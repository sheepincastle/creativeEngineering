/**
 * @file main.cpp
 * @brief 팔 구동부 (Forearm Actuator & Server) - ESP32 DevKitC V4
 * @details 1. 플렉스 센서(3kΩ 분배) 기반 양끝 한계 탐색 및 실시간 현재 각도 피드백
 *          2. 가변 N단계 LUT(플렉스 선형화 또는 arccos 기본값) 기반 압력 목표 각도 산출
 *          3. '각도 변화 요구치(desiredAngle - currentAngle)'와 지수함수적 변화량(alpha)을 이용한 단일 통합 서보 제어
 *          4. 내장 버튼(GPIO 0) 클릭 시 보행 중 오작동 방지를 위한 일시정지/잠금(Pause & Lock) 토글 기능
 *          5. 통신 단절 시 신전 한계각 복귀 및 요구치 150° 초과 5초 지속 시 140° 안전 제한(Fail-Safe)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ESP32Servo.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <math.h>

#include "protocol.h"
#include "lut_calibrator.h"

// 핀 정의
#define SERVO_PIN 18            // PDI-6221MG 서보모터 신호 핀 (PWM)
#define FLEX_PIN 35             // 플렉스 센서 상시 피드백 및 보정용 핀 (ADC1_CH7, 3kΩ 풀다운)
#define PAUSE_BTN_PIN 0         // 일시정지/잠금 토글용 내장 BOOT 버튼 (GPIO 0, Active LOW)
#define STATUS_LED_PIN 2        // 동작/일시정지 상태 표시용 내장 LED (GPIO 2)

// 서보 펄스 폭 설정 (PDI-6221MG: 500us ~ 2500us)
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500

// 제어 주기 및 안전 타이머 상수
#define CONTROL_INTERVAL_MS 20      // 서보 제어 루프 주기 (20ms, 50Hz)
#define TIMEOUT_FAILSAFE_MS 300     // 300ms 패킷 단절 시 신전 한계각으로 복귀
#define STALL_PROTECT_MS 5000       // 각도 변화 요구치 과부하 지속 허용 시간 (5초)
#define STALL_CUTOFF_DEMAND 150     // 과부하 감지 커트라인 각도 변화 요구치 (150도 초과)
#define STALL_SAFE_HOLD_DEMAND 140  // 과부하 시 강제 조정할 안전 유지 요구치 (140도 -> +19도 유지)
#define BTN_DEBOUNCE_MS 250         // 버튼 채터링 방지 디바운스 시간 (250ms)
#define TELEMETRY_INTERVAL_MS 100   // 웹소켓 실시간 데이터 전송 주기

// 전역 객체
Servo fingerServo;
LutCalibrator calibrator;
Preferences prefs;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// 런타임 설정 파라미터
DeviceConfig config = {
    .ema_alpha = 0.25f,
    .max_step_alpha = 35.0f,    // 요구치 최대(180도)일 때 지수함수 최대 변화량 (35도 -> 요구치 140도일 때 +19도 확보)
    .exp_curve_k = 2.5f,        // 지수함수 곡률 상수 k (2.5)
    .extend_limit_angle = 0,
    .flex_limit_angle = 180
};

// 핵심 제어 변수
volatile uint16_t currentPressure = 0;
volatile unsigned long lastPacketTime = 0;

uint8_t desiredAngle = 0;       // 1. 압력 센서에 따른 목표 각도 (LUT 변환값)
uint8_t currentAngle = 0;       // 2. 플렉스 센서에 따른 현재 실제 서보모터 각도
uint8_t servoTargetAngle = 0;   // 3. 서보모터의 이번 주기 이동 목표 각도 (currentAngle + alpha)
int16_t angleDemand = 0;        // 4. 각도 변화 요구치 (desiredAngle - currentAngle)

// 일시정지/잠금(Pause & Lock) 및 안전 타이머 상태 변수
bool isPaused = false;          // true: 일시정지/잠금 상태 (보행 중 오작동 방지 및 서보 토크 해제)
bool lastBtnState = HIGH;
unsigned long lastBtnPressTime = 0;

unsigned long highDemandStartTime = 0;
bool isStallProtected = false;
unsigned long lastControlTime = 0;

/**
 * @brief 일시정지/잠금(Pause & Lock) 상태 전환 함수
 */
void togglePauseState() {
    isPaused = !isPaused;
    if (isPaused) {
        // 잠금 진입: 신전 한계각으로 편 뒤 서보 PWM 신호를 분리(detach)하여 모터 힘/전류 해제
        fingerServo.write(calibrator.extendLimitAngle);
        delay(150);
        fingerServo.detach();
        digitalWrite(STATUS_LED_PIN, LOW); // 상태 LED 소등
        highDemandStartTime = 0;
        isStallProtected = false;
        Serial.println("[LOCK] 일시정지/잠금 모드 활성화 (서보 토크 해제 및 입력 차단)");
    } else {
        // 잠금 해제: 서보 PWM 재연결 및 정상 구동 복귀
        fingerServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
        digitalWrite(STATUS_LED_PIN, HIGH); // 상태 LED 점등
        Serial.println("[UNLOCK] 일시정지 해제 -> 정상 구동 모드 재개");
    }
}

/**
 * @brief 내장 버튼(GPIO 0) 입력 확인 및 디바운싱 처리
 */
void checkPauseButton() {
    bool reading = digitalRead(PAUSE_BTN_PIN);
    unsigned long now = millis();

    if (reading == LOW && lastBtnState == HIGH && (now - lastBtnPressTime > BTN_DEBOUNCE_MS)) {
        lastBtnPressTime = now;
        togglePauseState();
    }
    lastBtnState = reading;
}

/**
 * @brief 각도 변화 요구치(demand)에 따른 지수함수적 변화량 alpha 계산
 */
int16_t computeExponentialAlpha(int16_t demand) {
    if (demand == 0) return 0;

    int16_t sign = (demand > 0) ? 1 : -1;
    float absDemand = (float)abs(demand);
    if (absDemand > 180.0f) absDemand = 180.0f;

    float normalized = absDemand / 180.0f; // 0.0 ~ 1.0
    float k = config.exp_curve_k;
    float expFactor = (expf(k * normalized) - 1.0f) / (expf(k) - 1.0f);

    float rawAlpha = config.max_step_alpha * expFactor;
    int16_t step = (int16_t)round(rawAlpha);

    if (step < 1) step = 1;

    return sign * step;
}

// NVS 설정 로드 및 저장
bool loadSettings() {
    prefs.begin("sixth_finger", false);
    config.ema_alpha = prefs.getFloat("alpha", 0.25f);
    config.max_step_alpha = prefs.getFloat("max_step", 35.0f);
    config.exp_curve_k = prefs.getFloat("exp_k", 2.5f);

    bool hasSavedLut = calibrator.loadFromNvs(prefs);
    config.extend_limit_angle = calibrator.extendLimitAngle;
    config.flex_limit_angle = calibrator.flexLimitAngle;
    return hasSavedLut;
}

void saveSettings() {
    prefs.putFloat("alpha", config.ema_alpha);
    prefs.putFloat("max_step", config.max_step_alpha);
    prefs.putFloat("exp_k", config.exp_curve_k);
    calibrator.saveToNvs(prefs);
    Serial.println("[NVS] 사용자 설정 및 가변 단계 LUT 영구 저장 완료");
}

// ESP-NOW 패킷 수신 콜백
void onDataReceived(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
    if (data_len == sizeof(FootSensorPacket)) {
        const FootSensorPacket *pkt = (const FootSensorPacket *)data;
        currentPressure = pkt->raw_pressure;
        lastPacketTime = millis();
    }
}

// WebSocket 이벤트 핸들러
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        data[len] = 0;
        DynamicJsonDocument doc(512);
        DeserializationError err = deserializeJson(doc, (char *)data);
        if (err) return;

        const char *cmd = doc["cmd"];
        if (strcmp(cmd, "set_alpha") == 0) {
            config.ema_alpha = doc["val"].as<float>();
        } else if (strcmp(cmd, "toggle_pause") == 0) {
            togglePauseState();
        } else if (strcmp(cmd, "reset_lut") == 0) {
            calibrator.generateArccosLut();
        } else if (strcmp(cmd, "start_calib") == 0) {
            if (isPaused) togglePauseState(); // 잠금 상태면 해제 후 캘리브레이션
            calibrator.runFullFlexCalibration(fingerServo);
            config.extend_limit_angle = calibrator.extendLimitAngle;
            config.flex_limit_angle = calibrator.flexLimitAngle;
        } else if (strcmp(cmd, "save_nvs") == 0) {
            saveSettings();
        }
    }
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_DATA) {
        handleWebSocketMessage(arg, data, len);
    }
}

// 실시간 텔레메트리 웹 브로드캐스트
void broadcastTelemetry() {
    if (ws.count() == 0) return;

    StaticJsonDocument<256> doc;
    doc["p"] = currentPressure;
    doc["a"] = currentAngle;
    doc["m"] = 0; // 웹 대시보드 호환용 필드
    doc["des"] = desiredAngle;
    doc["dem"] = angleDemand;
    doc["target"] = servoTargetAngle;
    doc["paused"] = isPaused;
    doc["stall"] = isStallProtected;
    doc["online"] = (millis() - lastPacketTime < TIMEOUT_FAILSAFE_MS);

    String output;
    serializeJson(doc, output);
    ws.textAll(output);
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n==========================================");
    Serial.println("  여섯 번째 손가락 - 팔 구동부 & 서버 기동");
    Serial.println("==========================================");

    // 핀 초기화 (플렉스 센서 ADC, 일시정지 버튼, 상태 LED)
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    pinMode(FLEX_PIN, INPUT);
    pinMode(PAUSE_BTN_PIN, INPUT_PULLUP);
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, HIGH); // 부팅 시 정상 구동 상태(LED ON)

    // 서보 모터 초기화 (PDI-6221MG)
    ESP32PWM::allocateTimer(0);
    fingerServo.setPeriodHertz(50);
    fingerServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);

    // NVS에서 LUT 및 양끝 한계 각도 로드 (저장된 설정이 없으면 부팅 시 자동 한계 탐색 수행)
    bool hasSavedLut = loadSettings();
    if (!hasSavedLut) {
        calibrator.runFullFlexCalibration(fingerServo);
        config.extend_limit_angle = calibrator.extendLimitAngle;
        config.flex_limit_angle = calibrator.flexLimitAngle;
    }

    // 초기 위치를 신전 한계각(extendLimitAngle)으로 정렬
    servoTargetAngle = calibrator.extendLimitAngle;
    fingerServo.write(servoTargetAngle);

    // LittleFS 마운트
    LittleFS.begin(true);

    // Wi-Fi SoftAP 및 ESP-NOW 초기화
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP("Sixth-Finger-AP", "", WIFI_ESP_NOW_CHANNEL);
    esp_wifi_set_channel(WIFI_ESP_NOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() == ESP_OK) {
        esp_now_register_recv_cb(onDataReceived);
    }

    // 웹서버 시작
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
    server.begin();
}

void loop() {
    unsigned long now = millis();
    ws.cleanupClients();

    // 0. 일시정지/잠금 버튼(GPIO 0) 입력 확인
    checkPauseButton();

    // 20ms(50Hz) 주기로 서보모터 제어 루프 실행
    if (now - lastControlTime >= CONTROL_INTERVAL_MS) {
        lastControlTime = now;

        // 1. 플렉스 센서를 통해 현재 실제 서보모터 각도(currentAngle) 측정
        // (만약 플렉스 센서 미보정 상태라면 직전 서보 목표 각도를 현재 각도로 활용)
        if (calibrator.isCalibrated) {
            currentAngle = calibrator.readCurrentServoAngle();
        } else {
            currentAngle = servoTargetAngle;
        }

        // [일시정지/잠금 모드 체크] 잠금 상태면 모터 제어를 건너뛰어 대기 상태 유지
        if (isPaused) {
            return;
        }

        // 2. 통신 단절 Fail-Safe 검사 (300ms 타임아웃)
        if (now - lastPacketTime > TIMEOUT_FAILSAFE_MS) {
            desiredAngle = calibrator.extendLimitAngle;
            highDemandStartTime = 0;
            isStallProtected = false;
        } else {
            // 정상 수신 시: 가변 N단계 LUT로 압력 목표 각도 산출
            desiredAngle = calibrator.mapPressureToAngle(currentPressure);
        }

        // 3. '각도 변화 요구치(angleDemand)' 산출
        angleDemand = (int16_t)desiredAngle - (int16_t)currentAngle;

        // 4. 과부하(Stall) 커트라인 검사 (150도 초과 5초 지속 시 -> 안전 유지 요구치 140도로 제한)
        if (angleDemand > STALL_CUTOFF_DEMAND) {
            if (highDemandStartTime == 0) {
                highDemandStartTime = now;
            } else if (now - highDemandStartTime >= STALL_PROTECT_MS) {
                isStallProtected = true;
                angleDemand = STALL_SAFE_HOLD_DEMAND;
            }
        } else {
            highDemandStartTime = 0;
            isStallProtected = false;
        }

        // 5. 각도 변화 요구치에 따른 지수함수적 변화량(alpha) 계산 및 서보 이동 목표 각도 갱신
        int16_t alpha = computeExponentialAlpha(angleDemand);
        int16_t nextServoAngle = (int16_t)currentAngle + alpha;

        servoTargetAngle = (uint8_t)constrain(
            nextServoAngle,
            (int16_t)calibrator.extendLimitAngle,
            (int16_t)calibrator.flexLimitAngle
        );

        // 6. 서보모터 PWM 출력 (양방향 왕복 와이어 구동)
        fingerServo.write(servoTargetAngle);
    }

    // 실시간 텔레메트리 웹 브로드캐스트 (100ms 주기)
    static unsigned long lastTelemetryTime = 0;
    if (now - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
        lastTelemetryTime = now;
        broadcastTelemetry();
    }
}
