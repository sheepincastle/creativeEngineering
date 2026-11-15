/**
 * @file main.cpp
 * @brief 팔 구동부 (Forearm Actuator & Server) - ESP32 DevKitC V4 (듀얼 모터 / 롤백 호환)
 * @details 1. 2개의 서보모터(1번: 바깥쪽 1·2번 마디, 2번: 손날 쪽 3번 마디) 및 2개의 플렉스 센서 피드백
 *          2. 채널별 가변 N단계 LUT(플렉스 선형화 또는 arccos 기본값) 기반 압력 목표 각도 산출
 *          3. '각도 변화 요구치(desiredAngle - currentAngle)'와 지수함수적 변화량(alpha) 제어
 *          4. 1번 모터 작동 시 3번 마디 간섭을 사전 계산하여 1·2번 모터를 동시 구동하는 전향 교차 결합 보상(Feedforward Decoupling)
 *          5. 내장 버튼(GPIO 0) 일시정지/잠금(Pause & Lock), 통신 단절 복귀 및 과부하(Stall) 안전 제한
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
#include <stdarg.h>

#include "protocol.h"
#include "lut_calibrator.h"

// 핀 정의 (ADC1 전용 입력 핀만 사용하여 Wi-Fi/ESP-NOW 충돌 원천 방지)
#define SERVO_PIN_1 18          // 1번 서보모터 신호 핀 (바깥쪽 1, 2번 마디 구동)
#define SERVO_PIN_2 19          // 2번 서보모터 신호 핀 (손날 쪽 3번 마디 구동)
#define FLEX_PIN_1  35          // 1번 플렉스 센서 핀 (1, 2번 마디 각도 측정, ADC1_CH7, 3kΩ 풀다운)
#define FLEX_PIN_2  34          // 2번 플렉스 센서 핀 (3번 마디 각도 측정, ADC1_CH6, 3kΩ 풀다운)
#define PAUSE_BTN_PIN 0         // 일시정지/잠금 토글용 내장 BOOT 버튼 (GPIO 0, Active LOW)
#define STATUS_LED_PIN 2        // 동작/일시정지 상태 표시용 내장 LED (GPIO 2)

#if NUM_MOTORS >= 2
const uint8_t SERVO_PINS[NUM_MOTORS] = {SERVO_PIN_1, SERVO_PIN_2};
const uint8_t FLEX_PINS[NUM_MOTORS]  = {FLEX_PIN_1, FLEX_PIN_2};
const char*   NVS_PREFIX[NUM_MOTORS] = {"m1_", "m2_"};
#else
const uint8_t SERVO_PINS[NUM_MOTORS] = {SERVO_PIN_1};
const uint8_t FLEX_PINS[NUM_MOTORS]  = {FLEX_PIN_1};
const char*   NVS_PREFIX[NUM_MOTORS] = {"m1_"};
#endif

// 서보 펄스 폭 설정 (PDI-6221MG: 500us ~ 2500us)
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500

// 제어 주기 및 안전 타이머 기본 상수
#define CONTROL_INTERVAL_MS 20          // 서보 제어 루프 주기 (20ms, 50Hz)
#define TIMEOUT_FAILSAFE_MS 300         // 300ms 패킷 단절 시 신전 한계각으로 복귀
#define DEFAULT_STALL_PROTECT_MS 5000   // 각도 변화 요구치 과부하 지속 허용 기본 시간 (5초)
#define DEFAULT_STALL_CUTOFF 150        // 과부하 감지 커트라인 기본 요구치 (150도 초과)
#define DEFAULT_STALL_SAFE_HOLD 140     // 과부하 시 강제 조정할 안전 유지 기본 요구치 (140도)
#define BTN_DEBOUNCE_MS 250             // 버튼 채터링 방지 디바운스 시간 (250ms)
#define TELEMETRY_INTERVAL_MS 100       // 웹소켓 실시간 데이터 전송 주기

// 전역 객체
Servo fingerServo[NUM_MOTORS];
LutCalibrator calibrator[NUM_MOTORS];
Preferences prefs;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// 런타임 설정 파라미터
DeviceConfig config = {
    .ema_alpha = 0.25f,
    .max_step_alpha = 35.0f,                    // 요구치 최대(180도)일 때 지수함수 최대 변화량 (기본 35도)
    .exp_curve_k = 2.5f,                        // 지수함수 곡률 상수 k (기본 2.5)
    .coupling_gamma = 0.25f,                    // 1번 모터 작동 시 3번 마디에 미치는 간섭 보상 계수 γ (기본 0.25)
    .stall_cutoff_demand = DEFAULT_STALL_CUTOFF,// 과부하 감지 커트라인 요구치 (기본 150도)
    .stall_safe_demand = DEFAULT_STALL_SAFE_HOLD,// 과부하 시 제한할 안전 유지 요구치 (기본 140도)
    .stall_protect_ms = DEFAULT_STALL_PROTECT_MS,// 과부하 지속 허용 제한 시간 (기본 5000ms)
#if NUM_MOTORS >= 2
    .extend_limit_angle = {0, 0},
    .flex_limit_angle = {180, 180}
#else
    .extend_limit_angle = {0},
    .flex_limit_angle = {180}
#endif
};

// 채널별 핵심 제어 변수 ([0]: 1번 모터/1·2번 마디, [1]: 2번 모터/3번 마디)
volatile uint16_t currentPressure[NUM_MOTORS] = {0};
volatile unsigned long lastPacketTime = 0;

uint8_t desiredAngle[NUM_MOTORS] = {0};       // 1. 각 압력 센서에 따른 목표 각도 (LUT 변환값)
uint8_t currentAngle[NUM_MOTORS] = {0};       // 2. 각 플렉스 센서에 따른 현재 실제 마디 각도
uint8_t servoTargetAngle[NUM_MOTORS] = {0};   // 3. 각 서보모터의 이번 주기 최종 지령 각도
int16_t angleDemand[NUM_MOTORS] = {0};        // 4. 각도 변화 요구치 (desiredAngle - currentAngle)
int16_t appliedAlpha[NUM_MOTORS] = {0};       // 5. 지수함수 및 전향 보상을 거쳐 도출된 실제 각도 변화 목표량(α)
int16_t rawPureAlpha[NUM_MOTORS] = {0};       // 6. 전향 보상 적용 전 순수 지수함수 변화량

// 일시정지/잠금(Pause & Lock) 및 안전 타이머 상태 변수
bool isPaused = false;                        // true: 일시정지/잠금 상태
bool isCalibratingNow = false;                // true: 캘리브레이션 스윕 진행 중
bool calibRequested = false;                  // 비동기 웹소켓 캘리브레이션 요청 플래그
bool lastBtnState = HIGH;
unsigned long lastBtnPressTime = 0;

unsigned long highDemandStartTime[NUM_MOTORS] = {0};
bool isStallProtected[NUM_MOTORS] = {false};
unsigned long lastControlTime = 0;

/**
 * @brief 시리얼 모니터 및 웹 대시보드 터미널로 시스템 로그 동시 전송
 */
void sendSysLog(const char *level, const char *fmt, ...) {
    char msgBuf[192];
    va_list args;
    va_start(args, fmt);
    vsnprintf(msgBuf, sizeof(msgBuf), fmt, args);
    va_end(args);

    Serial.println(msgBuf);

    if (ws.count() > 0) {
        StaticJsonDocument<256> doc;
        doc["type"] = "log";
        doc["level"] = level;
        doc["msg"] = msgBuf;
        String out;
        serializeJson(doc, out);
        ws.textAll(out);
    }
}

/**
 * @brief 발 송신부로 EMA 필터 계수(ema_alpha) 동기화 패킷 전송
 */
void syncFootConfig() {
    FootConfigPacket pkt = { .ema_alpha = config.ema_alpha };
    esp_now_send(broadcastAddress, (uint8_t *)&pkt, sizeof(pkt));
}

/**
 * @brief 웹 클라이언트에 현재 설정된 파라미터 전체 전송
 */
void broadcastConfig() {
    if (ws.count() == 0) return;

    StaticJsonDocument<384> doc;
    doc["type"] = "config";
    doc["ema_alpha"] = config.ema_alpha;
    doc["max_step"] = config.max_step_alpha;
    doc["exp_k"] = config.exp_curve_k;
    doc["gamma"] = config.coupling_gamma;
    doc["stall_cut"] = config.stall_cutoff_demand;
    doc["stall_safe"] = config.stall_safe_demand;
    doc["stall_ms"] = config.stall_protect_ms;
    doc["ext1"] = calibrator[MOTOR_OUTER_IDX].extendLimitAngle;
    doc["flx1"] = calibrator[MOTOR_OUTER_IDX].flexLimitAngle;
    doc["cal1"] = calibrator[MOTOR_OUTER_IDX].isCalibrated;
#if NUM_MOTORS >= 2
    doc["ext2"] = calibrator[MOTOR_BASE_IDX].extendLimitAngle;
    doc["flx2"] = calibrator[MOTOR_BASE_IDX].flexLimitAngle;
    doc["cal2"] = calibrator[MOTOR_BASE_IDX].isCalibrated;
#endif

    String out;
    serializeJson(doc, out);
    ws.textAll(out);
}

/**
 * @brief 일시정지/잠금(Pause & Lock) 상태 전환 함수 (모든 서보 동시 잠금/해제)
 */
void togglePauseState() {
    isPaused = !isPaused;
    if (isPaused) {
        // 잠금 진입: 모든 모터를 신전 한계각으로 편 뒤 서보 PWM 신호를 분리(detach)하여 모터 힘/전류 해제
        for (int i = 0; i < NUM_MOTORS; i++) {
            fingerServo[i].write(calibrator[i].extendLimitAngle);
            highDemandStartTime[i] = 0;
            isStallProtected[i] = false;
            appliedAlpha[i] = 0;
        }
        delay(150);
        for (int i = 0; i < NUM_MOTORS; i++) {
            fingerServo[i].detach();
        }
        digitalWrite(STATUS_LED_PIN, LOW); // 상태 LED 소등
        sendSysLog("warn", "[LOCK] 일시정지/잠금 모드 활성화 (모든 서보 토크 해제 및 입력 차단)");
    } else {
        // 잠금 해제: 모든 서보 PWM 재연결 및 정상 구동 복귀
        for (int i = 0; i < NUM_MOTORS; i++) {
            fingerServo[i].attach(SERVO_PINS[i], SERVO_MIN_US, SERVO_MAX_US);
        }
        digitalWrite(STATUS_LED_PIN, HIGH); // 상태 LED 점등
        sendSysLog("info", "[UNLOCK] 일시정지 해제 -> 정상 구동 모드 재개");
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
    if (k < 0.05f) k = 0.05f;
    float expFactor = (expf(k * normalized) - 1.0f) / (expf(k) - 1.0f);

    float rawAlpha = config.max_step_alpha * expFactor;
    int16_t step = (int16_t)round(rawAlpha);

    if (step < 1) step = 1;

    return sign * step;
}

/**
 * @brief 다중 모터 순차 캘리브레이션 수행
 */
void runSystemCalibration() {
    isCalibratingNow = true;
#if NUM_MOTORS >= 2
    sendSysLog("calib", "[CALIB] 듀얼 모터 순차 캘리브레이션 시작 (2번 모터[3번 마디] -> 1번 모터[1·2번 마디])");
    // 1. 1번 모터를 중간 각도(90도)에 두고 2번 모터(3번 마디) 먼저 캘리브레이션
    fingerServo[MOTOR_OUTER_IDX].write(90);
    delay(300);
    calibrator[MOTOR_BASE_IDX].runFullFlexCalibration(fingerServo[MOTOR_BASE_IDX]);
    config.extend_limit_angle[MOTOR_BASE_IDX] = calibrator[MOTOR_BASE_IDX].extendLimitAngle;
    config.flex_limit_angle[MOTOR_BASE_IDX]   = calibrator[MOTOR_BASE_IDX].flexLimitAngle;
    sendSysLog("calib", "[CALIB #2] 3번 마디 완료: 가동범위 %u° ~ %u° (%u단계 LUT, 보정=%s)",
               calibrator[MOTOR_BASE_IDX].extendLimitAngle,
               calibrator[MOTOR_BASE_IDX].flexLimitAngle,
               calibrator[MOTOR_BASE_IDX].lutSteps,
               calibrator[MOTOR_BASE_IDX].isCalibrated ? "성공" : "기본값");

    // 2. 2번 모터가 신전 한계각을 유지한 상태에서 1번 모터(1·2번 마디) 캘리브레이션
    calibrator[MOTOR_OUTER_IDX].runFullFlexCalibration(fingerServo[MOTOR_OUTER_IDX]);
    config.extend_limit_angle[MOTOR_OUTER_IDX] = calibrator[MOTOR_OUTER_IDX].extendLimitAngle;
    config.flex_limit_angle[MOTOR_OUTER_IDX]   = calibrator[MOTOR_OUTER_IDX].flexLimitAngle;
    sendSysLog("calib", "[CALIB #1] 1·2번 마디 완료: 가동범위 %u° ~ %u° (%u단계 LUT, 보정=%s)",
               calibrator[MOTOR_OUTER_IDX].extendLimitAngle,
               calibrator[MOTOR_OUTER_IDX].flexLimitAngle,
               calibrator[MOTOR_OUTER_IDX].lutSteps,
               calibrator[MOTOR_OUTER_IDX].isCalibrated ? "성공" : "기본값");
#else
    sendSysLog("calib", "[CALIB] 단일 모터 캘리브레이션 시작...");
    calibrator[0].runFullFlexCalibration(fingerServo[0]);
    config.extend_limit_angle[0] = calibrator[0].extendLimitAngle;
    config.flex_limit_angle[0]   = calibrator[0].flexLimitAngle;
    sendSysLog("calib", "[CALIB #1] 완료: 가동범위 %u° ~ %u° (%u단계 LUT)",
               calibrator[0].extendLimitAngle, calibrator[0].flexLimitAngle, calibrator[0].lutSteps);
#endif

    for (int i = 0; i < NUM_MOTORS; i++) {
        servoTargetAngle[i] = calibrator[i].extendLimitAngle;
        fingerServo[i].write(servoTargetAngle[i]);
    }
    isCalibratingNow = false;
    broadcastConfig();
}

// NVS 설정 로드 및 저장
bool loadSettings() {
    prefs.begin("sixth_finger", false);
    config.ema_alpha = prefs.getFloat("alpha", 0.25f);
    config.max_step_alpha = prefs.getFloat("max_step", 35.0f);
    config.exp_curve_k = prefs.getFloat("exp_k", 2.5f);
    config.coupling_gamma = prefs.getFloat("gamma", 0.25f);
    config.stall_cutoff_demand = prefs.getUChar("stl_cut", DEFAULT_STALL_CUTOFF);
    config.stall_safe_demand = prefs.getUChar("stl_safe", DEFAULT_STALL_SAFE_HOLD);
    config.stall_protect_ms = prefs.getUInt("stl_ms", DEFAULT_STALL_PROTECT_MS);

    bool allLoaded = true;
    for (int i = 0; i < NUM_MOTORS; i++) {
        bool ok = calibrator[i].loadFromNvs(prefs);
        config.extend_limit_angle[i] = calibrator[i].extendLimitAngle;
        config.flex_limit_angle[i]   = calibrator[i].flexLimitAngle;
        if (!ok) allLoaded = false;
    }
    return allLoaded;
}

void saveSettings() {
    prefs.putFloat("alpha", config.ema_alpha);
    prefs.putFloat("max_step", config.max_step_alpha);
    prefs.putFloat("exp_k", config.exp_curve_k);
    prefs.putFloat("gamma", config.coupling_gamma);
    prefs.putUChar("stl_cut", config.stall_cutoff_demand);
    prefs.putUChar("stl_safe", config.stall_safe_demand);
    prefs.putUInt("stl_ms", config.stall_protect_ms);

    for (int i = 0; i < NUM_MOTORS; i++) {
        calibrator[i].saveToNvs(prefs);
    }
    sendSysLog("info", "[NVS] 파라미터(α=%.2f, max=%.1f°, k=%.2f, γ=%.2f, 제한=%u°/%lums) 및 LUT 영구 저장 완료",
               config.ema_alpha, config.max_step_alpha, config.exp_curve_k,
               config.coupling_gamma, config.stall_safe_demand, (unsigned long)config.stall_protect_ms);
}

// ESP-NOW 패킷 수신 콜백
void onDataReceived(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
    if (data_len == sizeof(FootSensorPacket)) {
        const FootSensorPacket *pkt = (const FootSensorPacket *)data;
        for (int i = 0; i < NUM_MOTORS; i++) {
            currentPressure[i] = pkt->raw_pressure[i];
        }
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
        if (!cmd) return;

        if (strcmp(cmd, "set_alpha") == 0) {
            config.ema_alpha = constrain(doc["val"].as<float>(), 0.05f, 0.95f);
            syncFootConfig();
            sendSysLog("info", "[PARAM] 발 압력 EMA 필터 계수(ema_alpha) = %.2f 적용", config.ema_alpha);
        } else if (strcmp(cmd, "set_max_step") == 0) {
            config.max_step_alpha = constrain(doc["val"].as<float>(), 1.0f, 90.0f);
            sendSysLog("info", "[PARAM] 지수함수 최대 변화량(max_step_alpha) = %.1f° 적용", config.max_step_alpha);
        } else if (strcmp(cmd, "set_exp_k") == 0) {
            config.exp_curve_k = constrain(doc["val"].as<float>(), 0.1f, 8.0f);
            sendSysLog("info", "[PARAM] 지수함수 곡률 상수(exp_curve_k) = %.2f 적용", config.exp_curve_k);
        } else if (strcmp(cmd, "set_gamma") == 0) {
            config.coupling_gamma = constrain(doc["val"].as<float>(), 0.0f, 1.0f);
            sendSysLog("info", "[PARAM] 3번 마디 전향 간섭 보상 계수(coupling_gamma) = %.2f 적용", config.coupling_gamma);
        } else if (strcmp(cmd, "set_stall_params") == 0) {
            if (doc.containsKey("safe")) {
                config.stall_safe_demand = (uint8_t)constrain(doc["safe"].as<int>(), 30, 175);
            }
            if (doc.containsKey("cutoff")) {
                config.stall_cutoff_demand = (uint8_t)constrain(doc["cutoff"].as<int>(), (int)config.stall_safe_demand, 180);
            } else {
                config.stall_cutoff_demand = (uint8_t)constrain((int)config.stall_safe_demand + 10, 40, 180);
            }
            if (doc.containsKey("ms")) {
                config.stall_protect_ms = (uint32_t)constrain(doc["ms"].as<long>(), 500L, 30000L);
            }
            sendSysLog("info", "[PARAM] 과부하 제한 설정: 감지>%u°, 제한각도=%u°, 제한시간=%lums",
                       config.stall_cutoff_demand, config.stall_safe_demand, (unsigned long)config.stall_protect_ms);
        } else if (strcmp(cmd, "get_config") == 0) {
            broadcastConfig();
        } else if (strcmp(cmd, "toggle_pause") == 0) {
            togglePauseState();
        } else if (strcmp(cmd, "reset_lut") == 0) {
            for (int i = 0; i < NUM_MOTORS; i++) {
                calibrator[i].generateArccosLut();
            }
            sendSysLog("warn", "[LUT] 모든 채널이 역코사인(arccos) 기본 LUT로 복원되었습니다.");
            broadcastConfig();
        } else if (strcmp(cmd, "start_calib") == 0) {
            calibRequested = true; // 메인 loop()에서 안전하게 실행하도록 플래그 설정
        } else if (strcmp(cmd, "save_nvs") == 0) {
            saveSettings();
        }
    }
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        broadcastConfig();
        sendSysLog("info", "[WS] 웹 대시보드 클라이언트 연결됨 (ID: %u)", client->id());
    } else if (type == WS_EVT_DATA) {
        handleWebSocketMessage(arg, data, len);
    }
}

// 실시간 텔레메트리 웹 브로드캐스트
void broadcastTelemetry() {
    if (ws.count() == 0) return;

    StaticJsonDocument<512> doc;
    doc["type"] = "telemetry";

    // 1번 모터 (바깥쪽 1, 2번 마디) 상태
    doc["p"]      = currentPressure[MOTOR_OUTER_IDX];
    doc["des"]    = desiredAngle[MOTOR_OUTER_IDX];
    doc["a"]      = currentAngle[MOTOR_OUTER_IDX];
    doc["dem"]    = angleDemand[MOTOR_OUTER_IDX];
    doc["step"]   = appliedAlpha[MOTOR_OUTER_IDX];
    doc["target"] = servoTargetAngle[MOTOR_OUTER_IDX];
    doc["ext"]    = calibrator[MOTOR_OUTER_IDX].extendLimitAngle;
    doc["flx"]    = calibrator[MOTOR_OUTER_IDX].flexLimitAngle;
    doc["cal"]    = calibrator[MOTOR_OUTER_IDX].isCalibrated;
    doc["stall"]  = isStallProtected[MOTOR_OUTER_IDX];

#if NUM_MOTORS >= 2
    // 2번 모터 (손날 쪽 3번 마디) 상태
    doc["p2"]      = currentPressure[MOTOR_BASE_IDX];
    doc["des2"]    = desiredAngle[MOTOR_BASE_IDX];
    doc["a2"]      = currentAngle[MOTOR_BASE_IDX];
    doc["dem2"]    = angleDemand[MOTOR_BASE_IDX];
    doc["step2"]   = appliedAlpha[MOTOR_BASE_IDX];
    doc["pure2"]   = rawPureAlpha[MOTOR_BASE_IDX];
    doc["target2"] = servoTargetAngle[MOTOR_BASE_IDX];
    doc["ext2"]    = calibrator[MOTOR_BASE_IDX].extendLimitAngle;
    doc["flx2"]    = calibrator[MOTOR_BASE_IDX].flexLimitAngle;
    doc["cal2"]    = calibrator[MOTOR_BASE_IDX].isCalibrated;
    doc["stall2"]  = isStallProtected[MOTOR_BASE_IDX];
#endif

    // 시스템 공통 상태
    doc["paused"]      = isPaused;
    doc["calibrating"] = isCalibratingNow;
    doc["online"]      = (millis() - lastPacketTime < TIMEOUT_FAILSAFE_MS);

    String output;
    serializeJson(doc, output);
    ws.textAll(output);
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n==========================================");
    Serial.printf("  여섯 번째 손가락 - 팔 구동부 기동 (%d모터 모드)\n", NUM_MOTORS);
    Serial.println("==========================================");

    // 핀 초기화 (플렉스 센서 ADC, 일시정지 버튼, 상태 LED)
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    pinMode(PAUSE_BTN_PIN, INPUT_PULLUP);
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, HIGH); // 부팅 시 정상 구동 상태(LED ON)

    // 채널별 플렉스 센서 및 서보 모터 초기화 (PDI-6221MG)
    for (int i = 0; i < NUM_MOTORS; i++) {
        pinMode(FLEX_PINS[i], INPUT);
        calibrator[i].configure(FLEX_PINS[i], NVS_PREFIX[i], i + 1);

        ESP32PWM::allocateTimer(i);
        fingerServo[i].setPeriodHertz(50);
        fingerServo[i].attach(SERVO_PINS[i], SERVO_MIN_US, SERVO_MAX_US);
    }

    // NVS에서 LUT 및 양끝 한계 각도 로드 (저장된 설정이 없으면 부팅 시 순차 한계 탐색 수행)
    bool hasSavedLut = loadSettings();
    if (!hasSavedLut) {
        runSystemCalibration();
    } else {
        for (int i = 0; i < NUM_MOTORS; i++) {
            servoTargetAngle[i] = calibrator[i].extendLimitAngle;
            fingerServo[i].write(servoTargetAngle[i]);
        }
    }

    // LittleFS 마운트
    LittleFS.begin(true);

    // Wi-Fi SoftAP 및 ESP-NOW 초기화
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP("Sixth-Finger-AP", "", WIFI_ESP_NOW_CHANNEL);
    esp_wifi_set_channel(WIFI_ESP_NOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() == ESP_OK) {
        esp_now_register_recv_cb(onDataReceived);

        // 발 송신부로 설정(ema_alpha) 전송을 위한 브로드캐스트 피어 등록
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, broadcastAddress, 6);
        peerInfo.channel = WIFI_ESP_NOW_CHANNEL;
        peerInfo.encrypt = false;
        esp_now_add_peer(&peerInfo);
        syncFootConfig();
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

    // 0-A. 일시정지/잠금 버튼(GPIO 0) 입력 확인
    checkPauseButton();

    // 0-B. 웹소켓 캘리브레이션 요청 처리 (비동기 콜백 밖에서 안전하게 실행)
    if (calibRequested) {
        calibRequested = false;
        if (isPaused) togglePauseState();
        runSystemCalibration();
        lastControlTime = millis();
        return;
    }

    // 20ms(50Hz) 주기로 서보모터 제어 루프 실행
    if (now - lastControlTime >= CONTROL_INTERVAL_MS) {
        lastControlTime = now;

        // 1. 각 플렉스 센서를 통해 현재 실제 마디 각도(currentAngle) 측정
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (calibrator[i].isCalibrated) {
                currentAngle[i] = calibrator[i].readCurrentServoAngle();
            } else {
                currentAngle[i] = servoTargetAngle[i];
            }
        }

        // [일시정지/잠금 모드 체크] 잠금 상태면 모터 제어를 건너뛰어 대기 상태 유지
        if (isPaused) {
            return;
        }

        // 2. 통신 단절 Fail-Safe 검사 (300ms 타임아웃) 및 압력 -> 목표 각도 LUT 변환
        bool isTimeout = (now - lastPacketTime > TIMEOUT_FAILSAFE_MS);

        for (int i = 0; i < NUM_MOTORS; i++) {
            if (isTimeout) {
                desiredAngle[i] = calibrator[i].extendLimitAngle;
                highDemandStartTime[i] = 0;
                isStallProtected[i] = false;
            } else {
                desiredAngle[i] = calibrator[i].mapPressureToAngle(currentPressure[i]);
            }

            // 3. 채널별 '각도 변화 요구치(angleDemand)' 산출
            angleDemand[i] = (int16_t)desiredAngle[i] - (int16_t)currentAngle[i];

            // 4. 채널별 과부하(Stall) 커트라인 검사 (동적 설정된 커트라인 및 제한 시간 적용)
            if (angleDemand[i] > (int16_t)config.stall_cutoff_demand) {
                if (highDemandStartTime[i] == 0) {
                    highDemandStartTime[i] = now;
                } else if (now - highDemandStartTime[i] >= config.stall_protect_ms) {
                    if (!isStallProtected[i]) {
                        isStallProtected[i] = true;
                        sendSysLog("warn", "[STALL #%d] 요구치 %u° 초과 %lums 지속 -> 안전 요구치 %u°로 자동 제한",
                                   i + 1, config.stall_cutoff_demand,
                                   (unsigned long)config.stall_protect_ms, config.stall_safe_demand);
                    }
                    angleDemand[i] = (int16_t)config.stall_safe_demand;
                }
            } else {
                if (isStallProtected[i]) {
                    sendSysLog("info", "[STALL #%d] 과부하 보호 해제 -> 정상 요구치 복귀", i + 1);
                }
                highDemandStartTime[i] = 0;
                isStallProtected[i] = false;
            }

            // 5-A. 각 모터의 순수 지수함수적 변화량(rawPureAlpha) 산출
            rawPureAlpha[i] = computeExponentialAlpha(angleDemand[i]);
        }

        // 5-B. 1번 모터(바깥쪽 1, 2번 마디) 이동 목표 각도 산출
        int16_t alpha1 = rawPureAlpha[MOTOR_OUTER_IDX];
        appliedAlpha[MOTOR_OUTER_IDX] = alpha1;
        int16_t nextServo1 = (int16_t)currentAngle[MOTOR_OUTER_IDX] + alpha1;
        servoTargetAngle[MOTOR_OUTER_IDX] = (uint8_t)constrain(
            nextServo1,
            (int16_t)calibrator[MOTOR_OUTER_IDX].extendLimitAngle,
            (int16_t)calibrator[MOTOR_OUTER_IDX].flexLimitAngle
        );

#if NUM_MOTORS >= 2
        // 5-C. 2번 모터(손날 쪽 3번 마디) 전향 교차 결합 보상(Feedforward Decoupling) 제어
        int16_t couplingComp = (int16_t)round(config.coupling_gamma * (float)alpha1);
        int16_t alpha2_final = rawPureAlpha[MOTOR_BASE_IDX] - couplingComp;
        appliedAlpha[MOTOR_BASE_IDX] = alpha2_final;

        // 외력(1번 모터 장력 등)에 의해 3번 마디가 목표 각도(desiredAngle[1]) 너머로 끌려갔을 때 기준점 고정(Anchor)
        int16_t baseAngle2 = (int16_t)currentAngle[MOTOR_BASE_IDX];
        int16_t target2    = (int16_t)desiredAngle[MOTOR_BASE_IDX];
        int16_t prevServo2 = (int16_t)servoTargetAngle[MOTOR_BASE_IDX];

        if (baseAngle2 > target2 && prevServo2 <= target2) {
            baseAngle2 = target2;
        } else if (baseAngle2 < target2 && prevServo2 >= target2) {
            baseAngle2 = target2;
        }

        int16_t nextServo2 = baseAngle2 + alpha2_final;
        servoTargetAngle[MOTOR_BASE_IDX] = (uint8_t)constrain(
            nextServo2,
            (int16_t)calibrator[MOTOR_BASE_IDX].extendLimitAngle,
            (int16_t)calibrator[MOTOR_BASE_IDX].flexLimitAngle
        );
#endif

        // 6. 모든 서보모터 동시 PWM 출력 (양방향 왕복 와이어 구동)
        for (int i = 0; i < NUM_MOTORS; i++) {
            fingerServo[i].write(servoTargetAngle[i]);
        }
    }

    // 실시간 텔레메트리 웹 브로드캐스트 (100ms 주기)
    static unsigned long lastTelemetryTime = 0;
    if (now - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
        lastTelemetryTime = now;
        broadcastTelemetry();
    }
}
