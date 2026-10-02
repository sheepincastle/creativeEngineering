/**
 * @file main.cpp
 * @brief 팔 구동부 (Forearm Actuator & Server) - ESP32 DevKitC V4
 * @details ESP-NOW 패킷 수신, 하이브리드 서보 제어(연속/히스테리시스),
 *          Svelte 웹 대시보드 호스팅 및 실시간 WebSocket 통신
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

#include "protocol.h"
#include "lut_calibrator.h"

// 핀 정의
#define SERVO_PIN 18            // PDI-6221MG 서보모터 신호 핀 (PWM)
#define FLEX_PIN 35             // 플렉스 센서 보정용 핀 (ADC)

// 서보 펄스 폭 설정 (PDI-6221MG: 500us ~ 2500us)
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500

// 안전 타이머 상수
#define TIMEOUT_FAILSAFE_MS 300     // 300ms 패킷 단절 시 릴리즈
#define STALL_PROTECT_MS 5000       // 5초 연속 최대 굽힘 시 감압
#define TELEMETRY_INTERVAL_MS 100   // 웹소켓 실시간 데이터 전송 주기

// 전역 객체
Servo fingerServo;
LutCalibrator calibrator;
Preferences prefs;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// 런타임 제어 상태
DeviceConfig config = {
    .ema_alpha = 0.25f,
    .threshold_high = 2200,     // 12-bit 기준 (약 53%)
    .threshold_low = 800,       // 12-bit 기준 (약 19%)
    .control_mode = MODE_CONTINUOUS,
    .zero_offset = 0
};

volatile uint16_t currentPressure = 0;
volatile unsigned long lastPacketTime = 0;
uint8_t targetAngle = 0;
uint8_t currentAngle = 0;
bool isGripping = false; // Hysteresis 모드 상태 변수
unsigned long highTorqueStartTime = 0;
bool isStallProtected = false;

// NVS 설정 로드 및 저장
void loadSettings() {
    prefs.begin("sixth_finger", false);
    if (prefs.isKey("mode")) {
        config.control_mode = prefs.getUChar("mode", MODE_CONTINUOUS);
        config.threshold_high = prefs.getUShort("th_high", 2200);
        config.threshold_low = prefs.getUShort("th_low", 800);
        config.ema_alpha = prefs.getFloat("alpha", 0.25f);
        Serial.println("[NVS] 사용자 설정 로드 완료");
    }
    calibrator.loadFromNvs(prefs);
}

void saveSettings() {
    prefs.putUChar("mode", config.control_mode);
    prefs.putUShort("th_high", config.threshold_high);
    prefs.putUShort("th_low", config.threshold_low);
    prefs.putFloat("alpha", config.ema_alpha);
    calibrator.saveToNvs(prefs);
    Serial.println("[NVS] 사용자 설정 영구 저장 완료");
}

// ESP-NOW 패킷 수신 콜백
void onDataReceived(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
    if (data_len == sizeof(FootSensorPacket)) {
        FootSensorPacket *pkt = (FootSensorPacket *)data;
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
        if (strcmp(cmd, "set_mode") == 0) {
            config.control_mode = doc["val"].as<uint8_t>();
            Serial.printf("[WEB] 제어 모드 변경: %s\n", 
                          config.control_mode == MODE_CONTINUOUS ? "연속 비례" : "듀얼 임계값");
        } else if (strcmp(cmd, "set_threshold") == 0) {
            config.threshold_high = doc["high"].as<uint16_t>();
            config.threshold_low = doc["low"].as<uint16_t>();
            Serial.printf("[WEB] 임계값 변경: HIGH=%u, LOW=%u\n", 
                          config.threshold_high, config.threshold_low);
        } else if (strcmp(cmd, "set_alpha") == 0) {
            config.ema_alpha = doc["val"].as<float>();
            Serial.printf("[WEB] EMA 알파 변경: %.2f\n", config.ema_alpha);
        } else if (strcmp(cmd, "reset_lut") == 0) {
            calibrator.generateArccosLut();
            Serial.println("[WEB] arccos 기본 LUT로 초기화");
        } else if (strcmp(cmd, "start_calib") == 0) {
            calibrator.startFlexCalibration();
        } else if (strcmp(cmd, "finish_calib") == 0) {
            calibrator.finishFlexCalibration();
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
    doc["m"] = config.control_mode;
    doc["th_h"] = config.threshold_high;
    doc["th_l"] = config.threshold_low;
    doc["alpha"] = config.ema_alpha;
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

    // NVS 및 플렉스 센서 핀 초기화
    pinMode(FLEX_PIN, INPUT);
    loadSettings();

    // 서보 모터 초기화 (PDI-6221MG)
    ESP32PWM::allocateTimer(0);
    fingerServo.setPeriodHertz(50); // 표준 50Hz 서보
    fingerServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
    fingerServo.write(0);           // 부팅 시 완전 이완 위치(0도)

    // LittleFS 마운트 (Svelte 정적 파일 서빙용)
    if (!LittleFS.begin(true)) {
        Serial.println("[ERR] LittleFS 마운트 실패!");
    } else {
        Serial.println("[OK] LittleFS 파일시스템 준비 완료");
    }

    // Wi-Fi SoftAP 모드 시작 (SSID: Sixth-Finger-AP, 비밀번호 없음, 채널 1 고정)
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP("Sixth-Finger-AP", "", WIFI_ESP_NOW_CHANNEL);
    esp_wifi_set_channel(WIFI_ESP_NOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    Serial.print("[AP] Wi-Fi AP 개설 완료. IP 주소: ");
    Serial.println(WiFi.softAPIP());

    // ESP-NOW 초기화
    if (esp_now_init() != ESP_OK) {
        Serial.println("[ERR] ESP-NOW 초기화 실패!");
    } else {
        esp_now_register_recv_cb(onDataReceived);
        Serial.println("[OK] ESP-NOW 수신 모듈 가동");
    }

    // 웹서버 및 웹소켓 설정
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

    server.begin();
    Serial.println("[WEB] Svelte 대시보드 웹 서버 시작 (포트 80)");
}

void loop() {
    unsigned long now = millis();
    ws.cleanupClients();

    // 1. Fail-Safe 검사: 패킷 타임아웃
    if (now - lastPacketTime > TIMEOUT_FAILSAFE_MS) {
        targetAngle = 0; // 통신 단절 시 자동 릴리즈
    } 
    // 2. 모터 제어 파이프라인
    else {
        // [모드 A] 연속 비례 제어 (Continuous Mode)
        if (config.control_mode == MODE_CONTINUOUS) {
            targetAngle = calibrator.mapPressureToAngle(currentPressure);
        }
        // [모드 B] 듀얼 임계값 제어 (Hysteresis Mode)
        else {
            if (currentPressure >= config.threshold_high) {
                isGripping = true;
            } else if (currentPressure <= config.threshold_low) {
                isGripping = false;
            }
            targetAngle = isGripping ? 180 : 0;
        }

        // 3. Fail-Safe 검사: 스톨 방지 타이머 (고토크 장시간 지속 시 모터 보호)
        if (targetAngle >= 175) {
            if (highTorqueStartTime == 0) {
                highTorqueStartTime = now;
            } else if (now - highTorqueStartTime > STALL_PROTECT_MS) {
                isStallProtected = true;
                targetAngle = 110; // 60% 수준으로 장력 완화
            }
        } else {
            highTorqueStartTime = 0;
            isStallProtected = false;
        }
    }

    // 4. 서보 각도 출력
    if (currentAngle != targetAngle) {
        currentAngle = targetAngle;
        fingerServo.write(currentAngle);
    }

    // 5. 실시간 텔레메트리 웹 브로드캐스트 (100ms 주기)
    static unsigned long lastTelemetryTime = 0;
    if (now - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
        lastTelemetryTime = now;
        broadcastTelemetry();
    }
}
