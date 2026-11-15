/**
 * @file main.cpp
 * @brief 발 송신부 (Foot Transmitter Module) - ESP32 DevKitC V4
 * @details 2개의 FSR MD30-60 압력 센서(1번: 1·2번 마디용, 2번: 3번 마디용) 전압값을 읽어
 *          각각 무부하 영점 보정 및 EMA 필터링 후 20ms(50Hz) 주기로 ESP-NOW 브로드캐스트 전송
 *          (protocol.h의 NUM_MOTORS를 1로 변경하면 즉시 단일 센서 모드로 롤백 가능)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "protocol.h"

// 핀 정의 (Wi-Fi/ESP-NOW 활성 시 충돌 없는 ADC1 전용 입력 핀 사용)
#define FSR_ADC_PIN_1 34        // 1번 FSR 센서: 바깥쪽 1, 2번 마디(1번 모터) 제어용 (ADC1_CH6, 1kΩ 풀다운)
#define FSR_ADC_PIN_2 35        // 2번 FSR 센서: 손날 쪽 3번 마디(2번 모터) 제어용 (ADC1_CH7, 1kΩ 풀다운)
#define STATUS_LED_PIN 2        // 내장 상태 표시 LED

#if NUM_MOTORS >= 2
const uint8_t FSR_PINS[NUM_MOTORS] = {FSR_ADC_PIN_1, FSR_ADC_PIN_2};
#else
const uint8_t FSR_PINS[NUM_MOTORS] = {FSR_ADC_PIN_1};
#endif

// 통신 및 샘플링 설정
#define SEND_INTERVAL_MS 20     // 전송 주기 20ms (50Hz)
#define ZERO_SAMPLE_COUNT 50    // 부팅 시 무부하 영점 샘플링 횟수 (0.5초)

// 전역 변수
FootSensorPacket txPacket;
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // 브로드캐스트 전송
float filteredPressure[NUM_MOTORS] = {0.0f};
float emaAlpha = 0.25f;         // 기본 EMA 알파 계수
uint16_t zeroOffset[NUM_MOTORS] = {0}; // 각 FSR 센서의 무부하 기준 영점 오프셋
unsigned long lastSendTime = 0;

// 초기 무부하 영점 보정 함수 (모든 활성 FSR 센서 동시 샘플링)
void calibrateZeroOffset() {
    uint32_t sum[NUM_MOTORS] = {0};
    for (int i = 0; i < ZERO_SAMPLE_COUNT; i++) {
        for (int ch = 0; ch < NUM_MOTORS; ch++) {
            sum[ch] += analogRead(FSR_PINS[ch]);
        }
        delay(10);
    }
    for (int ch = 0; ch < NUM_MOTORS; ch++) {
        zeroOffset[ch] = (uint16_t)(sum[ch] / ZERO_SAMPLE_COUNT);
        filteredPressure[ch] = 0.0f;
        Serial.printf("[FSR %d (GPIO %u)] 무부하 영점 보정 완료: Offset = %u\n",
                      ch + 1, FSR_PINS[ch], zeroOffset[ch]);
    }
}

// ESP-NOW 전송 완료 콜백
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN)); // 토글 점멸
    }
}

// 팔 수신부로부터 EMA 필터 계수(ema_alpha) 변경 패킷 수신 콜백
void onConfigReceived(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
    if (data_len == sizeof(FootConfigPacket)) {
        const FootConfigPacket *cfg = (const FootConfigPacket *)data;
        if (cfg->ema_alpha >= 0.01f && cfg->ema_alpha <= 1.0f) {
            emaAlpha = cfg->ema_alpha;
            Serial.printf("[CFG] 팔 수신부로부터 EMA 필터 계수 동기화 완료: ema_alpha = %.2f\n", emaAlpha);
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n==========================================");
    Serial.printf("  여섯 번째 손가락 - 발 송신부 기동 (%d채널)\n", NUM_MOTORS);
    Serial.println("==========================================");

    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);

    for (int ch = 0; ch < NUM_MOTORS; ch++) {
        pinMode(FSR_PINS[ch], INPUT);
    }

    // ADC 설정 (12-bit: 0 ~ 4095, 3.3V 감쇠 11dB)
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // 부팅 시 무부하 기준 영점 자동 캘리브레이션
    Serial.println("무부하 상태를 측정합니다.");
    Serial.println("모든 FSR 센서 위에 아무것도 올려놓지 마세요.");
    Serial.println("영점 조정은 3초 후에 시작됩니다...");
    delay(3000);
    calibrateZeroOffset();

    // Wi-Fi 스테이션 모드 초기화 및 채널 고정
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_channel(WIFI_ESP_NOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    // ESP-NOW 초기화
    if (esp_now_init() != ESP_OK) {
        Serial.println("[ERR] ESP-NOW 초기화 실패! 재부팅합니다...");
        ESP.restart();
    }
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onConfigReceived);

    // 피어(Peer) 등록 - 브로드캐스트 주소
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = WIFI_ESP_NOW_CHANNEL;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[ERR] 피어 등록 실패!");
    } else {
        Serial.println("[OK] ESP-NOW 브로드캐스트 피어 등록 완료");
    }

    Serial.printf("[WIFI] ESP-NOW 채널: %d, MAC: %s\n", 
                  WIFI_ESP_NOW_CHANNEL, WiFi.macAddress().c_str());
}

void loop() {
    unsigned long currentMillis = millis();

    if (currentMillis - lastSendTime >= SEND_INTERVAL_MS) {
        lastSendTime = currentMillis;

        for (int ch = 0; ch < NUM_MOTORS; ch++) {
            // 1. 각 센서 ADC 원시 데이터 수집
            int rawAdc = analogRead(FSR_PINS[ch]);

            // 2. 영점 오프셋 감산 (음수 방지 클램핑)
            int zeroAdjusted = rawAdc - zeroOffset[ch];
            if (zeroAdjusted < 0) zeroAdjusted = 0;

            // 3. 지수 이동 평균(EMA) 필터링: S_t = α * Y_t + (1 - α) * S_{t-1}
            filteredPressure[ch] = (emaAlpha * (float)zeroAdjusted) + ((1.0f - emaAlpha) * filteredPressure[ch]);

            // 4. 전송 패킷 조립
            txPacket.raw_pressure[ch] = (uint16_t)constrain((int)round(filteredPressure[ch]), 0, 4095);
        }

        // 5. ESP-NOW 브로드캐스트 전송
        esp_now_send(broadcastAddress, (uint8_t *)&txPacket, sizeof(txPacket));
    }
}
