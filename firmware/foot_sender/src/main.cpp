/**
 * @file main.cpp
 * @brief 발 송신부 (Foot Transmitter Module) - ESP32 DevKitC V4
 * @details FSR MD30-60 압력 센서의 전압값을 읽어 EMA 필터링 후 20ms 주기로 ESP-NOW 브로드캐스트 전송
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "protocol.h"

// 핀 정의
#define FSR_ADC_PIN 34          // FSR 센서 전압 분배점 (ADC1_CH6, 입력 전용 핀)
#define STATUS_LED_PIN 2        // 내장 상태 표시 LED

// 통신 및 샘플링 설정
#define SEND_INTERVAL_MS 20     // 전송 주기 20ms (50Hz)
#define ZERO_SAMPLE_COUNT 50    // 부팅 시 영점 샘플링 횟수

// 전역 변수
FootSensorPacket txPacket;
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // 브로드캐스트 전송
float filteredPressure = 0.0f;
float emaAlpha = 0.25f;         // 기본 EMA 알파 계수
uint16_t zeroOffset = 0;        // 무부하 기준 영점 오프셋
unsigned long lastSendTime = 0;
uint8_t packetSeq = 0;

// 초기 무부하 영점 보정 함수
void calibrateZeroOffset() {
    uint32_t sum = 0;
    for (int i = 0; i < ZERO_SAMPLE_COUNT; i++) {
        sum += analogRead(FSR_ADC_PIN);
        delay(10);
    }
    zeroOffset = sum / ZERO_SAMPLE_COUNT;
    filteredPressure = (float)zeroOffset;
    Serial.printf("[FSR] 무부하 영점 보정 완료: Offset = %u\n", zeroOffset);
}

// ESP-NOW 전송 완료 콜백
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN)); // 토글 점멸
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n==========================================");
    Serial.println("  여섯 번째 손가락 - 발 송신부 기동");
    Serial.println("==========================================");

    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);

    // ADC 설정 (12-bit: 0 ~ 4095, 3.3V 감쇠 11dB)
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // 부팅 시 무부하 기준 영점 자동 캘리브레이션
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

        // 1. 센서 ADC 원시 데이터 수집
        int rawAdc = analogRead(FSR_ADC_PIN);

        // 2. 영점 오프셋 감산 (음수 방지 클램핑)
        int zeroAdjusted = rawAdc - zeroOffset;
        if (zeroAdjusted < 0) zeroAdjusted = 0;

        // 3. 지수 이동 평균(EMA) 필터링: S_t = α * Y_t + (1 - α) * S_{t-1}
        filteredPressure = (emaAlpha * (float)zeroAdjusted) + ((1.0f - emaAlpha) * filteredPressure);

        // 4. 전송 패킷 조립
        txPacket.raw_pressure = (uint16_t)filteredPressure;
        txPacket.sequence_id = packetSeq++;
        txPacket.flags = 0x00;

        // 5. ESP-NOW 전송
        esp_now_send(broadcastAddress, (uint8_t *)&txPacket, sizeof(txPacket));
    }
}
