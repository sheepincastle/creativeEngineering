#ifndef SIXTH_FINGER_PROTOCOL_H
#define SIXTH_FINGER_PROTOCOL_H

#include <stdint.h>

// Wi-Fi / ESP-NOW 통신 채널 (SoftAP와 ESP-NOW 공존을 위해 채널 1로 고정)
#define WIFI_ESP_NOW_CHANNEL 1

// 발 송신부 -> 팔 수신부 ESP-NOW 전송 패킷 (2 Bytes 초경량 최적화: 시간/불필요 필드 제거)
typedef struct __attribute__((packed)) {
    uint16_t raw_pressure;  // EMA 필터 및 영점 보정이 완료된 12-bit 압력값 (0 ~ 4095)
} FootSensorPacket;

// 런타임 및 NVS 저장용 디바이스 파라미터 구조체
typedef struct {
    float    ema_alpha;           // 발 센서 EMA 필터 계수 (기본값: 0.25)
    float    max_step_alpha;      // 요구치 최대(180도)일 때 지수함수 최대 변화량 (기본값: 35.0도)
    float    exp_curve_k;         // 지수함수 곡률 상수 k (기본값: 2.5 -> 요구치 140도 시 +19도 보장)
    uint8_t  extend_limit_angle;  // 1단계 플렉스 센서로 찾은 완전 신전(펴짐) 기준 각도 (기본값: 0도)
    uint8_t  flex_limit_angle;    // 1단계 플렉스 센서로 찾은 최대 굴곡(굽힘) 한계 각도 (기본값: 180도)
} DeviceConfig;

#endif // SIXTH_FINGER_PROTOCOL_H
