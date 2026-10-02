#ifndef SIXTH_FINGER_PROTOCOL_H
#define SIXTH_FINGER_PROTOCOL_H

#include <stdint.h>

// Wi-Fi / ESP-NOW 통신 채널 (SoftAP와 ESP-NOW 공존을 위해 채널 1로 고정)
#define WIFI_ESP_NOW_CHANNEL 1

// 제어 모드 정의
enum ControlMode : uint8_t {
    MODE_CONTINUOUS = 0,    // 연속 비례 제어 (0° ~ 180° 압력 추종)
    MODE_HYSTERESIS = 1     // 듀얼 임계값 기반 파지/복원 스위칭 모드
};

// 발 송신부 -> 팔 수신부 ESP-NOW 전송 패킷 (4 Bytes 최적화)
typedef struct __attribute__((packed)) {
    uint16_t raw_pressure;  // 12-bit ADC 읽기 값 (0 ~ 4095)
    uint8_t  sequence_id;   // 패킷 유실 감지용 시퀀스 넘버 (0 ~ 255)
    uint8_t  flags;         // bit 0: 보정 모드 활성, bit 1: 긴급 정지
} FootSensorPacket;

// 웹 대시보드와 주고받는 튜닝 파라미터 구조체
typedef struct {
    float    ema_alpha;         // EMA 필터 계수 (0.01 ~ 1.0, 기본값: 0.25)
    uint16_t threshold_high;    // 굽힘 시작 임계값 (Hysteresis 모드)
    uint16_t threshold_low;     // 이완 복귀 임계값 (Hysteresis 모드)
    uint8_t  control_mode;      // 0: CONTINUOUS, 1: HYSTERESIS
    uint16_t zero_offset;       // 부팅 시 보정된 영점 오프셋
} DeviceConfig;

#endif // SIXTH_FINGER_PROTOCOL_H
