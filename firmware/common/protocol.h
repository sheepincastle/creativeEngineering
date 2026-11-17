#ifndef SIXTH_FINGER_PROTOCOL_H
#define SIXTH_FINGER_PROTOCOL_H

#include <stdint.h>

// Wi-Fi / ESP-NOW 통신 채널 (SoftAP와 ESP-NOW 공존을 위해 채널 1로 고정)
#define WIFI_ESP_NOW_CHANNEL 1

// 구동 모터 및 센서 채널 수 설정
// - 2: 듀얼 모터 모드 (1번 모터: 바깥쪽 1·2번 마디, 2번 모터: 손날 쪽 3번 마디)
// - 1: 단일 모터 모드 (시간 부족 등으로 원래 계획으로 롤백 시 이 값만 1로 변경)
#define NUM_MOTORS 1

// 모터 인덱스 정의
#define MOTOR_OUTER_IDX 0 // 1번 모터: 바깥쪽 1, 2번 마디 구동 및 제어 채널
#define MOTOR_BASE_IDX 1  // 2번 모터: 손날에 가까운 3번 마디 구동 및 제어 채널

// 발 송신부 -> 팔 수신부 ESP-NOW 전송 패킷 (채널당 2 Bytes, 듀얼 모드 시 총 4
// Bytes)
typedef struct __attribute__((packed)) {
  uint16_t raw_pressure[NUM_MOTORS]; // 각 FSR 센서의 EMA 필터 및 영점 보정이
                                     // 완료된 12-bit 압력값 (0 ~ 4095)
} FootSensorPacket;

// 팔 수신부 -> 발 송신부 파라미터 동기화용 ESP-NOW 패킷 (웹에서 ema_alpha 조절
// 시 실시간 반영)
typedef struct __attribute__((packed)) {
  float ema_alpha; // 발 센서 EMA 필터 계수 (0.05 ~ 0.95)
} FootConfigPacket;

// 런타임 및 NVS 저장용 디바이스 파라미터 구조체
typedef struct {
  float ema_alpha;      // 1. 발 센서 EMA 필터 계수 (기본값: 0.25)
  float max_step_alpha; // 2. 정규화 요구치 최대(180도)일 때 지수함수 최대
                        // 변화량 (기본값: 35.0도)
  float exp_curve_k;    // 3. 지수함수 곡률 상수 k (기본값: 2.5 -> 정규화 요구치
                        // 140도 시 +19도 보장)
  float coupling_gamma; // 4. 1번 모터 장력 반작용으로 3번 마디가 함께 움직이는
                        // 것을 방지하는 전향 보상 계수 γ (기본값: 0.25)
  uint8_t stall_cutoff_demand; // 5-A. 가동 범위 정규화 요구치 기준 과부하 감지
                               // 커트라인 (양방향, 기본값: 150도)
  uint8_t stall_safe_demand;   // 5-B. 과부하 시 강제 제한할 정규화 안전 유지
                               // 요구치 (양방향, 기본값: 140도)
  uint32_t stall_protect_ms; // 5-C. 과부하 지속 허용 제한 시간 (기본값: 5000ms)
  uint8_t extend_limit_angle[NUM_MOTORS]; // 각 모터별 플렉스 센서로 찾은 신전
                                          // 한계각 (기본값: 0도)
  uint8_t flex_limit_angle[NUM_MOTORS];   // 각 모터별 플렉스 센서로 찾은 굴곡
                                          // 한계각 (기본값: 180도)
} DeviceConfig;

#endif // SIXTH_FINGER_PROTOCOL_H
