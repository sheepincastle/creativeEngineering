/**
 * @file main.cpp
 * @brief 팔 구동부 (Forearm Actuator & Server) - ESP32 DevKitC V4 (단독 유선
 * FSR 직결 / ESP-NOW 무선 롤백 및 단일·듀얼 모터 호환)
 * @details 1. 2개의 서보모터(1번: 바깥쪽 1·2번 마디, 2번: 손날 쪽 3번 마디) 및
 * 2개의 플렉스 센서 피드백 (+ USE_DIRECT_FSR=1 시 팔 ESP32 ADC1 핀에 FSR 직접 연결)
 *          2. 채널별 가변 N단계 LUT(플렉스 선형화 또는 arccos 기본값) 기반 압력
 * 목표 각도 산출 및 센서 유효성 기반 상시 폐루프 제어
 *          3. 가동 범위 정규화 요구치(effectiveDemand) 기반 지수함수적 변화량(alpha)
 * 및 미세 구간 와이어 유격 보상(slackOffset)
 *          4. 1번 모터 작동/파지 시 와이어 장력 반작용으로 3번 마디가 함께 움직이는
 * 것을 방지하는 전향 교차 결합 보상(Feedforward Decoupling) 및 역주행 차단 Anchor
 *          5. 내장 버튼(GPIO 0) 일시정지/잠금(Pause & Lock), 중단 가능한 캘리브레이션,
 * 무부하 FSR 영점 보정, 양방향 과부하(Stall) 안전 제한 및 웹 HIL 시뮬레이터 지원
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <WiFi.h>
#include <math.h>
#include <stdarg.h>

#include "lut_calibrator.h"
#include "protocol.h"

#if !USE_DIRECT_FSR
#include <esp_idf_version.h>
#include <esp_now.h>
#include <esp_wifi.h>
#endif

// 핀 정의 (ADC1 전용 입력 핀만 사용하여 Wi-Fi/ESP-NOW 충돌 원천 방지)
#define SERVO_PIN_1 18 // 1번 서보모터 신호 핀 (바깥쪽 1, 2번 마디 구동)
#define SERVO_PIN_2 19 // 2번 서보모터 신호 핀 (손날 쪽 3번 마디 구동)
#define FLEX_PIN_1                                                             \
  35 // 1번 플렉스 센서 핀 (1, 2번 마디 각도 측정, ADC1_CH7, 3kΩ 풀다운)
#define FLEX_PIN_2                                                             \
  34 // 2번 플렉스 센서 핀 (3번 마디 각도 측정, ADC1_CH6, 3kΩ 풀다운)
#if USE_DIRECT_FSR
#define FSR_PIN_1                                                              \
  32 // 1번 FSR 센서 핀 (바깥쪽 1, 2번 마디 제어용, ADC1_CH4, 1kΩ 풀다운)
#define FSR_PIN_2                                                              \
  33 // 2번 FSR 센서 핀 (손날 쪽 3번 마디 제어용, ADC1_CH5, 1kΩ 풀다운)
#endif
#define PAUSE_BTN_PIN                                                          \
  0 // 일시정지/잠금 토글용 내장 BOOT 버튼 (GPIO 0, Active LOW)
#define STATUS_LED_PIN 2 // 동작/일시정지 상태 표시용 내장 LED (GPIO 2)

#if NUM_MOTORS >= 2
const uint8_t SERVO_PINS[NUM_MOTORS] = {SERVO_PIN_1, SERVO_PIN_2};
const uint8_t FLEX_PINS[NUM_MOTORS] = {FLEX_PIN_1, FLEX_PIN_2};
#if USE_DIRECT_FSR
const uint8_t FSR_PINS[NUM_MOTORS] = {FSR_PIN_1, FSR_PIN_2};
#endif
const char *NVS_PREFIX[NUM_MOTORS] = {"m1_", "m2_"};
#else
const uint8_t SERVO_PINS[NUM_MOTORS] = {SERVO_PIN_1};
const uint8_t FLEX_PINS[NUM_MOTORS] = {FLEX_PIN_1};
#if USE_DIRECT_FSR
const uint8_t FSR_PINS[NUM_MOTORS] = {FSR_PIN_1};
#endif
const char *NVS_PREFIX[NUM_MOTORS] = {"m1_"};
#endif

// 서보 펄스 폭 설정 (PDI-6221MG: 500us ~ 2500us)
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500

// 제어 주기 및 안전 타이머 기본 상수
#define CONTROL_INTERVAL_MS 20  // 서보 제어 루프 주기 (20ms, 50Hz)
#if USE_DIRECT_FSR
#define FSR_ZERO_SAMPLE_COUNT 50 // 부팅/재보정 시 무부하 영점 샘플링 횟수 (0.5초)
#define FSR_ADC_AVG_SAMPLES 8    // Wi-Fi SoftAP 공존 시 ADC 노이즈 억제용 평균 횟수
#else
#define TIMEOUT_FAILSAFE_MS 300 // 300ms 패킷 단절 시 신전 한계각으로 복귀
#define CONFIG_RESYNC_INTERVAL_MS 3000 // 발 송신부 ema_alpha 주기적 재동기화 간격 (3초)
#endif
#define DEFAULT_STALL_PROTECT_MS                                               \
  5000 // 각도 변화 요구치 과부하 지속 허용 기본 시간 (5초)
#define DEFAULT_STALL_CUTOFF                                                   \
  150 // 과부하 감지 커트라인 기본 요구치 (150도 초과)
#define DEFAULT_STALL_SAFE_HOLD                                                \
  140 // 과부하 시 강제 조정할 안전 유지 기본 요구치 (140도)
#define BTN_DEBOUNCE_MS 250       // 버튼 채터링 방지 디바운스 시간 (250ms)
#define TELEMETRY_INTERVAL_MS 100 // 웹소켓 실시간 데이터 전송 주기
#define SLACK_COMP_ZONE_DEG 20         // 미세 유격 보상 활성화 요구치 구간 (20도 이하)
#define SLACK_COMP_STEP_DEG 0.4f       // 정지 시 주기당 유격 보상 누적량 (도)
#define SLACK_COMP_MAX_DEG 4.0f        // 최대 유격 보상 상한 (±4도)

// 전역 객체
Servo fingerServo[NUM_MOTORS];
LutCalibrator calibrator[NUM_MOTORS];
Preferences prefs;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
#if !USE_DIRECT_FSR
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
#endif

// 런타임 설정 파라미터
DeviceConfig config = {
    .ema_alpha = 0.25f,
    .max_step_alpha =
        35.0f, // 요구치 최대(180도)일 때 지수함수 최대 변화량 (기본 35도)
    .exp_curve_k = 2.5f,     // 지수함수 곡률 상수 k (기본 2.5)
    .coupling_gamma = 0.25f, // 1번 모터 작동 시 3번 마디에 미치는 간섭 보상
                             // 계수 γ (기본 0.25)
    .stall_cutoff_demand =
        DEFAULT_STALL_CUTOFF, // 과부하 감지 커트라인 요구치 (기본 150도)
    .stall_safe_demand = DEFAULT_STALL_SAFE_HOLD, // 과부하 시 제한할 안전 유지
                                                  // 요구치 (기본 140도)
    .stall_protect_ms =
        DEFAULT_STALL_PROTECT_MS, // 과부하 지속 허용 제한 시간 (기본 5000ms)
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
#if USE_DIRECT_FSR
float filteredPressure[NUM_MOTORS] = {0.0f};
uint16_t fsrZeroOffset[NUM_MOTORS] = {0};
#else
volatile unsigned long lastPacketTime = 0;
#endif

uint8_t desiredAngle[NUM_MOTORS] = {
    0}; // 1. 각 압력 센서에 따른 목표 각도 (LUT 변환값)
uint8_t currentAngle[NUM_MOTORS] = {
    0}; // 2. 각 플렉스 센서에 따른 현재 실제 마디 각도
uint8_t servoTargetAngle[NUM_MOTORS] = {
    0}; // 3. 각 서보모터의 이번 주기 최종 지령 각도
int16_t angleDemand[NUM_MOTORS] = {
    0}; // 4. 각도 변화 요구치 (desiredAngle - currentAngle)
int16_t appliedAlpha[NUM_MOTORS] = {
    0}; // 5. 지수함수 및 전향 보상을 거쳐 도출된 실제 각도 변화 목표량(α)
int16_t rawPureAlpha[NUM_MOTORS] = {
    0}; // 6. 전향 보상 적용 전 순수 지수함수 변화량

// 일시정지/잠금(Pause & Lock), 비동기 웹 명령 및 안전 타이머 상태 변수
bool isPaused = false;         // true: 일시정지/잠금 상태
bool isCalibratingNow = false; // true: 캘리브레이션 스윕 진행 중
volatile bool calibRequested = false;       // 비동기 웹소켓 캘리브레이션 요청 플래그
volatile bool pauseToggleRequested = false; // 비동기 웹소켓 일시정지 토글 요청 플래그
volatile bool resetLutRequested = false;    // 비동기 웹소켓 arccos 복원 요청 플래그
volatile bool saveNvsRequested = false;     // 비동기 웹소켓 NVS 저장 요청 플래그
#if USE_DIRECT_FSR
volatile bool fsrZeroRequested = false;     // 비동기 웹소켓 FSR 무부하 영점 보정 요청 플래그
#else
volatile bool syncConfigRequested = false;  // 비동기 웹소켓 발 송신부 설정 동기화 요청 플래그
#endif
volatile bool webSimEnabled = false;        // 웹 대시보드 가상 압력 제어(HIL) 활성화 여부
volatile uint16_t webSimPressure[NUM_MOTORS] = {0}; // 웹 슬라이더 가상 압력값 (0 ~ 4095)
volatile bool webSimBlock[NUM_MOTORS] = {false};    // 웹 '물체 접촉(현재각 고정)' 플래그
bool lastBtnState = HIGH;
unsigned long lastBtnPressTime = 0;

unsigned long highDemandStartTime[NUM_MOTORS] = {0};
bool isStallProtected[NUM_MOTORS] = {false};
float slackOffset[NUM_MOTORS] = {0.0f};
uint8_t prevMeasuredAngle[NUM_MOTORS] = {0};
int8_t prevDemandSign[NUM_MOTORS] = {0};
unsigned long lastControlTime = 0;
#if !USE_DIRECT_FSR
unsigned long lastConfigSyncTime = 0;
bool wasFootOnline = false;
#endif

// 전방 선언
void broadcastTelemetry();
void saveSettings();
void checkPauseButton();

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

#if USE_DIRECT_FSR
/**
 * @brief 유선 직결 FSR 센서 ADC 평균 읽기 (Wi-Fi 공존 노이즈 억제)
 */
uint16_t readFsrAdc(int ch) {
  uint32_t sum = 0;
  for (int i = 0; i < FSR_ADC_AVG_SAMPLES; i++) {
    sum += analogRead(FSR_PINS[ch]);
  }
  return (uint16_t)(sum / FSR_ADC_AVG_SAMPLES);
}

/**
 * @brief 유선 직결 FSR 센서 초기 무부하 영점 보정 (모든 활성 채널 동시 샘플링)
 */
void calibrateFsrZeroOffset() {
  uint32_t sum[NUM_MOTORS] = {0};
  for (int i = 0; i < FSR_ZERO_SAMPLE_COUNT; i++) {
    for (int ch = 0; ch < NUM_MOTORS; ch++) {
      sum[ch] += readFsrAdc(ch);
    }
    delay(10);
  }
  for (int ch = 0; ch < NUM_MOTORS; ch++) {
    fsrZeroOffset[ch] = (uint16_t)(sum[ch] / FSR_ZERO_SAMPLE_COUNT);
    filteredPressure[ch] = 0.0f;
    currentPressure[ch] = 0;
    sendSysLog("calib",
               "[FSR #%d (GPIO %u)] 무부하 영점 보정 완료: Offset = %u",
               ch + 1, FSR_PINS[ch], fsrZeroOffset[ch]);
  }
}

/**
 * @brief 20ms 제어 주기마다 유선 직결 FSR 압력값을 읽고 영점 보정 및 EMA 필터링 수행
 */
void updateLocalFsrPressure() {
  float currentAlpha = config.ema_alpha;
  for (int ch = 0; ch < NUM_MOTORS; ch++) {
    int rawAdc = (int)readFsrAdc(ch);
    int zeroAdjusted = rawAdc - (int)fsrZeroOffset[ch];
    if (zeroAdjusted < 0)
      zeroAdjusted = 0;

    filteredPressure[ch] = (currentAlpha * (float)zeroAdjusted) +
                           ((1.0f - currentAlpha) * filteredPressure[ch]);
    currentPressure[ch] =
        (uint16_t)constrain((int)round(filteredPressure[ch]), 0, 4095);
  }
}
#else
/**
 * @brief 발 송신부로 EMA 필터 계수(ema_alpha) 동기화 패킷 전송
 */
void syncFootConfig() {
  FootConfigPacket pkt = {.ema_alpha = config.ema_alpha};
  esp_now_send(broadcastAddress, (uint8_t *)&pkt, sizeof(pkt));
}
#endif

/**
 * @brief 웹 클라이언트에 현재 설정된 파라미터 전체 전송
 */
void broadcastConfig() {
  if (ws.count() == 0)
    return;

  StaticJsonDocument<640> doc;
  doc["type"] = "config";
  doc["motors"] = NUM_MOTORS;
  doc["direct_fsr"] = (bool)(USE_DIRECT_FSR != 0);
  doc["sim"] = (bool)webSimEnabled;
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
#if USE_DIRECT_FSR
  doc["zero1"] = fsrZeroOffset[MOTOR_OUTER_IDX];
#endif
#if NUM_MOTORS >= 2
  doc["ext2"] = calibrator[MOTOR_BASE_IDX].extendLimitAngle;
  doc["flx2"] = calibrator[MOTOR_BASE_IDX].flexLimitAngle;
  doc["cal2"] = calibrator[MOTOR_BASE_IDX].isCalibrated;
#if USE_DIRECT_FSR
  doc["zero2"] = fsrZeroOffset[MOTOR_BASE_IDX];
#endif
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
    // 잠금 진입: 모든 모터를 신전 한계각으로 편 뒤 서보 PWM 신호를
    // 분리(detach)하여 모터 힘/전류 해제
    for (int i = 0; i < NUM_MOTORS; i++) {
      fingerServo[i].write(calibrator[i].extendLimitAngle);
      highDemandStartTime[i] = 0;
      isStallProtected[i] = false;
      appliedAlpha[i] = 0;
      slackOffset[i] = 0.0f;
      prevDemandSign[i] = 0;
    }
    delay(150);
    for (int i = 0; i < NUM_MOTORS; i++) {
      fingerServo[i].detach();
    }
    digitalWrite(STATUS_LED_PIN, LOW); // 상태 LED 소등
    sendSysLog(
        "warn",
        "[LOCK] 일시정지/잠금 모드 활성화 (모든 서보 토크 해제 및 입력 차단)");
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

  if (reading == LOW && lastBtnState == HIGH &&
      (now - lastBtnPressTime > BTN_DEBOUNCE_MS)) {
    lastBtnPressTime = now;
    togglePauseState();
  }
  lastBtnState = reading;
}

/**
 * @brief 각도 변화 요구치(demand)에 따른 지수함수적 변화량 alpha 계산
 */
int16_t computeExponentialAlpha(int16_t demand) {
  if (demand == 0)
    return 0;

  int16_t sign = (demand > 0) ? 1 : -1;
  float absDemand = (float)abs(demand);
  if (absDemand > 180.0f)
    absDemand = 180.0f;

  float normalized = absDemand / 180.0f; // 0.0 ~ 1.0
  float k = config.exp_curve_k;
  if (k < 0.05f)
    k = 0.05f;
  float expFactor = (expf(k * normalized) - 1.0f) / (expf(k) - 1.0f);

  float rawAlpha = config.max_step_alpha * expFactor;
  int16_t step = (int16_t)round(rawAlpha);

  if (step < 1)
    step = 1;

  return sign * step;
}

/**
 * @brief 캘리브레이션 스윕 중 비상 정지 버튼 확인 및 웹 대시보드 실시간 텔레메트리 송신
 * @return true: 일시정지/비상정지 요청으로 스윕 중단 필요, false: 정상 계속
 */
bool calibStepCallback() {
  if (ws.count() > 0) {
    ws.cleanupClients();
  }
  checkPauseButton();

  if (pauseToggleRequested) {
    pauseToggleRequested = false;
    togglePauseState();
  }

  if (isPaused) {
    return true;
  }

  static unsigned long lastCalibTelemetryTime = 0;
  unsigned long now = millis();
  if (now - lastCalibTelemetryTime >= TELEMETRY_INTERVAL_MS) {
    lastCalibTelemetryTime = now;
    broadcastTelemetry();
  }
  return false;
}

/**
 * @brief 다중 모터 순차 캘리브레이션 수행 (비상 정지 시 즉시 중단 및 완료 시 NVS 자동 저장)
 */
void runSystemCalibration() {
  isCalibratingNow = true;
  broadcastTelemetry();

#if NUM_MOTORS >= 2
  sendSysLog("calib", "[CALIB] 듀얼 모터 순차 캘리브레이션 시작 (2번 모터[3번 "
                      "마디] -> 1번 모터[1·2번 마디])");
  // 1. 1번 모터를 중간 각도(90도)에 두고 2번 모터(3번 마디) 먼저 캘리브레이션
  fingerServo[MOTOR_OUTER_IDX].write(90);
  if (LutCalibrator::waitWithAbortCheck(300, calibStepCallback)) {
    sendSysLog("warn", "[CALIB] 비상 정지 요청으로 캘리브레이션이 중단되었습니다.");
    isCalibratingNow = false;
    broadcastConfig();
    broadcastTelemetry();
    return;
  }

  if (!calibrator[MOTOR_BASE_IDX].runFullFlexCalibration(
          fingerServo[MOTOR_BASE_IDX], calibStepCallback)) {
    sendSysLog("warn", "[CALIB #2] 중단됨 -> 기존 한계각 및 LUT를 보존합니다.");
    isCalibratingNow = false;
    broadcastConfig();
    broadcastTelemetry();
    return;
  }
  config.extend_limit_angle[MOTOR_BASE_IDX] =
      calibrator[MOTOR_BASE_IDX].extendLimitAngle;
  config.flex_limit_angle[MOTOR_BASE_IDX] =
      calibrator[MOTOR_BASE_IDX].flexLimitAngle;
  sendSysLog(
      "calib",
      "[CALIB #2] 3번 마디 완료: 가동범위 %u° ~ %u° (%u단계 LUT, 보정=%s)",
      calibrator[MOTOR_BASE_IDX].extendLimitAngle,
      calibrator[MOTOR_BASE_IDX].flexLimitAngle,
      calibrator[MOTOR_BASE_IDX].lutSteps,
      calibrator[MOTOR_BASE_IDX].isCalibrated ? "성공" : "기본값");

  // 2. 2번 모터가 신전 한계각을 유지한 상태에서 1번 모터(1·2번 마디)
  // 캘리브레이션
  if (!calibrator[MOTOR_OUTER_IDX].runFullFlexCalibration(
          fingerServo[MOTOR_OUTER_IDX], calibStepCallback)) {
    sendSysLog("warn", "[CALIB #1] 중단됨 -> 1번 모터 기존 보정 데이터를 보존합니다.");
    isCalibratingNow = false;
    broadcastConfig();
    broadcastTelemetry();
    return;
  }
  config.extend_limit_angle[MOTOR_OUTER_IDX] =
      calibrator[MOTOR_OUTER_IDX].extendLimitAngle;
  config.flex_limit_angle[MOTOR_OUTER_IDX] =
      calibrator[MOTOR_OUTER_IDX].flexLimitAngle;
  sendSysLog(
      "calib",
      "[CALIB #1] 1·2번 마디 완료: 가동범위 %u° ~ %u° (%u단계 LUT, 보정=%s)",
      calibrator[MOTOR_OUTER_IDX].extendLimitAngle,
      calibrator[MOTOR_OUTER_IDX].flexLimitAngle,
      calibrator[MOTOR_OUTER_IDX].lutSteps,
      calibrator[MOTOR_OUTER_IDX].isCalibrated ? "성공" : "기본값");
#else
  sendSysLog("calib", "[CALIB] 단일 모터 캘리브레이션 시작...");
  if (!calibrator[0].runFullFlexCalibration(fingerServo[0], calibStepCallback)) {
    sendSysLog("warn", "[CALIB #1] 중단됨 -> 기존 한계각 및 LUT를 보존합니다.");
    isCalibratingNow = false;
    broadcastConfig();
    broadcastTelemetry();
    return;
  }
  config.extend_limit_angle[0] = calibrator[0].extendLimitAngle;
  config.flex_limit_angle[0] = calibrator[0].flexLimitAngle;
  sendSysLog("calib", "[CALIB #1] 완료: 가동범위 %u° ~ %u° (%u단계 LUT, 보정=%s)",
             calibrator[0].extendLimitAngle, calibrator[0].flexLimitAngle,
             calibrator[0].lutSteps,
             calibrator[0].isCalibrated ? "성공" : "기본값");
#endif

  bool allCalibrated = true;
  for (int i = 0; i < NUM_MOTORS; i++) {
    servoTargetAngle[i] = calibrator[i].extendLimitAngle;
    fingerServo[i].write(servoTargetAngle[i]);
    slackOffset[i] = 0.0f;
    prevDemandSign[i] = 0;
    prevMeasuredAngle[i] = calibrator[i].extendLimitAngle;
    if (!calibrator[i].isCalibrated) {
      allCalibrated = false;
    }
  }
  isCalibratingNow = false;
  if (allCalibrated) {
    saveSettings();
  }
  broadcastConfig();
  broadcastTelemetry();
}

// NVS 설정 로드 및 저장
bool loadSettings() {
  prefs.begin("sixth_finger", false);
  config.ema_alpha = prefs.getFloat("alpha", 0.25f);
  config.max_step_alpha = prefs.getFloat("max_step", 35.0f);
  config.exp_curve_k = prefs.getFloat("exp_k", 2.5f);
  config.coupling_gamma = prefs.getFloat("gamma", 0.25f);
  config.stall_cutoff_demand = prefs.getUChar("stl_cut", DEFAULT_STALL_CUTOFF);
  config.stall_safe_demand =
      prefs.getUChar("stl_safe", DEFAULT_STALL_SAFE_HOLD);
  config.stall_protect_ms = prefs.getUInt("stl_ms", DEFAULT_STALL_PROTECT_MS);

  bool allLoaded = true;
  for (int i = 0; i < NUM_MOTORS; i++) {
    bool ok = calibrator[i].loadFromNvs(prefs);
    config.extend_limit_angle[i] = calibrator[i].extendLimitAngle;
    config.flex_limit_angle[i] = calibrator[i].flexLimitAngle;
    if (!ok)
      allLoaded = false;
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
  sendSysLog("info",
             "[NVS] 파라미터(α=%.2f, max=%.1f°, k=%.2f, γ=%.2f, "
             "제한=%u°/%lums) 및 LUT 영구 저장 완료",
             config.ema_alpha, config.max_step_alpha, config.exp_curve_k,
             config.coupling_gamma, config.stall_safe_demand,
             (unsigned long)config.stall_protect_ms);
}

#if !USE_DIRECT_FSR
static void handleReceivedPacket(const uint8_t *data, int data_len) {
  if (data_len == sizeof(FootSensorPacket)) {
    const FootSensorPacket *pkt = (const FootSensorPacket *)data;
    for (int i = 0; i < NUM_MOTORS; i++) {
      currentPressure[i] = pkt->raw_pressure[i];
    }
    lastPacketTime = millis();
  }
}

// ESP-NOW 패킷 수신 콜백
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void onDataReceived(const esp_now_recv_info_t *esp_now_info,
                    const uint8_t *data, int data_len) {
  (void)esp_now_info;
  handleReceivedPacket(data, data_len);
}
#else
void onDataReceived(const uint8_t *mac_addr, const uint8_t *data,
                    int data_len) {
  (void)mac_addr;
  handleReceivedPacket(data, data_len);
}
#endif
#endif

// WebSocket 이벤트 핸들러 (async_tcp 태스크 컨텍스트에서 실행되므로 길이를 명시해 파싱하고
// 서보/플래시/블로킹 작업은 메인 loop() 플래그로 위임)
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len &&
      info->opcode == WS_TEXT) {
    DynamicJsonDocument doc(512);
    DeserializationError err = deserializeJson(doc, (const char *)data, len);
    if (err)
      return;

    const char *cmd = doc["cmd"];
    if (!cmd)
      return;

    if (strcmp(cmd, "set_alpha") == 0) {
      config.ema_alpha = constrain(doc["val"].as<float>(), 0.05f, 0.95f);
#if USE_DIRECT_FSR
      sendSysLog("info",
                 "[PARAM] 발 압력 EMA 필터 계수(ema_alpha) = %.2f 즉시 적용",
                 config.ema_alpha);
#else
      syncConfigRequested = true;
      sendSysLog("info", "[PARAM] 발 압력 EMA 필터 계수(ema_alpha) = %.2f 적용",
                 config.ema_alpha);
#endif
    } else if (strcmp(cmd, "set_max_step") == 0) {
      config.max_step_alpha = constrain(doc["val"].as<float>(), 1.0f, 90.0f);
      sendSysLog("info",
                 "[PARAM] 지수함수 최대 변화량(max_step_alpha) = %.1f° 적용",
                 config.max_step_alpha);
    } else if (strcmp(cmd, "set_exp_k") == 0) {
      config.exp_curve_k = constrain(doc["val"].as<float>(), 0.1f, 8.0f);
      sendSysLog("info", "[PARAM] 지수함수 곡률 상수(exp_curve_k) = %.2f 적용",
                 config.exp_curve_k);
    } else if (strcmp(cmd, "set_gamma") == 0) {
      config.coupling_gamma = constrain(doc["val"].as<float>(), 0.0f, 1.0f);
      sendSysLog(
          "info",
          "[PARAM] 3번 마디 전향 간섭 보상 계수(coupling_gamma) = %.2f 적용",
          config.coupling_gamma);
    } else if (strcmp(cmd, "set_stall_params") == 0) {
      if (doc.containsKey("safe")) {
        config.stall_safe_demand =
            (uint8_t)constrain(doc["safe"].as<int>(), 30, 175);
      }
      if (doc.containsKey("cutoff")) {
        config.stall_cutoff_demand = (uint8_t)constrain(
            doc["cutoff"].as<int>(), (int)config.stall_safe_demand, 180);
      } else {
        config.stall_cutoff_demand =
            (uint8_t)constrain((int)config.stall_safe_demand + 10, 40, 180);
      }
      if (doc.containsKey("ms")) {
        config.stall_protect_ms =
            (uint32_t)constrain(doc["ms"].as<long>(), 500L, 30000L);
      }
      sendSysLog(
          "info",
          "[PARAM] 과부하 제한 설정: 감지>%u°, 제한각도=%u°, 제한시간=%lums",
          config.stall_cutoff_demand, config.stall_safe_demand,
          (unsigned long)config.stall_protect_ms);
    } else if (strcmp(cmd, "get_config") == 0) {
      broadcastConfig();
    } else if (strcmp(cmd, "toggle_pause") == 0) {
      pauseToggleRequested = true;
    } else if (strcmp(cmd, "reset_lut") == 0) {
      resetLutRequested = true;
    } else if (strcmp(cmd, "start_calib") == 0) {
      calibRequested = true; // 메인 loop()에서 안전하게 실행하도록 플래그 설정
    } else if (strcmp(cmd, "zero_fsr") == 0) {
#if USE_DIRECT_FSR
      fsrZeroRequested = true;
#else
      sendSysLog(
          "warn",
          "[FSR] 유선 직결 모드(USE_DIRECT_FSR=1)에서만 영점 재보정이 가능합니다.");
#endif
    } else if (strcmp(cmd, "save_nvs") == 0) {
      saveNvsRequested = true;
    } else if (strcmp(cmd, "set_sim") == 0) {
      bool prevSim = webSimEnabled;
      bool en = doc["enable"] | false;
      if (en) {
        int p1Val = constrain(doc["p1"] | 0, 0, 4095);
        webSimPressure[MOTOR_OUTER_IDX] = (uint16_t)p1Val;
        webSimBlock[MOTOR_OUTER_IDX] = (bool)(doc["b1"] | false);
#if NUM_MOTORS >= 2
        int p2Val = constrain(doc["p2"] | 0, 0, 4095);
        webSimPressure[MOTOR_BASE_IDX] = (uint16_t)p2Val;
        webSimBlock[MOTOR_BASE_IDX] = (bool)(doc["b2"] | false);
#endif
        webSimEnabled = true;
        if (!prevSim) {
          sendSysLog("warn",
                     "[SIM] 웹 가상 압력 제어 모드 활성화 (실제 서보모터 연동)");
        }
      } else {
        webSimEnabled = false;
        for (int i = 0; i < NUM_MOTORS; i++) {
          webSimPressure[i] = 0;
          webSimBlock[i] = false;
        }
        if (prevSim) {
#if USE_DIRECT_FSR
          sendSysLog("info",
                     "[SIM] 웹 가상 압력 제어 모드 해제 -> 유선 직결 FSR 입력 복귀");
#else
          sendSysLog("info",
                     "[SIM] 웹 가상 압력 제어 모드 해제 -> 발 송신부 입력 복귀");
#endif
        }
      }
    }
  }
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    broadcastConfig();
    sendSysLog("info", "[WS] 웹 대시보드 클라이언트 연결됨 (ID: %u)",
               client->id());
  } else if (type == WS_EVT_DISCONNECT) {
    if (webSimEnabled && ws.count() == 0) {
      webSimEnabled = false;
      for (int i = 0; i < NUM_MOTORS; i++) {
        webSimPressure[i] = 0;
        webSimBlock[i] = false;
      }
      Serial.println(
          "[SIM] 웹 클라이언트 연결 종료 -> 가상 압력 모드 자동 해제");
    }
  } else if (type == WS_EVT_DATA) {
    handleWebSocketMessage(arg, data, len);
  }
}

// 실시간 텔레메트리 웹 브로드캐스트
void broadcastTelemetry() {
  if (ws.count() == 0)
    return;

  StaticJsonDocument<768> doc;
  doc["type"] = "telemetry";
  doc["motors"] = NUM_MOTORS;
  doc["direct_fsr"] = (bool)(USE_DIRECT_FSR != 0);
  doc["sim"] = (bool)webSimEnabled;

  // 1번 모터 (바깥쪽 1, 2번 마디) 상태
  doc["p"] = webSimEnabled ? webSimPressure[MOTOR_OUTER_IDX]
                           : currentPressure[MOTOR_OUTER_IDX];
  doc["des"] = desiredAngle[MOTOR_OUTER_IDX];
  doc["a"] = currentAngle[MOTOR_OUTER_IDX];
  doc["dem"] = angleDemand[MOTOR_OUTER_IDX];
  doc["step"] = appliedAlpha[MOTOR_OUTER_IDX];
  doc["target"] = servoTargetAngle[MOTOR_OUTER_IDX];
  doc["ext"] = calibrator[MOTOR_OUTER_IDX].extendLimitAngle;
  doc["flx"] = calibrator[MOTOR_OUTER_IDX].flexLimitAngle;
  doc["cal"] = calibrator[MOTOR_OUTER_IDX].isCalibrated;
  doc["stall"] = isStallProtected[MOTOR_OUTER_IDX];

#if NUM_MOTORS >= 2
  // 2번 모터 (손날 쪽 3번 마디) 상태
  doc["p2"] = webSimEnabled ? webSimPressure[MOTOR_BASE_IDX]
                            : currentPressure[MOTOR_BASE_IDX];
  doc["des2"] = desiredAngle[MOTOR_BASE_IDX];
  doc["a2"] = currentAngle[MOTOR_BASE_IDX];
  doc["dem2"] = angleDemand[MOTOR_BASE_IDX];
  doc["step2"] = appliedAlpha[MOTOR_BASE_IDX];
  doc["pure2"] = rawPureAlpha[MOTOR_BASE_IDX];
  doc["target2"] = servoTargetAngle[MOTOR_BASE_IDX];
  doc["ext2"] = calibrator[MOTOR_BASE_IDX].extendLimitAngle;
  doc["flx2"] = calibrator[MOTOR_BASE_IDX].flexLimitAngle;
  doc["cal2"] = calibrator[MOTOR_BASE_IDX].isCalibrated;
  doc["stall2"] = isStallProtected[MOTOR_BASE_IDX];
#endif

  // 시스템 공통 상태
#if USE_DIRECT_FSR
  doc["online"] = true;
#else
  unsigned long nowMs = millis();
  unsigned long pktTime = lastPacketTime;
  bool isOnline =
      (pktTime > 0) && ((nowMs < pktTime) || (nowMs - pktTime < TIMEOUT_FAILSAFE_MS));
  doc["online"] = isOnline;
#endif
  doc["paused"] = isPaused;
  doc["calibrating"] = isCalibratingNow;

  String output;
  serializeJson(doc, output);
  ws.textAll(output);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n==========================================");
  Serial.printf("  여섯 번째 손가락 - 팔 구동부 기동 (%d모터 / %s)\n",
                NUM_MOTORS,
                USE_DIRECT_FSR ? "유선 FSR 직결" : "ESP-NOW 무선");
  Serial.println("==========================================");

  // 핀 초기화 (플렉스/FSR 센서 ADC, 일시정지 버튼, 상태 LED)
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(PAUSE_BTN_PIN, INPUT_PULLUP);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH); // 부팅 시 정상 구동 상태(LED ON)

  // 채널별 센서 및 서보 모터 초기화 (PDI-6221MG)
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(FLEX_PINS[i], INPUT);
#if USE_DIRECT_FSR
    pinMode(FSR_PINS[i], INPUT);
#endif
    calibrator[i].configure(FLEX_PINS[i], NVS_PREFIX[i], i + 1);

    ESP32PWM::allocateTimer(i);
    fingerServo[i].setPeriodHertz(50);
    fingerServo[i].attach(SERVO_PINS[i], SERVO_MIN_US, SERVO_MAX_US);
  }

  // NVS에서 LUT 및 양끝 한계 각도 로드 (저장된 설정이 없으면 부팅 시 순차 한계
  // 탐색 수행)
  bool hasSavedLut = loadSettings();
  if (!hasSavedLut) {
    runSystemCalibration();
  } else {
    for (int i = 0; i < NUM_MOTORS; i++) {
      servoTargetAngle[i] = calibrator[i].extendLimitAngle;
      prevMeasuredAngle[i] = calibrator[i].extendLimitAngle;
      fingerServo[i].write(servoTargetAngle[i]);
    }
  }

  // LittleFS 마운트
  LittleFS.begin(true);

#if USE_DIRECT_FSR
  // Wi-Fi SoftAP 단독 초기화 (유선 FSR 직결 모드에서는 ESP-NOW 미사용)
  WiFi.mode(WIFI_AP);
  WiFi.softAP("Sixth-Finger-AP", "", WIFI_ESP_NOW_CHANNEL);

  // Wi-Fi SoftAP 활성화 상태의 3.3V 기준선에 맞춰 유선 FSR 무부하 영점 보정
  calibrateFsrZeroOffset();
#else
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
#endif

  // 웹서버 시작
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
  server.begin();
  lastControlTime = millis();
}

void loop() {
  unsigned long now = millis();
  ws.cleanupClients();

  // 모든 웹 클라이언트 연결이 끊겼다면 가상 압력 제어 모드를 즉시 해제해 안전 복귀
  if (webSimEnabled && ws.count() == 0) {
    webSimEnabled = false;
    for (int i = 0; i < NUM_MOTORS; i++) {
      webSimPressure[i] = 0;
      webSimBlock[i] = false;
    }
    Serial.println("[SIM] 웹 클라이언트 없음 감지 -> 가상 압력 모드 자동 해제");
  }

  // 0-A. 일시정지/잠금 버튼(GPIO 0) 및 비동기 웹소켓 명령 플래그 처리
  checkPauseButton();

  if (pauseToggleRequested) {
    pauseToggleRequested = false;
    togglePauseState();
  }

  if (resetLutRequested) {
    resetLutRequested = false;
    for (int i = 0; i < NUM_MOTORS; i++) {
      calibrator[i].generateArccosLut();
      slackOffset[i] = 0.0f;
      prevDemandSign[i] = 0;
    }
    sendSysLog(
        "warn",
        "[LUT] 모든 채널이 역코사인(arccos) 기본 LUT로 복원되었습니다.");
    broadcastConfig();
  }

  if (saveNvsRequested) {
    saveNvsRequested = false;
    saveSettings();
  }

#if USE_DIRECT_FSR
  if (fsrZeroRequested) {
    fsrZeroRequested = false;
    calibrateFsrZeroOffset();
    broadcastConfig();
    broadcastTelemetry();
    lastControlTime = millis();
  }
#else
  if (syncConfigRequested) {
    syncConfigRequested = false;
    syncFootConfig();
    lastConfigSyncTime = now;
  }
#endif

  // 0-B. 웹소켓 캘리브레이션 요청 처리 (비동기 콜백 밖에서 안전하게 실행)
  if (calibRequested) {
    calibRequested = false;
    if (isPaused)
      togglePauseState();
    runSystemCalibration();
    lastControlTime = millis();
    return;
  }

  // 20ms(50Hz) 주기로 서보모터 제어 루프 실행
  if (now - lastControlTime >= CONTROL_INTERVAL_MS) {
    lastControlTime = now;

#if USE_DIRECT_FSR
    // 0-C. 유선 직결 FSR 압력 센서 상시 샘플링 및 EMA 필터링
    //      (일시정지 상태에서도 대시보드 실시간 압력 모니터링 유지)
    updateLocalFsrPressure();
#endif

    // 1. 각 플렉스 센서를 통해 현재 실제 마디 각도(currentAngle) 측정
    //    (보정 여부와 분리해 센서 하드웨어가 유효하면 arccos 기본 LUT 상태에서도 폐루프 피드백 유지,
    //     단선/쇼트 시에만 servoTargetAngle 오픈루프로 안전 폴백.
    //     웹 시뮬레이터의 '물체 접촉(현재각 고정)' 체크 시에는 직전 각도 유지)
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (webSimEnabled && webSimBlock[i]) {
        continue;
      }
      uint16_t flexAdc = calibrator[i].readFlexAdc();
      if (calibrator[i].isFlexSensorValid(flexAdc)) {
        currentAngle[i] = calibrator[i].readCurrentServoAngle(flexAdc);
      } else {
        currentAngle[i] = servoTargetAngle[i];
      }
    }

    // [일시정지/잠금 모드 체크] 잠금 상태면 모터 제어를 건너뛰어 대기 상태 유지
    if (isPaused) {
      for (int i = 0; i < NUM_MOTORS; i++) {
        prevMeasuredAngle[i] = currentAngle[i];
      }
      return;
    }

    // 2. 압력 입력 소스 검사 (무선 모드 시 300ms 통신 단절 Fail-Safe 포함) 및 압력 -> 목표 각도 LUT 변환
#if USE_DIRECT_FSR
    const bool isTimeout = false;
#else
    unsigned long pktTime = lastPacketTime;
    unsigned long elapsedSincePkt = (now >= pktTime) ? (now - pktTime) : 0;
    bool isFootTimeout = (pktTime == 0) || (elapsedSincePkt > TIMEOUT_FAILSAFE_MS);
    bool isFootOnline = !isFootTimeout;
    bool isTimeout = isFootTimeout && !webSimEnabled;

    if (isFootOnline && (!wasFootOnline || (now - lastConfigSyncTime >= CONFIG_RESYNC_INTERVAL_MS))) {
      syncFootConfig();
      lastConfigSyncTime = now;
    }
    wasFootOnline = isFootOnline;
#endif

    for (int i = 0; i < NUM_MOTORS; i++) {
      if (isTimeout) {
        desiredAngle[i] = calibrator[i].extendLimitAngle;
        highDemandStartTime[i] = 0;
        isStallProtected[i] = false;
        slackOffset[i] = 0.0f;
        prevDemandSign[i] = 0;
      } else {
        uint16_t activePressure =
            webSimEnabled ? webSimPressure[i] : currentPressure[i];
        desiredAngle[i] = calibrator[i].mapPressureToAngle(activePressure);
      }

      // 3. 채널별 '각도 변화 요구치(angleDemand)' 및 가동 범위 정규화 요구치(effectiveDemand) 산출
      angleDemand[i] = (int16_t)desiredAngle[i] - (int16_t)currentAngle[i];

      int16_t span = (int16_t)calibrator[i].flexLimitAngle -
                     (int16_t)calibrator[i].extendLimitAngle;
      if (span < 18)
        span = 18;
      int16_t effectiveDemand = (int16_t)constrain(
          (int)round((float)angleDemand[i] * 180.0f / (float)span), -180, 180);

      // 4. 채널별 양방향 과부하(Stall) 커트라인 검사 (정규화 요구치 절댓값 기준)
      if (!isTimeout &&
          abs(effectiveDemand) > (int16_t)config.stall_cutoff_demand) {
        if (highDemandStartTime[i] == 0) {
          highDemandStartTime[i] = now;
        } else if (now - highDemandStartTime[i] >= config.stall_protect_ms) {
          if (!isStallProtected[i]) {
            isStallProtected[i] = true;
            sendSysLog("warn",
                       "[STALL #%d] 정규화 요구치 %u° 초과 %lums 지속 -> 안전 요구치 "
                       "±%u°로 자동 제한",
                       i + 1, config.stall_cutoff_demand,
                       (unsigned long)config.stall_protect_ms,
                       config.stall_safe_demand);
          }
          int16_t sign = (effectiveDemand > 0) ? 1 : -1;
          effectiveDemand = sign * (int16_t)config.stall_safe_demand;
          angleDemand[i] =
              (int16_t)round((float)effectiveDemand * (float)span / 180.0f);
        }
      } else {
        if (isStallProtected[i]) {
          sendSysLog("info", "[STALL #%d] 과부하 보호 해제 -> 정상 요구치 복귀",
                     i + 1);
        }
        highDemandStartTime[i] = 0;
        isStallProtected[i] = false;
      }

      // 5-A. 각 모터의 순수 지수함수적 변화량(rawPureAlpha) 산출
      rawPureAlpha[i] = computeExponentialAlpha(effectiveDemand);

      // 5-B. 목표 각도 근접 미세 구간(0 < |angleDemand| <= 20도)에서의 마찰/유격 보상
      int8_t curSign = (angleDemand[i] > 0) ? 1 : ((angleDemand[i] < 0) ? -1 : 0);
      if (curSign == 0 ||
          (prevDemandSign[i] != 0 && curSign != prevDemandSign[i]) ||
          abs(angleDemand[i]) > SLACK_COMP_ZONE_DEG || isStallProtected[i]) {
        slackOffset[i] = 0.0f;
      } else if (currentAngle[i] == prevMeasuredAngle[i]) {
        slackOffset[i] += (float)curSign * SLACK_COMP_STEP_DEG;
        slackOffset[i] =
            constrain(slackOffset[i], -SLACK_COMP_MAX_DEG, SLACK_COMP_MAX_DEG);
      }
      prevDemandSign[i] = curSign;
      prevMeasuredAngle[i] = currentAngle[i];
    }

    // 5-C. 1번 모터(바깥쪽 1, 2번 마디) 이동 목표 각도 산출
    int16_t alpha1 = rawPureAlpha[MOTOR_OUTER_IDX];
    int16_t slack1 = (int16_t)round(slackOffset[MOTOR_OUTER_IDX]);
    appliedAlpha[MOTOR_OUTER_IDX] = alpha1 + slack1;
    int16_t nextServo1 =
        (int16_t)currentAngle[MOTOR_OUTER_IDX] + appliedAlpha[MOTOR_OUTER_IDX];
    servoTargetAngle[MOTOR_OUTER_IDX] = (uint8_t)constrain(
        nextServo1, (int16_t)calibrator[MOTOR_OUTER_IDX].extendLimitAngle,
        (int16_t)calibrator[MOTOR_OUTER_IDX].flexLimitAngle);

#if NUM_MOTORS >= 2
    // 5-D. 2번 모터(손날 쪽 3번 마디) 전향 교차 결합 보상(Feedforward Decoupling)
    //      (1번 모터 작동/파지 시 와이어 장력 반작용으로 3번 마디가 함께 움직이는 것을 방지)
    int16_t couplingComp =
        (int16_t)round(config.coupling_gamma * (float)alpha1);
    int16_t slack2 = (int16_t)round(slackOffset[MOTOR_BASE_IDX]);
    int16_t alpha2_final =
        rawPureAlpha[MOTOR_BASE_IDX] + slack2 - couplingComp;
    appliedAlpha[MOTOR_BASE_IDX] = alpha2_final;

    int16_t baseAngle2 = (int16_t)currentAngle[MOTOR_BASE_IDX];
    int16_t target2 = (int16_t)desiredAngle[MOTOR_BASE_IDX];
    int16_t prevServo2 = (int16_t)servoTargetAngle[MOTOR_BASE_IDX];

    // (a) 외력(1번 모터 장력 등)에 의해 3번 마디가 목표 각도(desiredAngle[1]) 너머로 끌려갔을 때 기준점 고정
    if ((alpha1 > 0 && baseAngle2 > target2 && prevServo2 <= target2) ||
        (alpha1 < 0 && baseAngle2 < target2 && prevServo2 >= target2)) {
      baseAngle2 = target2;
    }

    // (b) 전향 간섭 보상항(-γ·α1)으로 인해 2번 모터가 목표 각도 반대쪽으로 계속 걸어 내려가는 역주행 차단
    if (((int32_t)angleDemand[MOTOR_BASE_IDX] * (int32_t)alpha2_final <= 0) &&
        ((alpha2_final < 0 && baseAngle2 <= target2) ||
         (alpha2_final > 0 && baseAngle2 >= target2))) {
      baseAngle2 = target2;
    }

    int16_t nextServo2 = baseAngle2 + alpha2_final;
    servoTargetAngle[MOTOR_BASE_IDX] = (uint8_t)constrain(
        nextServo2, (int16_t)calibrator[MOTOR_BASE_IDX].extendLimitAngle,
        (int16_t)calibrator[MOTOR_BASE_IDX].flexLimitAngle);
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
