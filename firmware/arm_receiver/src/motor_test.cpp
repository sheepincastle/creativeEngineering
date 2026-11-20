/**
 * @file motor_test.cpp
 * @brief 서보모터 전체 가동범위(0°~180°) 왕복 작동 테스트 및 버튼 정지/재개
 * 스케치
 * @details PlatformIO 환경 `motor_test` 로 빌드/업로드하여 실행합니다.
 */

#include "protocol.h"
#include <Arduino.h>
#include <ESP32Servo.h>

#define SERVO_PIN_1 18
#define SERVO_PIN_2 19
#define PAUSE_BTN_PIN 0
#define STATUS_LED_PIN 2

#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
#define BTN_DEBOUNCE_MS 250
#define STEP_DELAY_MS 15
#define CYCLE_PAUSE_MS 1000

#if NUM_MOTORS >= 2
const uint8_t SERVO_PINS[NUM_MOTORS] = {SERVO_PIN_1, SERVO_PIN_2};
#else
const uint8_t SERVO_PINS[NUM_MOTORS] = {SERVO_PIN_1};
#endif

Servo testServo[NUM_MOTORS];
bool isStopped = false;
bool lastBtnState = HIGH;
unsigned long lastBtnPressTime = 0;

void setStopState(bool stop) {
  isStopped = stop;
  if (isStopped) {
    for (int i = 0; i < NUM_MOTORS; i++) {
      testServo[i].detach();
    }
    digitalWrite(STATUS_LED_PIN, LOW);
    Serial.println("[STOP] 버튼 입력 감지 -> 모터 정지 (토크 해제)");
  } else {
    for (int i = 0; i < NUM_MOTORS; i++) {
      testServo[i].attach(SERVO_PINS[i], SERVO_MIN_US, SERVO_MAX_US);
    }
    digitalWrite(STATUS_LED_PIN, HIGH);
    Serial.println("[START] 버튼 재입력 감지 -> 모터 테스트 재개");
  }
}

void checkButton() {
  bool reading = digitalRead(PAUSE_BTN_PIN);
  unsigned long now = millis();

  if (reading == LOW && lastBtnState == HIGH &&
      (now - lastBtnPressTime > BTN_DEBOUNCE_MS)) {
    lastBtnPressTime = now;
    setStopState(!isStopped);
  }
  lastBtnState = reading;
}

bool waitWithButtonCheck(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    checkButton();
    if (isStopped) {
      return false;
    }
    delay(5);
  }
  return true;
}

void writeAllServos(int angle) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    testServo[i].write(angle);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n==========================================");
  Serial.printf("  모터 작동 테스트 (%d모터 모드, BOOT 버튼으로 정지/재개)\n",
                NUM_MOTORS);
  Serial.println("==========================================");

  pinMode(PAUSE_BTN_PIN, INPUT_PULLUP);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);

  for (int i = 0; i < NUM_MOTORS; i++) {
    ESP32PWM::allocateTimer(i);
    testServo[i].setPeriodHertz(50);
    testServo[i].attach(SERVO_PINS[i], SERVO_MIN_US, SERVO_MAX_US);
    testServo[i].write(0);
  }
  delay(500);
}

void loop() {
  writeAllServos(0);
  /*checkButton();
  if (isStopped) {
    delay(10);
    return;
  }

  Serial.println("[TEST] 0° -> 180° 이동");
  for (int angle = 0; angle <= 180; angle++) {
    writeAllServos(angle);
    if (!waitWithButtonCheck(STEP_DELAY_MS))
      return;
  }

  Serial.println("[TEST] 180° -> 0° 복귀");
  for (int angle = 180; angle >= 0; angle--) {
    writeAllServos(angle);
    if (!waitWithButtonCheck(STEP_DELAY_MS))
      return;
  }

  Serial.println("[TEST] 1초 대기...");
  waitWithButtonCheck(CYCLE_PAUSE_MS);*/
}