/**
 * @file lut_calibrator.h
 * @brief 가변 단계(flexLimit - extendLimit + 1) 비선형 보정 룩업 테이블(LUT) 관리 및
 *        플렉스 센서(3kΩ 분배회로) 기반 양끝 한계 탐색 / 선형 역매핑 보정 엔진 (다중 모터/센서 채널 지원)
 */

#ifndef LUT_CALIBRATOR_H
#define LUT_CALIBRATOR_H

#include <Arduino.h>
#include <Preferences.h>
#include <ESP32Servo.h>
#include <math.h>
#include <stdio.h>

#define MAX_LUT_SIZE 181        // 서보모터 최대 가동 범위(0°~180°)를 수용하는 배열 최대 크기
#define MAX_ADC_VAL 4095.0f
#define DEFAULT_FLEX_PIN 35     // 기본 플렉스 센서 핀 (1번 플렉스 센서: GPIO 35)

// 한계점 탐색 시 누적 변화량 기반 기구적 끝점 도달 판정 상수
// (3kΩ 분배회로 기준 1°당 약 3.2 ADC 변화 -> 약 4~5° 이동 시 15 ADC 이상 변화하면 정상 이동 중으로 판정)
#define FLEX_MOVE_MIN_DELTA 15  // 기준점 대비 이 값 이상 ADC가 변하면 손가락이 계속 움직이는 것으로 판정
#define LIMIT_STALL_STEPS 12    // 서보가 12° 이상 회전하는 동안 누적 ADC 변화가 없으면 기구적 한계 도달로 확정

// 3kΩ 풀다운 분배회로 기준 플렉스 센서 정상 연결 ADC 유효 범위 (단선 < 50, 쇼트 > 4000)
#define FLEX_ADC_MIN_VALID 50
#define FLEX_ADC_MAX_VALID 4000

class LutCalibrator {
public:
    uint8_t  flexSensorPin;          // 할당된 플렉스 센서 ADC1 핀 번호 (예: 35, 34)
    uint8_t  motorId;                // 모터 식별 번호 (1: 1·2번 마디용, 2: 3번 마디용)
    char     nvsPrefix[6];           // NVS 키 충돌 방지용 접두사 (예: "m1_", "m2_")

    uint8_t  angleLut[MAX_LUT_SIZE]; // 각 단계에 대응하는 서보 목표 각도 (extendLimit ~ flexLimit)
    uint16_t lutSteps;               // 실제 활성화된 LUT 단계 수 = (flexLimitAngle - extendLimitAngle + 1)
    bool     isCalibrated;           // 플렉스 센서 실측 캘리브레이션 완료 여부

    // 플렉스 센서 양끝단 ADC 값 및 서보 한계 각도
    uint16_t flexAdcExtend;     // 신전 한계각에서의 플렉스 센서 ADC 값
    uint16_t flexAdcFlex;       // 굴곡 한계각에서의 플렉스 센서 ADC 값
    uint8_t  extendLimitAngle;  // 1단계에서 찾은 실제 신전 한계각 (기본 0도)
    uint8_t  flexLimitAngle;    // 1단계에서 찾은 실제 굴곡 한계각 (기본 180도)

    LutCalibrator(uint8_t pin = DEFAULT_FLEX_PIN, const char *prefix = "m1_", uint8_t id = 1)
        : flexSensorPin(pin),
          motorId(id),
          lutSteps(181),
          isCalibrated(false),
          flexAdcExtend(950),   // 3kΩ 풀다운 기준 평상시 약 945 ADC
          flexAdcFlex(370),     // 3kΩ 풀다운 기준 최대 굽힘 시 약 372 ADC
          extendLimitAngle(0),
          flexLimitAngle(180) {
        snprintf(nvsPrefix, sizeof(nvsPrefix), "%s", prefix);
        generateArccosLut();
    }

    /**
     * @brief 채널별 플렉스 센서 핀 번호 및 NVS 키 접두사 설정
     */
    void configure(uint8_t pin, const char *prefix, uint8_t id) {
        flexSensorPin = pin;
        motorId = id;
        snprintf(nvsPrefix, sizeof(nvsPrefix), "%s", prefix);
    }

    /**
     * @brief 플렉스 센서 ADC 평균 읽기 (노이즈 억제용 16회 평균)
     */
    uint16_t readFlexAdc() {
        uint32_t sum = 0;
        for (int i = 0; i < 16; i++) {
            sum += analogRead(flexSensorPin);
        }
        return (uint16_t)(sum / 16);
    }

    /**
     * @brief 플렉스 센서 하드웨어 연결 및 기준 ADC 유효성 확인
     * @details 보정 여부(isCalibrated)와 분리하여, arccos 기본 LUT 상태에서도
     *          센서가 정상 연결되어 있으면 폐루프 각도 피드백이 유지되도록 판별함.
     */
    bool isFlexSensorValid(uint16_t adc) const {
        return (adc >= FLEX_ADC_MIN_VALID && adc <= FLEX_ADC_MAX_VALID &&
                abs((int)flexAdcExtend - (int)flexAdcFlex) >= 30);
    }

    bool isFlexSensorValid() {
        return isFlexSensorValid(readFlexAdc());
    }

    /**
     * @brief 측정된 플렉스 ADC 값을 통해 현재 실제 서보모터 등가 각도(extendLimitAngle ~ flexLimitAngle)를 산출
     */
    uint8_t readCurrentServoAngle(uint16_t adc) {
        if (abs((int)flexAdcExtend - (int)flexAdcFlex) < 30) return extendLimitAngle;

        // 플렉스 센서의 현재 굽힘 비율 (0.0: 신전 한계각 ~ 1.0: 굴곡 한계각) 계산
        float ratio = (float)((int)adc - (int)flexAdcExtend) / (float)((int)flexAdcFlex - (int)flexAdcExtend);
        if (ratio < 0.0f) ratio = 0.0f;
        if (ratio > 1.0f) ratio = 1.0f;

        // 보정 완료 여부(선형 LUT 또는 arccos 기본 LUT)와 무관하게 항상 angleLut과 동일 스케일로 역보간
        if (lutSteps >= 2) {
            float idxFloat = ratio * (float)(lutSteps - 1);
            int idx = (int)idxFloat;
            float frac = idxFloat - idx;
            if (idx >= (int)lutSteps - 1) return angleLut[lutSteps - 1];
            float deg = (1.0f - frac) * angleLut[idx] + frac * angleLut[idx + 1];
            return (uint8_t)constrain((int)round(deg), extendLimitAngle, flexLimitAngle);
        }

        float currentDeg = extendLimitAngle + ratio * (float)(flexLimitAngle - extendLimitAngle);
        return (uint8_t)constrain((int)round(currentDeg), extendLimitAngle, flexLimitAngle);
    }

    uint8_t readCurrentServoAngle() {
        return readCurrentServoAngle(readFlexAdc());
    }

    /**
     * @brief 캘리브레이션 미실시 기본값: (flexLimit - extendLimit + 1) 단계의 arccos LUT 생성
     */
    void generateArccosLut() {
        lutSteps = (uint16_t)(flexLimitAngle - extendLimitAngle + 1);
        if (lutSteps < 2) lutSteps = 2;

        for (uint16_t i = 0; i < lutSteps; i++) {
            float u = (float)i / (float)(lutSteps - 1); // 0.0 ~ 1.0 (가변 N단계)
            float acosInput = 1.0f - (2.0f * u);        // 1.0 ~ -1.0

            if (acosInput > 1.0f) acosInput = 1.0f;
            if (acosInput < -1.0f) acosInput = -1.0f;

            float normalizedAngle = acosf(acosInput) / M_PI; // 0.0 ~ 1.0
            float deg = extendLimitAngle + normalizedAngle * (float)(flexLimitAngle - extendLimitAngle);
            angleLut[i] = (uint8_t)constrain((int)round(deg), extendLimitAngle, flexLimitAngle);
        }
        isCalibrated = false;
        Serial.printf("[LUT #%u] %u단계(%u°~%u°) 역코사인(arccos) 기본 LUT 생성 완료\n",
                      motorId, lutSteps, extendLimitAngle, flexLimitAngle);
    }

    /**
     * @brief 캘리브레이션 대기 중 비상 정지/텔레메트리 콜백 주기적 확인
     * @return true면 중단 요청(Abort), false면 정상 진행
     */
    static bool waitWithAbortCheck(unsigned long ms, bool (*onStepCheckAbort)()) {
        unsigned long start = millis();
        while (millis() - start < ms) {
            if (onStepCheckAbort && onStepCheckAbort()) {
                return true;
            }
            delay(5);
        }
        return (onStepCheckAbort && onStepCheckAbort());
    }

    /**
     * @brief 서보모터 왕복 스윕을 통한 양끝 한계 지점 탐색 및 (flexLimit - extendLimit + 1) 단계 선형 LUT 생성
     * @details 1. 서보모터를 신전 방향과 굴곡 방향으로 움직여 누적 플렉스 ADC 변화가 멈추는 실제 기구적 한계점을 찾음
     *          2. 신전 한계각 ~ 굴곡 한계각 사이의 모든 정수 각도 개수(lutSteps)만큼 LUT 단계를 설정하고,
     *             압력 패킷에 따라 플렉스 센서가 선형으로 변하도록 서보 각도를 역매핑함
     *          3. 스윕 도중 onStepCheckAbort()가 true를 반환하면 기존 한계각/LUT를 보존한 채 즉시 중단함
     * @return true: 스윕 정상 완료(미감지 시 arccos 복원 포함), false: 일시정지/비상정지 요청으로 중단됨
     */
    bool runFullFlexCalibration(Servo &servo, bool (*onStepCheckAbort)() = nullptr) {
        Serial.printf("[CALIB #%u (GPIO %u)] 플렉스 센서 기반 양끝 한계 탐색 및 선형화 캘리브레이션 시작...\n",
                      motorId, flexSensorPin);

        uint16_t sweepAdc[MAX_LUT_SIZE] = {0};

        // 0. 중간 각도(90도)에서 출발
        servo.write(90);
        if (waitWithAbortCheck(500, onStepCheckAbort)) {
            Serial.printf("[CALIB #%u] 중단 요청 감지 -> 기존 보정 데이터를 유지하고 스윕을 취소합니다.\n", motorId);
            return false;
        }

        // 1-A. 신전 한계각 탐색 (90도 -> 0도 방향 스윕)
        uint16_t anchorAdc = readFlexAdc();
        uint8_t lastMovingAngle = 90;
        int stallSteps = 0;

        for (int angle = 90; angle >= 0; angle--) {
            servo.write(angle);
            if (waitWithAbortCheck(25, onStepCheckAbort)) {
                Serial.printf("[CALIB #%u] 신전 탐색 중 중단 요청 감지 -> 스윕 취소\n", motorId);
                return false;
            }
            uint16_t curAdc = readFlexAdc();

            if (abs((int)curAdc - (int)anchorAdc) >= FLEX_MOVE_MIN_DELTA) {
                // 손가락이 정상적으로 계속 펴지고 있음 -> 기준점 갱신
                anchorAdc = curAdc;
                lastMovingAngle = (uint8_t)angle;
                stallSteps = 0;
            } else if (angle < 90 - 15) {
                // 초기 와이어 유격 구간(15도) 통과 후 LIMIT_STALL_STEPS(12도) 동안 변화가 없으면 신전 한계로 확정
                stallSteps++;
                if (stallSteps >= LIMIT_STALL_STEPS) {
                    break;
                }
            }
        }
        uint8_t newExtendLimit = lastMovingAngle;
        servo.write(newExtendLimit);
        if (waitWithAbortCheck(300, onStepCheckAbort)) {
            Serial.printf("[CALIB #%u] 신전 한계 정착 중 중단 요청 감지 -> 스윕 취소\n", motorId);
            return false;
        }
        uint16_t newAdcExtend = readFlexAdc();

        // 1-B. 굴곡 한계각 탐색 및 각도별 플렉스 응답 측정 (newExtendLimit -> 180도 스윕)
        anchorAdc = newAdcExtend;
        lastMovingAngle = newExtendLimit;
        stallSteps = 0;

        for (int angle = newExtendLimit; angle <= 180; angle++) {
            servo.write(angle);
            if (waitWithAbortCheck(25, onStepCheckAbort)) {
                Serial.printf("[CALIB #%u] 굴곡 탐색 중 중단 요청 감지 -> 스윕 취소\n", motorId);
                return false;
            }
            uint16_t curAdc = readFlexAdc();
            sweepAdc[angle] = curAdc;

            if (abs((int)curAdc - (int)anchorAdc) >= FLEX_MOVE_MIN_DELTA) {
                // 손가락이 정상적으로 계속 굽혀지고 있음 -> 기준점 갱신
                anchorAdc = curAdc;
                lastMovingAngle = (uint8_t)angle;
                stallSteps = 0;
            } else if (angle > newExtendLimit + 20) {
                stallSteps++;
                if (stallSteps >= LIMIT_STALL_STEPS) {
                    break;
                }
            }
        }
        uint8_t newFlexLimit = lastMovingAngle;
        if (newFlexLimit <= newExtendLimit + 15) {
            // 센서가 미연결되었거나 변화가 감지되지 않은 경우 안전 기본 범위(0~180도) 복원
            extendLimitAngle = 0;
            flexLimitAngle = 180;
            generateArccosLut();
            servo.write(extendLimitAngle);
            Serial.printf("[CALIB #%u] 플렉스 센서 변화가 감지되지 않아 기본 arccos LUT(0°~180°)를 유지합니다.\n", motorId);
            return true;
        }
        uint16_t newAdcFlex = sweepAdc[newFlexLimit];

        // 모든 스윕이 정상 완료된 시점에만 멤버 변수 커밋
        extendLimitAngle = newExtendLimit;
        flexLimitAngle = newFlexLimit;
        flexAdcExtend = newAdcExtend;
        flexAdcFlex = newAdcFlex;

        // 2. 신전 한계각 ~ 굴곡 한계각 사이의 모든 정수 각도 수(lutSteps)로 LUT 크기 설정
        lutSteps = (uint16_t)(flexLimitAngle - extendLimitAngle + 1);

        // 3. 플렉스 센서 값이 선형으로 변화하도록 대응하는 서보모터 각도를 lutSteps 단계 LUT에 역매핑
        for (uint16_t i = 0; i < lutSteps; i++) {
            float targetRatio = (float)i / (float)(lutSteps - 1); // 0.0 ~ 1.0 선형 목표 비율
            float targetFlexAdc = flexAdcExtend + targetRatio * ((float)flexAdcFlex - (float)flexAdcExtend);

            uint8_t bestServoAngle = extendLimitAngle;
            float minDiff = 99999.0f;
            for (int a = extendLimitAngle; a <= flexLimitAngle; a++) {
                float diff = fabsf((float)sweepAdc[a] - targetFlexAdc);
                if (diff < minDiff) {
                    minDiff = diff;
                    bestServoAngle = (uint8_t)a;
                }
            }
            angleLut[i] = bestServoAngle;
        }

        // 캘리브레이션 종료 후 신전 한계각 위치로 복귀
        servo.write(extendLimitAngle);
        isCalibrated = true;

        Serial.printf("[CALIB #%u] 완료! 신전 한계각=%u°(ADC:%u), 굴곡 한계각=%u°(ADC:%u) -> 총 %u단계 LUT 생성\n",
                      motorId, extendLimitAngle, flexAdcExtend, flexLimitAngle, flexAdcFlex, lutSteps);
        return true;
    }

    /**
     * @brief 입력 압력값(0~4095)을 활성화된 N단계(lutSteps) LUT를 통해 목표 각도(desiredAngle)로 변환
     */
    uint8_t mapPressureToAngle(uint16_t pressure) {
        if (pressure >= 4095) return angleLut[lutSteps - 1];

        // 0 ~ 4095 압력을 0 ~ (lutSteps - 1) 인덱스로 선형 매핑 및 보간
        float normalized = ((float)pressure / MAX_ADC_VAL) * (lutSteps - 1);
        int idx = (int)normalized;
        float frac = normalized - idx;

        if (idx >= (int)lutSteps - 1) return angleLut[lutSteps - 1];

        float angle = (1.0f - frac) * angleLut[idx] + frac * angleLut[idx + 1];
        return (uint8_t)constrain((int)round(angle), extendLimitAngle, flexLimitAngle);
    }

    /**
     * @brief NVS(비휘발성 메모리)에 채널별 가변 단계 LUT 및 한계 지점 저장
     */
    void saveToNvs(Preferences &prefs) {
        char key[16];
        snprintf(key, sizeof(key), "%slut_tbl", nvsPrefix);
        prefs.putBytes(key, angleLut, sizeof(angleLut));

        snprintf(key, sizeof(key), "%slut_stp", nvsPrefix);
        prefs.putUShort(key, lutSteps);

        snprintf(key, sizeof(key), "%sext_lim", nvsPrefix);
        prefs.putUChar(key, extendLimitAngle);

        snprintf(key, sizeof(key), "%sflx_lim", nvsPrefix);
        prefs.putUChar(key, flexLimitAngle);

        snprintf(key, sizeof(key), "%sadc_ext", nvsPrefix);
        prefs.putUShort(key, flexAdcExtend);

        snprintf(key, sizeof(key), "%sadc_flx", nvsPrefix);
        prefs.putUShort(key, flexAdcFlex);

        snprintf(key, sizeof(key), "%sis_cal", nvsPrefix);
        prefs.putBool(key, isCalibrated);

        Serial.printf("[NVS #%u] 가변 단계 LUT 및 한계 각도 플래시 메모리 저장 완료\n", motorId);
    }

    /**
     * @brief NVS에서 저장된 채널별 가변 단계 LUT 및 한계 지점 로드 (무결성 검증 포함)
     * @return true: 실제 플렉스 보정 완료 데이터가 유효하게 로드됨, false: 미보정 또는 저장 데이터 없음
     */
    bool loadFromNvs(Preferences &prefs) {
        char keyTbl[16];
        snprintf(keyTbl, sizeof(keyTbl), "%slut_tbl", nvsPrefix);

        if (prefs.isKey(keyTbl) && prefs.getBytesLength(keyTbl) == sizeof(angleLut)) {
            prefs.getBytes(keyTbl, angleLut, sizeof(angleLut));

            char key[16];
            snprintf(key, sizeof(key), "%sext_lim", nvsPrefix);
            extendLimitAngle = prefs.getUChar(key, 0);

            snprintf(key, sizeof(key), "%sflx_lim", nvsPrefix);
            flexLimitAngle = prefs.getUChar(key, 180);

            // 한계각 역전 및 최대 각도(180도) 초과 방어
            if (flexLimitAngle > 180) flexLimitAngle = 180;
            if (extendLimitAngle >= flexLimitAngle) {
                extendLimitAngle = 0;
                flexLimitAngle = 180;
            }

            // lutSteps를 (flexLimitAngle - extendLimitAngle + 1) 및 [2, MAX_LUT_SIZE] 범위로 강제 정규화
            lutSteps = (uint16_t)(flexLimitAngle - extendLimitAngle + 1);
            if (lutSteps < 2) lutSteps = 2;
            if (lutSteps > MAX_LUT_SIZE) lutSteps = MAX_LUT_SIZE;

            snprintf(key, sizeof(key), "%sadc_ext", nvsPrefix);
            flexAdcExtend = prefs.getUShort(key, 950);

            snprintf(key, sizeof(key), "%sadc_flx", nvsPrefix);
            flexAdcFlex = prefs.getUShort(key, 370);

            snprintf(key, sizeof(key), "%sis_cal", nvsPrefix);
            isCalibrated = prefs.getBool(key, false);

            // 로드된 LUT 테이블 값의 안전 범위 클램핑
            for (uint16_t i = 0; i < lutSteps; i++) {
                if (angleLut[i] < extendLimitAngle) angleLut[i] = extendLimitAngle;
                if (angleLut[i] > flexLimitAngle) angleLut[i] = flexLimitAngle;
            }

            if (!isCalibrated) {
                generateArccosLut();
            }

            Serial.printf("[NVS #%u] 저장된 %u단계(%u°~%u°) LUT 로드 완료 (보정=%s)\n",
                          motorId, lutSteps, extendLimitAngle, flexLimitAngle,
                          isCalibrated ? "성공" : "미보정");
            return isCalibrated;
        }
        Serial.printf("[NVS #%u] 저장된 LUT가 없어 부팅 시 초기 탐색 또는 arccos 기본값을 사용합니다.\n", motorId);
        return false;
    }
};

#endif // LUT_CALIBRATOR_H
