/**
 * @file lut_calibrator.h
 * @brief 가변 단계(flexLimit - extendLimit + 1) 비선형 보정 룩업 테이블(LUT) 관리 및
 *        플렉스 센서(3kΩ 분배회로) 기반 양끝 한계 탐색 / 선형 역매핑 보정 엔진
 */

#ifndef LUT_CALIBRATOR_H
#define LUT_CALIBRATOR_H

#include <Arduino.h>
#include <Preferences.h>
#include <ESP32Servo.h>
#include <math.h>

#define MAX_LUT_SIZE 181        // 서보모터 최대 가동 범위(0°~180°)를 수용하는 배열 최대 크기
#define MAX_ADC_VAL 4095.0f
#define FLEX_SENSOR_PIN 35      // 플렉스 센서 상시 연결 핀 (ADC1_CH7, 3kΩ 풀다운 저항)

// 한계점 탐색 시 누적 변화량 기반 기구적 끝점 도달 판정 상수
// (3kΩ 분배회로 기준 1°당 약 3.2 ADC 변화 -> 약 4~5° 이동 시 15 ADC 이상 변화하면 정상 이동 중으로 판정)
#define FLEX_MOVE_MIN_DELTA 15  // 기준점 대비 이 값 이상 ADC가 변하면 손가락이 계속 움직이는 것으로 판정
#define LIMIT_STALL_STEPS 12    // 서보가 12° 이상 회전하는 동안 누적 ADC 변화가 없으면 기구적 한계 도달로 확정

class LutCalibrator {
public:
    uint8_t  angleLut[MAX_LUT_SIZE]; // 각 단계에 대응하는 서보 목표 각도 (extendLimit ~ flexLimit)
    uint16_t lutSteps;               // 실제 활성화된 LUT 단계 수 = (flexLimitAngle - extendLimitAngle + 1)
    bool     isCalibrated;           // 플렉스 센서 실측 캘리브레이션 완료 여부

    // 플렉스 센서 양끝단 ADC 값 및 서보 한계 각도
    uint16_t flexAdcExtend;     // 신전 한계각에서의 플렉스 센서 ADC 값
    uint16_t flexAdcFlex;       // 굴곡 한계각에서의 플렉스 센서 ADC 값
    uint8_t  extendLimitAngle;  // 1단계에서 찾은 실제 신전 한계각 (기본 0도)
    uint8_t  flexLimitAngle;    // 1단계에서 찾은 실제 굴곡 한계각 (기본 180도)

    LutCalibrator()
        : lutSteps(181),
          isCalibrated(false),
          flexAdcExtend(950),   // 3kΩ 풀다운 기준 평상시 약 945 ADC
          flexAdcFlex(370),     // 3kΩ 풀다운 기준 최대 굽힘 시 약 372 ADC
          extendLimitAngle(0),
          flexLimitAngle(180) {
        generateArccosLut();
    }

    /**
     * @brief 플렉스 센서 ADC 평균 읽기 (노이즈 억제용 16회 평균)
     */
    uint16_t readFlexAdc() {
        uint32_t sum = 0;
        for (int i = 0; i < 16; i++) {
            sum += analogRead(FLEX_SENSOR_PIN);
        }
        return (uint16_t)(sum / 16);
    }

    /**
     * @brief 플렉스 센서 값을 통해 현재 실제 서보모터 각도(extendLimitAngle ~ flexLimitAngle)를 산출
     */
    uint8_t readCurrentServoAngle() {
        uint16_t adc = readFlexAdc();
        if (abs((int)flexAdcExtend - (int)flexAdcFlex) < 30) return extendLimitAngle;

        // 플렉스 센서의 현재 굽힘 비율 (0.0: 신전 한계각 ~ 1.0: 굴곡 한계각) 계산
        float ratio = (float)((int)adc - (int)flexAdcExtend) / (float)((int)flexAdcFlex - (int)flexAdcExtend);
        if (ratio < 0.0f) ratio = 0.0f;
        if (ratio > 1.0f) ratio = 1.0f;

        // 보정 완료 시: 플렉스 비율(ratio)에 대응하는 실제 서보모터 각도를 LUT에서 보간하여 반환
        if (isCalibrated && lutSteps >= 2) {
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
        Serial.printf("[LUT] %u단계(%u°~%u°) 역코사인(arccos) 기본 LUT 생성 완료\n",
                      lutSteps, extendLimitAngle, flexLimitAngle);
    }

    /**
     * @brief 서보모터 왕복 스윕을 통한 양끝 한계 지점 탐색 및 (flexLimit - extendLimit + 1) 단계 선형 LUT 생성
     * @details 1. 서보모터를 신전 방향과 굴곡 방향으로 움직여 누적 플렉스 ADC 변화가 멈추는 실제 기구적 한계점을 찾음
     *          2. 신전 한계각 ~ 굴곡 한계각 사이의 모든 정수 각도 개수(lutSteps)만큼 LUT 단계를 설정하고,
     *             압력 패킷에 따라 플렉스 센서가 선형(0, 20, 40...)으로 변하도록 서보 각도를 역매핑함
     */
    void runFullFlexCalibration(Servo &servo) {
        Serial.println("[CALIB] 플렉스 센서 기반 양끝 한계 탐색 및 가변 단계 선형화 캘리브레이션 시작...");

        uint16_t sweepAdc[MAX_LUT_SIZE] = {0};

        // 0. 중간 각도(90도)에서 출발
        servo.write(90);
        delay(500);

        // 1-A. 신전 한계각 탐색 (90도 -> 0도 방향 스윕)
        uint16_t anchorAdc = readFlexAdc();
        uint8_t lastMovingAngle = 90;
        int stallSteps = 0;

        for (int angle = 90; angle >= 0; angle--) {
            servo.write(angle);
            delay(25);
            uint16_t curAdc = readFlexAdc();

            if (abs((int)curAdc - (int)anchorAdc) >= FLEX_MOVE_MIN_DELTA) {
                // 손가락이 정상적으로 계속 펴지고 있음 -> 기준점 갱신
                anchorAdc = curAdc;
                lastMovingAngle = (uint8_t)angle;
                stallSteps = 0;
            } else {
                stallSteps++;
                if (stallSteps >= LIMIT_STALL_STEPS) {
                    // LIMIT_STALL_STEPS(12도) 동안 플렉스 변화가 없으므로 직전 이동 각도를 신전 한계각으로 확정
                    break;
                }
            }
        }
        extendLimitAngle = lastMovingAngle;
        servo.write(extendLimitAngle);
        delay(300);
        flexAdcExtend = readFlexAdc();

        // 1-B. 굴곡 한계각 탐색 및 각도별 플렉스 응답 측정 (extendLimitAngle -> 180도 스윕)
        anchorAdc = flexAdcExtend;
        lastMovingAngle = extendLimitAngle;
        stallSteps = 0;

        for (int angle = extendLimitAngle; angle <= 180; angle++) {
            servo.write(angle);
            delay(25);
            uint16_t curAdc = readFlexAdc();
            sweepAdc[angle] = curAdc;

            if (abs((int)curAdc - (int)anchorAdc) >= FLEX_MOVE_MIN_DELTA) {
                // 손가락이 정상적으로 계속 굽혀지고 있음 -> 기준점 갱신
                anchorAdc = curAdc;
                lastMovingAngle = (uint8_t)angle;
                stallSteps = 0;
            } else if (angle > extendLimitAngle + 20) {
                stallSteps++;
                if (stallSteps >= LIMIT_STALL_STEPS) {
                    break;
                }
            }
        }
        flexLimitAngle = lastMovingAngle;
        if (flexLimitAngle <= extendLimitAngle + 15) {
            // 센서가 미연결되었거나 변화가 감지되지 않은 경우 안전 기본 범위(0~180도) 복원
            extendLimitAngle = 0;
            flexLimitAngle = 180;
            generateArccosLut();
            servo.write(extendLimitAngle);
            Serial.println("[CALIB] 플렉스 센서 변화가 감지되지 않아 기본 arccos LUT(0°~180°)를 유지합니다.");
            return;
        }
        flexAdcFlex = sweepAdc[flexLimitAngle];

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

        Serial.printf("[CALIB] 완료! 신전 한계각=%u°(ADC:%u), 굴곡 한계각=%u°(ADC:%u) -> 총 %u단계 LUT 생성\n",
                      extendLimitAngle, flexAdcExtend, flexLimitAngle, flexAdcFlex, lutSteps);
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
     * @brief NVS(비휘발성 메모리)에 가변 단계 LUT 및 한계 지점 저장
     */
    void saveToNvs(Preferences &prefs) {
        prefs.putBytes("lut_table", angleLut, sizeof(angleLut));
        prefs.putUShort("lut_steps", lutSteps);
        prefs.putUChar("ext_lim", extendLimitAngle);
        prefs.putUChar("flx_lim", flexLimitAngle);
        prefs.putUShort("adc_ext", flexAdcExtend);
        prefs.putUShort("adc_flx", flexAdcFlex);
        prefs.putBool("is_calib", isCalibrated);
        Serial.println("[NVS] 가변 단계 LUT 및 한계 각도 플래시 메모리 저장 완료");
    }

    /**
     * @brief NVS에서 저장된 가변 단계 LUT 및 한계 지점 로드
     */
    bool loadFromNvs(Preferences &prefs) {
        if (prefs.isKey("lut_table") && prefs.getBytesLength("lut_table") == sizeof(angleLut)) {
            prefs.getBytes("lut_table", angleLut, sizeof(angleLut));
            extendLimitAngle = prefs.getUChar("ext_lim", 0);
            flexLimitAngle = prefs.getUChar("flx_lim", 180);
            lutSteps = prefs.getUShort("lut_steps", (uint16_t)(flexLimitAngle - extendLimitAngle + 1));
            flexAdcExtend = prefs.getUShort("adc_ext", 950);
            flexAdcFlex = prefs.getUShort("adc_flx", 370);
            isCalibrated = prefs.getBool("is_calib", false);
            Serial.printf("[NVS] 저장된 %u단계(%u°~%u°) LUT 로드 성공\n",
                          lutSteps, extendLimitAngle, flexLimitAngle);
            return true;
        }
        Serial.println("[NVS] 저장된 LUT가 없어 부팅 시 초기 탐색 또는 arccos 기본값을 사용합니다.");
        return false;
    }
};

#endif // LUT_CALIBRATOR_H
