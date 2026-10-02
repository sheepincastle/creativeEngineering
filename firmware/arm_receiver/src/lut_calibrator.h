/**
 * @file lut_calibrator.h
 * @brief 비선형 보정 룩업 테이블(LUT) 관리 및 arccos / 플렉스 센서 보정 엔진
 */

#ifndef LUT_CALIBRATOR_H
#define LUT_CALIBRATOR_H

#include <Arduino.h>
#include <Preferences.h>
#include <math.h>

#define LUT_SIZE 128            // 128단계 정밀 LUT (ADC 0~4095 매핑)
#define MAX_ADC_VAL 4095.0f
#define FLEX_SENSOR_PIN 35      // 플렉스 센서 임시 연결 핀 (ADC1_CH7)

class LutCalibrator {
public:
    uint8_t angleLut[LUT_SIZE]; // 각 압력 단계에 대응하는 서보 각도 (0 ~ 180)
    bool isCalibrating;
    unsigned long calibStartTime;

    LutCalibrator() : isCalibrating(false), calibStartTime(0) {
        generateArccosLut();
    }

    /**
     * @brief 1단계 기본 보정: arccos(역코사인) 수식 기반 비선형 LUT 생성
     * @details 압력이 0일 때 0도, 최대일 때 180도가 되도록 arccos 곡선 매핑
     *          u = i / (LUT_SIZE - 1) -> input = 1.0 - 2.0 * u
     *          angle = (acos(input) / PI) * 180.0
     */
    void generateArccosLut() {
        for (int i = 0; i < LUT_SIZE; i++) {
            float u = (float)i / (float)(LUT_SIZE - 1); // 0.0 ~ 1.0
            float acosInput = 1.0f - (2.0f * u);        // 1.0 ~ -1.0
            
            // 수치 안정성을 위한 클램핑
            if (acosInput > 1.0f) acosInput = 1.0f;
            if (acosInput < -1.0f) acosInput = -1.0f;

            float rad = acosf(acosInput);               // 0 ~ PI
            float deg = (rad / M_PI) * 180.0f;          // 0° ~ 180°
            angleLut[i] = (uint8_t)round(deg);
        }
        Serial.println("[LUT] 역코사인(arccos) 기반 비선형 기본 테이블 생성 완료");
    }

    /**
     * @brief 입력 압력값(0~4095)을 LUT를 통해 서보 각도(0~180도)로 변환
     */
    uint8_t mapPressureToAngle(uint16_t pressure) {
        if (pressure >= 4095) return angleLut[LUT_SIZE - 1];
        
        // LUT 인덱스 계산 및 선형 보간 (Linear Interpolation)
        float normalized = ((float)pressure / MAX_ADC_VAL) * (LUT_SIZE - 1);
        int idx = (int)normalized;
        float frac = normalized - idx;

        if (idx >= LUT_SIZE - 1) return angleLut[LUT_SIZE - 1];

        float angle = (1.0f - frac) * angleLut[idx] + frac * angleLut[idx + 1];
        return (uint8_t)constrain((int)round(angle), 0, 180);
    }

    /**
     * @brief 플렉스 센서 보정 모드 시작
     */
    void startFlexCalibration() {
        isCalibrating = true;
        calibStartTime = millis();
        Serial.println("[CALIB] 플렉스 센서 2단계 실측 보정 모드 시작");
    }

    /**
     * @brief 플렉스 센서 보정 모드 종료 및 실측 데이터 기반 LUT 갱신
     */
    void finishFlexCalibration() {
        if (!isCalibrating) return;
        isCalibrating = false;
        Serial.println("[CALIB] 플렉스 센서 실측 보정 종료 및 LUT 반영 완료");
    }

    /**
     * @brief NVS(비휘발성 메모리)에 LUT 저장
     */
    void saveToNvs(Preferences &prefs) {
        prefs.putBytes("lut_table", angleLut, sizeof(angleLut));
        Serial.println("[NVS] LUT 테이블 플래시 메모리 저장 완료");
    }

    /**
     * @brief NVS에서 저장된 LUT 로드
     */
    bool loadFromNvs(Preferences &prefs) {
        if (prefs.isKey("lut_table")) {
            prefs.getBytes("lut_table", angleLut, sizeof(angleLut));
            Serial.println("[NVS] 저장된 사용자 맞춤 LUT 로드 성공");
            return true;
        }
        Serial.println("[NVS] 저장된 LUT가 없어 arccos 기본값을 유지합니다.");
        return false;
    }
};

#endif // LUT_CALIBRATOR_H
