#pragma once
#include <vector>
#include <math.h>

// 陀螺儀輔助俯仰角估測器 (互補/預測-修正型)
class PitchEstimator {
public:
    double alpha;        // 角度修正權重 (0~1)，建議 0.01 ~ 0.05
    double angle;        // 估測出的俯仰角 (deg)
    bool   initialized;

    PitchEstimator(double a = 0.02) {
        alpha = a;
        angle = 0.0;
        initialized = false;
    }

    // measuredAngle_deg : IMU 輸出的俯仰角 (deg)
    // gyroRate_dps      : 同一軸的角速度 (deg/s)，符號必須與角度一致
    // dt_s              : 距離上次呼叫的時間 (s)
    double update(double measuredAngle_deg, double gyroRate_dps, double dt_s) {
        if (!initialized) {
            angle = measuredAngle_deg;
            initialized = true;
            return angle;
        }
        // 保護：dt 異常時不做積分，只做修正，避免大跳動
        if (dt_s > 0.0 && dt_s < 0.2) {
            angle += gyroRate_dps * dt_s;              // 預測
        }
        angle += alpha * (measuredAngle_deg - angle);  // 修正
        return angle;
    }

    void reset(double value) {
        angle = value;
        initialized = true;
    }
};

// ============================================================
// 1. 卡爾曼濾波器 (Kalman Filter) — 保留，但平衡迴圈已不再使用
//    這是簡化版的1D卡爾曼濾波器 (假設狀態近似「隨機漫步」)，
//    等效於一個固定增益的低通，會帶來明顯相位延遲，
//    不適合直接放在不穩定系統的回授路徑上。
// ============================================================
class KalmanFilter {
public:
    double err_measure;      // 測量誤差 (R)
    double err_estimate;     // 估計誤差 (P)
    double q;                // 過程雜訊 (Q)
    double current_estimate; // 當前估計值
    double last_estimate;    // 上次估計值

    KalmanFilter(double mea_e = 1.0, double est_e = 1.0, double q_val = 0.05) {
        err_measure = mea_e;
        err_estimate = est_e;
        q = q_val;
        current_estimate = 0.0;
        last_estimate = 0.0;
    }

    double update(double measurement) {
        double kalman_gain = err_estimate / (err_estimate + err_measure);
        current_estimate = last_estimate + kalman_gain * (measurement - last_estimate);
        err_estimate = (1.0 - kalman_gain) * err_estimate + q;
        last_estimate = current_estimate;
        return current_estimate;
    }

    void reset(double value) {
        current_estimate = value;
        last_estimate = value;
    }
};

// ============================================================
// 2. 一階低通濾波器 (Low-pass Filter)
// ============================================================
class LowPassFilter {
public:
    double alpha;
    double output;
    bool initialized;

    LowPassFilter(double a = 0.3) {
        alpha = a;
        output = 0.0;
        initialized = false;
    }

    double update(double measurement) {
        if (!initialized) {
            output = measurement;
            initialized = true;
        } else {
            output = (alpha * measurement) + ((1.0 - alpha) * output);
        }
        return output;
    }

    void reset(double value) {
        output = value;
        initialized = true;
    }
};

// ============================================================
// 3. 滑動均值濾波器 (Moving Average Filter)
//
//    【注意】視窗長度 = 群延遲約 (N-1)/2 個取樣。
//    原本輪速用 window=10，在 67Hz 下等於 ~67ms 的群延遲，
//    而 MPC 把 v 當成「當下」的量測，這是另一個相位落後來源。
//    建議把輪速視窗降到 3~4（見 config.cpp）。
// ============================================================
class MovingAverageFilter {
public:
    int window_size;
    std::vector<double> buffer;
    double sum;
    int index;

    MovingAverageFilter(int size = 5) {
        window_size = size;
        buffer.assign(size, 0.0);
        sum = 0.0;
        index = 0;
    }

    double update(double measurement) {
        sum -= buffer[index];
        buffer[index] = measurement;
        sum += measurement;

        index++;
        if (index >= window_size) index = 0;

        return sum / window_size;
    }

    void reset(double value) {
        buffer.assign(window_size, value);
        sum = value * window_size;
        index = 0;
    }
};