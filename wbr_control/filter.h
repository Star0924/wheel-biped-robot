#pragma once
#include <vector>
#include <math.h>

// ============================================================
// 0. 【新增，最重要】陀螺儀輔助俯仰角估測器 (互補/預測-修正型)
//
// 為什麼要加這個：
//   原本的做法是 rawPitch -> KalmanFilter(q=0.05,R=1) -> LowPassFilter(alpha=0.3)。
//   這兩級都是「純平滑」，沒有任何動態模型，等效增益分別約 0.2 與 0.3，
//   在 67Hz 下合起來造成 ~70~95ms 的相位延遲。
//   實測 log 也驗證了這件事：pitch_deg 相對於 gyro 落後約 5 個取樣 (75ms)，
//   兩者在 lag=0 的相關係數只有 0.27。
//
//   結果就是送進 MPC 的狀態向量自相矛盾：
//     theta 是 75ms 前的、omega 是「現在」的，
//   MPC 卻假設它們是同一瞬間的量測。對倒單擺這種不穩定系統，
//   這種等級的相位落後幾乎必然造成低頻搖擺(實測 ~0.9Hz)。
//
// 這個估測器的做法：
//   預測步：用陀螺儀積分推進角度   -> 高頻幾乎零延遲
//   修正步：用 IMU 輸出的角度慢慢拉回 -> 消除陀螺儀積分漂移
//   alpha 越小，越信任陀螺儀(更低延遲、但漂移修正更慢)。
//   alpha=0.02 在 67Hz 下，漂移修正時間常數約 0.015/0.02 = 0.75 秒，
//   對平衡控制來說綽綽有餘。
// ============================================================
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