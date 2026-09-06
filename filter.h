#pragma once
#include <vector>

// ============================================================
// 1. 卡爾曼濾波器 (Kalman Filter)
//    這是簡化版的1D卡爾曼濾波器 (假設狀態近似「隨機漫步」，
//    沒有額外的狀態轉移模型)，適合用來平滑一個緩慢變化+帶雜訊的
//    純量訊號(例如這裡的俯仰角)，不是完整的多狀態卡爾曼濾波器。
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

    // 傳入新量測值，回傳濾波後的估計值
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
    double alpha;     // 權重 (0 ~ 1)，越大代表越信任新量測值(反應快但較不平滑)
    double output;     // 濾波後的結果
    bool initialized;  // 是否已收過第一筆資料

    LowPassFilter(double a = 0.3) {
        alpha = a;
        output = 0.0;
        initialized = false;
    }

    double update(double measurement) {
        if (!initialized) {
            output = measurement;   // 第一筆資料直接當結果，避免從0開始的暫態
            initialized = true;
        } else {
            // 新結果 = (新資料 * alpha) + (舊結果 * (1-alpha))
            output = (alpha * measurement) + ((1.0 - alpha) * output);
        }
        return output;
    }

    void reset(double value) {
        output = value;
        initialized = true;   // 標記為已初始化，避免下次 update 又把它當成第一筆
    }
};

// ============================================================
// 3. 滑動均值濾波器 (Moving Average Filter)
// ============================================================
class MovingAverageFilter {
public:
    int window_size;              // 窗口大小 (取幾筆資料做平均)
    std::vector<double> buffer;   // 環形緩衝區
    double sum;                   // 緩衝區內資料總和 (避免每次都重新加總，節省運算)
    int index;                    // 目前寫入位置

    MovingAverageFilter(int size = 5) {
        window_size = size;
        buffer.assign(size, 0.0);
        sum = 0.0;
        index = 0;
    }

    double update(double measurement) {
        sum -= buffer[index];        // 先減掉即將被覆蓋的舊資料
        buffer[index] = measurement; // 寫入新資料
        sum += measurement;

        index++;
        if (index >= window_size) index = 0;   // 環形緩衝區繞回開頭

        return sum / window_size;
    }

    void reset(double value) {
        buffer.assign(window_size, value);
        sum = value * window_size;
        index = 0;
    }
};