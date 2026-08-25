#pragma once
#include <vector>

// ============================================================
// 1. 卡爾曼濾波器 (Kalman Filter)
// ============================================================
class KalmanFilter {
public:
    double err_measure;     // 測量誤差
    double err_estimate;    // 估計誤差
    double q;               // 過程雜訊
    double current_estimate; // 當前估計值
    double last_estimate;    // 上次估計值

    // 建構子：在這裡設定初始值 (看起來比較像其他語言的寫法)
    KalmanFilter(double mea_e = 1.0, double est_e = 1.0, double q_val = 0.05) {
        err_measure = mea_e;
        err_estimate = est_e;
        q = q_val;
        current_estimate = 0.0;
        last_estimate = 0.0;
    }

    // 傳入新數值，算出濾波後的數值
    double update(double measurement) {
        // 算權重
        double kalman_gain = err_estimate / (err_estimate + err_measure);
        // 更新數值
        current_estimate = last_estimate + kalman_gain * (measurement - last_estimate);
        // 更新誤差
        err_estimate = (1.0 - kalman_gain) * err_estimate + q;
        // 把這次的結果存起來給下次用
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
    double alpha;     // 權重 (0 ~ 1)
    double output;    // 濾波後的結果
    bool initialized; // 記錄是不是第一筆資料

    LowPassFilter(double a = 0.3) {
        alpha = a;
        output = 0.0;
        initialized = false;
    }

    double update(double measurement) {
        if (initialized == false) {
            output = measurement; // 第一筆資料直接當結果
            initialized = true;
        } else {
            // 公式： 新結果 = (新資料 * 權重) + (舊結果 * 剩下的權重)
            output = (alpha * measurement) + ((1.0 - alpha) * output);
        }
        return output;
    }

    void reset(double value) {
        output = value;
        initialized = true; // 標記為已初始化，避免下次 update 覆蓋
    }
};

// ============================================================
// 3. 滑動均值濾波器 (Moving Average Filter)
// ============================================================
class MovingAverageFilter {
public:
    int window_size;              // 窗口大小 (要平均幾筆資料)
    std::vector<double> buffer;   // 用來裝資料的陣列
    double sum;                   // 資料的總和
    int index;                    // 目前寫入的位置

    MovingAverageFilter(int size = 5) {
        window_size = size;
        buffer.assign(size, 0.0); // 準備一個大小為 size 的陣列，初始值為 0
        sum = 0.0;
        index = 0;
    }

    double update(double measurement) {
        sum = sum - buffer[index];       // 先減去最舊的那筆資料
        buffer[index] = measurement;     // 存入最新的這筆資料
        sum = sum + measurement;         // 加上最新的資料
        
        index = index + 1;               // 格子往下移動
        if (index >= window_size) {
            index = 0;                   // 如果到底了，就繞回第 0 格 (環形陣列)
        }
        
        return sum / window_size;        // 回傳平均值
    }

    void reset(double value) {
        buffer.assign(window_size, value);
        sum = value * window_size;
        index = 0;
    }    
};