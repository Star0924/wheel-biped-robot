#pragma once
#include <math.h>
#include <stdint.h>

// ============================================================
// IMU 互補濾波：加速度計(重力向量) + 陀螺儀 -> pitch / roll
// 不依賴 Arduino，可直接用 g++ 在電腦上測試。
//
// 架構 (PI 型互補濾波 / Mahony 的 2D 簡化版)：
//   pitch_dot = (gyro - bias) + Kp * (accPitch - pitch)
//   bias_dot  = -Ki * (accPitch - pitch)
//
// 加速度計補償有兩層：
//   (1) 線加速度補償：輪式倒單擺加速/減速時，加速度計量到的是
//       「重力 + 車體加速度」，直接用 atan2 算角度會有 atan(a/g) 的誤差
//       (0.1g -> 約 5.7°)。這裡把輪速微分得到的前向加速度扣掉。
//   (2) 自適應增益：|a| 偏離 1g 越多，代表加速度計越不可信，
//       Kp/Ki 就越小(甚至為0，完全靠陀螺儀積分)。
// ============================================================

struct ImuFusionParams {
    double tau_s         = 0.5;    // 互補濾波時間常數 (s)，越大越信任陀螺儀
    double kiRatio       = 0.05;   // Ki = kiRatio * Kp^2 (0.25 為臨界阻尼，取小一點讓偏置慢慢收斂)
    double normTol_g     = 0.15;   // |a|-1g 超過這個值，加速度計信任度降為0
    double accPitchSign  = +1.0;   // 加速度算出的 pitch 方向 (要與感測器 angle[1] 同號)
    double accRollSign   = +1.0;
    double gyroPitchSign = +1.0;   // 陀螺儀 Y 軸方向 (要與 pitch 同號)
    double gyroRollSign  = +1.0;
    double linAccGain    = 0.0;    // 線加速度補償係數：0=關閉，+1/-1=開啟(正負號依安裝方向)
};

class ImuFusion {
public:
    ImuFusionParams p;

    // ---- 輸出 ----
    double pitch = 0.0, roll = 0.0;            // 融合後角度 (deg)
    double pitchRate = 0.0, rollRate = 0.0;    // 扣除偏置、換算成歐拉角速度 (deg/s)
    // ---- 診斷 ----
    double accPitchRaw  = 0.0;                 // 只用重力向量算出的 pitch (未補償)
    double accPitchComp = 0.0;                 // 扣掉線加速度後的 pitch
    double accNorm = 1.0;                      // 補償後的 |a| (g)
    double trust   = 0.0;                      // 加速度計信任度 0~1
    double gyroBias[3] = {0.0, 0.0, 0.0};      // 陀螺儀零偏 (deg/s)
    double pitchOffset = 0.0, rollOffset = 0.0;// 靜態校正得到的安裝角度偏移 (deg)
    double dt = 0.005;                         // 目前使用的取樣週期估計 (s)
    bool   initialized = false;

    // 靜態校正：車體保持不動，約 n 筆 gyro 取樣 (200Hz 約 1 秒)
    // 完成後：gyro 零偏 = 平均值；目前姿態 = pitch 0 / roll 0
    //   setAngleZero=false 時只估陀螺儀零偏，不動 pitch/roll 的零點 (開機自動校正用)
    void startCalibration(int n = 200, bool setAngleZero = true) {
        calN = n; calCount = 0; calState = 1; calSetZero = setAngleZero;
        for (int i = 0; i < 3; i++) { sumG[i] = 0; mn[i] = 1e9; mx[i] = -1e9; }
        sumP = sumR = 0;
    }
    int calStatus() const { return calState; }   // 0 閒置, 1 進行中, 2 成功, -1 失敗(校正中有移動)

    // 以 micros() 時間戳驅動 (dt 用移動平均平滑，避免序列埠一次吐多幀造成 dt 抖動)
    void stepMicros(const double a[3], const double g[3], uint32_t nowUs, double linAcc_g = 0.0) {
        if (!haveLast) {
            haveLast = true; lastUs = nowUs;
            update(a, g, 0.0, linAcc_g);
            return;
        }
        double d = (uint32_t)(nowUs - lastUs) * 1e-6;
        lastUs = nowUs;
        if (d > 1e-3 && d < 0.1) dt += 0.1 * (d - dt);
        update(a, g, (d >= 0.1) ? 0.0 : dt, linAcc_g);
    }

    // a: 加速度 (g)；g: 角速度 (deg/s)；linAcc_g: 車體前向線加速度 (g, 前進加速為正)
    void update(const double a[3], const double g[3], double dt_s, double linAcc_g = 0.0) {
        const double R2D = 57.29577951308232, D2R = 1.0 / R2D;
        if (dt_s < 0.0) dt_s = 0.0;

        // ---- 重力向量 -> 傾角 (Euler ZYX：pitch 不受 roll 影響) ----
        double ax = a[0], ay = a[1], az = a[2];
        double yz = sqrt(ay * ay + az * az);
        double pitchAbsRaw = p.accPitchSign * atan2(-ax, yz) * R2D;
        double rollAbs     = p.accRollSign  * atan2(ay, az)  * R2D;

        // ---- 線加速度補償：把車體前向加速度從 ax 扣掉 ----
        double axc = ax - p.linAccGain * linAcc_g;
        double pitchAbsComp = p.accPitchSign * atan2(-axc, yz) * R2D;

        accPitchRaw  = pitchAbsRaw  - pitchOffset;
        accPitchComp = pitchAbsComp - pitchOffset;
        double accRoll = rollAbs - rollOffset;

        // ---- 自適應信任度 ----
        accNorm = sqrt(axc * axc + ay * ay + az * az);
        double dev = fabs(accNorm - 1.0) / p.normTol_g;
        trust = (dev >= 1.0) ? 0.0 : (1.0 - dev * dev);

        // ---- 陀螺儀：扣偏置並轉成歐拉角速度 ----
        double gx = g[0] - gyroBias[0];
        double gy = g[1] - gyroBias[1];
        double gz = g[2] - gyroBias[2];
        double gxE = p.gyroRollSign  * gx;
        double gyE = p.gyroPitchSign * gy;
        double phi = roll * D2R, th = pitch * D2R;
        double cth = cos(th); if (cth < 0.2) cth = 0.2;       // 避免接近 ±90° 時奇異
        pitchRate = gyE * cos(phi) - gz * sin(phi);
        rollRate  = gxE + (gyE * sin(phi) + gz * cos(phi)) * sin(th) / cth;

        if (!initialized) {
            pitch = accPitchComp; roll = accRoll; initialized = true;
        } else if (dt_s > 0.0) {
            double kp0 = 1.0 / p.tau_s;
            double kp  = kp0 * trust;
            double ki  = p.kiRatio * kp0 * kp0 * trust;
            double errP = accPitchComp - pitch;
            double errR = accRoll - roll;

            pitch += (pitchRate + kp * errP) * dt_s;
            roll  += (rollRate  + kp * errR) * dt_s;

            // 只自動修正 pitch 軸偏置 (roll 方向若符號有誤會讓偏置發散，所以不自動調)
            gyroBias[1] += -p.gyroPitchSign * ki * errP * dt_s;
            if (gyroBias[1] >  10.0) gyroBias[1] =  10.0;
            if (gyroBias[1] < -10.0) gyroBias[1] = -10.0;
        }

        if (calState == 1) collect(g, pitchAbsRaw, rollAbs);
    }

private:
    bool haveLast = false; uint32_t lastUs = 0;
    int calState = 0, calN = 0, calCount = 0;
    bool calSetZero = true;
    double sumG[3], mn[3], mx[3], sumP = 0, sumR = 0;

    void collect(const double g[3], double pAbs, double rAbs) {
        for (int i = 0; i < 3; i++) {
            sumG[i] += g[i];
            if (g[i] < mn[i]) mn[i] = g[i];
            if (g[i] > mx[i]) mx[i] = g[i];
        }
        sumP += pAbs; sumR += rAbs;
        if (++calCount < calN) return;

        bool still = true;
        for (int i = 0; i < 3; i++) if (mx[i] - mn[i] > 3.0) still = false;   // 3 dps 以內視為靜止
        if (still) {
            for (int i = 0; i < 3; i++) gyroBias[i] = sumG[i] / calN;
            if (calSetZero) {
                pitchOffset = sumP / calN;
                rollOffset  = sumR / calN;
            }
            initialized = false;       // 下一幀以新的零點重新初始化
            calState = 2;
        } else {
            calState = -1;
        }
    }
};

// ============================================================
// 原始資料調理 (Raw data conditioning)
//   acc : 去突波 (3點中值) -> 2階 Butterworth 低通 (預設 10Hz)
//   gyro: 去突波 (3點中值) -> 2階 Butterworth 低通 (預設 40Hz)
//
// 為什麼加速度可以濾很兇、陀螺儀不行：
//   加速度只走「慢速修正」那條路 (時間常數 tau=0.5s)，10Hz 低通的延遲(~30ms)
//   遠小於 tau，幾乎無影響；但陀螺儀的 pitchRate 會直接進 MPC 的回授，
//   濾得越兇相位延遲越大，會讓不穩定系統更難控，所以只濾掉高頻雜訊。
// ============================================================
struct Biquad {
    double b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, z1 = 0, z2 = 0;
    bool init = false;

    void setLowpass(double fc, double fs) {
        if (fc <= 0.0 || fc >= 0.45 * fs) { b0 = 1; b1 = b2 = a1 = a2 = 0; return; }   // 旁路
        const double PI_ = 3.14159265358979323846;
        double K = tan(PI_ * fc / fs), q = 0.70710678118654752;
        double norm = 1.0 / (1.0 + K / q + K * K);
        b0 = K * K * norm; b1 = 2.0 * b0; b2 = b0;
        a1 = 2.0 * (K * K - 1.0) * norm;
        a2 = (1.0 - K / q + K * K) * norm;
    }
    double process(double x) {
        if (!init) { z1 = x * (1.0 - b0); z2 = x * (b2 - a2); init = true; }   // 以第一筆當穩態，避免啟動暫態
        double y = b0 * x + z1;
        z1 = b1 * x - a1 * y + z2;
        z2 = b2 * x - a2 * y;
        return y;
    }
};

struct Median3 {
    double v[3] = {0, 0, 0}; bool init = false;
    double process(double x) {
        if (!init) { v[0] = v[1] = v[2] = x; init = true; }
        else { v[0] = v[1]; v[1] = v[2]; v[2] = x; }
        double a = v[0], b = v[1], c = v[2];
        return fmax(fmin(a, b), fmin(fmax(a, b), c));
    }
};

struct ImuConditionerParams {
    double accCutoffHz  = 10.0;
    double gyroCutoffHz = 40.0;
    bool   median       = true;    // 加速度 3 點中值
    bool   gyroMedian   = false;   // 陀螺儀 3 點中值：在斜坡訊號上會多 1 個取樣的延遲，回授用的 gyro 預設關閉
};

class ImuConditioner {
public:
    ImuConditionerParams p;
    void reconfigure() { lastFs = 0.0; }          // 改了 p 之後呼叫

    // fs: 目前 IMU 幀率 (Hz)；fs 變動超過10%會自動重算濾波係數
    void process(const double a[3], const double g[3], double fs, double aOut[3], double gOut[3]) {
        if (fs > 0.0 && (lastFs <= 0.0 || fabs(fs - lastFs) > 0.1 * lastFs)) {
            lastFs = fs;
            for (int i = 0; i < 3; i++) {
                accLp[i].setLowpass(p.accCutoffHz, fs);
                gyroLp[i].setLowpass(p.gyroCutoffHz, fs);
            }
            linLp.setLowpass(p.accCutoffHz, fs);
        }
        for (int i = 0; i < 3; i++) {
            double xa = p.median ? accMed[i].process(a[i]) : a[i];
            double xg = p.gyroMedian ? gyroMed[i].process(g[i]) : g[i];
            aOut[i] = accLp[i].process(xa);
            gOut[i] = gyroLp[i].process(xg);
        }
    }
    // 輪速微分得到的線加速度，走和加速度計「完全相同」的 中值+低通，延遲才會對齊
    // (否則在 ax 上扣掉一個沒延遲的修正量，反而會製造出假的瞬態誤差)。要在 process() 之後呼叫。
    double processLin(double x) {
        double xl = p.median ? linMed.process(x) : x;
        return linLp.process(xl);
    }
private:
    double lastFs = 0.0;
    Biquad accLp[3], gyroLp[3], linLp;
    Median3 accMed[3], gyroMed[3], linMed;
};

// 把兩次控制取樣之間所有 IMU 取樣取平均 (抗混疊的降頻)。
// IMU 約 200Hz、控制 100Hz：只取「最新一筆」會讓 >50Hz 的雜訊折疊進 0~50Hz；
// 取平均 = 2~3 點 boxcar，額外延遲約 (n-1)/2 個 IMU 取樣 (~2.5ms)。
struct SampleAverager {
    double sum = 0.0; int n = 0; double last = 0.0;
    void add(double x) { sum += x; n++; }
    double take() { if (n > 0) { last = sum / n; sum = 0.0; n = 0; } return last; }
    void reset() { sum = 0.0; n = 0; }
};

// 在線統計 (Welford)，用來量「穩定度」
struct RunningStat {
    long n = 0; double mean = 0, m2 = 0, mn = 1e30, mx = -1e30;
    void add(double x) {
        n++; double d = x - mean; mean += d / n; m2 += d * (x - mean);
        if (x < mn) mn = x; if (x > mx) mx = x;
    }
    double stddev() const { return n > 1 ? sqrt(m2 / (n - 1)) : 0.0; }
    double range()  const { return n > 0 ? mx - mn : 0.0; }
};