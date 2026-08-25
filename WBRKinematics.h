#ifndef WBR_KINEMATICS_H
#define WBR_KINEMATICS_H

#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 關節角度空間 (單位: deg)
struct JointSpace {
    double theta1 = 0.0; // 髖關節角度 (度)
    double theta2 = 0.0; // 膝關節角度 (度)
};

// 末端位置空間 (單位: mm)
struct GoalPos {
    double x = 0.0; // 水平偏移 (mm)
    double z = 0.0; // 垂直深度 (mm)
    double L = 0.0; // 虛擬腿長 (mm)
};

class WBRKinematics {
public:
    
    WBRKinematics(double thigh_len = 130.0, double calf_len = 130.0);

    // 設定腿長
    void setLegLengths(double thigh_len, double calf_len);

    // 正向與逆向運動學
    GoalPos fk(const JointSpace& joints) const;
    bool ik(const GoalPos& goal, JointSpace& joints) const;

private:
    double l1_; // 大腿長度 (Thigh length, mm)
    double l2_; // 小腿長度 (Calf length, mm)

    // 內部輔助：角度與弧度轉換
    inline double deg2rad(double deg) const { return deg * M_PI / 180.0; }
    inline double rad2deg(double rad) const { return rad * 180.0 / M_PI; }
};

#endif // WBR_KINEMATICS_H