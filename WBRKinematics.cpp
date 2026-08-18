#include "WBRKinematics.h"

WBRKinematics::WBRKinematics(double thigh_len, double calf_len){
    l1_ = thigh_len; 
    l2_ = calf_len;
}

void WBRKinematics::setLegLengths(double thigh_len, double calf_len) {
    l1_ = thigh_len;
    l2_ = calf_len;
}

GoalPos WBRKinematics::fk(const JointSpace& joints) const {
    double r1 = deg2rad(joints.theta1);
    double r2 = deg2rad(joints.theta2);

    GoalPos goal;
    // 分別乘上大腿長度 (l1_) 與小腿長度 (l2_)
    goal.x =  l1_ * std::sin(r1) + l2_ * std::sin(r1 + r2);
    goal.z = -l1_ * std::cos(r1) - l2_ * std::cos(r1 + r2);
    goal.L =  std::hypot(goal.x, goal.z);

    return goal;
}

bool WBRKinematics::ik(const GoalPos& goal, JointSpace& joints) const {
    double dist_sq = goal.x * goal.x + goal.z * goal.z;
    double dist    = std::hypot(goal.x, goal.z);

    double max_len = l1_ + l2_;
    double min_len = std::abs(l1_ - l2_);

    // 1. 檢查目標點是否超出工作範圍 [ |L1 - L2|, L1 + L2 ]
    if (dist > max_len || dist < min_len) {
        return false;
    }

    // 2. 通用餘弦定理求膝關節角度 theta2: cos(θ2) = (d - L1^2 - L2^2) / (2 * L1 * L2)
    double cos_theta2 = (dist_sq - l1_ * l1_ - l2_ * l2_) / (2.0 * l1_ * l2_);
    cos_theta2 = std::max(-1.0, std::min(1.0, cos_theta2)); // 數值防護
    
    double theta2_rad = std::acos(cos_theta2);
    joints.theta2 = rad2deg(theta2_rad);

    // 3. 計算大腿與虛擬腿夾角 beta_rad (使用 atan2 比 acos 更穩定)
    double alpha_rad = std::atan2(goal.x, -goal.z);
    double beta_rad  = std::atan2(l2_ * std::sin(theta2_rad), l1_ + l2_ * std::cos(theta2_rad));
    
    joints.theta1 = rad2deg(alpha_rad - beta_rad);

    return true;
}