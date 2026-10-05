"""
com_balance.py —— 腿部姿態 -> 整機 COM -> 平衡角 theta_eq (+ Jacobian) -> 高度 IK

座標：輪軸座標系，原點 = 輪軸中心，x 往前為正，z 往上為正。
角度：q1 = 0 腿垂直；q2 = 0 上下腿伸直。  所有角度 API 一律用 rad。

連桿約定 (照你給的公式)：從輪軸往上數
    link1 (l1, 質量 m1) : 輪軸 -> 膝      (你文中叫 thigh)
    link2 (l2, 質量 m2) : 膝   -> Hip     (你文中叫 shank)
    Hip = (-(l1 sin q1 + l2 sin(q1+q2)),  l1 cos q1 + l2 cos(q1+q2))
  [假設] 你貼上來的文字裡 x_h 前面的「∗」是亂碼的「−」號 (x1、x2 都是負號)，
         這裡全部統一用負號。若你的 q1 正方向相反，把 X_SIGN 改成 +1。

質量：mb / m1 / m2 / mw 都是「整台機器人兩條腿的合計」。
      兩條腿姿態對稱時，一條腿的模型就等於整機。
"""

from dataclasses import dataclass
import numpy as np

X_SIGN = +1.0   # x 方向符號約定


# ============================================================ 核心模型
def compute_balance(q1, q2, l1, l2, mb, m1, m2, mw, dx, dz):
    """
    輸入 (q1, q2 單位 rad)，回傳 dict:
        x_com, z_com, h_com (= z_com), l_com (= 輪軸到 COM 距離，給 MPC 當擺長),
        theta_eq_rad, theta_eq_deg, J_theta (shape (2,) = [d/dq1, d/dq2], 單位 rad/rad)
        total_mass
    theta_eq = -atan2(x_com, z_com)
    """
    s1, c1 = np.sin(q1), np.cos(q1)
    s12, c12 = np.sin(q1 + q2), np.cos(q1 + q2)
    xs = X_SIGN

    # ---- 各質量塊位置 ----
    x_h = xs * (l1 * s1 + l2 * s12)
    z_h = l1 * c1 + l2 * c12
    x_b, z_b = x_h + dx, z_h + dz
    x_1, z_1 = xs * (l1 / 2) * s1, (l1 / 2) * c1
    x_2 = xs * (l1 * s1 + (l2 / 2) * s12)
    z_2 = l1 * c1 + (l2 / 2) * c12
    x_w = z_w = 0.0

    M = mb + m1 + m2 + mw
    x_com = (mb * x_b + m1 * x_1 + m2 * x_2 + mw * x_w) / M
    z_com = (mb * z_b + m1 * z_1 + m2 * z_2 + mw * z_w) / M

    theta_eq = -np.arctan2(x_com, z_com)

    # ---- 解析 Jacobian ----
    # 各位置對 (q1, q2) 的偏微分
    dxh = np.array([xs * (l1 * c1 + l2 * c12), xs * (l2 * c12)])
    dzh = np.array([-(l1 * s1 + l2 * s12),     -(l2 * s12)])
    dx1 = np.array([xs * (l1 / 2) * c1, 0.0])
    dz1 = np.array([-(l1 / 2) * s1,     0.0])
    dx2 = np.array([xs * (l1 * c1 + (l2 / 2) * c12), xs * (l2 / 2) * c12])
    dz2 = np.array([-(l1 * s1 + (l2 / 2) * s12),     -(l2 / 2) * s12])

    dx_com = (mb * dxh + m1 * dx1 + m2 * dx2) / M    # body COM 對 q 的導數 = Hip 的導數
    dz_com = (mb * dzh + m1 * dz1 + m2 * dz2) / M

    r2 = x_com**2 + z_com**2
    J_theta = -(z_com * dx_com - x_com * dz_com) / r2

    return {
        "x_com": float(x_com), "z_com": float(z_com),
        "h_com": float(z_com), "l_com": float(np.sqrt(r2)),
        "theta_eq_rad": float(theta_eq), "theta_eq_deg": float(np.degrees(theta_eq)),
        "J_theta": J_theta, "total_mass": float(M),
    }


def theta_tilde(theta_imu, theta_eq):
    """MPC 應追蹤 theta_tilde = 0，而不是 theta_imu = 0。"""
    return theta_imu - theta_eq


# ============================================================ 高度 IK
def ik_height(h, l1, l2, knee_sign=+1.0):
    """
    Hip 正好在輪軸正上方 (x_h = 0)、高度 z_h = h 的 (q1, q2)，單位 rad。
    knee_sign = +1 / -1 選膝蓋彎曲方向。h 會被夾在 [|l1-l2|+eps, l1+l2-eps]。
    """
    h = float(np.clip(h, abs(l1 - l2) + 1e-4, l1 + l2 - 1e-4))
    c = (h * h - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    q2 = knee_sign * np.arccos(np.clip(c, -1.0, 1.0))
    # Hip 向量 = e^{i q1} (l1 + l2 e^{i q2}) (以 z 軸為 0 度)，要求 x_h = 0 -> 相位為 0
    q1 = -np.arctan2(l2 * np.sin(q2), l1 + l2 * np.cos(q2))
    return float(q1), float(q2)


@dataclass
class LegBalanceModel:
    # ---- 幾何 / 質量：【以下數值是佔位值，請換成你實測的】----
    l1: float = 0.12
    l2: float = 0.12
    mb: float = 7.6
    m1: float = 0.8
    m2: float = 0.8
    mw: float = 3.165
    dx: float = 0.010      # Body COM 相對 Hip：往前為正
    dz: float = 0.050      # Body COM 相對 Hip：往上為正
    knee_sign: float = +1.0
    h_min: float = 0.12    # Hip 高度可動範圍 (m)
    h_max: float = 0.22

    def clamp_h(self, h):
        return float(np.clip(h, self.h_min, self.h_max))

    def at_q(self, q1, q2):
        return compute_balance(q1, q2, self.l1, self.l2, self.mb, self.m1,
                               self.m2, self.mw, self.dx, self.dz)

    def at_height(self, h):
        """Hip 高度 -> dict(q1, q2, + compute_balance 全部輸出)"""
        h = self.clamp_h(h)
        q1, q2 = ik_height(h, self.l1, self.l2, self.knee_sign)
        out = self.at_q(q1, q2)
        out.update({"h_hip": h, "q1": q1, "q2": q2})
        return out


# ============================================================ 測試
def _fd_jacobian(q1, q2, args, eps=1e-6):
    f = lambda a, b: compute_balance(a, b, *args)["theta_eq_rad"]
    return np.array([(f(q1 + eps, q2) - f(q1 - eps, q2)) / (2 * eps),
                     (f(q1, q2 + eps) - f(q1, q2 - eps)) / (2 * eps)])


if __name__ == "__main__":
    np.set_printoptions(precision=5, suppress=True)
    args = (0.12, 0.12, 7.6, 0.8, 0.8, 3.165, 0.010, 0.050)   # l1 l2 mb m1 m2 mw dx dz

    print("=== 測試 1：腿完全伸直垂直 (q1=q2=0) ===")
    r = compute_balance(0.0, 0.0, *args)
    for k, v in r.items():
        print(f"  {k:13s} {v}")
    # 手算驗證：x_com = M^-1 * mb*dx = 7.6*0.01/12.365 ；腿垂直時其餘 x 全 0
    assert abs(r["x_com"] - 7.6 * 0.010 / 12.365) < 1e-9
    assert abs(r["theta_eq_rad"] + np.arctan2(r["x_com"], r["z_com"])) < 1e-12

    print("\n=== 測試 2：dx=0 且腿垂直伸直 -> theta_eq 必為 0；膝蓋彎曲則 COM 會前移 ===")
    a0 = (0.12, 0.12, 7.6, 0.8, 0.8, 3.165, 0.0, 0.050)
    r = compute_balance(0.0, 0.0, *a0)
    assert abs(r["theta_eq_rad"]) < 1e-12
    q1, q2 = ik_height(0.17, 0.12, 0.12)
    r = compute_balance(q1, q2, *a0)
    print(f"  彎膝 h=0.17: q1={np.degrees(q1):+.2f}deg q2={np.degrees(q2):+.2f}deg "
          f"x_com={r['x_com']:+.5f}m  theta_eq={r['theta_eq_deg']:+.3f}deg (連桿 COM 往前凸出所致)")

    print("\n=== 測試 3：Jacobian 解析解 vs 有限差分 ===")
    for (qa, qb) in [(0.0, 0.0), (-0.4, 0.8), (0.3, -0.6), (-0.7, 1.2)]:
        Ja = compute_balance(qa, qb, *args)["J_theta"]
        Jn = _fd_jacobian(qa, qb, args)
        print(f"  q=({qa:+.2f},{qb:+.2f})  analytic={Ja}  numeric={Jn}  |err|={np.abs(Ja-Jn).max():.2e}")
        assert np.abs(Ja - Jn).max() < 1e-6

    print("\n=== 測試 4：不同 Hip 高度 (IK, Hip 在輪軸正上方) ===")
    m = LegBalanceModel()
    print("  h_hip[m]  q1[deg]  q2[deg]  z_com[m]  l_com[m]  theta_eq[deg]  dth/dq1  dth/dq2")
    for h in np.linspace(m.h_min, m.h_max, 6):
        o = m.at_height(h)
        print(f"  {o['h_hip']:.3f}    {np.degrees(o['q1']):+7.2f}  {np.degrees(o['q2']):+7.2f}  "
              f"{o['z_com']:.4f}   {o['l_com']:.4f}   {o['theta_eq_deg']:+8.4f}      "
              f"{o['J_theta'][0]:+.3f}  {o['J_theta'][1]:+.3f}")

    print("\n=== theta_tilde ===")
    print("  theta_imu=2.0deg, theta_eq=1.4deg ->", theta_tilde(2.0, 1.4), "deg (MPC 追蹤這個 = 0)")
    print("\n全部測試通過")
