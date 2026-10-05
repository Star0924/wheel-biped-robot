"""
balance_mpc_yaw.py —— 在原本 4 階 (p, v, theta, omega) 上擴增 yaw，變成 6 階

狀態 x = [p, v, theta, omega, psi_err, psi_dot]^T
    psi_err : 航向誤差 (rad) = wrap(psi - psi_target)，由 bridge 端算好再送進來
    psi_dot : 偏航角速度 (rad/s)
輸入 u = [u_s, u_d]^T
    u_s = uL + uR   (合計扭矩，推動前後 / 平衡，跟原本的 u 完全一樣)
    u_d = uR - uL   (差動扭矩，產生 yaw；正值 = 左轉，右輪比左輪出力大)

線性化下 yaw 與矢狀面完全解耦，所以 A 是 blockdiag(A_sagittal, A_yaw)，
只有 B 多了一欄 (u_d -> psi_ddot)。
    psi_ddot = k_yaw * u_d,   k_yaw = torque_gain_yaw * d / (2 r I_eff)
    I_eff    = I_z + m_w d^2/2 + I_w d^2/(2 r^2)
    (I_w d^2/(2r^2) 是「輪子被迫差速轉動」的反射慣量，數值上不小，別漏掉)

「左右輪各自 |u|<=u_max/2」 等價於 |u_s| + |u_d| <= u_max，仍是凸的線性約束。
"""

import time
import numpy as np
import cvxpy as cp
from dataclasses import dataclass
from scipy.linalg import expm

# ---------------------------------------------------------------- 調參區
#            [p,   v,    theta, omega, psi_err, psi_dot]
Q_DIAG = [10.0, 12.0, 90.0, 9.5, 8.5, 5.0]
QF_SCALE = 20.0
R_U_DIAG = [0.09, 0.5]      # [u_s, u_d]
R_DU_DIAG = [1.80, 1.20]     # [u_s, u_d]  u_s 維持原值，行為才會跟舊版一致


@dataclass
class RobotParams:
    m_w: float = 3.165
    I_w: float = 0.0256
    m_b: float = 7.6
    I_b: float = 0.308
    r: float = 0.108
    g: float = 9.81
    torque_gain: float = 0.7

    track_width: float = 0.441     # d：左右輪接地點距離 (m) 
    I_z: float = 0.2928             # 機身繞垂直軸轉動慣量 (kg*m^2) 
    torque_gain_yaw: float = 0.7

    u_max: float = 8.0            # |u_s|+|u_d| 上限
    du_max: float = 4.0           # u_s 每步變化率上限
    du_max_d: float = 3.0         # u_d 每步變化率上限
    theta_max: float = 0.35


def continuous_model(params: RobotParams, l: float):
    p = params
    a = p.m_w + p.m_b + p.I_w / p.r**2
    b = p.m_b * l
    c = p.I_b + p.m_b * l**2
    det = a * c - b**2

    A = np.zeros((6, 6))
    A[0, 1] = 1.0
    A[1, 2] = -b * p.m_b * p.g * l / det
    A[2, 3] = 1.0
    A[3, 2] = a * p.m_b * p.g * l / det
    A[4, 5] = 1.0                       # psi_err' = psi_dot

    I_eff = (p.I_z + p.m_w * p.track_width**2 / 2.0
             + p.I_w * p.track_width**2 / (2.0 * p.r**2))
    k_yaw = p.torque_gain_yaw * p.track_width / (2.0 * p.r * I_eff)

    B = np.zeros((6, 2))
    B[1, 0] = (c / p.r + b) / det * p.torque_gain
    B[3, 0] = -(b / p.r + a) / det * p.torque_gain
    B[5, 1] = k_yaw
    return A, B


def discretize(A, B, Ts):
    n, m = A.shape[0], B.shape[1]
    M = np.zeros((n + m, n + m))
    M[:n, :n] = A
    M[:n, n:] = B
    Md = expm(M * Ts)
    return Md[:n, :n], Md[:n, n:]


class BalanceMPC:
    def __init__(self, params: RobotParams, N=20, Ts=0.015, l_nominal=0.181,
                 osqp_eps=1e-2, osqp_max_iter=4000):
        self.p, self.N, self.Ts = params, N, Ts
        self.nx, self.nu = 6, 2
        self.osqp_eps, self.osqp_max_iter = osqp_eps, osqp_max_iter

        self.Q = np.diag(Q_DIAG)
        self.Qf = self.Q * QF_SCALE
        self.R = np.diag(R_U_DIAG)
        self.R_delta = np.diag(R_DU_DIAG)
        self.du_vec = np.array([params.du_max, params.du_max_d])

        self.u_prev = np.zeros(2)
        self.solve_times, self.qp_solve_times = [], []
        self.rebuild_count = 0
        self._l_current = None
        self._build_problem(l_nominal)

    def _build_problem(self, l):
        p, N, nx, nu = self.p, self.N, self.nx, self.nu
        A_c, B_c = continuous_model(p, l)
        A_d, B_d = discretize(A_c, B_c, self.Ts)
        self.A_c, self.B_c, self.A_d, self.B_d = A_c, B_c, A_d, B_d
        self._l_current = l

        self.X = cp.Variable((nx, N + 1))
        self.U = cp.Variable((nu, N))
        self.x0_param = cp.Parameter(nx)
        self.v_ref_param = cp.Parameter(N)
        self.p_ref_param = cp.Parameter(N)
        self.psid_ref_param = cp.Parameter(N)     # 偏航角速度參考
        self.u_prev_param = cp.Parameter(nu)

        cost = 0
        cons = [self.X[:, 0] == self.x0_param]
        for k in range(N):
            x_ref_k = cp.hstack([self.p_ref_param[k], self.v_ref_param[k],
                                 0.0, 0.0, 0.0, self.psid_ref_param[k]])
            cost += cp.quad_form(self.X[:, k], self.Q) - 2 * (self.Q @ x_ref_k) @ self.X[:, k]
            cost += cp.quad_form(self.U[:, k], self.R)
            if k == 0:
                cost += (cp.quad_form(self.U[:, k], self.R_delta)
                         - 2 * (self.R_delta @ self.u_prev_param) @ self.U[:, k])
                cons += [cp.abs(self.U[:, k] - self.u_prev_param) <= self.du_vec]
            else:
                cost += cp.quad_form(self.U[:, k] - self.U[:, k - 1], self.R_delta)
                cons += [cp.abs(self.U[:, k] - self.U[:, k - 1]) <= self.du_vec]

            cons += [self.X[:, k + 1] == A_d @ self.X[:, k] + B_d @ self.U[:, k]]
            cons += [cp.abs(self.U[0, k]) + cp.abs(self.U[1, k]) <= p.u_max]
            if k >= 1:
                cons += [cp.abs(self.X[2, k]) <= p.theta_max]

        x_ref_N = cp.hstack([self.p_ref_param[N - 1], self.v_ref_param[N - 1],
                             0.0, 0.0, 0.0, self.psid_ref_param[N - 1]])
        cost += cp.quad_form(self.X[:, N], self.Qf) - 2 * (self.Qf @ x_ref_N) @ self.X[:, N]

        self.prob = cp.Problem(cp.Minimize(cost), cons)
        assert self.prob.is_dpp(), "問題不符合DPP規則"

    def _predict_delay(self, x0, delay_s):
        if delay_s <= 1e-4:
            return x0
        delay_s = float(min(delay_s, 0.06))
        Ad, Bd = discretize(self.A_c, self.B_c, delay_s)
        return Ad @ np.asarray(x0, float) + Bd @ self.u_prev

    def set_applied_u(self, u_s: float, u_d: float):
        self.u_prev = np.array([u_s, u_d], dtype=float)

    def solve(self, x0, l_current, v_ref_traj, p_ref_traj=None,
              psid_ref_traj=None, delay_s=0.0):
        t_start = time.perf_counter()
        rebuilt = False
        if self._l_current is None or abs(l_current - self._l_current) > 1e-3:
            self._build_problem(l_current)
            self.rebuild_count += 1
            rebuilt = True

        x0_pred = self._predict_delay(x0, delay_s)
        self.x0_param.value = np.asarray(x0_pred, float)
        self.v_ref_param.value = np.asarray(v_ref_traj, float)
        self.p_ref_param.value = np.zeros(self.N) if p_ref_traj is None else np.asarray(p_ref_traj, float)
        self.psid_ref_param.value = (np.zeros(self.N) if psid_ref_traj is None
                                     else np.asarray(psid_ref_traj, float))
        self.u_prev_param.value = self.u_prev.copy()

        t_qp = time.perf_counter()
        self.prob.solve(solver=cp.OSQP, warm_start=True, verbose=False,
                        eps_abs=self.osqp_eps, eps_rel=self.osqp_eps,
                        max_iter=self.osqp_max_iter)
        qp_ms = (time.perf_counter() - t_qp) * 1000.0
        total_ms = (time.perf_counter() - t_start) * 1000.0
        self.solve_times.append(total_ms)
        self.qp_solve_times.append(qp_ms)
        timing = {"total_ms": total_ms, "qp_ms": qp_ms, "rebuilt": rebuilt,
                  "status": self.prob.status, "delay_ms": delay_s * 1000.0}

        if self.prob.status not in ("optimal", "optimal_inaccurate"):
            print(f"[BalanceMPC] QP 求解失敗: {self.prob.status}")
            return self.u_prev.copy(), None, timing

        u0 = np.array(self.U.value[:, 0], dtype=float)
        self.u_prev = u0.copy()
        return u0, (self.X.value, self.U.value), timing


def run_simulation(sim_steps=500, delay_s=0.025):
    """yaw 階躍測試：step 150 起要求 +0.6 rad/s 偏航率，step 350 回 0。"""
    params = RobotParams()
    mpc = BalanceMPC(params, N=20, Ts=0.015)
    x = np.array([0.0, 0.0, 0.03, 0.0, 0.0, 0.0])
    delay_steps = max(1, int(round(delay_s / mpc.Ts)))
    buf = [np.zeros(2)] * delay_steps
    psi_target = 0.0
    for step in range(sim_steps):
        psid_cmd = 0.6 if 150 <= step < 350 else 0.0
        psi_target += psid_cmd * mpc.Ts
        u0, _, tm = mpc.solve(x, 0.181, np.zeros(mpc.N),
                              psid_ref_traj=np.full(mpc.N, psid_cmd),
                              delay_s=delay_steps * mpc.Ts)
        buf.append(u0)
        u = buf.pop(0)
        x = mpc.A_d @ x + mpc.B_d @ u
        x[4] -= psid_cmd * mpc.Ts              # psi_err 相對移動目標
        if step % 50 == 0:
            print(f"step={step:4d} theta={np.degrees(x[2]):+.2f}deg "
                  f"psi_dot={x[5]:+.3f} psi_err={x[4]:+.3f} "
                  f"u_s={u0[0]:+.2f} u_d={u0[1]:+.2f} solve={tm['total_ms']:.1f}ms")


if __name__ == "__main__":
    run_simulation()
