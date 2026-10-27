"""
wbr_mpc.py (實車辨識增益版)

與前一版相比的關鍵修正：
  1. [根因] 俯仰輸入增益：由 log 辨識，實車「扭矩 -> 俯仰角加速度」約為教科書模型的 2.9 倍
     (速度通道約 1.3 倍)。舊模型低估增益 -> MPC 輸出過大 -> 20ms 延遲下產生 ~5.5Hz 飽和極限環。
     以 pitch_input_gain / vel_input_gain 修正 B 矩陣。
  2. 延遲預測：依「在途指令」逐段積分 (最近 2~3 拍指令尚未作用)，不再只用兩段近似。
  3. Yaw 慣量還原為物理值 (舊版改成 0.362 並無 log 依據：該 log 的 yaw 幾乎沒被激發)。
  4. 權重 / 限幅依新模型重新整定；du_max 不再過度壓低，以免吃掉相位裕度。
"""
import time
from collections import deque
import numpy as np
import scipy.sparse as sp
import osqp
from dataclasses import dataclass
from scipy.linalg import expm, solve_discrete_are

# ---------------------------------------------------------------- 調參區
#            [p,    v,   theta, omega, psi_err, psi_dot]
Q_DIAG    = [10.0, 5.0, 200.0, 15.0, 15.0, 2.0]   # theta/omega 為主；yaw 頻寬刻意壓低 (~2Hz)，對 2~3.5x 增益誤差與 50ms 延遲仍穩定
R_U_DIAG  = [0.3, 0.3]    # [u_l, u_r]
R_DU_DIAG = [0.5, 0.5]   # [u_l, u_r]

OSQP_EPS = 1e-3
OSQP_MAX_ITER = 1500


@dataclass
class RobotParams:
    m_w: float = 3.165
    I_w: float = 0.0256
    m_b: float = 7.6
    I_b: float = 0.308
    r: float = 0.108
    g: float = 9.81
    d: float = 0.441                # 左右輪接地點距離 (m)
    I_z: float = 0.2928             # 機身繞垂直軸轉動慣量 (kg*m^2)
    torque_gain_yaw: float = 1.0
    torque_gain: float = 1.0
    pitch_input_gain: float = 2.9   # 實測 / 模型：扭矩 -> omega 的增益比 (由 log 辨識)
    vel_input_gain: float = 1.3      # 實測 / 模型：扭矩 -> v 的增益比
    yaw_input_gain: float = 2.7      # 實測 / 模型：差動扭矩 -> yaw 角加速度 (由 20261027 log 的 5Hz 極限環估得 2.5~3，需用開迴路激勵確認)

    u_max: float = 6.0             # 單輪扭矩上限 (Nm)；馬達 1.6Nm/A，7.5A 上限約 12Nm
    du_max: float = 3.0              # 每拍 (10ms) 最大扭矩變化
    du_max_d: float = 3.0
    theta_max: float = 0.35


def continuous_model(params: RobotParams, l: float):
    p = params
    # 注意：兩輪總等效質量需與實車動態匹配
    a = p.m_w + p.m_b + p.I_w / p.r**2
    b = p.m_b * l
    c = p.I_b + p.m_b * l**2
    d = p.m_b * p.g * l
    e = (p.d**2 * p.m_w) / 2 + (p.d**2 * p.I_w) / (2 * p.r**2) + p.I_z
    det = a * c - b**2

    A = np.zeros((6, 6))
    A[0, 1] = 1.0
    A[1, 2] = -b * d / det
    A[2, 3] = 1.0
    A[3, 2] = a * d / det
    A[4, 5] = 1.0                       # psi_err' = psi_dot

    yaw_coeff = p.torque_gain_yaw * p.yaw_input_gain * p.d / (2.0 * p.r * e)

    B = np.zeros((6, 2))
    gv = p.torque_gain * p.vel_input_gain
    go = p.torque_gain * p.pitch_input_gain
    B[1, 0] = c / p.r / det * gv
    B[1, 1] = c / p.r / det * gv
    B[3, 0] = -b / p.r / det * go
    B[3, 1] = -b / p.r / det * go
    B[5, 0] = -yaw_coeff
    B[5, 1] = yaw_coeff

    return A, B


def discretize(A, B, Ts):
    n, m = A.shape[0], B.shape[1]
    M = np.zeros((n + m, n + m))
    M[:n, :n] = A
    M[:n, n:] = B
    Md = expm(M * Ts)
    return Md[:n, :n], Md[:n, n:]


class BalanceMPC:
    def __init__(self, params: RobotParams, N=15, Ts=0.01, l_nominal=0.170):
        self.p, self.N, self.Ts = params, N, Ts
        self.nx, self.nu = 6, 2

        self.Q = np.diag(Q_DIAG)
        self.R = np.diag(R_U_DIAG)
        self.R_delta = np.diag(R_DU_DIAG)
        self.du_vec = np.array([params.du_max, params.du_max_d])

        self.u_prev = np.zeros(2)
        self.u_hist = deque([np.zeros(2)] * 4, maxlen=4)  # 過去指令，[-1] 為上一拍 u_{n-1}
        self.solve_times = deque(maxlen=1000)
        self.qp_solve_times = deque(maxlen=1000)
        self.rebuild_count = 0
        self._l_current = None
        self._delay_cache = {}
        self._build_problem(l_nominal)

    # ------------------------------------------------------------ 模型
    def _update_model(self, l):
        A_c, B_c = continuous_model(self.p, l)
        A_d, B_d = discretize(A_c, B_c, self.Ts)
        self.A_c, self.B_c, self.A_d, self.B_d = A_c, B_c, A_d, B_d
        self._l_current = l
        self._delay_cache.clear()

    def _compute_terminal(self):
        try:
            Pm = solve_discrete_are(self.A_d, self.B_d, self.Q, self.R)
            self.K_lqr = np.linalg.solve(self.R + self.B_d.T @ Pm @ self.B_d,
                                         self.B_d.T @ Pm @ self.A_d)
            return Pm
        except Exception as e:
            print(f"[BalanceMPC] DARE 失敗 ({e})，終端代價改用 2Q，無 LQR 後備")
            self.K_lqr = None
            return 2.0 * self.Q

    # ------------------------------------------------------------ QP 建構
    def _build_problem(self, l):
        p, N, nx, nu = self.p, self.N, self.nx, self.nu
        self._update_model(l)
        Pm = self._compute_terminal()
        self.nz = N * (nx + nu)
        ix = lambda k: slice((k - 1) * nx, k * nx)
        iu = lambda k: slice(N * nx + k * nu, N * nx + (k + 1) * nu)

        P = sp.lil_matrix((self.nz, self.nz))
        for k in range(1, N):
            P[ix(k), ix(k)] = 2.0 * self.Q
        P[ix(N), ix(N)] = 2.0 * Pm
        for k in range(N):
            P[iu(k), iu(k)] = P[iu(k), iu(k)] + 2.0 * self.R + 2.0 * self.R_delta
            if k >= 1:
                P[iu(k - 1), iu(k - 1)] = P[iu(k - 1), iu(k - 1)] + 2.0 * self.R_delta
                P[iu(k), iu(k - 1)] = -2.0 * self.R_delta
                P[iu(k - 1), iu(k)] = -2.0 * self.R_delta
        self.P = sp.csc_matrix(P)
        self.Pm = Pm

        n_dyn, n_rate = N * nx, N * nu
        A_dyn = sp.lil_matrix((n_dyn, self.nz))
        for k in range(N):
            r0 = k * nx
            A_dyn[r0:r0 + nx, ix(k + 1)] = np.eye(nx)
            if k >= 1:
                A_dyn[r0:r0 + nx, ix(k)] = -self.A_d
            A_dyn[r0:r0 + nx, iu(k)] = -self.B_d

        A_rate = sp.lil_matrix((n_rate, self.nz))
        for k in range(N):
            r0 = k * nu
            A_rate[r0:r0 + nu, iu(k)] = np.eye(nu)
            if k >= 1:
                A_rate[r0:r0 + nu, iu(k - 1)] = -np.eye(nu)

        A_box = sp.eye(self.nz, format="lil")
        A = sp.vstack([A_dyn, A_rate, A_box], format="csc")

        lo = np.full(self.nz, -np.inf)
        hi = np.full(self.nz, np.inf)
        for k in range(1, N):
            lo[(k - 1) * nx + 2] = -p.theta_max
            hi[(k - 1) * nx + 2] = p.theta_max
        lo[N * nx:] = -p.u_max
        hi[N * nx:] = p.u_max
        self._box_lo, self._box_hi = lo, hi

        self._n_dyn, self._n_rate = n_dyn, n_rate
        self._ix, self._iu = ix, iu
        self._lo = np.concatenate([np.zeros(n_dyn), np.zeros(n_rate), lo])
        self._hi = np.concatenate([np.zeros(n_dyn), np.zeros(n_rate), hi])

        self.solver = osqp.OSQP()
        self.solver.setup(P=self.P, q=np.zeros(self.nz), A=A, l=self._lo, u=self._hi,
                          eps_abs=OSQP_EPS, eps_rel=OSQP_EPS, max_iter=OSQP_MAX_ITER,
                          polishing=False, adaptive_rho=True, check_termination=5,
                          warm_starting=True, verbose=False)
        self._z_last = np.zeros(self.nz)

    # ------------------------------------------------------------ 延遲預測 (在途指令逐段積分)
    def _delay_mats(self, dur_s):
        key = int(round(dur_s * 1e4))              # 0.1 ms 為單位快取
        m = self._delay_cache.get(key)
        if m is None:
            m = discretize(self.A_c, self.B_c, key * 1e-4)
            self._delay_cache[key] = m
        return m

    def _predict_delay(self, x0, delay_s):
        """
        量測時刻 t_n 的狀態 x0，預測到「本拍指令開始作用」的時刻 t_n + D。
        第 j 拍前送出的指令 u_{n-j} 從 t_n - j*Ts + D 起作用、持續 Ts。
        只需處理仍在途 (尚未作用或作用中) 的 j = 1..ceil(D/Ts)。
        """
        x = np.asarray(x0, float)
        D = float(min(delay_s, 0.05))
        if D <= 5e-4:
            return x
        Ts = self.Ts
        jmax = int(np.ceil(D / Ts))
        for j in range(jmax, 0, -1):                # 由舊到新
            seg0 = max(0.0, D - j * Ts)             # 此指令生效起點 (相對 t_n)
            seg1 = min(D, D - (j - 1) * Ts)         # 生效終點 (下一拍指令接手或到達 t_n+D)
            dur = seg1 - seg0
            if dur <= 1e-5:
                continue
            u = self.u_hist[-j] if j <= len(self.u_hist) else np.zeros(2)
            Ad, Bd = self._delay_mats(dur)
            x = Ad @ x + Bd @ u
        return x

    def set_applied_u(self, u_l: float, u_r: float):
        """設定純控制力矩 (不含摩擦前饋！避免正回授發散)"""
        u = np.array([u_l, u_r], dtype=float)
        self.u_prev = u.copy()
        self.u_hist.append(u.copy())
        if u_l == 0.0 and u_r == 0.0:
            self.u_hist.extend([np.zeros(2)] * 4)
            self._z_last = np.zeros(self.nz)

    # ------------------------------------------------------------ 求解
    def solve(self, x0, l_current, X_ref_traj, delay_s=0.0):
        t_start = time.perf_counter()
        N, nx, nu = self.N, self.nx, self.nu
        rebuilt = False

        if abs(l_current - self._l_current) > 1e-4:
            self._build_problem(l_current)
            self.rebuild_count += 1
            rebuilt = True

        x0p = np.asarray(self._predict_delay(x0, delay_s), float)

        q = np.zeros(self.nz)
        Xr = np.asarray(X_ref_traj, float)
        if np.any(Xr):
            for k in range(1, N):
                q[(k - 1) * nx:k * nx] = -2.0 * self.Q @ Xr[:, k]
            q[(N - 1) * nx:N * nx] = -2.0 * self.Pm @ Xr[:, N - 1]
        q[N * nx:N * nx + nu] += -2.0 * self.R_delta @ self.u_prev

        lo, hi = self._lo.copy(), self._hi.copy()
        lo[:self._n_dyn][:nx] = hi[:self._n_dyn][:nx] = self.A_d @ x0p
        r = self._n_dyn
        lo[r:r + nu] = self.u_prev - self.du_vec
        hi[r:r + nu] = self.u_prev + self.du_vec
        lo[r + nu:r + self._n_rate] = -np.tile(self.du_vec, N - 1)
        hi[r + nu:r + self._n_rate] = np.tile(self.du_vec, N - 1)

        t_qp = time.perf_counter()
        self.solver.update(q=q, l=lo, u=hi)
        self.solver.warm_start(x=self._z_last)
        res = self.solver.solve()
        qp_status = res.info.status
        ok = (res.x is not None and np.all(np.isfinite(res.x))
              and (res.info.status_val in (1, 2)
                   or (res.info.status_val == -2 and res.info.prim_res < 5e-2)))   # 迭代用盡但殘差夠小 -> 仍可用 (暖啟動下一拍會接續收斂)
        qp_ms = (time.perf_counter() - t_qp) * 1000.0

        timing = {"total_ms": 0.0, "qp_ms": qp_ms, "rebuilt": rebuilt,
                  "status": "optimal" if ok else qp_status, "delay_ms": delay_s * 1000.0,
                  "iter": res.info.iter}

        if not ok:
            print(f"[BalanceMPC] QP 求解失敗: {qp_status}")
            if self.K_lqr is not None:
                u_fb = np.clip(-self.K_lqr @ x0p, -self.p.u_max, self.p.u_max)
            else:
                u_fb = self.u_prev.copy()
            self.set_applied_u(u_fb[0], u_fb[1])
            self._z_last = np.zeros(self.nz)
            timing["total_ms"] = (time.perf_counter() - t_start) * 1000.0
            self.solve_times.append(timing["total_ms"]); self.qp_solve_times.append(qp_ms)
            return u_fb, None, timing

        z = res.x
        self._z_last = z.copy()
        u0 = np.array(z[N * nx:N * nx + nu], dtype=float)
        self.set_applied_u(u0[0], u0[1])

        timing["total_ms"] = (time.perf_counter() - t_start) * 1000.0
        self.solve_times.append(timing["total_ms"])
        self.qp_solve_times.append(qp_ms)
        X = np.column_stack([x0p] + [z[(k - 1) * nx:k * nx] for k in range(1, N + 1)])
        U = z[N * nx:].reshape(N, nu).T
        return u0, (X, U), timing