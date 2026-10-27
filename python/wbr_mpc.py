"""
wbr_mpc.py (實車辨識增益版，CVXPY 寫法)

QP 以 CVXPY 依數學式直接寫出 (見 _build_problem)，底層仍用 OSQP 求解，
每拍只更新 cp.Parameter，不重新建模。需要 cvxpy >= 1.3 與 osqp。

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
import cvxpy as cp
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

    # ------------------------------------------------------------ QP 建構 (CVXPY)
    def _build_problem(self, l):
        """
        把 MPC 直接照數學式寫出來，CVXPY 負責轉成 OSQP 的 P, A, l, u。
        每拍會變的量 (x0, u_prev, 參考軌跡) 宣告成 cp.Parameter，
        問題只在腿長 l 改變時重建一次；之後每拍只更新參數 (DPP)，OSQP 沿用同一份分解並暖啟動。
        """
        p, N, nx, nu = self.p, self.N, self.nx, self.nu
        self._update_model(l)
        Pm = self._compute_terminal()
        self.Pm = Pm
        A, B = self.A_d, self.B_d
        Q, R, Rd = self.Q, self.R, self.R_delta

        # 每拍更新的參數
        self.x0_par = cp.Parameter(nx, name="x0")             # 延遲補償後的初始狀態
        self.u_prev_par = cp.Parameter(nu, name="u_prev")     # 上一拍送出的指令
        self.x_ref_par = cp.Parameter((nx, N), name="x_ref")  # 參考軌跡 (欄 k 對應 x_k)

        # 決策變數
        x = cp.Variable((nx, N + 1), name="x")
        u = cp.Variable((nu, N), name="u")
        self._x_var, self._u_var = x, u

        # 追蹤代價 (x - r)' W (x - r) 展開成 x'Wx - 2 r'W x (常數項不影響最佳解)。
        # 不直接寫 cp.quad_form(x - r, W)：參數 r 放在 quad_form 裡不符合 DPP，CVXPY 會每拍重新編譯。
        def track(v, r, W):
            return cp.quad_form(v, W) - 2 * (r @ W) @ v

        cost = 0
        constraints = [x[:, 0] == self.x0_par]                # 初始條件

        for k in range(N):
            # 代價：輸入大小 + 輸入變化量 (du_0 = u_0 - u_prev)
            cost += cp.quad_form(u[:, k], R)
            if k == 0:
                du = u[:, 0] - self.u_prev_par
                cost += track(u[:, 0], self.u_prev_par, Rd)
            else:
                du = u[:, k] - u[:, k - 1]
                cost += cp.quad_form(du, Rd)

            # 動態
            constraints += [x[:, k + 1] == A @ x[:, k] + B @ u[:, k]]

            # 輸入限幅與變化率限幅
            constraints += [-p.u_max <= u[:, k], u[:, k] <= p.u_max,
                            -self.du_vec <= du, du <= self.du_vec]

        for k in range(1, N):
            # 狀態追蹤代價 (x_1 .. x_{N-1}) 與俯仰角限幅
            cost += track(x[:, k], self.x_ref_par[:, k], Q)
            constraints += [-p.theta_max <= x[2, k], x[2, k] <= p.theta_max]

        # 終端代價 (與原版相同，終端參考取 x_ref 最後一欄)
        cost += track(x[:, N], self.x_ref_par[:, N - 1], 0.5 * (Pm + Pm.T))

        self.problem = cp.Problem(cp.Minimize(cost), constraints)
        assert self.problem.is_dpp()
        self.nz = N * (nx + nu)
        self._needs_cold_start = True

    def _solve_qp(self):
        """呼叫 OSQP；回傳 (是否可用, 狀態字串, 迭代數)"""
        if self._needs_cold_start:
            self.problem._solver_cache.clear()     # 丟掉舊的暖啟動，下次重新 setup
            self._needs_cold_start = False
        try:
            self.problem.solve(solver=cp.OSQP, warm_start=True,
                               eps_abs=OSQP_EPS, eps_rel=OSQP_EPS, max_iter=OSQP_MAX_ITER,
                               polishing=False, adaptive_rho=True, check_termination=5,
                               verbose=False)
        except cp.error.SolverError as e:
            return False, f"solver_error: {e}", 0
        info = self.problem.solver_stats.extra_stats.info
        status = self.problem.status
        ok = (self._u_var.value is not None and np.all(np.isfinite(self._u_var.value))
              and (status in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE)
                   or (status == cp.USER_LIMIT and info.prim_res < 5e-2)))  # 迭代用盡但殘差夠小 -> 仍可用
        return ok, info.status, info.iter

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
            self._needs_cold_start = True

    # ------------------------------------------------------------ 求解
    def solve(self, x0, l_current, X_ref_traj, delay_s=0.0):
        t_start = time.perf_counter()
        rebuilt = False

        if abs(l_current - self._l_current) > 1e-4:
            self._build_problem(l_current)
            self.rebuild_count += 1
            rebuilt = True

        x0p = np.asarray(self._predict_delay(x0, delay_s), float)

        self.x0_par.value = x0p
        self.u_prev_par.value = self.u_prev
        self.x_ref_par.value = np.asarray(X_ref_traj, float)[:, :self.N]

        t_qp = time.perf_counter()
        ok, qp_status, n_iter = self._solve_qp()
        qp_ms = (time.perf_counter() - t_qp) * 1000.0

        timing = {"total_ms": 0.0, "qp_ms": qp_ms, "rebuilt": rebuilt,
                  "status": "optimal" if ok else qp_status, "delay_ms": delay_s * 1000.0,
                  "iter": n_iter}

        if not ok:
            print(f"[BalanceMPC] QP 求解失敗: {qp_status}")
            if self.K_lqr is not None:
                u_fb = np.clip(-self.K_lqr @ x0p, -self.p.u_max, self.p.u_max)
            else:
                u_fb = self.u_prev.copy()
            self.set_applied_u(u_fb[0], u_fb[1])
            self._needs_cold_start = True
            timing["total_ms"] = (time.perf_counter() - t_start) * 1000.0
            self.solve_times.append(timing["total_ms"]); self.qp_solve_times.append(qp_ms)
            return u_fb, None, timing

        X = np.array(self._x_var.value, dtype=float)
        X[:, 0] = x0p
        U = np.array(self._u_var.value, dtype=float)
        u0 = U[:, 0].copy()
        self.set_applied_u(u0[0], u0[1])

        timing["total_ms"] = (time.perf_counter() - t_start) * 1000.0
        self.solve_times.append(timing["total_ms"])
        self.qp_solve_times.append(qp_ms)
        return u0, (X, U), timing
