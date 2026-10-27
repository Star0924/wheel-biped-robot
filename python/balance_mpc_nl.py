"""
balance_mpc_nl.py  --  非線性動態版 BalanceMPC (SQP / real-time iteration)

與 balance_mpc_yaw.py 的差別
  * 預測模型用「完整非線性」輪式倒單擺 (sin/cos 重力項、離心項、質量矩陣隨 theta 變化)，
    yaw 部分維持原本的線性模型。
  * 每次 solve：沿著 (上一次解平移後的) 輸入序列做 RK4 forward rollout -> 對每個 stage
    用 complex-step 取得精確離散 Jacobian A_k, B_k -> 組成「時變線性化」QP 再用 OSQP 解。
    QP 的大小/結構和線性版完全一樣 (N、nx、nu、限制式都相同)，只是 A,B 每個 stage 不同、多一個仿射項 c_k。
  * sqp_iters=1 就是 RTI (real-time iteration)；>1 則同一個取樣時間內重複線性化+解 QP。
  * 介面與 BalanceMPC.solve 相同，可直接換進 mpc_serial_bridge_yaw.py。

timing dict 比線性版多了：
  lin_ms  = rollout + Jacobian (非線性多出來的成本，與 QP 後端無關)
  qp_ms   = 所有 SQP 迭代中 OSQP 的總時間 (含 cvxpy 填參數)
  iters   = 實際 SQP 迭代數

直接執行本檔：用「非線性被控體」做閉迴路模擬，比較 線性MPC vs 非線性MPC 的求解時間與表現。
"""
import math
import time
import numpy as np

from balance_mpc_yaw import RobotParams, Q_DIAG, QF_SCALE, R_U_DIAG, R_DU_DIAG


# ------------------------------------------------------------------ 非線性模型
class NLModel:
    """狀態 x = [p, v, theta, omega, psi_err, psi_dot]，輸入 u = [u_l, u_r]。

    與線性版同一組係數 (a,b,c,d)；差別只在：
        M(theta) = [[a, b*cos th], [b*cos th, c]]
        f1 = T/r + b*sin th * omega^2          (離心項)
        f2 = d*sin th - T                       (重力 sin，不再用 th 近似)
        [p'', th''] = M(theta)^-1 [f1, f2],  T = (u_l + u_r) * torque_gain
    在 theta=0 線性化會剛好回到 continuous_model() 的 A、B。
    """

    def __init__(self, params: RobotParams, l: float):
        self.p = params
        self.set_l(l)

    def set_l(self, l: float):
        p = self.p
        self.l = l
        self.a = p.m_w + p.m_b + p.I_w / p.r**2
        self.b = p.m_b * l
        self.c = p.I_b + p.m_b * l**2
        self.d = p.m_b * p.g * l
        self.yaw = p.torque_gain_yaw * p.wheel_dis / (2.0 * p.r * p.I_z)
        self.ir = 1.0 / p.r
        self.tg = p.torque_gain

    # --- 向量版 (支援 complex，用於 complex-step 求 Jacobian) ---
    def f_vec(self, x, u):
        th, om = x[..., 2], x[..., 3]
        T = (u[..., 0] + u[..., 1]) * self.tg
        s, co = np.sin(th), np.cos(th)
        f1 = T * self.ir + self.b * s * om * om
        f2 = self.d * s - T
        det = self.a * self.c - (self.b * co) ** 2
        ax = (self.c * f1 - self.b * co * f2) / det
        ath = (-self.b * co * f1 + self.a * f2) / det
        return np.stack([x[..., 1], ax, x[..., 3], ath, x[..., 5],
                         self.yaw * (u[..., 1] - u[..., 0])], axis=-1)

    def rk4_vec(self, x, u, h):
        k1 = self.f_vec(x, u)
        k2 = self.f_vec(x + 0.5 * h * k1, u)
        k3 = self.f_vec(x + 0.5 * h * k2, u)
        k4 = self.f_vec(x + h * k3, u)
        return x + (h / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    # --- 純量版 (python float，rollout / 被控體模擬用，比 numpy 小陣列快很多) ---
    def f_s(self, x, u):
        th, om = x[2], x[3]
        T = (u[0] + u[1]) * self.tg
        s, co = math.sin(th), math.cos(th)
        f1 = T * self.ir + self.b * s * om * om
        f2 = self.d * s - T
        det = self.a * self.c - (self.b * co) ** 2
        return [x[1],
                (self.c * f1 - self.b * co * f2) / det,
                x[3],
                (-self.b * co * f1 + self.a * f2) / det,
                x[5],
                self.yaw * (u[1] - u[0])]

    def rk4_s(self, x, u, h):
        k1 = self.f_s(x, u)
        k2 = self.f_s([xi + 0.5 * h * ki for xi, ki in zip(x, k1)], u)
        k3 = self.f_s([xi + 0.5 * h * ki for xi, ki in zip(x, k2)], u)
        k4 = self.f_s([xi + h * ki for xi, ki in zip(x, k3)], u)
        return [xi + (h / 6.0) * (a + 2 * b + 2 * c + d)
                for xi, a, b, c, d in zip(x, k1, k2, k3, k4)]

    def jac(self, X, U, h):
        """離散化 (RK4 一步) 的精確 Jacobian。X:(N,6) U:(N,2) -> A:(N,6,6) B:(N,6,2)。"""
        Z = np.concatenate([X, U], axis=1)                       # (N,8)
        Zc = np.repeat(Z[:, None, :], 8, axis=1).astype(complex)  # (N,8,8)
        idx = np.arange(8)
        eps = 1e-30
        Zc[:, idx, idx] += 1j * eps
        xn = self.rk4_vec(Zc[..., :6], Zc[..., 6:], h)           # (N,8,6)
        J = xn.imag / eps                                         # J[k,j,i] = d xn_i / d z_j
        A = np.swapaxes(J[:, :6, :], 1, 2).copy()
        B = np.swapaxes(J[:, 6:, :], 1, 2).copy()
        return A, B


# ------------------------------------------------------------------ 非線性 MPC
class NonlinearBalanceMPC:
    def __init__(self, params: RobotParams, N=20, Ts=0.015, l_nominal=0.181,
                 sqp_iters=1, osqp_eps=1e-2, osqp_max_iter=4000, build_qp=True):
        self.p, self.N, self.Ts = params, N, Ts
        self.nx, self.nu = 6, 2
        self.sqp_iters = int(sqp_iters)
        self.osqp_eps, self.osqp_max_iter = osqp_eps, osqp_max_iter

        self.Q = np.diag(Q_DIAG)
        self.Qf = self.Q * QF_SCALE
        self.R = np.diag(R_U_DIAG)
        self.R_delta = np.diag(R_DU_DIAG)
        self.du_vec = np.array([params.du_max, params.du_max_d])

        self.model = NLModel(params, l_nominal)
        self.u_prev = np.zeros(2)
        self.U_guess = np.zeros((N, 2))
        self.has_guess = False
        self.solve_times, self.qp_solve_times, self.lin_times = [], [], []
        if build_qp:
            self._build_qp()

    # ---------------------------------------------------------------- QP (cvxpy + OSQP)
    def _build_qp(self):
        import cvxpy as cp
        self._cp = cp
        p, N, nx, nu = self.p, self.N, self.nx, self.nu
        # 時變線性化：每個 stage 一組 Parameter (DPP：param @ var 合法)
        self.A_p = [cp.Parameter((nx, nx)) for _ in range(N)]
        self.B_p = [cp.Parameter((nx, nu)) for _ in range(N)]
        self.c_p = [cp.Parameter(nx) for _ in range(N)]

        self.X = cp.Variable((nx, N + 1))
        self.U = cp.Variable((nu, N))
        self.x0_param = cp.Parameter(nx)
        self.v_ref_param = cp.Parameter(N)
        self.p_ref_param = cp.Parameter(N)
        self.psid_ref_param = cp.Parameter(N)
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

            cons += [self.X[:, k + 1] == self.A_p[k] @ self.X[:, k]
                     + self.B_p[k] @ self.U[:, k] + self.c_p[k]]
            cons += [cp.abs(self.U[0, k]) <= p.u_max]
            cons += [cp.abs(self.U[1, k]) <= p.u_max]
            if k >= 1:
                cons += [cp.abs(self.X[2, k]) <= p.theta_max]

        x_ref_N = cp.hstack([self.p_ref_param[N - 1], self.v_ref_param[N - 1],
                             0.0, 0.0, 0.0, self.psid_ref_param[N - 1]])
        cost += cp.quad_form(self.X[:, N], self.Qf) - 2 * (self.Qf @ x_ref_N) @ self.X[:, N]

        self.prob = cp.Problem(cp.Minimize(cost), cons)
        assert self.prob.is_dpp(), "問題不符合DPP規則"

    def _solve_qp(self, x0, A, B, c, v_ref, p_ref, psid_ref):
        """回傳 (X:(N+1,6), U:(N,2), status, qp_ms)；失敗時 X,U 為 None。"""
        cp = self._cp
        for k in range(self.N):
            self.A_p[k].value = A[k]
            self.B_p[k].value = B[k]
            self.c_p[k].value = c[k]
        self.x0_param.value = x0
        self.v_ref_param.value = v_ref
        self.p_ref_param.value = p_ref
        self.psid_ref_param.value = psid_ref
        self.u_prev_param.value = self.u_prev.copy()
        t0 = time.perf_counter()
        self.prob.solve(solver=cp.OSQP, warm_start=True, verbose=False,
                        eps_abs=self.osqp_eps, eps_rel=self.osqp_eps,
                        max_iter=self.osqp_max_iter)
        qp_ms = (time.perf_counter() - t0) * 1000.0
        if self.prob.status not in ("optimal", "optimal_inaccurate"):
            return None, None, self.prob.status, qp_ms
        return (np.array(self.X.value).T, np.array(self.U.value).T,
                self.prob.status, qp_ms)

    # ---------------------------------------------------------------- 非線性部分
    def _rollout(self, x0, U):
        N, h, m = self.N, self.Ts, self.model
        X = np.empty((N + 1, self.nx))
        x = [float(v) for v in x0]
        X[0] = x
        for k in range(N):
            x = m.rk4_s(x, (U[k, 0], U[k, 1]), h)
            X[k + 1] = x
        return X

    def _predict_delay(self, x0, delay_s):
        """用非線性模型把狀態往前推 delay_s (輸入固定為上一個已送出的 u)。"""
        if delay_s <= 1e-4:
            return np.asarray(x0, float)
        delay_s = float(min(delay_s, 0.06))
        n = max(1, int(math.ceil(delay_s / 0.005)))
        h = delay_s / n
        x = [float(v) for v in x0]
        u = (self.u_prev[0], self.u_prev[1])
        for _ in range(n):
            x = self.model.rk4_s(x, u, h)
        return np.array(x)

    def set_applied_u(self, u_l: float, u_r: float):
        self.u_prev = np.array([u_l, u_r], dtype=float)

    def solve(self, x0, l_current, v_ref_traj, p_ref_traj=None,
              psid_ref_traj=None, delay_s=0.0):
        t_start = time.perf_counter()
        if abs(l_current - self.model.l) > 1e-4:
            self.model.set_l(l_current)

        x0_pred = self._predict_delay(x0, delay_s)
        N = self.N
        v_ref = np.asarray(v_ref_traj, float)
        p_ref = np.zeros(N) if p_ref_traj is None else np.asarray(p_ref_traj, float)
        psid_ref = np.zeros(N) if psid_ref_traj is None else np.asarray(psid_ref_traj, float)

        U = self.U_guess.copy() if self.has_guess else np.tile(self.u_prev, (N, 1))
        lin_ms = qp_ms = 0.0
        X_sol = U_sol = None
        status, iters = "none", 0
        for _ in range(self.sqp_iters):
            t0 = time.perf_counter()
            Xbar = self._rollout(x0_pred, U)                       # (N+1,6)
            A, B = self.model.jac(Xbar[:-1], U, self.Ts)           # (N,6,6) (N,6,2)
            c = (Xbar[1:] - np.einsum('kij,kj->ki', A, Xbar[:-1])
                 - np.einsum('kij,kj->ki', B, U))
            lin_ms += (time.perf_counter() - t0) * 1000.0
            Xs, Us, status, q_ms = self._solve_qp(x0_pred, A, B, c, v_ref, p_ref, psid_ref)
            qp_ms += q_ms
            iters += 1
            if Xs is None:
                break
            X_sol, U_sol = Xs, Us
            U = Us

        total_ms = (time.perf_counter() - t_start) * 1000.0
        self.solve_times.append(total_ms)
        self.qp_solve_times.append(qp_ms)
        self.lin_times.append(lin_ms)
        timing = {"total_ms": total_ms, "qp_ms": qp_ms, "lin_ms": lin_ms,
                  "iters": iters, "rebuilt": False, "status": status,
                  "delay_ms": delay_s * 1000.0}

        if U_sol is None:
            print(f"[NonlinearBalanceMPC] QP 求解失敗: {status}")
            self.has_guess = False
            return self.u_prev.copy(), None, timing

        u0 = np.array(U_sol[0], dtype=float)
        self.u_prev = u0.copy()
        self.U_guess = np.vstack([U_sol[1:], U_sol[-1:]])           # 平移 warm start
        self.has_guess = True
        return u0, (X_sol.T, U_sol.T), timing


# ------------------------------------------------------------------ 閉迴路比較
def simulate(controller, plant: NLModel, steps=500, delay_s=0.025,
             theta0=0.03, yaw_step=True, l=0.181, warmup=5):
    """被控體永遠是「非線性模型」，比較不同控制器。"""
    Ts = controller.Ts
    delay_steps = max(1, int(round(delay_s / Ts)))
    buf = [np.zeros(2)] * delay_steps
    x = [0.0, 0.0, theta0, 0.0, 0.0, 0.0]
    th_log, u_log, times, lin_times, qp_times = [], [], [], [], []
    N = controller.N
    for step in range(steps):
        psid_cmd = 0.6 if (yaw_step and 150 <= step < 350) else 0.0
        u0, _, tm = controller.solve(np.array(x), l, np.zeros(N),
                                     psid_ref_traj=np.full(N, psid_cmd),
                                     delay_s=delay_steps * Ts)
        buf.append(u0)
        u = buf.pop(0)
        for _ in range(5):                                    # 被控體用 5 個 RK4 子步
            x = plant.rk4_s(x, (u[0], u[1]), Ts / 5)
        x[4] -= psid_cmd * Ts
        th_log.append(x[2])
        u_log.append(np.abs(u).max())
        if step >= warmup:
            times.append(tm["total_ms"])
            qp_times.append(tm["qp_ms"])
            lin_times.append(tm.get("lin_ms", 0.0))
        if abs(x[2]) > 0.6:
            print("   !! 倒下 (|theta|>0.6 rad) at step", step)
            break
    t = np.array(times)
    return {"max_theta_deg": math.degrees(max(abs(v) for v in th_log)),
            "rms_theta_deg_last": math.degrees(float(np.sqrt(np.mean(np.square(th_log[-100:]))))),
            "max_u": float(max(u_log)),
            "mean": t.mean(), "median": float(np.median(t)),
            "p95": float(np.percentile(t, 95)), "max": t.max(),
            "qp_mean": float(np.mean(qp_times)), "lin_mean": float(np.mean(lin_times)),
            "n": len(t)}


def _print_row(name, r, budget_ms):
    over = ""
    if r["p95"] > budget_ms:
        over = "  <-- p95 超過控制週期"
    print(f"{name:26s} 求解 mean={r['mean']:6.2f} med={r['median']:6.2f} p95={r['p95']:6.2f} "
          f"max={r['max']:6.2f} ms | QP={r['qp_mean']:5.2f} 線性化={r['lin_mean']:5.2f} ms | "
          f"max|th|={r['max_theta_deg']:5.1f}deg rms_end={r['rms_theta_deg_last']:.2f}deg "
          f"max|u|={r['max_u']:.1f}{over}")


def run_comparison(N=20, Ts=0.015, delay_s=0.025):
    params = RobotParams()
    plant = NLModel(params, 0.181)
    budget = Ts * 1000.0
    for theta0_deg in (1.7, 12.0):
        print(f"\n=== 初始傾角 {theta0_deg:.1f} deg, N={N}, Ts={Ts*1000:.0f} ms (控制週期預算 {budget:.0f} ms) ===")
        th0 = math.radians(theta0_deg)
        try:
            from balance_mpc_yaw import BalanceMPC
            lin = BalanceMPC(params, N=N, Ts=Ts)
            _print_row("線性 MPC (原版)", simulate(lin, plant, theta0=th0, delay_s=delay_s), budget)
        except Exception as e:                                # 沒有原版檔案就略過
            print("線性 MPC 略過:", e)
        for iters in (1, 2, 3):
            nl = NonlinearBalanceMPC(params, N=N, Ts=Ts, sqp_iters=iters)
            _print_row(f"非線性 MPC (SQP x{iters})",
                       simulate(nl, plant, theta0=th0, delay_s=delay_s), budget)


if __name__ == "__main__":
    run_comparison()
