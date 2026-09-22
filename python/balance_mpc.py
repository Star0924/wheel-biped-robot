"""
六馬達串聯式輪足機器人 —— 矢狀面平衡 / 前後移動 MPC

本版相對於原始檔的主要修改（都標了 [FIX]）：
  [FIX-1] RobotParams 新增 torque_gain：修正「模型以為送出多少扭矩 / 實際吃到多少」的落差。
  [FIX-2] solve() 新增 delay_s 參數：用模型把量測狀態往前推算一個死區時間，
          補償「量測 -> 序列埠 -> 求解 -> 回傳 -> 下一個控制tick才套用」的整體延遲。
  [FIX-3] 新增 set_applied_u()：讓 Δu 懲罰與 du_max 約束是對「實際送出的扭矩」做的，
          而不是對「MPC 想送的扭矩」做的。
  [FIX-4] 權重重新整理並集中到檔案上方，方便調參；預設值往「增加阻尼、減少速度環搶權」調。

狀態 x = [p, v, theta, omega]^T
    p     : 水平位置 (m)
    v     : 水平速度 (m/s)
    theta : 機身俯仰角，相對垂直方向 (rad)
    omega : 俯仰角速度 (rad/s)
控制輸入 u = 左右輪合計驅動力矩 (N*m)

依賴套件：
    pip install numpy scipy cvxpy osqp
"""

import time
import numpy as np
import cvxpy as cp
from dataclasses import dataclass
from scipy.linalg import expm


# ----------------------------------------------------------------------
# 0. 調參區（集中放這裡，方便一次看完）
# ----------------------------------------------------------------------
# 代價權重 [p, v, theta, omega]
#   p     : 不追蹤絕對位置，固定 0
#   v     : 原本 40。速度回授本身有濾波延遲，權重太大會跟平衡環搶權，
#           實測 log 的 0.9Hz 慢速搖擺就有這個成分，先降到 12。
#   theta : 原本 60 -> 90，讓姿態優先。
#   omega : 原本 5 -> 12，這是「阻尼」項，是抑制搖擺最直接的旋鈕。
#
# [2nd 調整] 第二次實機 log 的結論：低頻(真正的平衡問題)大幅改善，
#   pitch LF RMS 1.93deg -> 0.48deg、pitch_rate LF RMS 9.3 -> 2.4dps；
#   但高頻變差了：u0 HF RMS 0.56 -> 1.60Nm，出現 5.8Hz 的極限環
#   (pitch ±0.53deg / rate ±19.7dps / u0 ±2.0Nm)。
#   原因是我上一版把 omega 權重從 5 拉到 12、又把 R_DU 從 0.75 砍到 0.20，
#   這兩個動作都是在「墊高高頻迴路增益」。有 ~16ms 死區時間的系統
#   在 5.8Hz 已經沒有相位餘裕，控制器越用力只會把它激得更大。
#   所以這一版把高頻增益壓回去，低頻的部分維持不動。
Q_DIAG = [3.0, 12.0, 90.0, 6.0]   # [3rd] p 權重 0 -> 3 (需搭配 bridge 的 TRACK_POSITION=True)
QF_SCALE = 20.0
R_U = 0.08
# R_DU 是 MPC 裡最乾淨的「控制訊號低通」旋鈕：它懲罰 u[k]-u[k-1]，
# 等效權重正比於 |1-e^{-jwT}| = 2*sin(wT/2)。
#   在 0.5Hz: 係數 0.047  -> 幾乎不影響低頻控制力
#   在 5.8Hz: 係數 0.539  -> 強力壓制極限環
# 跟量測濾波不同，它完全不會增加任何相位延遲，所以優先用這個而不是濾陀螺儀。
R_DU = 1.20         # 0.20 -> 1.20 (原始版本是 0.75)


# ----------------------------------------------------------------------
# 1. 機器人物理參數
# ----------------------------------------------------------------------
@dataclass
class RobotParams:
    m_w: float = 3.165     # 單輪質量 (kg)
    I_w: float = 0.0256    # 單輪對輪軸轉動慣量 (kg*m^2)
    m_b: float = 7.6       # 機身等效質量 (kg)
    I_b: float = 0.308     # 機身對自身質心轉動慣量 (kg*m^2)
    r: float = 0.108       # 輪半徑 (m)
    g: float = 9.81        # 重力加速度 (m/s^2)

    # [FIX-1] 扭矩傳遞有效增益。
    # 用實機 log 做最小平方辨識：domega/dt 對 u 的靈敏度只有模型預測的 54~74%，
    # 代表模型高估了控制權限，MPC 會一直「以為自己出力夠了」而實際上不夠。
    # 先填 0.7；等你用測力計實測校正 MOTOR_TORQUE_CONSTANT 之後再調回 1.0。
    torque_gain: float = 0.7

    # input安全限制
    u_max: float = 8.0        # 左右輪合計最大扭矩 (N*m)

    # [2nd] 4.0 -> 2.5。log 顯示 |du| 的 p99 已經到 4.10、有 1.31% 的時間打到上限，
    # 代表變化率限制正在「參與」極限環而不是在保護系統。
    # 2.5Nm/step @ 67Hz = 167Nm/s，對搶救傾倒來說依然非常快。
    # [3rd] 2.5 -> 4.0 還原：v3 的 |du| p99 只有 0.515，完全沒碰到上限，
    # 平滑已經由 R_DU 負責，這條限制留大一點給搶救傾倒用。
    du_max: float = 4.0       # 每步扭矩變化率限制 (N*m / step)
    theta_max: float = 0.35   # 最大允許俯仰角 (rad) ~= 20 deg


# ----------------------------------------------------------------------
# 2. 連續時間線性化模型
# ----------------------------------------------------------------------
def continuous_model(params: RobotParams, l: float):
    """
    回傳線性化後的連續時間 A, B 矩陣（在 theta=0, theta_dot=0 平衡點附近）
    l: 當前虛擬腿長 (輪軸到機身質心距離, m)

    !! 提醒 !! l 必須跟「關節自鎖後的實際姿態」對應。
    command.cpp 的 lockJoints() 目前是把四顆關節都寫 0，
    但註解裡的設計姿態是 -45 / 70.2 / 45 / -70.2，兩者的質心高度不會一樣。
    l 填錯會直接讓 A/B 的增益錯掉，請先量測鎖腿後「輪軸到機身質心」的實際距離。
    """
    p = params
    a = p.m_w + p.m_b + p.I_w / p.r**2
    b = p.m_b * l
    c = p.I_b + p.m_b * l**2
    det = a * c - b**2

    A = np.array([
        [0.0, 1.0, 0.0,                          0.0],
        [0.0, 0.0, -b * p.m_b * p.g * l / det,    0.0],
        [0.0, 0.0, 0.0,                          1.0],
        [0.0, 0.0,  a * p.m_b * p.g * l / det,    0.0],
    ])

    B = np.array([
        [0.0],
        [(c / p.r + b) / det],
        [0.0],
        [-(b / p.r + a) / det],
    ]) * p.torque_gain          # [FIX-1]

    return A, B


def discretize(A: np.ndarray, B: np.ndarray, Ts: float):
    """零階保持 (ZOH) 離散化。"""
    n = A.shape[0]
    m = B.shape[1]
    M = np.zeros((n + m, n + m))
    M[:n, :n] = A
    M[:n, n:] = B
    Md = expm(M * Ts)
    return Md[:n, :n], Md[:n, n:]


# ----------------------------------------------------------------------
# 3. 線性 MPC（QP 形式）
# ----------------------------------------------------------------------
class BalanceMPC:
    def __init__(self, params: RobotParams, N: int = 20, Ts: float = 0.015,
                 l_nominal: float = 0.181,
                 osqp_eps: float = 1e-2, osqp_max_iter: int = 4000):
        self.p = params
        self.N = N
        self.Ts = Ts
        self.nx = 4
        self.nu = 1
        self.osqp_eps = osqp_eps
        self.osqp_max_iter = osqp_max_iter

        self.Q = np.diag(Q_DIAG)
        self.Qf = self.Q * QF_SCALE
        self.R = np.diag([R_U])
        self.R_delta = np.diag([R_DU])

        self.u_prev = 0.0

        self.solve_times = []
        self.qp_solve_times = []
        self.rebuild_count = 0

        self._l_current = None
        self._build_problem(l_nominal)

    # ------------------------------------------------------------------
    def _build_problem(self, l: float):
        p = self.p
        N, nx, nu = self.N, self.nx, self.nu

        A_c, B_c = continuous_model(p, l)
        A_d, B_d = discretize(A_c, B_c, self.Ts)
        # [FIX-2] 留著給死區時間預測用
        self.A_c, self.B_c = A_c, B_c
        self.A_d, self.B_d = A_d, B_d
        self._l_current = l

        self.X = cp.Variable((nx, N + 1))
        self.U = cp.Variable((nu, N))

        self.x0_param     = cp.Parameter(nx)
        self.v_ref_param  = cp.Parameter(N)
        self.p_ref_param  = cp.Parameter(N)
        self.u_prev_param = cp.Parameter(nu)

        cost = 0
        constraints = [self.X[:, 0] == self.x0_param]

        for k in range(N):
            x_ref_k = cp.hstack([self.p_ref_param[k], self.v_ref_param[k], 0.0, 0.0])

            cost += cp.quad_form(self.X[:, k], self.Q) - 2 * (self.Q @ x_ref_k) @ self.X[:, k]
            cost += cp.quad_form(self.U[:, k], self.R)

            if k == 0:
                cost += (cp.quad_form(self.U[:, k], self.R_delta)
                         - 2 * (self.R_delta @ self.u_prev_param) @ self.U[:, k])
            else:
                cost += cp.quad_form(self.U[:, k] - self.U[:, k - 1], self.R_delta)

            constraints += [self.X[:, k + 1] == A_d @ self.X[:, k] + B_d @ self.U[:, k]]
            constraints += [cp.abs(self.U[:, k]) <= p.u_max]

            if k == 0:
                constraints += [cp.abs(self.U[:, k] - self.u_prev_param) <= p.du_max]
            else:
                constraints += [cp.abs(self.U[:, k] - self.U[:, k - 1]) <= p.du_max]

            if k >= 1:
                constraints += [cp.abs(self.X[2, k]) <= p.theta_max]

        x_ref_N = cp.hstack([self.p_ref_param[N - 1], self.v_ref_param[N - 1], 0.0, 0.0])
        cost += cp.quad_form(self.X[:, N], self.Qf) - 2 * (self.Qf @ x_ref_N) @ self.X[:, N]

        self.prob = cp.Problem(cp.Minimize(cost), constraints)
        assert self.prob.is_dpp(), "問題不符合DPP規則，Parameter用法需要檢查"

    # ------------------------------------------------------------------
    # [FIX-2] 死區時間補償
    # ------------------------------------------------------------------
    def _predict_delay(self, x0: np.ndarray, delay_s: float) -> np.ndarray:
        """
        把「量測當下的狀態」沿著模型往前推 delay_s，得到「這次算出來的扭矩
        真正會被套用的那一瞬間」的狀態估計。

        為什麼一定要做：
          Teensy 在 t 時刻送出狀態 -> 序列埠 -> PC 求解(實測 p50 5.7ms / p95 10.3ms)
          -> 回傳 -> Teensy 下一個 15ms tick 才真的寫進馬達。
          整條鏈路的死區時間大約 20~30ms。倒單擺是不穩定系統，
          未補償的死區時間會直接吃掉相位餘裕，這是 log 裡低頻搖擺的主因之一。

        推算期間假設仍在送上一次的扭矩 u_prev（零階保持），這符合實際行為。
        """
        if delay_s <= 1e-4:
            return x0
        # 限制最大補償量，避免 PC 卡頓時外推過頭反而發散
        delay_s = float(min(delay_s, 0.06))
        Ad, Bd = discretize(self.A_c, self.B_c, delay_s)
        return Ad @ np.asarray(x0, dtype=float) + Bd.flatten() * self.u_prev

    def set_applied_u(self, u_applied: float):
        """
        [FIX-3] 告訴 MPC「上一拍實際送進馬達的總扭矩是多少」。
        Teensy 端會再加上摩擦前饋，所以實際值跟 MPC 算出來的 u0 不同；
        Δu 懲罰與 du_max 若用錯的基準，會讓控制器對自己的輸出有錯誤認知。
        """
        self.u_prev = float(u_applied)

    # ------------------------------------------------------------------
    def solve(self, x0: np.ndarray, l_current: float,
              v_ref_traj: np.ndarray, p_ref_traj: np.ndarray | None = None,
              delay_s: float = 0.0):
        """
        x0        : 當前狀態量測 [p, v, theta, omega]
        l_current : 當前虛擬腿長 (m)
        v_ref_traj: 長度 N 的速度參考序列 (m/s)
        delay_s   : [FIX-2] 從「量測」到「扭矩真正被套用」的估計死區時間 (s)

        回傳: u0, 完整預測序列(debug用), timing
        """
        t_start = time.perf_counter()

        rebuilt = False
        if self._l_current is None or abs(l_current - self._l_current) > 1e-3:
            self._build_problem(l_current)
            self.rebuild_count += 1
            rebuilt = True

        x0_pred = self._predict_delay(x0, delay_s)

        self.x0_param.value = np.asarray(x0_pred, dtype=float)
        self.v_ref_param.value = np.asarray(v_ref_traj, dtype=float)
        self.p_ref_param.value = (np.zeros(self.N) if p_ref_traj is None
                                  else np.asarray(p_ref_traj, dtype=float))
        self.u_prev_param.value = np.array([self.u_prev])

        t_qp_start = time.perf_counter()
        self.prob.solve(solver=cp.OSQP, warm_start=True, verbose=False,
                        eps_abs=self.osqp_eps, eps_rel=self.osqp_eps,
                        max_iter=self.osqp_max_iter)
        qp_ms = (time.perf_counter() - t_qp_start) * 1000.0

        total_ms = (time.perf_counter() - t_start) * 1000.0
        self.solve_times.append(total_ms)
        self.qp_solve_times.append(qp_ms)
        timing = {"total_ms": total_ms, "qp_ms": qp_ms, "rebuilt": rebuilt,
                  "status": self.prob.status, "delay_ms": delay_s * 1000.0}

        if self.prob.status not in ("optimal", "optimal_inaccurate"):
            print(f"[BalanceMPC] QP 求解失敗，狀態: {self.prob.status}  "
                  f"(耗時 total={total_ms:.2f}ms, qp={qp_ms:.2f}ms)")
            return self.u_prev, None, timing

        u0 = float(self.U.value[0, 0])
        self.u_prev = u0
        return u0, (self.X.value, self.U.value), timing

    def timing_summary(self) -> dict:
        if not self.solve_times:
            return {}
        arr = np.array(self.solve_times)
        qp_arr = np.array(self.qp_solve_times)
        return {
            "count": len(arr),
            "total_mean_ms": float(arr.mean()),
            "total_max_ms": float(arr.max()),
            "total_p95_ms": float(np.percentile(arr, 95)),
            "qp_mean_ms": float(qp_arr.mean()),
            "qp_max_ms": float(qp_arr.max()),
        }


# ----------------------------------------------------------------------
# 4. 模擬範例
# ----------------------------------------------------------------------
def run_simulation(sim_steps: int = 500, delay_s: float = 0.025):
    """
    模擬時刻意加入 delay_s 的實際死區時間，並讓 MPC 用同樣的值做補償，
    可以用來比較「有補償 / 無補償」的差別：把 mpc.solve(..., delay_s=0.0)
    改掉就能看到未補償時的搖擺變大。
    """
    params = RobotParams()
    mpc = BalanceMPC(params, N=20, Ts=0.015)

    x = np.array([0.0, 0.0, 0.03, 0.0])
    l_current = 0.181

    delay_steps = max(1, int(round(delay_s / mpc.Ts)))
    u_buffer = [0.0] * delay_steps          # 模擬致動延遲

    v_command = 0.0
    v_current_ref = 0.0

    history = {k: np.zeros(sim_steps) for k in
               ["t", "p", "v", "theta", "omega", "u", "v_command", "solve_ms", "qp_ms"]}

    for step in range(sim_steps):
        if step == 200:
            v_command = 0.1
        if step == 400:
            v_command = -0.1

        v_current_ref += 0.05 * (v_command - v_current_ref)
        v_ref_traj = np.full(mpc.N, v_current_ref)

        u0, debug, timing = mpc.solve(x, l_current, v_ref_traj,
                                      delay_s=delay_steps * mpc.Ts)

        u_buffer.append(u0)
        u_applied = u_buffer.pop(0)

        A_d, B_d = mpc.A_d, mpc.B_d
        x = A_d @ x + B_d.flatten() * u_applied

        history["t"][step] = step * mpc.Ts
        history["p"][step] = x[0]
        history["v"][step] = x[1]
        history["theta"][step] = x[2]
        history["omega"][step] = x[3]
        history["u"][step] = u0
        history["v_command"][step] = v_command
        history["solve_ms"][step] = timing["total_ms"]
        history["qp_ms"][step] = timing["qp_ms"]

        if step % 50 == 0:
            print(f"step={step:4d}  p={x[0]:+.3f}  v={x[1]:+.3f}  "
                  f"theta={np.degrees(x[2]):+.2f}deg  u={u0:+.3f}  "
                  f"solve={timing['total_ms']:.2f}ms")

    summary = mpc.timing_summary()
    print(f"\n[timing_summary] N={mpc.N}, Ts={mpc.Ts * 1000:.1f}ms")
    print(f"  total_ms  mean={summary['total_mean_ms']:.2f}  "
          f"p95={summary['total_p95_ms']:.2f}  max={summary['total_max_ms']:.2f}")
    print(f"  theta RMS = {np.degrees(np.sqrt(np.mean(history['theta']**2))):.3f} deg")

    return history


if __name__ == "__main__":
    run_simulation(sim_steps=500)