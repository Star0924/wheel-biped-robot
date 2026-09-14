"""
六馬達串聯式輪足機器人 —— 矢狀面平衡 / 前後移動 MPC 骨架

模型：輪型倒單擺 (Wheeled Inverted Pendulum, WIP)
狀態 x = [p, v, theta, omega]^T
    p     : 水平位置 (m)
    v     : 水平速度 (m/s)
    theta : 機身俯仰角，相對垂直方向 (rad)
    omega : 俯仰角速度 (rad/s)
控制輸入 u = 輪馬達驅動力矩 (N*m)，本骨架假設左右輪同步（矢狀面簡化）

依賴套件：
    pip install numpy scipy cvxpy osqp
"""

import time
import numpy as np
import cvxpy as cp
from dataclasses import dataclass


# ----------------------------------------------------------------------
# 1. 機器人物理參數
# ----------------------------------------------------------------------
@dataclass
class RobotParams:
    m_w: float = 3.165     # 單輪質量 (kg)
    I_w: float = 0.0256    # 單輪對輪軸轉動慣量 (kg*m^2)
    m_b: float = 7.6      # 機身等效質量 (kg)
    I_b: float = 0.308     # 機身對自身質心轉動慣量 (kg*m^2)
    r: float = 0.108       # 輪半徑 (m)
    g: float = 9.81       # 重力加速度 (m/s^2)

    # 致動器與安全限制
    u_max: float = 8.0        # 單輪最大扭矩 (N*m)
    du_max: float = 4.0       # 每步扭矩變化率限制 (N*m / step)
    theta_max: float = 0.35   # 最大允許俯仰角 (rad) ~= 20 deg


# ----------------------------------------------------------------------
# 2. 連續時間線性化模型（腿長 l 為可變參數，每步依當前量測重新計算）
# ----------------------------------------------------------------------
def continuous_model(params: RobotParams, l: float):
    """
    回傳線性化後的連續時間 A, B 矩陣（在 theta=0, theta_dot=0 平衡點附近）
    l: 當前虛擬腿長 (輪軸到機身質心距離, m)，由下層 VMC / 腿長量測提供
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
    ])

    return A, B


def discretize(A: np.ndarray, B: np.ndarray, Ts: float):
    """
    零階保持 (ZOH) 離散化。狀態維度小，直接用矩陣指數法（精確）。
    """
    n = A.shape[0]
    m = B.shape[1]
    M = np.zeros((n + m, n + m))
    M[:n, :n] = A
    M[:n, n:] = B
    from scipy.linalg import expm
    Md = expm(M * Ts)
    A_d = Md[:n, :n]
    B_d = Md[:n, n:]
    return A_d, B_d


# ----------------------------------------------------------------------
# 3. 線性 MPC（QP 形式，每步重新線性化 = LTV-MPC）
# ----------------------------------------------------------------------
class BalanceMPC:
    def __init__(self, params: RobotParams, N: int = 60, Ts: float = 0.01,
                 l_nominal: float = 0.20,
                 osqp_eps: float = 1e-3, osqp_max_iter: int = 4000):
        """
        l_nominal: 初始/預設腿長 (m)。因為關節通常是鎖死的，A_d/B_d 在腿長不變
                   的情況下是常數，所以整個 QP 問題只在 __init__ 跟腿長真的改變
                   時才需要重新建立(慢)；平常 solve() 只更新數值再重解(快)。
        osqp_eps      : OSQP 收斂容忍度 (eps_abs/eps_rel)。數字越大代表容許誤差越大，
                         換取更快收斂/更少迭代次數。對平衡控制這種每10ms就會重解一次
                         的應用，不需要追求高精度，1e-3 通常夠用，還是不夠快可以放到1e-2。
        osqp_max_iter : 單次求解最多迭代次數，避免極端情況下OSQP卡很久，
                         逾時就回傳「盡力而為」的次佳解 (optimal_inaccurate)。
        """
        self.p = params
        self.N = N
        self.Ts = Ts
        self.nx = 4
        self.nu = 1
        self.osqp_eps = osqp_eps
        self.osqp_max_iter = osqp_max_iter

        # 權重矩陣（可依實機調整）
        # [p, v, theta, omega]
        # 注意：這個加速版本假設你跟原本一樣「不追蹤絕對位置」(p權重固定0)。
        # 如果之後真的需要動態切換有無追蹤位置，Q[0,0] 也要改成 cp.Parameter。
        self.Q = np.diag([0.0, 35.0, 80.0, 5.0])
        self.Qf = self.Q * 20.0

        self.R = np.diag([0.1])
        self.R_delta = np.diag([0.75])

        self.u_prev = 0.0

        self.solve_times = []
        self.qp_solve_times = []
        self.rebuild_count = 0   # 累積被觸發重新建模的次數，理論上鎖腿情況下應該只有1(初始化那次)

        self._l_current = None
        self._build_problem(l_nominal)

    def _build_problem(self, l: float):
        """建立(或重建)整個cvxpy問題。只有第一次跟腿長真的改變時才會呼叫，較慢。"""
        p = self.p
        N, nx, nu = self.N, self.nx, self.nu

        A_c, B_c = continuous_model(p, l)
        A_d, B_d = discretize(A_c, B_c, self.Ts)
        self._l_current = l

        self.X = cp.Variable((nx, N + 1))
        self.U = cp.Variable((nu, N))

        # ---- 會在每次 solve() 變動的量，改用 Parameter，避免重新canonicalize ----
        self.x0_param     = cp.Parameter(nx)
        self.v_ref_param  = cp.Parameter(N)
        self.p_ref_param  = cp.Parameter(N)
        self.u_prev_param = cp.Parameter(nu)

        cost = 0
        constraints = [self.X[:, 0] == self.x0_param]

        for k in range(N):
            x_ref_k = cp.hstack([self.p_ref_param[k], self.v_ref_param[k], 0.0, 0.0])

            # 注意：quad_form(X-x_ref, Q) 展開會出現 x_ref^T@Q@x_ref 這種「純參數二次項」，
            # 這個項不符合DPP規則(cvxpy會偷偷退回每次都整個重建，完全沒加速到)。
            # 這一項是常數(不含Variable)，不影響argmin U，所以直接拿掉即可，
            # 只保留跟Variable有關的部分：quad_form(X,Q) - 2*(Q@x_ref)@X
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

            # 注意：X[:,0] 是「已經量測到的當下狀態」(見上面 constraints[0] 的等式約束)，
            # 不是決策變數，對它設 theta_max 沒有意義──真實傾角一旦已經超過這個值
            # (例如快摔倒的瞬間)，這條約束會讓整個QP在k=0直接無解(infeasible)，
            # solve()判定失敗後只能回傳上一次的舊扭矩，等於在最需要出力搶救的
            # 瞬間反而放棄動作。這裡改成只約束 k>=1 (MPC真正能影響、規劃出來的
            # 未來狀態)，讓MPC在任何當下角度都至少會盡力算出一個非零的修正扭矩。
            if k >= 1:
                constraints += [cp.abs(self.X[2, k]) <= p.theta_max]

        x_ref_N = cp.hstack([self.p_ref_param[N - 1], self.v_ref_param[N - 1], 0.0, 0.0])
        cost += cp.quad_form(self.X[:, N], self.Qf) - 2 * (self.Qf @ x_ref_N) @ self.X[:, N]

        self.prob = cp.Problem(cp.Minimize(cost), constraints)
        # 確認真的符合DPP規則，否則cvxpy會每次還是整個重建(等於沒加速到)
        assert self.prob.is_dpp(), "問題不符合DPP規則，Parameter用法需要檢查"

    def solve(self, x0: np.ndarray, l_current: float,
              v_ref_traj: np.ndarray, p_ref_traj: np.ndarray | None = None):
        """
        x0          : 當前狀態量測 [p, v, theta, omega]
        l_current   : 當前腿長量測 (m)。只有跟上次不同(超過1mm)才會觸發重新建模(慢)，
                      腿長沒變的話這裡幾乎零成本。
        v_ref_traj  : 長度 N 的速度參考序列 (m/s)
        p_ref_traj  : 長度 N 的位置參考序列 (m)，可選；不給則視為0(等同不追蹤位置)

        回傳: u0, 完整預測序列(debug用), timing = {"total_ms","qp_ms"}
        """
        t_start = time.perf_counter()

        # 腿長真的變了才重建問題；沒變就跳過（一般鎖腿情況下這裡永遠不會進去）
        rebuilt = False
        if self._l_current is None or abs(l_current - self._l_current) > 1e-3:
            self._build_problem(l_current)
            self.rebuild_count += 1
            rebuilt = True

        self.x0_param.value = np.asarray(x0, dtype=float)
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
                   "status": self.prob.status}

        if self.prob.status not in ("optimal", "optimal_inaccurate"):
            print(f"[BalanceMPC] QP 求解失敗，狀態: {self.prob.status}  "
                  f"(耗時 total={total_ms:.2f}ms, qp={qp_ms:.2f}ms)")
            return self.u_prev, None, timing

        u0 = float(self.U.value[0, 0])
        self.u_prev = u0
        return u0, (self.X.value, self.U.value), timing

    def timing_summary(self) -> dict:
        """回傳目前累積的求解耗時統計 (ms)，方便判斷是否能即時運行(耗時應遠小於 Ts)。"""
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
# 4. 使用範例（模擬迴路骨架，實機時把 plant.step 換成真實狀態回授）
#    本區塊額外把每步的狀態/輸入記錄下來，方便事後把數據圖像化：
#      - 存成 CSV，可用 Excel / pandas 自行繪圖
#      - 用 matplotlib 直接輸出 PNG 圖表
# ----------------------------------------------------------------------
def run_simulation(sim_steps: int = 500):
    """
    執行模擬並回傳歷史資料字典，每個 key 對應一條長度為 sim_steps 的陣列：
        t, p, v, theta, omega, u, v_command
    """
    params = RobotParams()
    mpc = BalanceMPC(params, N=20, Ts=0.2)

    x = np.array([0.0, 0.0, 0.01, 0.0])   # 初始狀態：小幅初始傾角擾動
    l_current = 0.181                       # 假設腿長固定 20cm

    v_command = 0.0
    v_current_ref = 0.0

    # 預先配置歷史紀錄陣列
    history = {
        "t": np.zeros(sim_steps),
        "p": np.zeros(sim_steps),
        "v": np.zeros(sim_steps),
        "theta": np.zeros(sim_steps),
        "omega": np.zeros(sim_steps),
        "u": np.zeros(sim_steps),
        "v_command": np.zeros(sim_steps),
        "solve_ms": np.zeros(sim_steps),     # 每步 MPC 總求解耗時 (ms)
        "qp_ms": np.zeros(sim_steps),        # 每步純 QP 求解耗時 (ms)
    }

    for step in range(sim_steps):
        if step == 200:
            v_command = 0.1    # 前進指令
        if step == 400:
            v_command = -0.1   # 後退指令

        v_current_ref += 0.05 * (v_command - v_current_ref)

        v_ref_traj = np.full(mpc.N, v_current_ref)
        u0, debug, timing = mpc.solve(x, l_current, v_ref_traj)

        # ---- 這裡串接下層 VMC：把 u0 (輪扭矩) 與腿部虛擬力一併送給關節馬達 ----
        # tau_hip, tau_knee = vmc.compute(F_leg_ref, l_ref, ...)
        # tau_wheel_left  = u0 / 2.0
        # tau_wheel_right = u0 / 2.0

        # ---- 模擬 plant（實機時這段換成 IMU / 編碼器狀態回授）----
        A_c, B_c = continuous_model(params, l_current)
        A_d, B_d = discretize(A_c, B_c, mpc.Ts)
        x = A_d @ x + B_d.flatten() * u0

        # ---- 記錄本步資料 ----
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
                  f"theta={x[2]:+.3f}  u={u0:+.3f}  "
                  f"solve={timing['total_ms']:.2f}ms  qp={timing['qp_ms']:.2f}ms")

    # --- 模擬結束後印出耗時統計，方便判斷即時性 (應遠小於 Ts) ---
    summary = mpc.timing_summary()
    print(f"\n[timing_summary] N={mpc.N}, Ts={mpc.Ts * 1000:.1f}ms")
    print(f"  total_ms  mean={summary['total_mean_ms']:.2f}  "
          f"p95={summary['total_p95_ms']:.2f}  max={summary['total_max_ms']:.2f}")
    print(f"  qp_ms     mean={summary['qp_mean_ms']:.2f}  "
          f"max={summary['qp_max_ms']:.2f}")

    return history


def save_history_csv(history: dict, path: str = "mpc_sim_log.csv"):
    """把歷史資料存成 CSV，方便用 Excel / pandas / 其他工具畫圖。"""
    import csv
    keys = list(history.keys())
    n = len(history[keys[0]])
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(keys)
        for i in range(n):
            writer.writerow([history[k][i] for k in keys])
    print(f"[save_history_csv] 已輸出 {path}")


def plot_history(history: dict, path: str = "mpc_sim_plot.png"):
    """
    用 matplotlib 把模擬結果畫成 4 個子圖並存成 PNG：
        1. 水平位置 p 與速度 v（含速度指令對照）
        2. 機身俯仰角 theta
        3. 俯仰角速度 omega
        4. 輪馬達扭矩 u
    """
    import matplotlib
    matplotlib.use("Agg")   # 無顯示環境也能存檔
    import matplotlib.pyplot as plt

    # 注意：matplotlib 預設字型 (DejaVu Sans) 不含中文字形，
    # 這裡圖表文字一律用英文，避免存出的 PNG 出現缺字方框。
    t = history["t"]
    fig, axes = plt.subplots(5, 1, figsize=(9, 10), sharex=True)

    axes[0].plot(t, history["p"], label="p (position, m)")
    axes[0].plot(t, history["v"], label="v (velocity, m/s)")
    axes[0].plot(t, history["v_command"], "--", label="v_ref (command)")
    axes[0].set_ylabel("position / velocity")
    axes[0].legend(loc="upper left")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, np.degrees(history["theta"]), color="tab:red")
    axes[1].set_ylabel("theta (deg)")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t, np.degrees(history["omega"]), color="tab:orange")
    axes[2].set_ylabel("omega (deg/s)")
    axes[2].grid(True, alpha=0.3)

    axes[3].plot(t, history["u"], color="tab:green")
    axes[3].set_ylabel("u (N*m)")
    axes[3].set_xlabel("time (s)")
    axes[3].grid(True, alpha=0.3)

    axes[4].plot(t, history["solve_ms"], label="total solve time (ms)")
    axes[4].set_ylabel("solve time (ms)")
    axes[4].set_xlabel("time (s)")
    axes[4].grid(True, alpha=0.3)

    fig.suptitle("Balance MPC simulation result")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"[plot_history] 已輸出 {path}")


if __name__ == "__main__":
    history = run_simulation(sim_steps=500)
    save_history_csv(history, "mpc_sim_log.csv")
    plot_history(history, "mpc_sim_plot.png")