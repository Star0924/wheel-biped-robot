"""
plot_mpc_yaw_log.py
---------------------------------------------------------------
讀取 mpc_serial_bridge_yaw.py 產生的 CSV，畫成多子圖 PNG。
也能讀舊版 (4 階) 的 log：缺的欄位會自動略過那幾張圖。

用法：
    python plot_mpc_yaw_log.py mpc_bridge_log_20260101_120000.csv
    (不帶參數則自動抓當前資料夾裡最新的一個 mpc_bridge_log_*.csv)

新版 log 欄位：
    seq,t_wall_s,t_teensy_ms,pitch_deg,pitch_rate_dps,vL_dps,vR_dps,v_mps,pos_m,
    yaw_deg,yaw_rate_dps,psi_err_rad,u_s,u_d,u_s_applied,u_d_applied,
    delay_ms,solve_total_ms,solve_qp_ms,dropped,rebuilt,status
---------------------------------------------------------------
"""

import sys
import glob
import os
import csv
import numpy as np

# 只用來做「輪速差 vs 陀螺儀 yaw rate」的交叉檢查，需與 bridge / balance_mpc_yaw 一致
WHEEL_RADIUS_M = 0.108
TRACK_WIDTH_M = 0.40      # 佔位值；下面的摘要會給你一個由 log 推出的估計
U_MAX = 8.0               # |u_s|+|u_d| 上限，需與 RobotParams.u_max 一致


def find_latest_log():
    candidates = sorted(glob.glob("mpc_bridge_log_*.csv"))
    if not candidates:
        raise FileNotFoundError(
            "目前資料夾找不到 mpc_bridge_log_*.csv，"
            "請確認 mpc_serial_bridge_yaw.py 有正常執行過，"
            "或直接把檔名當第一個參數傳進來。"
        )
    return candidates[-1]


def load_csv(path):
    with open(path, "r", newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"{path} 是空的，沒有資料可以畫")
    cols = rows[0].keys()

    def col(name, cast=float, default=None):
        """欄位不存在 -> 回傳 None；存在但某列壞掉 -> nan"""
        if name not in cols:
            return default
        out = []
        for r in rows:
            try:
                out.append(cast(r[name]))
            except (ValueError, TypeError):
                out.append(np.nan if cast is float else 0)
        return np.array(out)

    d = {
        "seq": col("seq", int),
        "t": col("t_wall_s"),
        "pitch_deg": col("pitch_deg"),
        "pitch_rate_dps": col("pitch_rate_dps"),
        "v_mps": col("v_mps"),
        "pos_m": col("pos_m"),
        "solve_total_ms": col("solve_total_ms"),
        "solve_qp_ms": col("solve_qp_ms"),
        "delay_ms": col("delay_ms"),
        "dropped": col("dropped", int),
        "rebuilt": col("rebuilt", int),
        "status": np.array([r.get("status", "n/a") or "n/a" for r in rows]),
        # ---- 新版欄位（舊版 log 會是 None）----
        "vL_dps": col("vL_dps"),
        "vR_dps": col("vR_dps"),
        "yaw_deg": col("yaw_deg"),
        "yaw_rate_dps": col("yaw_rate_dps"),
        "psi_err_rad": col("psi_err_rad"),
        "u_s": col("u_s"),
        "u_d": col("u_d"),
        "u_s_app": col("u_s_applied"),
        "u_d_app": col("u_d_applied"),
    }
    # ---- 舊版 log 相容：單一輪速、單一 u ----
    if d["vL_dps"] is None:
        ws = col("wheel_speed_dps")
        d["wheel_speed_dps"] = ws
    else:
        d["wheel_speed_dps"] = 0.5 * (d["vL_dps"] + d["vR_dps"])
    if d["u_s"] is None:
        d["u_s"] = col("u0_Nm_combined")
        d["u_s_app"] = col("u_applied_Nm")
    d["has_yaw"] = d["yaw_rate_dps"] is not None
    return d


def _panel(ax, t, series, ylabel, zero_line=True):
    """series: list of (y, label, color, alpha)；y 為 None 就跳過"""
    for y, label, color, alpha in series:
        if y is not None:
            ax.plot(t, y, color=color, alpha=alpha, label=label, linewidth=1.0)
    if zero_line:
        ax.axhline(0, color="gray", linewidth=0.8, linestyle="--")
    ax.set_ylabel(ylabel)
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(True, alpha=0.3)


def plot(d, out_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = d["t"]
    n_panels = 9 if d["has_yaw"] else 5
    fig, axes = plt.subplots(n_panels, 1, figsize=(11, 2.3 * n_panels + 1), sharex=True)
    i = 0

    # 1. pitch / pitch rate
    _panel(axes[i], t, [(d["pitch_deg"], "pitch (deg)", "tab:red", 1.0)], "pitch (deg)"); i += 1
    _panel(axes[i], t, [(d["pitch_rate_dps"], "pitch rate (deg/s)", "tab:orange", 1.0)],
           "pitch rate (deg/s)"); i += 1

    # 2. 輪速（新版左右分開）+ 位置
    if d["vL_dps"] is not None:
        _panel(axes[i], t, [(d["vL_dps"], "vL (deg/s)", "tab:blue", 0.9),
                            (d["vR_dps"], "vR (deg/s)", "tab:green", 0.9)], "wheel speed (deg/s)")
    else:
        _panel(axes[i], t, [(d["wheel_speed_dps"], "wheel speed (deg/s)", "tab:blue", 1.0)],
               "wheel speed (deg/s)")
    axb = axes[i].twinx()
    axb.plot(t, d["pos_m"], color="tab:gray", alpha=0.6, label="pos (m)")
    axb.set_ylabel("pos (m)")
    axb.legend(loc="upper right", fontsize=8)
    i += 1

    if d["has_yaw"]:
        # 3. yaw 角度 / 誤差
        ax = axes[i]
        ax.plot(t, d["yaw_deg"], color="tab:brown", label="yaw (deg)")
        ax.plot(t, np.degrees(d["psi_err_rad"]), color="tab:red", alpha=0.7, label="yaw error (deg)")
        ax.axhline(0, color="gray", linewidth=0.8, linestyle="--")
        ax.set_ylabel("yaw (deg)"); ax.legend(loc="upper left", fontsize=8); ax.grid(True, alpha=0.3)
        i += 1

        # 4. yaw rate：陀螺儀 vs 輪速差推得
        kin = None
        if d["vL_dps"] is not None:
            kin = np.degrees(WHEEL_RADIUS_M * np.radians(d["vR_dps"] - d["vL_dps"]) / TRACK_WIDTH_M)
        _panel(axes[i], t, [(d["yaw_rate_dps"], "yaw rate gyro (deg/s)", "tab:purple", 1.0),
                            (kin, f"yaw rate from wheels (d={TRACK_WIDTH_M} m)", "tab:cyan", 0.6)],
               "yaw rate (deg/s)"); i += 1

    # 5. 扭矩
    _panel(axes[i], t, [(d["u_s"], "u_s = uL+uR (cmd)", "tab:green", 1.0),
                        (d["u_s_app"], "u_s applied (incl. friction FF)", "tab:olive", 0.6)],
           "u_s (N*m)"); i += 1

    if d["has_yaw"]:
        _panel(axes[i], t, [(d["u_d"], "u_d = uR-uL (cmd)", "tab:red", 1.0),
                            (d["u_d_app"], "u_d applied", "tab:orange", 0.6)], "u_d (N*m)")
        i += 1

        # 6. 各輪扭矩與飽和裕度：|u_s|+|u_d| 逼近 U_MAX 代表某一輪頂到上限
        ax = axes[i]
        uL = (d["u_s"] - d["u_d"]) / 2.0
        uR = (d["u_s"] + d["u_d"]) / 2.0
        ax.plot(t, uL, color="tab:blue", label="uL (N*m)", linewidth=1.0)
        ax.plot(t, uR, color="tab:green", label="uR (N*m)", linewidth=1.0)
        ax.axhline(U_MAX / 2, color="red", linestyle=":", linewidth=0.8)
        ax.axhline(-U_MAX / 2, color="red", linestyle=":", linewidth=0.8, label="per-wheel limit")
        ax.set_ylabel("per-wheel u (N*m)"); ax.legend(loc="upper left", fontsize=8); ax.grid(True, alpha=0.3)
        i += 1

    # 7. 求解時間
    ax = axes[i]
    ax.plot(t, d["solve_total_ms"], color="tab:purple", label="solve total (ms)")
    ax.plot(t, d["solve_qp_ms"], color="tab:pink", alpha=0.7, label="qp only (ms)")
    if d["delay_ms"] is not None:
        ax.plot(t, d["delay_ms"], color="tab:gray", alpha=0.6, label="delay comp (ms)")
    ax.axhline(15, color="red", linestyle=":", linewidth=0.8, label="control period 15 ms")
    idx = np.where(d["rebuilt"] == 1)[0]
    if len(idx) > 0:
        ax.scatter(t[idx], d["solve_total_ms"][idx], color="red", marker="x", s=60,
                   label="rebuild triggered", zorder=5)
    ax.set_ylabel("time (ms)"); ax.set_xlabel("time (s)")
    ax.legend(loc="upper left", fontsize=8); ax.grid(True, alpha=0.3)

    fig.suptitle("MPC serial bridge log" + (" (with yaw)" if d["has_yaw"] else ""))
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"[plot_mpc_yaw_log] 已輸出 {out_path}")


def _rng(name, x, unit="", fmt="+.3f"):
    print(f"{name:<22} min={x.min():{fmt}}  max={x.max():{fmt}}  mean={x.mean():{fmt}} {unit}")


def print_summary(d):
    print("\n===== 摘要統計 =====")
    print(f"樣本數: {len(d['seq'])}    時間長度: {d['t'][-1]:.2f} s")
    _rng("pitch (deg)", d["pitch_deg"])
    _rng("wheel speed (dps)", d["wheel_speed_dps"], fmt="+.1f")
    _rng("u_s (N*m)", d["u_s"])

    if d["has_yaw"]:
        _rng("yaw error (deg)", np.degrees(d["psi_err_rad"]))
        _rng("yaw rate (dps)", d["yaw_rate_dps"], fmt="+.1f")
        _rng("u_d (N*m)", d["u_d"])
        sat = (np.abs(d["u_s"]) + np.abs(d["u_d"])) >= 0.98 * U_MAX
        print(f"扭矩飽和比例 (|u_s|+|u_d| >= 98% u_max): {100 * sat.mean():.1f} %")

        # 由 log 估計有效輪距：yaw_rate_rad = (r/d) * (phi_dot_R - phi_dot_L)
        if d["vL_dps"] is not None:
            x = WHEEL_RADIUS_M * np.radians(d["vR_dps"] - d["vL_dps"])
            y = np.radians(d["yaw_rate_dps"])
            ok = np.isfinite(x) & np.isfinite(y)
            if ok.sum() > 20 and np.dot(x[ok], x[ok]) > 1e-9:
                slope = np.dot(x[ok], y[ok]) / np.dot(x[ok], x[ok])   # = 1/d
                if abs(slope) > 1e-6:
                    print(f"由輪速差 vs 陀螺儀估計的有效輪距: {1.0 / abs(slope):.3f} m"
                          f"  (符號{'一致' if slope > 0 else '相反 -> 檢查 YAW_SIGN/GYRO_YAW_SIGN'})")
                    print("    ※ 打滑會讓估計偏大；yaw 幾乎沒被激發時此估計不可靠")

    print(f"solve total (ms)   mean={d['solve_total_ms'].mean():.2f}  "
          f"p95={np.percentile(d['solve_total_ms'], 95):.2f}  max={d['solve_total_ms'].max():.2f}")
    if d["delay_ms"] is not None:
        print(f"delay 補償 (ms)    mean={d['delay_ms'].mean():.1f}  max={d['delay_ms'].max():.1f}")
    print(f"總共丟棄過期資料: {d['dropped'].sum()} 筆")
    print(f"總共觸發重建問題: {d['rebuilt'].sum()} 次")
    statuses, counts = np.unique(d["status"], return_counts=True)
    if not (len(statuses) == 1 and statuses[0] == "n/a"):
        print("求解狀態分佈:")
        for s, c in zip(statuses, counts):
            print(f"    {s}: {c}")


if __name__ == "__main__":
    csv_path = sys.argv[1] if len(sys.argv) > 1 else find_latest_log()
    data = load_csv(csv_path)
    print_summary(data)
    plot(data, os.path.splitext(csv_path)[0] + ".png")