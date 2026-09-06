"""
plot_mpc_log.py
---------------------------------------------------------------
讀取 mpc_serial_bridge_logging.py 產生的 CSV，畫成一張多子圖 PNG。

用法：
    python plot_mpc_log.py mpc_bridge_log_20260101_120000.csv
    (不帶參數則自動抓當前資料夾裡最新的一個 mpc_bridge_log_*.csv)
---------------------------------------------------------------
"""

import sys
import glob
import os
import csv
import numpy as np


def find_latest_log():
    candidates = sorted(glob.glob("mpc_bridge_log_*.csv"))
    if not candidates:
        raise FileNotFoundError(
            "目前資料夾找不到 mpc_bridge_log_*.csv，"
            "請確認 mpc_serial_bridge_logging.py 有正常執行過，"
            "或直接把檔名當第一個參數傳進來。"
        )
    return candidates[-1]


def load_csv(path):
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        raise ValueError(f"{path} 是空的，沒有資料可以畫")

    data = {
        "seq": np.array([int(r["seq"]) for r in rows]),
        "t_wall_s": np.array([float(r["t_wall_s"]) for r in rows]),
        "pitch_deg": np.array([float(r["pitch_deg"]) for r in rows]),
        "pitch_rate_dps": np.array([float(r["pitch_rate_dps"]) for r in rows]),
        "wheel_speed_dps": np.array([float(r["wheel_speed_dps"]) for r in rows]),
        "v_mps": np.array([float(r["v_mps"]) for r in rows]),
        "u0_Nm": np.array([float(r["u0_Nm"]) for r in rows]),
        "solve_total_ms": np.array([float(r["solve_total_ms"]) for r in rows]),
        "solve_qp_ms": np.array([float(r["solve_qp_ms"]) for r in rows]),
        "dropped": np.array([int(r["dropped"]) for r in rows]),
        "rebuilt": np.array([int(r["rebuilt"]) for r in rows]),
        # 舊版log沒有這欄，讀不到就統一填"n/a"，畫圖/統計都能正常跳過
        "status": np.array([r.get("status", "n/a") or "n/a" for r in rows]),
    }
    return data


def plot(data, out_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = data["t_wall_s"]
    fig, axes = plt.subplots(5, 1, figsize=(10, 13), sharex=True)

    axes[0].plot(t, data["pitch_deg"], color="tab:red", label="pitch (deg)")
    axes[0].axhline(0, color="gray", linewidth=0.8, linestyle="--")
    axes[0].set_ylabel("pitch (deg)")
    axes[0].legend(loc="upper left")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, data["pitch_rate_dps"], color="tab:orange", label="pitch rate (deg/s)")
    axes[1].axhline(0, color="gray", linewidth=0.8, linestyle="--")
    axes[1].set_ylabel("pitch rate (deg/s)")
    axes[1].legend(loc="upper left")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t, data["wheel_speed_dps"], color="tab:blue", label="wheel speed (deg/s)")
    ax2b = axes[2].twinx()
    ax2b.plot(t, data["v_mps"], color="tab:cyan", alpha=0.6, label="v (m/s)")
    axes[2].set_ylabel("wheel speed (deg/s)")
    ax2b.set_ylabel("v (m/s)")
    lines1, labels1 = axes[2].get_legend_handles_labels()
    lines2, labels2 = ax2b.get_legend_handles_labels()
    axes[2].legend(lines1 + lines2, labels1 + labels2, loc="upper left")
    axes[2].grid(True, alpha=0.3)

    axes[3].plot(t, data["u0_Nm"], color="tab:green", label="u (N*m, MPC torque cmd)")
    axes[3].axhline(0, color="gray", linewidth=0.8, linestyle="--")
    axes[3].set_ylabel("u (N*m)")
    axes[3].legend(loc="upper left")
    axes[3].grid(True, alpha=0.3)

    axes[4].plot(t, data["solve_total_ms"], color="tab:purple", label="solve total (ms)")
    axes[4].plot(t, data["solve_qp_ms"], color="tab:pink", alpha=0.7, label="qp only (ms)")
    rebuilt_idx = np.where(data["rebuilt"] == 1)[0]
    if len(rebuilt_idx) > 0:
        axes[4].scatter(t[rebuilt_idx], data["solve_total_ms"][rebuilt_idx],
                         color="red", marker="x", s=60, label="rebuild triggered", zorder=5)
    axes[4].set_ylabel("solve time (ms)")
    axes[4].set_xlabel("time (s)")
    axes[4].legend(loc="upper left")
    axes[4].grid(True, alpha=0.3)

    fig.suptitle("MPC serial bridge log")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"[plot_mpc_log] 已輸出 {out_path}")


def print_summary(data):
    print("\n===== 摘要統計 =====")
    print(f"樣本數: {len(data['seq'])}")
    print(f"時間長度: {data['t_wall_s'][-1]:.2f} s")
    print(f"pitch (deg)        min={data['pitch_deg'].min():+.3f}  "
          f"max={data['pitch_deg'].max():+.3f}  mean={data['pitch_deg'].mean():+.3f}")
    print(f"wheel speed (dps)  min={data['wheel_speed_dps'].min():+.1f}  "
          f"max={data['wheel_speed_dps'].max():+.1f}")
    print(f"u (N*m)            min={data['u0_Nm'].min():+.3f}  "
          f"max={data['u0_Nm'].max():+.3f}  mean={data['u0_Nm'].mean():+.3f}")
    print(f"solve total (ms)   mean={data['solve_total_ms'].mean():.2f}  "
          f"p95={np.percentile(data['solve_total_ms'], 95):.2f}  "
          f"max={data['solve_total_ms'].max():.2f}")
    print(f"總共丟棄過期資料: {data['dropped'].sum()} 筆")
    print(f"總共觸發重建問題: {data['rebuilt'].sum()} 次")
    statuses, counts = np.unique(data["status"], return_counts=True)
    if not (len(statuses) == 1 and statuses[0] == "n/a"):
        print("求解狀態分佈:")
        for s, c in zip(statuses, counts):
            print(f"    {s}: {c}")


if __name__ == "__main__":
    csv_path = sys.argv[1] if len(sys.argv) > 1 else find_latest_log()
    data = load_csv(csv_path)
    print_summary(data)
    out_png = os.path.splitext(csv_path)[0] + ".png"
    plot(data, out_png)