"""
mpc_serial_bridge_logging.py
---------------------------------------------------------------
跟 mpc_serial_bridge.py 完全相同的即時運行邏輯，唯一差異是：
每次 solve() 之後，把這一筆資料同時寫進一個 CSV 檔，方便事後用
plot_mpc_log.py 畫圖 / 用 Excel 檢視。

新增/修改的地方都標了 [LOG] 註解，其餘邏輯逐行照抄原始檔案，
沒有動任何控制邏輯，可以放心直接拿來取代原本的 mpc_serial_bridge.py。

輸出檔案：
    ./mpc_bridge_log_<YYYYMMDD_HHMMSS>.csv
    欄位：seq, t_wall_s, t_teensy_ms, pitch_deg, pitch_rate_dps,
          wheel_speed_dps, v_mps, u0_Nm, solve_total_ms, solve_qp_ms,
          dropped, rebuilt
---------------------------------------------------------------
"""

import csv
import time
import datetime
import numpy as np
import serial

from balance_mpc import BalanceMPC, RobotParams

# ================= 使用者需要確認/調整的設定 =================
MPC_PORT = "COM17"          # <<< 改成 SerialUSB1 對應的序列埠名稱
MPC_BAUD = 2000000         # 需與 Teensy 端 mpcLink.begin() 的 baud 一致

LEG_LENGTH_M = 0.181
PITCH_SIGN = -1.0
V_REF = 0.0

MPC_TIMEOUT_S = 0.3
PRINT_EVERY = 20

# [調整] 原本 N=40 時，實測 solve_total_ms 均值約50ms、尖峰近300ms，
# 跟 Ts=0.01(10ms) 假設差了5~10倍，是造成延遲失穩的主因之一。
# 先降到20試試，重新記log看 solve_total_ms 有沒有明顯壓低；
# 如果均值還是遠高於10ms，可以再往下調(例如15)，
# 或者乾脆把下面的 MPC_TS 也一併調大，讓模型的時間粒度貼近實測周期。
MPC_N = 20
MPC_TS = 0.015

MPC_OSQP_EPS = 1e-2
MPC_OSQP_MAX_ITER = 4000

# [LOG] CSV 檔名，預設帶時間戳記避免每次覆蓋掉上一次的紀錄
LOG_PATH = f"mpc_bridge_log_{datetime.datetime.now():%Y%m%d_%H%M%S}.csv"
LOG_FLUSH_EVERY = 20   # [LOG] 每幾筆資料強制flush一次，避免程式意外中斷時資料留在OS緩衝區沒寫進硬碟
# ================================================================


def parse_state_line(line: str):
    """解析 'S,<t_ms>,<pitch_deg>,<pitchRate_dps>,<wheelSpeed_dps>'"""
    parts = line.strip().split(",")
    if len(parts) != 5 or parts[0] != "S":
        return None
    try:
        t_ms = int(parts[1])
        pitch_deg = float(parts[2])
        pitch_rate_dps = float(parts[3])
        wheel_speed_dps = float(parts[4])
        return t_ms, pitch_deg, pitch_rate_dps, wheel_speed_dps
    except ValueError:
        return None


def main():
    print(f"[mpc_serial_bridge] 開啟序列埠 {MPC_PORT} @ {MPC_BAUD} baud ...")
    ser = serial.Serial(MPC_PORT, MPC_BAUD, timeout=0.05)
    time.sleep(1.0)

    params = RobotParams()
    mpc = BalanceMPC(params, N=MPC_N, Ts=MPC_TS,
                      osqp_eps=MPC_OSQP_EPS, osqp_max_iter=MPC_OSQP_MAX_ITER)
    wheel_radius_m = params.r

    print("[mpc_serial_bridge] 暖機中 ...")
    warmup_t = mpc.solve(np.zeros(4), LEG_LENGTH_M, np.zeros(mpc.N))[2]
    print(f"[mpc_serial_bridge] 暖機完成 (耗時 {warmup_t['total_ms']:.1f}ms)，之後每次應該快很多")

    seq = 0
    last_rx_time = time.time()
    warned_timeout = False
    t_wall_start = time.time()   # [LOG] 記錄開始時間，CSV裡的t_wall_s從0開始算

    # [LOG] 開檔並寫header，用 with 確保程式任何原因結束都會flush+關檔
    print(f"[mpc_serial_bridge] 資料將同時記錄到 {LOG_PATH}")
    with open(LOG_PATH, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "seq", "t_wall_s", "t_teensy_ms", "pitch_deg", "pitch_rate_dps",
            "wheel_speed_dps", "v_mps", "u0_Nm_combined",
            "solve_total_ms", "solve_qp_ms",
            "dropped", "rebuilt", "status",
        ])

        print("[mpc_serial_bridge] 開始運行，Ctrl+C 結束")
        try:
            while True:
                parsed = None
                dropped = 0
                raw = ser.readline().decode("ascii", errors="ignore")
                if raw:
                    p = parse_state_line(raw)
                    if p is not None:
                        parsed = p
                    while ser.in_waiting > 0:
                        raw2 = ser.readline().decode("ascii", errors="ignore")
                        p2 = parse_state_line(raw2)
                        if p2 is not None:
                            if parsed is not None:
                                dropped += 1
                            parsed = p2

                if parsed is None:
                    if time.time() - last_rx_time > MPC_TIMEOUT_S and not warned_timeout:
                        print("[mpc_serial_bridge] !!! 一段時間沒收到姿態資料，"
                              "請確認Teensy是否已切換到MPC模式(按m)並啟動輪子(按e) !!!")
                        warned_timeout = True
                    continue

                warned_timeout = False
                last_rx_time = time.time()
                t_ms, pitch_deg, pitch_rate_dps, wheel_speed_dps = parsed

                theta = np.radians(pitch_deg) * PITCH_SIGN
                omega = np.radians(pitch_rate_dps) * PITCH_SIGN
                v = np.radians(wheel_speed_dps) * wheel_radius_m

                x0 = np.array([0.0, v, theta, omega])
                v_ref_traj = np.full(mpc.N, V_REF)

                u0, _debug, timing = mpc.solve(x0, LEG_LENGTH_M, v_ref_traj)

                seq += 1
                ser.write(f"U,{seq},{u0:.4f}\n".encode("ascii"))

                # [LOG] 寫入這一筆資料
                writer.writerow([
                    seq,
                    f"{time.time() - t_wall_start:.4f}",
                    t_ms,
                    f"{pitch_deg:.4f}",
                    f"{pitch_rate_dps:.4f}",
                    f"{wheel_speed_dps:.4f}",
                    f"{v:.5f}",
                    f"{u0:.5f}",
                    f"{timing['total_ms']:.3f}",
                    f"{timing['qp_ms']:.3f}",
                    dropped,
                    int(timing["rebuilt"]),
                    timing.get("status", ""),
                ])
                if seq % LOG_FLUSH_EVERY == 0:
                    f.flush()

                if seq % PRINT_EVERY == 0:
                    backlog_note = f"  (剛丟棄{dropped}筆過期資料)" if dropped > 0 else ""
                    print(f"[{seq}] pitch={pitch_deg:+.2f}deg  v={v:+.3f}m/s  "
                          f"u0(combined)={u0:+.3f}Nm "
                          f"solve={timing['total_ms']:.2f}ms "
                          f"(qp={timing['qp_ms']:.2f}ms){backlog_note}")

                if timing["rebuilt"]:
                    print(f"[mpc_serial_bridge] !!! 第{seq}次求解觸發了重新建模(rebuild_count="
                          f"{mpc.rebuild_count})，這就是total比qp多很多的原因！"
                          f"如果LEG_LENGTH_M是常數，理論上除了第一次不該再發生 !!!")

                if timing["total_ms"] > 100:
                    print(f"[mpc_serial_bridge] 警告：本次求解耗時 {timing['total_ms']:.1f}ms 偏高，"
                          f"建議調小 MPC_N (目前 {mpc.N})")

        except KeyboardInterrupt:
            print("\n[mpc_serial_bridge] 收到 Ctrl+C，送出扭矩=0 並結束")
            try:
                ser.write(b"U,0,0.0\n")
            except Exception:
                pass
        finally:
            ser.close()
            print(f"[mpc_serial_bridge] 資料已存至 {LOG_PATH}")


if __name__ == "__main__":
    main()