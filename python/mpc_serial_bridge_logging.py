"""
mpc_serial_bridge_logging.py  (改良版)
---------------------------------------------------------------
相對於原版的修改（都標了 [FIX]）：

[FIX-A] 量測並補償整條鏈路的死區時間。
        原版把「Teensy 在 t 時刻量到的狀態」直接當成「扭矩會被套用的時刻」的狀態，
        但中間其實隔了：序列埠傳輸 + solve(p50 5.7ms / p95 10.3ms) + 回傳
        + Teensy 下一個 15ms tick 才寫馬達，合計約 20~30ms。
        倒單擺是不穩定系統，未補償的死區時間會直接吃掉相位餘裕。
        這裡用「上一拍實際量到的 solve 時間 + 固定鏈路延遲」估計死區時間，
        交給 mpc.solve(delay_s=...) 用模型往前推算。

[FIX-B] 用 Teensy 自己的時間戳記偵測「這筆狀態已經放多久」，
        如果因為 PC 卡頓導致積壓，會被算進 delay 一起補償，而不是默默用舊資料。

[FIX-C] 把 Teensy 端的摩擦前饋在 PC 端鏡像一份，用來告訴 MPC
        「上一拍實際送出去的扭矩」，讓 Δu 懲罰與 du_max 的基準正確。
        這裡的常數必須跟 config.h 保持一致。

[FIX-D] log 多記 delay_ms / u_applied，方便事後驗證補償量是否合理。

輸出檔案：
    ./mpc_bridge_log_<YYYYMMDD_HHMMSS>.csv
---------------------------------------------------------------
"""

import csv
import math
import time
import datetime
import numpy as np
import serial

from balance_mpc import BalanceMPC, RobotParams

# ================= 使用者需要確認/調整的設定 =================
MPC_PORT = "COM6"          # <<< 改成 SerialUSB1 對應的序列埠名稱
MPC_BAUD = 2000000

LEG_LENGTH_M = 0.181       # 請確認這是「鎖腿後輪軸到機身質心」的實際距離
PITCH_SIGN = -1.0
V_REF = 0.0

MPC_TIMEOUT_S = 0.3
PRINT_EVERY = 20

# 實測 solve_total_ms: p50 5.7 / p95 10.3 / max 13.3 (N=20, Ts=0.015)
MPC_N = 20
MPC_TS = 0.015

MPC_OSQP_EPS = 1e-2
MPC_OSQP_MAX_ITER = 4000

# [FIX-A] 除了 solve 時間之外的固定鏈路延遲估計 (s)
#   = 序列埠往返 + Teensy 下一個控制 tick 的量化延遲
#   Teensy 控制週期 15ms，平均量化延遲約 7.5ms，加上 USB 往返抓 3ms
FIXED_LINK_DELAY_S = 0.010
DELAY_CAP_S = 0.060        # 補償上限，避免 PC 卡頓時外推過頭

# [FIX-C] 必須與 config.h 的同名常數一致
FRICTION_LEFT_NM  = 0.503 * 0.7
FRICTION_RIGHT_NM = 0.391 * 0.7
FRICTION_SPEED_EPS_DPS = 20.0    # [2nd] 6.0 -> 20.0，需與 config.h 同步改
FRICTION_CMD_EPS_NM    = 0.05

# [2nd] 陀螺儀一階低通，只作用在送進 MPC 的 omega，不影響 Teensy 的角度積分。
#   1.0 = 完全不濾(預設)。放在 PC 端是因為調這個不用重新燒錄。
#   先靠 R_DU 壓 5.8Hz 極限環；如果還壓不下來，再從 0.6 開始往下調。
#   代價：alpha=0.6 在 67Hz 下截止頻率約 13Hz、群延遲約 7ms，會稍微吃掉相位餘裕。
OMEGA_LPF_ALPHA = 1.0

# [2nd] 位置保持。預設 False：x0 的 p 固定為 0，機器人只穩速度不穩位置，
#   所以會有 log 裡看到的 0.23Hz 慢速游走(整段 106 秒淨漂移 -0.09m，其實很小)。
#   想讓它「待在原地」再打開，並同時把 balance_mpc.py 的 Q_DIAG[0] 從 0 調成 2~8。
#   注意：位置是用輪速積分出來的，打滑會累積誤差，開了之後請留意有沒有緩慢跑掉。
TRACK_POSITION = True    # [3rd] 打開：v3 的殘留誤差已經幾乎都是位置游走
POS_LIMIT_M = 0.5        # [3rd] 積分位置夾限 (m)

LOG_PATH = f"mpc_bridge_log_{datetime.datetime.now():%Y%m%d_%H%M%S}.csv"
LOG_FLUSH_EVERY = 20
# ================================================================


def parse_state_line(line: str):
    """解析 'S,<t_ms>,<pitch_deg>,<pitchRate_dps>,<wheelSpeed_dps>'"""
    parts = line.strip().split(",")
    if len(parts) != 5 or parts[0] != "S":
        return None
    try:
        return (int(parts[1]), float(parts[2]), float(parts[3]), float(parts[4]))
    except ValueError:
        return None


def friction_ff(u_wheel_nm: float, speed_dps: float, fc_nm: float) -> float:
    """[FIX-C] 與 wbr_control.ino 的 applyFrictionFF() 完全相同的公式"""
    dir_speed = math.tanh(speed_dps / FRICTION_SPEED_EPS_DPS)
    dir_cmd = math.tanh(u_wheel_nm / FRICTION_CMD_EPS_NM)
    w = abs(dir_speed)
    return u_wheel_nm + fc_nm * (w * dir_speed + (1.0 - w) * dir_cmd)


def estimate_applied_total(u0_nm: float, wheel_speed_dps: float) -> float:
    """把 MPC 的總扭矩換算成 Teensy 實際會送出的總扭矩（含摩擦前饋）"""
    base = u0_nm / 2.0
    return (friction_ff(base, wheel_speed_dps, FRICTION_LEFT_NM)
            + friction_ff(base, wheel_speed_dps, FRICTION_RIGHT_NM))


def main():
    print(f"[bridge] 開啟序列埠 {MPC_PORT} @ {MPC_BAUD} baud ...")
    ser = serial.Serial(MPC_PORT, MPC_BAUD, timeout=0.05)
    time.sleep(1.0)

    params = RobotParams()
    mpc = BalanceMPC(params, N=MPC_N, Ts=MPC_TS,
                     l_nominal=LEG_LENGTH_M,
                     osqp_eps=MPC_OSQP_EPS, osqp_max_iter=MPC_OSQP_MAX_ITER)
    wheel_radius_m = params.r

    print("[bridge] 暖機中 ...")
    warmup_t = mpc.solve(np.zeros(4), LEG_LENGTH_M, np.zeros(mpc.N))[2]
    print(f"[bridge] 暖機完成 (耗時 {warmup_t['total_ms']:.1f}ms)")
    mpc.set_applied_u(0.0)

    seq = 0
    last_rx_time = time.time()
    warned_timeout = False
    t_wall_start = time.time()

    last_solve_s = 0.006       # [FIX-A] 上一拍的求解耗時，第一拍先給個合理初值
    t_teensy_prev = None
    omega_filt_prev = 0.0      # [2nd]
    pos = 0.0                  # [2nd]

    print(f"[bridge] 資料將同時記錄到 {LOG_PATH}")
    with open(LOG_PATH, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "seq", "t_wall_s", "t_teensy_ms", "pitch_deg", "pitch_rate_dps",
            "wheel_speed_dps", "v_mps", "pos_m", "u0_Nm_combined", "u_applied_Nm",
            "delay_ms", "solve_total_ms", "solve_qp_ms",
            "dropped", "rebuilt", "status",
        ])

        print("[bridge] 開始運行，Ctrl+C 結束")
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
                        p2 = parse_state_line(
                            ser.readline().decode("ascii", errors="ignore"))
                        if p2 is not None:
                            if parsed is not None:
                                dropped += 1
                            parsed = p2

                if parsed is None:
                    if time.time() - last_rx_time > MPC_TIMEOUT_S and not warned_timeout:
                        print("[bridge] !!! 一段時間沒收到姿態資料，"
                              "請確認Teensy是否已切換到MPC模式(按m)並啟動輪子(按e) !!!")
                        warned_timeout = True
                    continue

                warned_timeout = False
                last_rx_time = time.time()
                t_ms, pitch_deg, pitch_rate_dps, wheel_speed_dps = parsed

                theta = np.radians(pitch_deg) * PITCH_SIGN
                omega_raw = np.radians(pitch_rate_dps) * PITCH_SIGN
                v = np.radians(wheel_speed_dps) * wheel_radius_m

                # [2nd] 可選的 omega 低通
                if OMEGA_LPF_ALPHA >= 1.0:
                    omega = omega_raw
                else:
                    omega = (OMEGA_LPF_ALPHA * omega_raw
                             + (1.0 - OMEGA_LPF_ALPHA) * omega_filt_prev)
                omega_filt_prev = omega

                # [2nd] 可選的位置積分
                if TRACK_POSITION:
                    pos += v * MPC_TS
                    # [3rd] 夾限：打滑或被抱起來時積分會一路累積，
                    # 夾在 ±POS_LIMIT_M 之內，避免 MPC 為了「回家」而下猛藥。
                    pos = max(-POS_LIMIT_M, min(POS_LIMIT_M, pos))
                else:
                    pos = 0.0

                x0 = np.array([pos, v, theta, omega])
                v_ref_traj = np.full(mpc.N, V_REF)

                # [FIX-A][FIX-B] 估計這次算出來的扭矩「被套用時」距離量測有多久
                backlog_s = 0.0
                if dropped > 0 and t_teensy_prev is not None:
                    # 有積壓代表這筆已經不是最新的；用 Teensy 時間戳差估算額外延遲
                    backlog_s = max(0.0, (t_ms - t_teensy_prev) * 1e-3 - MPC_TS)
                t_teensy_prev = t_ms

                delay_s = min(DELAY_CAP_S,
                              last_solve_s + FIXED_LINK_DELAY_S + backlog_s)

                u0, _debug, timing = mpc.solve(x0, LEG_LENGTH_M, v_ref_traj,
                                               delay_s=delay_s)
                last_solve_s = timing["total_ms"] * 1e-3

                seq += 1
                ser.write(f"U,{seq},{u0:.4f}\n".encode("ascii"))

                # [FIX-C] 把「實際會被送出的扭矩」回寫給 MPC 當作下一拍的 u_prev
                u_applied = estimate_applied_total(u0, wheel_speed_dps)
                mpc.set_applied_u(u_applied)

                writer.writerow([
                    seq,
                    f"{time.time() - t_wall_start:.4f}",
                    t_ms,
                    f"{pitch_deg:.4f}",
                    f"{pitch_rate_dps:.4f}",
                    f"{wheel_speed_dps:.4f}",
                    f"{v:.5f}",
                    f"{pos:.5f}",
                    f"{u0:.5f}",
                    f"{u_applied:.5f}",
                    f"{delay_s * 1000:.2f}",
                    f"{timing['total_ms']:.3f}",
                    f"{timing['qp_ms']:.3f}",
                    dropped,
                    int(timing["rebuilt"]),
                    timing.get("status", ""),
                ])
                if seq % LOG_FLUSH_EVERY == 0:
                    f.flush()

                if seq % PRINT_EVERY == 0:
                    note = f"  (丟棄{dropped}筆過期資料)" if dropped > 0 else ""
                    print(f"[{seq}] pitch={pitch_deg:+.2f}deg  v={v:+.3f}m/s  "
                          f"u0={u0:+.3f}Nm  delay={delay_s*1000:.0f}ms  "
                          f"solve={timing['total_ms']:.2f}ms{note}")

                if timing["rebuilt"]:
                    print(f"[bridge] !!! 第{seq}次求解觸發重新建模"
                          f"(rebuild_count={mpc.rebuild_count})，腿長固定時不該發生 !!!")

                if timing["total_ms"] > 14:
                    print(f"[bridge] 警告：求解耗時 {timing['total_ms']:.1f}ms，"
                          f"已接近控制週期 {MPC_TS*1000:.0f}ms，建議調小 MPC_N (目前 {mpc.N})")

        except KeyboardInterrupt:
            print("\n[bridge] 收到 Ctrl+C，送出扭矩=0 並結束")
            try:
                ser.write(b"U,0,0.0\n")
            except Exception:
                pass
        finally:
            ser.close()
            print(f"[bridge] 資料已存至 {LOG_PATH}")


if __name__ == "__main__":
    main()