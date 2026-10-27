"""
mpc_serial_bridge.py (配合實車辨識增益版 wbr_mpc.py)
"""

import csv
import math
import sys
import time
import datetime
import numpy as np
import serial

from wbr_mpc import BalanceMPC, RobotParams

try:
    import msvcrt
except ImportError:
    msvcrt = None
    import select

# ================= 使用者需要確認/調整的設定 =================
MPC_PORT = "COM6"
MPC_BAUD = 2000000

LEG_LENGTH_M = 0.170    
PITCH_SIGN = -1.0
YAW_SIGN = +1.0
V_REF = 0.0

MPC_TIMEOUT_S = 0.3
PRINT_EVERY = 20

MPC_N = 10                 # 預測 150ms，兼顧穩定裕度與極速求解 (~0.25ms)
MPC_TS = 0.01

FIXED_LINK_DELAY_S = 0.024 # 狀態取樣齡 + USB + 馬達電流響應；log 顯示 cmd->電流約 1~2 拍，原 18ms 偏小
DELAY_CAP_S = 0.050

FRICTION_LEFT_NM  = 0.3   # 與 wbr_control.ino 完全對齊 (僅供 log 記錄對照)
FRICTION_RIGHT_NM = 0.3
FRICTION_SPEED_EPS_DPS = 20.0
FRICTION_CMD_EPS_NM    = 0.12

OMEGA_LPF_ALPHA = 0.7      # 對 pitch_rate 加入輕微一階低通，抑制微分高頻抖動
YAW_RATE_LPF_ALPHA = 0.85  # 韌體已把 yaw rate 對 10ms 內的 IMU 取樣取平均 (抗混疊)，這裡只留很輕的低通，少一點相位落後

TRACK_POSITION = True

# [OPT] 平衡點 (pitch 零點 / 重心偏移) 慢速補償：theta_off 以輪速積分，夾限、傾斜過大時凍結
THETA_OFF_KI = 0.1               # 0 = 關閉；模擬中 0.1~0.2 可降低位置游走，過大反而拖慢
THETA_OFF_LIMIT_DEG = 3.0
THETA_OFF_FREEZE_DEG = 6.0
POS_LIMIT_M = 0.4

YAW_RATE_REF = 0.0
PSI_LIMIT_RAD = math.radians(180)

LOG_PATH = f"mpc_bridge_log_{datetime.datetime.now():%Y%m%d_%H%M%S}.csv"
LOG_FLUSH_EVERY = 20

AUTO_LOCK_ON_START = True
COMMAND_KEYS = "dluzxc69"
KEY_HELP = (
    "[bridge] 按鍵：  e 啟動輪子   d 關閉輪子   l 關節自鎖   u 關節解鎖\n"
    "                z 偏航歸零   x XY歸零     c 加速度校正(輪子須關閉)   6/9 六軸/九軸\n"
    "                h 顯示說明"
)


def parse_state_line(line: str):
    parts = line.strip().split(",")
    if len(parts) not in (8, 15) or parts[0] != "S":
        return None
    try:
        return (int(parts[1]), *[float(x) for x in parts[2:]])
    except ValueError:
        return None


def parse_ack_line(line: str):
    parts = line.strip().split(",")
    if len(parts) != 3 or parts[0] != "A" or len(parts[1]) != 1:
        return None
    try:
        return parts[1], int(parts[2])
    except ValueError:
        return None


class CmdSender:
    def __init__(self, ser):
        self.ser = ser
        self.seq = 0

    def send(self, cmd: str):
        self.seq += 1
        self.ser.write(f"C,{self.seq},{cmd}\n".encode("ascii"))
        print(f"[bridge] -> 送出指令 '{cmd}'")


def poll_keys():
    keys = []
    if msvcrt is not None:
        while msvcrt.kbhit():
            keys.append(msvcrt.getwch().lower())
    else:
        while select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline()
            if not line:
                break
            keys.extend(line.strip().lower())
    return keys


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def friction_ff(u_wheel_nm: float, speed_dps: float, fc_nm: float) -> float:
    dir_speed = math.tanh(speed_dps / FRICTION_SPEED_EPS_DPS)
    dir_cmd = math.tanh(u_wheel_nm / FRICTION_CMD_EPS_NM)
    w = abs(dir_speed)
    return u_wheel_nm + fc_nm * (w * dir_speed + (1.0 - w) * dir_cmd)


def estimate_applied(u_l: float, u_r: float, vL_dps: float, vR_dps: float):
    aL = friction_ff(u_l, vL_dps, FRICTION_LEFT_NM)
    aR = friction_ff(u_r, vR_dps, FRICTION_RIGHT_NM)
    return aL, aR


def main():
    print(f"[bridge] 開啟序列埠 {MPC_PORT} @ {MPC_BAUD} baud ...")
    ser = serial.Serial(MPC_PORT, MPC_BAUD, timeout=0.05)
    time.sleep(1.0)

    params = RobotParams()
    mpc = BalanceMPC(params, N=MPC_N, Ts=MPC_TS, l_nominal=LEG_LENGTH_M)
    wheel_radius_m = params.r

    print("[bridge] 暖機中 ...")
    warmup_t = mpc.solve(np.zeros(6), LEG_LENGTH_M, np.zeros((mpc.nx, mpc.N)))[2]
    print(f"[bridge] 暖機完成 (耗時 {warmup_t['total_ms']:.2f}ms)")
    mpc.set_applied_u(0.0, 0.0)

    seq = 0
    last_rx_time = time.time()
    warned_timeout = False
    t_wall_start = time.time()

    last_solve_s = 0.001
    t_teensy_prev = None
    omega_filt_prev = 0.0
    psi_dot_filt_prev = 0.0
    pos = 0.0
    theta_off = 0.0
    psi_target = None

    sender = CmdSender(ser)
    wheels_requested = False
    if AUTO_LOCK_ON_START:
        print("[bridge] 啟動：請求關節自鎖 (腿移動需要幾秒，到位後再按 e)")
        sender.send('l')
    print(KEY_HELP)

    print(f"[bridge] 資料將同時記錄到 {LOG_PATH}")
    with open(LOG_PATH, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "seq", "t_wall_s", "t_teensy_ms", "pitch_deg", "pitch_rate_dps",
            "vL_dps", "vR_dps", "v_mps", "pos_m", "yaw_deg", "yaw_rate_dps", "psi_err_rad",
            "u_l", "u_r", "u_l_applied", "u_r_applied",
            "delay_ms", "solve_total_ms", "solve_qp_ms",
            "dropped", "rebuilt", "status",
            "iL_A", "iR_A", "cmdL_A", "cmdR_A", "wL_raw_dps", "wR_raw_dps", "motor_fresh",
            "theta_off_deg",
        ])

        print("[bridge] 開始運行，Ctrl+C 結束")
        try:
            while True:
                for k in poll_keys():
                    if k == 'e':
                        mpc.set_applied_u(0.0, 0.0)
                        pos = 0.0
                        theta_off = 0.0
                        psi_target = None
                        omega_filt_prev = 0.0
                        psi_dot_filt_prev = 0.0
                        t_teensy_prev = None
                        last_rx_time = time.time()
                        wheels_requested = True
                        sender.send('e')
                    elif k in COMMAND_KEYS:
                        if k in ('d', 'u'):
                            wheels_requested = False
                        sender.send(k)
                    elif k == 'h':
                        print(KEY_HELP)

                parsed = None
                dropped = 0
                lines = []
                raw = ser.readline().decode("ascii", errors="ignore")
                if raw:
                    lines.append(raw)
                    while ser.in_waiting > 0:
                        lines.append(ser.readline().decode("ascii", errors="ignore"))
                for ln in lines:
                    ack = parse_ack_line(ln)
                    if ack is not None:
                        a_cmd, a_res = ack
                        if a_cmd == 'F':
                            print("[bridge] !!! Teensy 傾倒保護觸發，輪子已關閉 (重新扶正後按 e) !!!")
                            wheels_requested = False
                        elif a_cmd == 'T':
                            print("[bridge] !!! Teensy 判定 PC 逾時，輪子已關閉 !!!")
                            wheels_requested = False
                        else:
                            print(f"[bridge] <- Teensy 回覆 '{a_cmd}': "
                                  f"{'已執行' if a_res == 1 else '被拒絕 (看 Teensy Serial 訊息)'}")
                            if a_cmd == 'e' and a_res != 1:
                                wheels_requested = False
                        continue
                    p = parse_state_line(ln)
                    if p is not None:
                        if parsed is not None:
                            dropped += 1
                        parsed = p

                if parsed is None:
                    if (wheels_requested and not warned_timeout
                            and time.time() - last_rx_time > MPC_TIMEOUT_S):
                        print("[bridge] !!! 已送出啟動指令，但一段時間沒收到姿態資料，"
                              "請確認 Teensy 是否已自鎖關節、序列埠是否正確 !!!")
                        warned_timeout = True
                    continue

                warned_timeout = False
                last_rx_time = time.time()
                t_ms, pitch_deg, pitch_rate_dps, vL_dps, vR_dps, yaw_deg, yaw_rate_dps, *motor_x = parsed
                wheel_speed_dps = 0.5 * (vL_dps + vR_dps)

                theta = np.radians(pitch_deg) * PITCH_SIGN
                omega_raw = np.radians(pitch_rate_dps) * PITCH_SIGN
                v = np.radians(wheel_speed_dps) * wheel_radius_m

                omega = OMEGA_LPF_ALPHA * omega_raw + (1.0 - OMEGA_LPF_ALPHA) * omega_filt_prev
                omega_filt_prev = omega

                if TRACK_POSITION:
                    pos += v * MPC_TS
                    pos = max(-POS_LIMIT_M, min(POS_LIMIT_M, pos))
                else:
                    pos = 0.0

                psi = math.radians(yaw_deg) * YAW_SIGN
                psi_dot_raw = math.radians(yaw_rate_dps) * YAW_SIGN
                psi_dot = YAW_RATE_LPF_ALPHA * psi_dot_raw + (1.0 - YAW_RATE_LPF_ALPHA) * psi_dot_filt_prev
                psi_dot_filt_prev = psi_dot

                if psi_target is None:
                    psi_target = psi
                psi_target = wrap_pi(psi_target + YAW_RATE_REF * MPC_TS)
                psi_err = wrap_pi(psi - psi_target)

                # [OPT] 平衡點補償：輪速持續為正 -> 真正的平衡點比目前假設更「前」，慢慢移動 theta 零點
                if THETA_OFF_KI > 0.0 and abs(math.degrees(theta)) < THETA_OFF_FREEZE_DEG:
                    lim = math.radians(THETA_OFF_LIMIT_DEG)
                    theta_off = max(-lim, min(lim, theta_off - THETA_OFF_KI * v * MPC_TS))

                x0 = np.array([pos, v, theta - theta_off, omega, psi_err, psi_dot])
                X_ref_traj = np.zeros((mpc.nx, mpc.N))
                X_ref_traj[1, :] = V_REF
                X_ref_traj[5, :] = YAW_RATE_REF

                backlog_s = 0.0
                if dropped > 0 and t_teensy_prev is not None:
                    backlog_s = max(0.0, (t_ms - t_teensy_prev) * 1e-3 - MPC_TS)
                t_teensy_prev = t_ms

                delay_s = min(DELAY_CAP_S, last_solve_s + FIXED_LINK_DELAY_S + backlog_s)

                u0, _debug, timing = mpc.solve(x0, LEG_LENGTH_M, X_ref_traj, delay_s=delay_s)
                u_l, u_r = float(u0[0]), float(u0[1])
                last_solve_s = 0.7 * last_solve_s + 0.3 * timing["total_ms"] * 1e-3

                seq += 1
                ser.write(f"U,{seq},{u_l:.4f},{u_r:.4f}\n".encode("ascii"))

                # 僅計算包含摩擦前饋的估計值供 CSV 記錄，絕不可回寫進 mpc.set_applied_u！
                u_l_app, u_r_app = estimate_applied(u_l, u_r, vL_dps, vR_dps)

                writer.writerow([
                    seq,
                    f"{time.time() - t_wall_start:.4f}",
                    t_ms,
                    f"{pitch_deg:.4f}",
                    f"{pitch_rate_dps:.4f}",
                    f"{vL_dps:.4f}",
                    f"{vR_dps:.4f}",
                    f"{v:.5f}",
                    f"{pos:.5f}",
                    f"{yaw_deg:.4f}",
                    f"{yaw_rate_dps:.4f}",
                    f"{psi_err:.5f}",
                    f"{u_l:.5f}",
                    f"{u_r:.5f}",
                    f"{u_l_app:.5f}",
                    f"{u_r_app:.5f}",
                    f"{delay_s * 1000:.2f}",
                    f"{timing['total_ms']:.3f}",
                    f"{timing['qp_ms']:.3f}",
                    dropped,
                    int(timing["rebuilt"]),
                    timing.get("status", ""),
                    *([f"{motor_x[0]:.3f}", f"{motor_x[1]:.3f}", f"{motor_x[2]:.3f}",
                       f"{motor_x[3]:.3f}", f"{motor_x[4]:.3f}", f"{motor_x[5]:.3f}",
                       int(motor_x[6])] if len(motor_x) == 7 else [""] * 7),
                    f"{math.degrees(theta_off):.3f}",
                ])
                if seq % LOG_FLUSH_EVERY == 0:
                    f.flush()

                if seq % PRINT_EVERY == 0:
                    note = f"  (丟棄{dropped}筆過期資料)" if dropped > 0 else ""
                    print(f"[{seq}] pitch={pitch_deg:+.2f}deg  v={v:+.3f}m/s  "
                          f"uL={u_l:+.3f} uR={u_r:+.3f}Nm  yawerr={math.degrees(psi_err):+.1f}deg  delay={delay_s*1000:.0f}ms  "
                          f"solve={timing['total_ms']:.2f}ms{note}")

                if timing["rebuilt"]:
                    print(f"[bridge] !!! 第{seq}次求解觸發重新建模"
                          f"(rebuild_count={mpc.rebuild_count})，腿長固定時不該發生 !!!")

                if timing["total_ms"] > 8:
                    print(f"[bridge] 警告：求解耗時 {timing['total_ms']:.1f}ms，"
                          f"已接近控制週期 {MPC_TS*1000:.0f}ms，建議調小 MPC_N (目前 {mpc.N})")

        except KeyboardInterrupt:
            print("\n[bridge] 收到 Ctrl+C，送出扭矩=0 並關閉輪子後結束")
            try:
                ser.write(b"U,0,0.0,0.0\n")
                for _ in range(3):
                    sender.send('d')
                    time.sleep(0.02)
            except Exception:
                pass
        finally:
            ser.close()
            print(f"[bridge] 資料已存至 {LOG_PATH}")


if __name__ == "__main__":
    main()