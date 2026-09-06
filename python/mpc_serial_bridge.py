"""
mpc_serial_bridge.py
---------------------------------------------------------------
在 PC 端執行：透過 Teensy 的 SerialUSB1 讀取即時姿態資料，餵給
balance_mpc.py 裡的 BalanceMPC 即時求解，再把扭矩指令送回 Teensy。

事前準備：
    1. Arduino IDE: Tools -> USB Type -> 選 "Dual Serial" 或 "Triple Serial"
       上傳新韌體後，Teensy 會多出一個 USB 序列埠 (SerialUSB1)。
    2. 把下面 MPC_PORT 改成 SerialUSB1 對應的裝置名稱：
         Windows: 裝置管理員看多出來的第二個 COM port
         macOS/Linux: `python -m serial.tools.list_ports` 或 `ls /dev/tty.usb*`
       原本用來下 e/d/l/m 等人工指令的那個 Serial 埠不受影響，可另開一個
       終端機 (Arduino Serial Monitor 或 `screen`) 照舊操作。
    3. pip install numpy scipy cvxpy osqp pyserial
    4. Teensy 端先按 m 切到 MPC 模式（輪子必須先是關閉狀態），再執行本程式，
       最後在 Teensy 端按 e 啟動輪子。

安全機制：
    - 超過 MPC_TIMEOUT_S 沒收到姿態資料只會印警告；真正的安全刹車在Teensy端
      （超過 MPC_TIMEOUT_MS 沒收到PC的扭矩指令會自動關輪子）。
    - Ctrl+C 離開前會送一次 torque=0。
---------------------------------------------------------------
"""

import time
import numpy as np
import serial

from balance_mpc import BalanceMPC, RobotParams

# ================= 使用者需要確認/調整的設定 =================
MPC_PORT = "COM17"          # <<< 改成 SerialUSB1 對應的序列埠名稱
MPC_BAUD = 2000000         # 需與 Teensy 端 mpcLink.begin() 的 baud 一致

# TODO: 請依實際鎖定姿態量測/計算出的虛擬腿長 (輪軸到機身質心距離，公尺)
LEG_LENGTH_M = 0.20

# 若實測發現機器人反應方向跟預期相反 (往前倒卻加速往前衝之類)，改成 -1.0
PITCH_SIGN = -1.0

# 目標前進速度 (m/s)，先固定 0 只驗證原地平衡；之後可接搖桿/鍵盤即時修改
V_REF = 0.0

MPC_TIMEOUT_S = 0.3        # 多久沒收到姿態資料就印警告（僅供PC端除錯用）
PRINT_EVERY = 20           # 每隔幾次求解印一次除錯訊息

# MPC 預測時域步數 / 每步代表的時間(s)。
# 求解耗時主要跟 N 有關（QP問題規模），Ts只影響預測準不準，不太影響耗時。
# N=40 大約是 N=60,Ts=0.01 的默認設定拿掉1/3步數，如果你的電腦跑起來
# solve_ms 還是常常超過20~30ms，把 N 再往下調（例如20~30）通常最有效。
MPC_N = 40
MPC_TS = 0.01

# OSQP 收斂容忍度，數字越大解得越粗但越快。求解時間如果還是偏高，
# 可以先試著把這個從 1e-3 調到 1e-2；1e-2 通常對平衡控制來說精度還夠。
MPC_OSQP_EPS = 1e-2
MPC_OSQP_MAX_ITER = 4000
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
    time.sleep(1.0)  # 等待Teensy USB CDC就緒

    params = RobotParams()
    mpc = BalanceMPC(params, N=MPC_N, Ts=MPC_TS,
                      osqp_eps=MPC_OSQP_EPS, osqp_max_iter=MPC_OSQP_MAX_ITER)
    wheel_radius_m = params.r

    # ---- 暖機：cvxpy第一次solve()要建立/canonicalize問題，比較慢(可能上百ms)。----
    # ---- 先用假資料跑一次，避免這個一次性成本發生在真正開始平衡的當下。 ----
    print("[mpc_serial_bridge] 暖機中 ...")
    warmup_t = mpc.solve(np.zeros(4), LEG_LENGTH_M, np.zeros(mpc.N))[2]
    print(f"[mpc_serial_bridge] 暖機完成 (耗時 {warmup_t['total_ms']:.1f}ms)，之後每次應該快很多")

    seq = 0
    last_rx_time = time.time()
    warned_timeout = False

    print("[mpc_serial_bridge] 開始運行，Ctrl+C 結束")
    try:
        while True:
            # ---- 把序列埠緩衝區裡排隊的資料一次讀光，只留「最新一筆」有效狀態 ----
            # 這是解決資料堆積的關鍵：如果 solve() 比感測資料送達的速度慢，
            # 緩衝區會一直塞新資料進來；如果每次只 readline() 一行，
            # 會一直在處理舊資料、越追越落後。這裡改成把buffer清空，
            # 永遠只拿"當下最新"的一筆去算，寧可降低更新頻率也不要用過期姿態。
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
            _t_ms, pitch_deg, pitch_rate_dps, wheel_speed_dps = parsed

            theta = np.radians(pitch_deg) * PITCH_SIGN
            omega = np.radians(pitch_rate_dps) * PITCH_SIGN
            v = np.radians(wheel_speed_dps) * wheel_radius_m

            x0 = np.array([0.0, v, theta, omega])   # p 不追蹤(Q_p=0)，固定填0即可
            v_ref_traj = np.full(mpc.N, V_REF)

            u0, _debug, timing = mpc.solve(x0, LEG_LENGTH_M, v_ref_traj)

            seq += 1
            ser.write(f"U,{seq},{u0:.4f}\n".encode("ascii"))

            if seq % PRINT_EVERY == 0:
                backlog_note = f"  (剛丟棄{dropped}筆過期資料)" if dropped > 0 else ""
                print(f"[{seq}] pitch={pitch_deg:+.2f}deg  v={v:+.3f}m/s  "
                      f"u={u0:+.3f}Nm  solve={timing['total_ms']:.2f}ms "
                      f"(qp={timing['qp_ms']:.2f}ms){backlog_note}")

            if timing["rebuilt"]:
                print(f"[mpc_serial_bridge] !!! 第{seq}次求解觸發了重新建模(rebuild_count="
                      f"{mpc.rebuild_count})，這就是total比qp多很多的原因！"
                      f"如果LEG_LENGTH_M是常數，理論上除了第一次不該再發生 !!!")

            # solve()耗時如果長期大於 Teensy 送資料的週期(約10ms)，
            # 代表現在跑的頻率已經追不上100Hz，是正常的降頻現象(不是bug)，
            # 但如果 solve_ms 動輒 >50~100ms，建議把上面的 MPC_N 調小。
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


if __name__ == "__main__":
    main()