#!/usr/bin/env python3
"""0xA2速度閉ループの動作可否・指令単位・応答帯域を実測するプローブ。

tilt_servo(速度制御による追跡)の前提検証:
  1. 0xA2に応答するか(本ファームはレガシーLK系。V3系0x42等は無応答の実績あり)
  2. 指令単位がロータ角か出力軸角か — 一定速度指令中の0x92角度の傾きで判定
  3. 速度ステップ応答(0→目標、+→−反転)の整定時間
  4. 100Hzループが回るか(1tickあたりのCAN往復時間)

安全策: limits.yamlの可動域+マージン内でのみ動作。毎tickで窓・過電流を監視し、
逸脱時は即0x81停止。終了時は開始位置へ復帰して保持停止。
出力は1行JSONの連続(機械可読)。

使い方(コンテナ内):
  python3 probe_speed.py            # 全フェーズ実行
  python3 probe_speed.py --dry      # 0xA2応答確認のみ(速度0指令、動かない)
"""
import argparse
import json
import sys
import time

import yaml

from rmd_can import RmdMotor, RmdComError, RmdGuardTrip


def emit(obj):
    print(json.dumps(obj, ensure_ascii=False))
    sys.stdout.flush()


class SpeedSession:
    """安全窓つき速度指令セッション。逸脱・過電流で即停止しRmdGuardTrip。"""

    def __init__(self, motor, lo, hi, current_limit_a):
        self.m = motor
        self.lo = lo
        self.hi = hi
        self.limit_a = current_limit_a

    def guard(self, pos, reply):
        if not (self.lo <= pos <= self.hi):
            self.m.command_speed(0.0)
            self.m.stop()
            raise RmdGuardTrip(f"window exceeded ({pos:+.2f} not in "
                               f"[{self.lo:+.1f}, {self.hi:+.1f}])", pos)
        if reply and abs(reply["current_a"]) > self.limit_a:
            self.m.command_speed(0.0)
            self.m.stop()
            raise RmdGuardTrip(
                f"over-current {reply['current_a']:.2f} A", pos)

    def run(self, dps, duration_s, tick_s=0.01):
        """一定速度で duration_s 動かし、サンプル列を返す。

        毎tick: 0xA2再送(応答=電流/速度) + 0x92角度読み → ガード判定。
        """
        samples = []
        t0 = time.perf_counter()
        next_t = t0
        while True:
            now = time.perf_counter()
            if now - t0 >= duration_s:
                break
            reply = self.m.command_speed(dps)
            pos = self.m.angle_deg()
            samples.append((now - t0, pos,
                            reply["speed_dps"] if reply else None,
                            reply["current_a"] if reply else None))
            self.guard(pos, reply)
            next_t += tick_s
            sleep = next_t - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
        return samples


def slope_dps(samples):
    """最小二乗で角度傾き[deg/s]を求める(整定後半分のみ使用)。"""
    tail = samples[len(samples) // 2:]
    n = len(tail)
    if n < 2:
        return 0.0
    st = sum(s[0] for s in tail)
    sp = sum(s[1] for s in tail)
    stt = sum(s[0] * s[0] for s in tail)
    stp = sum(s[0] * s[1] for s in tail)
    den = n * stt - st * st
    return (n * stp - st * sp) / den if den else 0.0


def settle_time(samples, target_dps, frac=0.9):
    """報告速度が目標のfrac倍に初到達する時刻[s]。未到達ならNone。"""
    for t, _, spd, _ in samples:
        if spd is None:
            continue
        if (target_dps >= 0 and spd >= frac * target_dps) or \
           (target_dps < 0 and spd <= frac * target_dps):
            return t
    return None


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--iface", default="can0")
    p.add_argument("--id", type=int, default=1)
    p.add_argument("--limits", default="/workspaces/velodyne-rmd/config/limits.yaml")
    p.add_argument("--limit-a", type=float, default=4.0)
    p.add_argument("--dry", action="store_true", help="0xA2応答確認のみ(動かさない)")
    args = p.parse_args()

    with open(args.limits) as f:
        lim = yaml.safe_load(f)

    try:
        m = RmdMotor(args.iface, args.id)
    except OSError as e:
        emit({"ok": False, "error": f"CAN open failed: {e}"})
        return 1

    try:
        # --- 水平復元と安全窓(ロータ角) ---
        level_mt, delta = m.level_multiturn_from_encoder(int(lim["level_encoder"]))
        if abs(delta) > float(lim["max_level_offset_deg"]):
            emit({"ok": False, "error": f"level offset {delta:+.1f} deg too large"})
            return 1
        margin = float(lim["margin_deg"])
        lo = level_mt + float(lim["lower_rel_deg"]) + margin
        hi = level_mt + float(lim["upper_rel_deg"]) - margin
        start = m.angle_deg()
        emit({"phase": "setup", "level_deg": round(level_mt, 2),
              "start_deg": round(start, 2),
              "window": [round(lo, 1), round(hi, 1)]})

        # --- Phase 1: 0xA2応答確認(速度0 → 動かない) ---
        m.enable()
        reply = m.command_speed(0.0)
        if reply is None:
            m.stop()
            emit({"ok": False, "phase": "a2_check", "a2_supported": False,
                  "note": "0xA2 no reply — fall back to high-rate 0xA4"})
            return 2
        emit({"phase": "a2_check", "a2_supported": True, "reply": reply})
        if args.dry:
            m.stop()
            emit({"ok": True, "note": "dry run — motor not moved"})
            return 0

        sess = SpeedSession(m, lo, hi, args.limit_a)

        # --- Phase 2: 単位判定 — 12.4 dps指令で2秒(ロータ解釈なら出力軸2dps) ---
        cmd = 12.4
        samples = sess.run(cmd, 2.0)
        m.command_speed(0.0)
        meas = slope_dps(samples)  # 0x92はロータ角なので傾きはロータdps
        ratio = meas / cmd if cmd else 0.0
        unit = "rotor" if abs(ratio - 1.0) < 0.3 else \
               ("output_shaft" if abs(ratio - float(lim["gear_ratio"])) < 1.0 else "unknown")
        ticks = [samples[i + 1][0] - samples[i][0] for i in range(len(samples) - 1)]
        emit({"phase": "unit_check", "cmd_dps": cmd,
              "measured_rotor_dps": round(meas, 2), "ratio": round(ratio, 3),
              "unit": unit, "n_samples": len(samples),
              "tick_ms_mean": round(1000 * sum(ticks) / len(ticks), 2),
              "tick_ms_max": round(1000 * max(ticks), 2)})

        # --- Phase 3: ステップ応答 0→+31dps(ロータ) ---
        time.sleep(0.3)
        step = 31.0
        samples = sess.run(step, 1.2)
        t90 = settle_time(samples, step)
        emit({"phase": "step_up", "cmd_dps": step,
              "t90_ms": round(1000 * t90, 1) if t90 is not None else None,
              "measured_rotor_dps": round(slope_dps(samples), 2)})

        # --- Phase 4: 反転 +31→-31dps ---
        samples = sess.run(-step, 1.2)
        m.command_speed(0.0)
        t90 = settle_time(samples, -step)
        emit({"phase": "reversal", "cmd_dps": -step,
              "t90_ms": round(1000 * t90, 1) if t90 is not None else None,
              "measured_rotor_dps": round(slope_dps(samples), 2)})

        # --- 復帰 ---
        pos, peak_a, secs = m.move_to(start, speed_dps=20,
                                      current_limit_a=args.limit_a)
        m.stop()
        emit({"ok": True, "phase": "done", "returned_to": round(pos, 2),
              "final_temp_c": m.status()["temp_c"]})
        return 0

    except RmdGuardTrip as g:
        emit({"ok": False, "guard": str(g), "angle_at_trip": g.angle})
        return 2
    except RmdComError as e:
        try:
            m.command_speed(0.0)
            m.stop()
        except Exception:
            pass
        emit({"ok": False, "error": str(e)})
        return 1
    finally:
        m.close()


if __name__ == "__main__":
    sys.exit(main())
