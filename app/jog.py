#!/usr/bin/env python3
"""RMD-X8 Pro ジョグCLI — 干渉限界(上限/下限)を安全に探索するためのツール。

ガード発動時(過電流=何かに接触した可能性)は自動で移動開始位置へ退避する。
出力は1行JSON(機械可読)。

例:
  jog.py status
  jog.py to 15.0          # 絶対角度15度へ(ゆっくり・ガード付き)
  jog.py stop             # 停止(通電保持)
  jog.py release          # 脱力(自重で動く可能性あり)
"""
import argparse
import json
import sys

from rmd_can import RmdMotor, RmdComError, RmdGuardTrip


def emit(obj):
    print(json.dumps(obj, ensure_ascii=False))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--iface", default="can0")
    p.add_argument("--id", type=int, default=1)
    p.add_argument("--speed", type=float, default=15.0, help="速度制限 [deg/s]")
    p.add_argument("--limit-a", type=float, default=4.0, help="過電流ガード [A]")
    p.add_argument("--tol", type=float, default=0.5, help="到達判定 [deg]")
    sub = p.add_subparsers(dest="cmd", required=True)
    p.add_argument("--limits", default="/workspaces/velodyne-rmd/config/limits.yaml")
    sub.add_parser("status")
    sub.add_parser("angle")
    sub.add_parser("enable")
    sub.add_parser("stop")
    sub.add_parser("release")
    sub.add_parser("level")
    st = sub.add_parser("to")
    st.add_argument("target", type=float, help="絶対角度 [deg]")
    args = p.parse_args()

    try:
        m = RmdMotor(args.iface, args.id)
    except OSError as e:
        emit({"ok": False, "error": f"CAN open failed: {e}"})
        return 1

    try:
        if args.cmd == "status":
            st_ = m.status()
            st_["angle_deg"] = round(m.angle_deg(), 2)
            emit({"ok": True, **st_})
        elif args.cmd == "angle":
            emit({"ok": True, "angle_deg": round(m.angle_deg(), 2)})
        elif args.cmd == "enable":
            m.enable()
            emit({"ok": True, "angle_deg": round(m.angle_deg(), 2)})
        elif args.cmd == "stop":
            m.stop()
            emit({"ok": True, "note": "stopped (holding, energized)"})
        elif args.cmd == "release":
            m.release()
            emit({"ok": True, "note": "released (de-energized)"})
        elif args.cmd == "level":
            # 較正済み単回転エンコーダ値から水平を復元して移動する
            import yaml
            with open(args.limits) as f:
                lim = yaml.safe_load(f)
            m.enable()
            level_mt, delta = m.level_multiturn_from_encoder(int(lim["level_encoder"]))
            try:
                pos, peak_a, secs = m.move_to(
                    level_mt, speed_dps=args.speed,
                    current_limit_a=args.limit_a, tol_deg=args.tol)
                emit({"ok": True, "angle_deg": round(pos, 2),
                      "level_deg": round(level_mt, 2),
                      "offset_was": round(delta, 2), "secs": round(secs, 2)})
            except RmdGuardTrip as g:
                m.stop()
                emit({"ok": False, "guard": str(g), "angle_at_trip": g.angle})
                return 2
        elif args.cmd == "to":
            start = m.angle_deg()
            target = args.target
            m.enable()
            try:
                pos, peak_a, secs = m.move_to(
                    target, speed_dps=args.speed,
                    current_limit_a=args.limit_a, tol_deg=args.tol)
                emit({"ok": True, "start_deg": round(start, 2),
                      "angle_deg": round(pos, 2), "target_deg": round(target, 2),
                      "peak_a": round(peak_a, 2), "secs": round(secs, 2)})
            except RmdGuardTrip as g:
                # 接触の可能性 → 移動開始位置へ退避(既知の安全域方向)
                retreat_note = "retreat ok"
                try:
                    m.move_to(start, speed_dps=args.speed,
                              current_limit_a=args.limit_a, tol_deg=args.tol)
                except (RmdGuardTrip, RmdComError):
                    m.stop()
                    retreat_note = "RETREAT FAILED — holding in place"
                emit({"ok": False, "guard": str(g),
                      "angle_at_trip": g.angle,
                      "angle_deg": round(m.angle_deg(), 2),
                      "retreat": retreat_note})
                return 2
    except RmdComError as e:
        emit({"ok": False, "error": str(e)})
        return 1
    finally:
        m.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
