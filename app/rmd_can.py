"""RMD-X8 Pro SocketCANドライバ(レガシーLK系ファームウェア用)。

rmd-demoセッション(2026-08-02)で実機検証済みのプロトコル:
  - CAN ID = 0x140 + motor_id、1 Mbps、標準フレーム8バイト
  - 0x92 多回転角度読み出し: DATA[1..7] の符号付き56bit LE、0.01 deg/LSB
    (V3ドキュメントのint32 DATA[4..7]ではない)
  - 0x9A 状態1(エラーフラグ u16 DATA[6..7])、0x9C 状態2(温度/電流/速度)
  - 0xA4 絶対位置閉ループ(速度制限付き。制限はソフトで多少オーバーシュートする)
  - 0x88 有効化 / 0x81 停止(通電保持) / 0x80 脱力
標準ライブラリのみ使用(python-can不要)。
"""
import socket
import struct
import time


class RmdComError(RuntimeError):
    """モータから応答が得られない。"""


class RmdGuardTrip(RuntimeError):
    """動作中ガード(過電流・エラーフラグ・タイムアウト)発動。"""

    def __init__(self, reason, angle=None):
        super().__init__(reason)
        self.angle = angle


class RmdMotor:
    def __init__(self, iface="can0", motor_id=1, timeout=0.5):
        self.can_id = 0x140 + motor_id
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((iface,))
        self.sock.settimeout(timeout)
        self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 0)

    def close(self):
        self.sock.close()

    def _xfer(self, cmd, payload=b"\x00" * 7, retries=2):
        """コマンド送信して同一コマンドバイトの応答を返す。応答なしならNone。"""
        for _ in range(retries + 1):
            self.sock.send(struct.pack("=IB3x8s", self.can_id, 8, bytes([cmd]) + payload))
            for _ in range(16):
                try:
                    raw = self.sock.recv(16)
                except socket.timeout:
                    break
                cid, dlc, data = struct.unpack("=IB3x8s", raw)
                if (cid & 0x1FFFFFFF) == self.can_id and data[0] == cmd:
                    return data[:dlc]
        return None

    def _require(self, cmd, payload=b"\x00" * 7):
        d = self._xfer(cmd, payload)
        if d is None:
            raise RmdComError(f"no reply to cmd 0x{cmd:02X}")
        return d

    @staticmethod
    def _s16(d, i):
        v = d[i] | (d[i + 1] << 8)
        return v - 0x10000 if v & 0x8000 else v

    @staticmethod
    def _u16(d, i):
        return d[i] | (d[i + 1] << 8)

    def angle_deg(self):
        """出力軸の多回転角度 [deg]。"""
        d = self._require(0x92)
        v = int.from_bytes(d[1:8], "little")
        if v & (1 << 55):
            v -= 1 << 56
        return v * 0.01

    def status(self):
        """dict(temp_c, current_a, speed_dps, encoder, error)を返す。"""
        d1 = self._require(0x9A)
        d2 = self._require(0x9C)
        return {
            "temp_c": struct.unpack("b", d2[1:2])[0],
            "current_a": self._s16(d2, 2) * 0.01,
            "speed_dps": self._s16(d2, 4),
            "encoder": self._u16(d2, 6),
            "error": self._u16(d1, 6),
        }

    def level_multiturn_from_encoder(self, level_encoder):
        """較正済み単回転エンコーダ値から、現フレームでの水平多回転角を復元する。

        多回転カウンタは電源断で基準を失うが、単回転エンコーダ(65536/回転)は
        絶対値なので、現在位置が水平から±180°(ロータ)以内なら水平を一意に特定できる。
        戻り値: (水平の多回転角, 現在位置-水平 のロータ角差)
        """
        st = self.status()
        m0 = self.angle_deg()
        diff = (st["encoder"] - level_encoder) % 65536
        if diff > 32768:
            diff -= 65536
        delta_deg = diff * 360.0 / 65536.0
        return m0 - delta_deg, delta_deg

    def enable(self):
        self._xfer(0x88)

    def stop(self):
        """停止(位置保持のまま通電継続)。"""
        self._xfer(0x81)

    def release(self):
        """脱力(通電オフ)。センサの自重で動く可能性があるので注意。"""
        self._xfer(0x81)
        self._xfer(0x80)

    def command_speed(self, dps):
        """速度閉ループ指令(0xA2)。0.01 dps/LSB int32。

        指令単位がロータ角か出力軸角かはファーム依存のため実測で判定すること
        (probe_speed.py)。応答は0x9C同等の温度/電流/速度/エンコーダ。
        応答が得られなければNone(本ファームで0xA2非対応の可能性)。
        """
        payload = b"\x00\x00\x00" + struct.pack("<i", int(round(dps * 100)))
        d = self._xfer(0xA2, payload, retries=0)
        if d is None:
            return None
        return {
            "temp_c": struct.unpack("b", d[1:2])[0],
            "current_a": self._s16(d, 2) * 0.01,
            "speed_dps": self._s16(d, 4),
            "encoder": self._u16(d, 6),
        }

    def command_position(self, target_deg, speed_dps):
        """位置指令を送るだけ(待たない)。"""
        counts = int(round(target_deg * 100))
        payload = bytes([0x00]) + struct.pack("<H", int(speed_dps)) + struct.pack("<i", counts)
        self.sock.send(struct.pack("=IB3x8s", self.can_id, 8, bytes([0xA4]) + payload))
        # 0xA4への応答フレームを読み捨てる(次のqueryに混ざらないように)
        try:
            raw = self.sock.recv(16)
            cid, _, data = struct.unpack("=IB3x8s", raw)
            if (cid & 0x1FFFFFFF) != self.can_id or data[0] != 0xA4:
                pass  # 別応答だった場合も_xfer側のドレインで回収される
        except socket.timeout:
            pass

    def move_to(self, target_deg, speed_dps=20, current_limit_a=4.0,
                tol_deg=0.5, timeout_margin_s=3.0, on_sample=None):
        """位置制御で移動し到達まで監視する。

        ガード(過電流/エラーフラグ/未到達タイムアウト)発動時は即0x81停止し
        RmdGuardTripを投げる。呼び出し側で退避を判断する。
        戻り値: (最終角度, ピーク電流A, 所要秒)
        """
        start = self.angle_deg()
        dist = abs(target_deg - start)
        timeout = dist / max(speed_dps, 1) + timeout_margin_s
        self.command_position(target_deg, speed_dps)

        t0 = time.perf_counter()
        peak_a = 0.0
        while time.perf_counter() - t0 < timeout:
            time.sleep(0.02)
            try:
                st = self.status()
            except RmdComError:
                self.stop()
                raise RmdGuardTrip("lost communication during motion")
            peak_a = max(peak_a, abs(st["current_a"]))
            if on_sample:
                on_sample(st)
            if st["error"]:
                self.stop()
                raise RmdGuardTrip(f"error flag 0x{st['error']:04X}", self.angle_deg())
            if abs(st["current_a"]) > current_limit_a:
                self.stop()
                raise RmdGuardTrip(
                    f"over-current {st['current_a']:.2f} A (limit {current_limit_a} A)",
                    self.angle_deg())
            pos = self.angle_deg()
            if abs(pos - target_deg) <= tol_deg:
                return pos, peak_a, time.perf_counter() - t0
        pos = self.angle_deg()
        self.stop()
        raise RmdGuardTrip(f"did not reach target in {timeout:.1f}s (at {pos:+.2f})", pos)
