"""Transport-compatible Pico simulator for serial diagnostics without hardware."""

from collections import deque
from math import isfinite
from threading import Lock
from time import monotonic, sleep

from controle_motores import MOTORS, PROTOCOL_HELLO, load_settings


class PicoMock:
    def __init__(self, *, drop=(), fragment=11, telemetry=False):
        self.port = "Pico simulada"
        self.drop = set(drop)
        self.fragment = fragment
        self.telemetry = telemetry
        self.cfg, _ = load_settings()
        self.published_at = 0.0
        self.encoder = [40.0, 40.0]
        self.references = [40.0, 40.0]
        self.goal = [40.0, 40.0]
        self.centered_at = 0.0
        self.writes = []
        self.buffer = bytearray()
        self.replies = deque()
        self.lock = Lock()
        self.hello = False
        self.link = self.session = self.sequence = self.state = 0
        self.last_target = 0
        self.targets = None
        self.closed = False

    @property
    def in_waiting(self):
        with self.lock:
            return len(self.replies)

    def inject(self, line):
        with self.lock:
            self.replies.extend(line + b"\n")

    def read(self, size):
        if self.closed:
            raise OSError("Pico desconectada")
        now = monotonic()
        if self.state and now - self.last_target > .45:
            self.state = 0
            self.inject(f"!BALLSTATE2,{self.session},2".encode())
        if self.state == 4 and now >= self.centered_at:
            self.state = 1
            self.inject(f"!BALLSTATE2,{self.session},1".encode())
        if self.telemetry and now - self.published_at >= .04:
            self.published_at = now
            self.publish(now)
        with self.lock:
            if self.replies:
                return bytes(self.replies.popleft() for _ in range(min(size, len(self.replies), self.fragment)))
        sleep(.001)
        return b""

    def close(self):
        self.closed = True

    def write(self, payload):
        if self.closed:
            raise OSError("Pico desconectada")
        for byte in payload:
            if byte == 0x18:
                self.buffer.clear()
            elif byte == 10:
                try:
                    self.handle(self.buffer.decode("ascii").split())
                except UnicodeError:
                    self.inject(b"!RXERR,frame")
                self.buffer.clear()
            elif len(self.buffer) < 192:
                self.buffer.append(byte)
        return len(payload)

    def publish(self, now):
        for index, motor in enumerate(MOTORS):
            active = self.state in (1, 4)
            if active:
                self.references[index] += .2 * (self.goal[index] - self.references[index])
            p0 = self.encoder[index]
            if active:
                self.encoder[index] += .2 * (self.references[index] - p0)
            speed = (self.encoder[index] - p0) / .04
            mode = 3 if active else 1 if self.state == 5 else 0
            self.inject((f"@,{motor},{int(now * 1e6)},{mode},0,3,0,0,0,"
                         f"{self.encoder[index]:.4f},{self.goal[index]:.4f},{self.references[index]:.4f},"
                         f"{speed:.4f},100,0,0,0,-1,0,0,0,0,0").encode())

    def settings(self, motor):
        self.inject((f"!CFG,{motor}," + ",".join(f"{value:.7g}" for value in self.cfg[motor])).encode())

    def handle(self, fields):
        if not fields:
            return
        command, *args = fields
        self.writes.append(" ".join(fields))
        if command == "STOP" and not args:
            self.state = 0
            self.inject(b"!OK,STOP")
            self.inject(f"!BALLSTATE2,{self.session},0".encode())
        elif command == "HELLO" and not args:
            self.hello = True
            self.link = self.session = self.sequence = self.state = 0
            self.inject(PROTOCOL_HELLO)
            self.inject(f"!BALLSTATE2,{self.session},0".encode())
            if self.telemetry:
                self.inject(b"!SCALE,0.28125")
                self.inject(b"!LIMITS,1")
                self.inject(b"!HOME,0")
                self.inject(b"!IMUCFG,0,0,1")
                for motor in MOTORS:
                    self.settings(motor)
        elif command == "CFG" and self.telemetry and len(args) == 12:
            motor = int(args[0])
            self.cfg[motor] = tuple(map(float, args[1:]))
            self.settings(motor)
        elif command in ("IMUGET", "IMUCFG") and self.telemetry:
            self.inject(("!IMUCFG," + (",".join(args) if args else "0,0,1")).encode())
        elif command == "BALLLINK2" and len(args) == 1 and self.hello and not self.link:
            self.link = self.read_id(args[0])
            self.inject(f"!BALLLINK2,{self.link}".encode())
        elif command == "BALLSTATUS":
            self.inject(f"!BALLSTATE2,{self.session},{self.state}".encode())
        elif command in ("BALLTEST2", "BALLSTART2"):
            try:
                session = self.read_id(args[0])
                if not self.hello or self.state or session <= self.session or self.read_id(args[1]) != self.link:
                    raise ValueError
                if command == "BALLSTART2":
                    limits = tuple(map(float, args[2:]))
                    if len(limits) != 8 or any(
                        not (1 <= low < neutral < high <= 99 and 0 < pwm <= 100)
                        for low, high, pwm, neutral in (limits[:4], limits[4:])
                    ):
                        raise ValueError
                elif len(args) != 2:
                    raise ValueError
            except (ValueError, IndexError):
                return
            self.session, self.sequence = session, 0
            if command == "BALLTEST2":
                self.state = 5
            elif limits[0] <= self.encoder[0] <= limits[1] and limits[4] <= self.encoder[1] <= limits[5]:
                self.state = 1
                self.goal[:] = self.encoder
            else:
                self.state = 4
                self.goal[:] = limits[3], limits[7]
            self.last_target = monotonic()
            self.centered_at = self.last_target + .25
            self.inject(f"!BALLSTATE2,{session},{self.state}".encode())
        elif command in ("BALL2", "BALLFAST"):
            self.target(args, acknowledge=command == "BALL2")
        elif command == "PING" and not self.hello:
            self.inject(b"!ERR,PING,sem_sessao")

    @staticmethod
    def read_id(value):
        if not value.isdecimal() or not 0 < int(value) < 2**32:
            raise ValueError
        return int(value)

    def target(self, args, acknowledge=True):
        session = sequence = 0
        reason = None
        try:
            session, sequence = map(self.read_id, args[:2])
            m1, m3 = map(float, args[2:])
            if not isfinite(m1) or not isfinite(m3):
                raise ValueError
        except (ValueError, IndexError):
            reason = "argumento_invalido"
        if reason is None:
            if session != self.session or not self.state:
                reason = "sessao_inativa"
            elif sequence <= self.sequence:
                reason = "sequencia_antiga"
            else:
                self.sequence = sequence
                if not (1 <= m1 <= 99 and 1 <= m3 <= 99):
                    reason = "bola_fora_da_faixa"
        if reason:
            self.inject(f"!BALLNACK,{session},{sequence},{reason}".encode())
            return
        self.last_target = monotonic()
        if self.state == 1:
            self.targets = m1, m3
            self.goal[:] = self.targets
        if acknowledge and sequence not in self.drop:
            self.inject(f"!BALLACK,{session},{sequence},{self.state},{m1:.4f},{m3:.4f},0".encode())
