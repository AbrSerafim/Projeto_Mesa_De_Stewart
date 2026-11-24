"""Malha externa de posição da bola e ajustes locais do Tkinter."""

import json
from collections import deque
from math import ceil, floor, isfinite
from pathlib import Path
from time import monotonic
import tkinter as tk
from tkinter import ttk
from typing import NamedTuple

from controle_motores import FAULTS


BALL_SETTINGS = Path(__file__).resolve().with_name("parametros_bola.txt")
LAWS = ("P", "PI", "PID", "PIDF")
HOME_PERCENT = 40.0
MAX_AGE = .32
SEND_PERIOD = .05
COURSE_WINDOW = 8


class AxisConfig(NamedTuple):
    mode: str = "P"
    kp: float = 0
    ki: float = 0
    kd: float = 0
    tf: float = .10
    limit: float = 2
    invert: bool = False
    course_min: float = 35
    course_max: float = 45
    pwm_max: float = 30

    def gains(self):
        return self.kp, self.ki if self.mode != "P" else 0, self.kd if self.mode in ("PID", "PIDF") else 0

    def validate(self):
        if (self.mode not in LAWS or not all(isfinite(x) and 0 <= x <= 1000 for x in self[1:5])
                or not .1 <= self.limit <= 59 or not isinstance(self.invert, bool)):
            raise ValueError("Use ganhos finitos e correção máxima de 0,1 a 59 pontos percentuais.")
        if not (1 <= round(self.course_min, 4) < HOME_PERCENT < round(self.course_max, 4) <= 99):
            raise ValueError("Use curso mínimo ≥ 1%, máximo ≤ 99% e deixe 40% dentro da faixa.")
        if not .1 <= self.pwm_max <= 100:
            raise ValueError("Use PWM máx. de 0,1% a 100%.")
        if self.mode == "PIDF" and self.kd > 0 and self.tf < .001:
            raise ValueError("Tf deve ser de pelo menos 0,001 s no PIDF com Kd ativo.")
        return self


def ball_center(boxes):
    if boxes:
        u0, v0, u1, v1 = max(boxes, key=lambda detection: detection.confidence).bounds
        return (u0 + u1) * .5, (v0 + v1) * .5
    return None


class AxisController:
    def __init__(self, config):
        self.config = config
        self.integral = self.derivative = 0
        self.previous = None
        self.limited = False

    def update(self, goal, measurement, dt):
        cfg = self.config
        sign = -1 if cfg.invert else 1
        error = sign * (goal - measurement)
        kp, ki, kd = cfg.gains()
        if self.previous is not None and dt > 0 and kd:
            tf = cfg.tf if cfg.mode == "PIDF" else 0
            self.derivative = (tf * self.derivative - kd * sign * (measurement - self.previous)) / (tf + dt)
        self.previous = measurement
        base = kp * error + self.derivative
        step = ki * error * dt
        candidate = base + self.integral + step
        low = max(-cfg.limit, round(cfg.course_min, 4) - HOME_PERCENT)
        high = min(cfg.limit, round(cfg.course_max, 4) - HOME_PERCENT)
        if not ((candidate > high and step > 0) or (candidate < low and step < 0)):
            self.integral = min(high, max(low, self.integral + step))
        correction = base + self.integral
        self.limited = correction <= low or correction >= high
        return HOME_PERCENT + min(high, max(low, correction))


class CoursePlot:
    def __init__(self, parent):
        frame = ttk.LabelFrame(parent, text="Alvo do motor (% do curso)", padding=4)
        frame.pack(fill="x", pady=(6, 0))
        self.canvas = tk.Canvas(frame, width=202, height=154, bg="white", highlightthickness=0)
        self.canvas.pack(fill="x")
        self.sent = deque(maxlen=256)
        self.reference = deque(maxlen=512)
        self.changed = True
        self.canvas.bind("<Configure>", self.resize)
        self.draw()

    def resize(self, _event):
        self.draw()

    def add(self, series, at, value):
        series.append((at, value))
        self.changed = True

    def clear(self):
        self.sent.clear()
        self.reference.clear()
        self.changed = True

    def draw(self):
        self.changed = False
        canvas = self.canvas
        canvas.delete("all")
        left, right = 28, max(canvas.winfo_reqwidth(), canvas.winfo_width()) - 9
        top, bottom = 13, 112
        end = max((series[-1][0] for series in (self.sent, self.reference) if series), default=0)
        start = end - COURSE_WINDOW
        values = [value for series in (self.sent, self.reference) for at, value in series if at >= start]
        low = 5 * floor(min(35, *values) / 5) if values else 35
        high = 5 * ceil(max(45, *values) / 5) if values else 45
        scale = (bottom - top) / (high - low)
        marks = (low, 40, high) if high - low <= 25 else (low, high)
        for mark in marks:
            y = bottom - (mark - low) * scale
            canvas.create_line(left, y, right, y, fill="#c5cbd0" if mark == HOME_PERCENT else "#e5e9ec")
            canvas.create_text(left - 5, y, anchor="e", text=str(mark), fill="#384550")
        canvas.create_line(left, top, left, bottom, fill="#c5cbd0")
        canvas.create_text(left, bottom + 12, anchor="w", text=f"−{COURSE_WINDOW} s", fill="#384550")
        canvas.create_text(right, bottom + 12, anchor="e", text="0 s", fill="#384550")
        for series, color, label, legend_x, steps in (
            (self.sent, "#155ec0", "Enviado", 7, True),
            (self.reference, "#a65d05", "Ref. limitada", 91, False),
        ):
            canvas.create_line(legend_x, 143, legend_x + 14, 143, fill=color, width=2)
            canvas.create_text(legend_x + 18, 143, anchor="w", text=label, fill=color)
            points = []
            for at, value in series:
                if at < start:
                    continue
                x = right - (end - at) * (right - left) / COURSE_WINDOW
                y = bottom - (value - low) * scale
                if steps and points:
                    points.extend((x, points[-1]))
                points.extend((x, y))
            if steps and points and points[-2] < right:
                points.extend((right, points[-1]))
            if len(points) >= 4:
                canvas.create_line(*points, fill=color, width=2)
            elif points:
                x, y = points
                canvas.create_oval(x - 2, y - 2, x + 2, y + 2, fill=color, outline="")


class AxisPanel:
    def __init__(self, parent, title, config):
        self.frame = ttk.LabelFrame(parent, text=title, padding=7)
        self.mode = tk.StringVar(value=config.mode)
        self.invert = tk.BooleanVar(value=config.invert)
        self.fields = []
        ttk.Label(self.frame, text="Modo").grid(row=0, column=0, sticky="w")
        ttk.Combobox(self.frame, textvariable=self.mode, values=LAWS, state="readonly",
                     width=7).grid(row=0, column=1, sticky="ew")
        labels = ("Kp", "Ki", "Kd", "Tf (s)", "Correção ± p.p.", "Curso mín. %", "Curso máx. %", "PWM máx. %")
        for row, (label, value) in enumerate(zip(labels, (*config[1:6], *config[7:])), start=1):
            var = tk.StringVar(value=f"{value:g}")
            self.fields.append(var)
            ttk.Label(self.frame, text=label).grid(row=row, column=0, sticky="w", pady=2, padx=(0, 4))
            ttk.Entry(self.frame, textvariable=var, width=8).grid(row=row, column=1, sticky="ew")
        ttk.Checkbutton(self.frame, text="Inverter sentido", variable=self.invert).grid(row=9, columnspan=2, sticky="w")
        self.status = tk.StringVar()
        ttk.Label(self.frame, text="Erro em px · saída em % do curso").grid(row=10, columnspan=2, sticky="w", pady=(7, 0))
        ttk.Label(self.frame, textvariable=self.status).grid(row=11, columnspan=2, sticky="w")
        self.frame.pack(fill="x")
        self.plot = CoursePlot(parent)

    def values(self):
        values = [float(var.get().replace(",", ".")) for var in self.fields]
        return AxisConfig(self.mode.get(), *values[:5], self.invert.get(), *values[5:]).validate()


class BallControl:
    def __init__(self, left, right, toolbar, confidence):
        self.confidence = confidence
        self.config, saved_confidence, warning = self.load()
        self.confidence.set(saved_confidence * 100)
        self.panels = (AxisPanel(left, "Bola X · Motor 3", self.config[0]),
                       AxisPanel(right, "Bola Y · Motor 1", self.config[1]))
        self.send = None
        self.prepare = None
        self.status = tk.StringVar(value=warning)
        self.connection = tk.StringVar(value="Conecte o Pico para liberar a autocorreção.")
        self.toggle_text = tk.StringVar(value="Iniciar autocorreção")
        self.button = ttk.Button(toolbar, textvariable=self.toggle_text, command=self.toggle, state="disabled")
        self.button.pack(side="left", padx=(0, 5))
        self.apply_button = ttk.Button(toolbar, text="Aplicar PID da bola", command=self.apply)
        self.apply_button.pack(side="left", padx=(0, 5))
        self.save_button = ttk.Button(toolbar, text="Salvar PID da bola", command=self.save)
        self.save_button.pack(side="left", padx=(0, 6))
        ttk.Label(toolbar, textvariable=self.status).pack(side="left")
        ttk.Label(toolbar, textvariable=self.connection).pack(side="left", padx=(8, 0))
        self.state = "idle"
        self.supported = False
        self.connected_at = None
        self.compatible = None
        self.status_requested = False
        self.frame_at = None
        self.observation = None
        self.previous_time = None
        self.last_sent_at = None
        self.started_at = 0
        self.start_acknowledged = False
        self.fault_detail = None
        self.controllers = ()
        self.draw_at = 0

    @staticmethod
    def load():
        try:
            data = json.loads(BALL_SETTINGS.read_text(encoding="utf-8"))
            configs = tuple(AxisConfig(**data[axis]).validate() for axis in ("x", "y"))
            confidence = float(data["confidence"])
            if not .01 <= confidence <= .99:
                raise ValueError("Confiança inválida.")
            return configs, confidence, ""
        except FileNotFoundError:
            return (AxisConfig(), AxisConfig()), .20, ""
        except (OSError, ValueError, TypeError, KeyError) as error:
            return (AxisConfig(), AxisConfig()), .20, f"Falha ao ler parâmetros ({error})."

    def refresh_button(self):
        self.button.state(["!disabled"] if self.supported else ["disabled"])
        self.toggle_text.set("Parar autocorreção" if self.state != "idle" else "Iniciar autocorreção")

    def apply(self):
        if self.state != "idle":
            self.status.set("Pare a autocorreção para aplicar outros ganhos.")
            return False
        try:
            self.config = tuple(panel.values() for panel in self.panels)
        except (ValueError, tk.TclError) as error:
            self.status.set(f"Parâmetros da bola inválidos ({error}).")
            return False
        self.status.set("Ajustes aplicados")
        return True

    def save(self):
        if not self.apply():
            return
        settings = dict(zip(("x", "y"), (config._asdict() for config in self.config)))
        settings["confidence"] = self.confidence.get() / 100
        temporary = BALL_SETTINGS.with_suffix(".tmp")
        try:
            temporary.write_text(json.dumps(settings, indent=2) + "\n", encoding="utf-8")
            temporary.replace(BALL_SETTINGS)
        except OSError as error:
            self.status.set(f"Falha ao salvar ({error}).")
            return
        self.status.set("Salvo em parametros_bola.txt")

    def toggle(self):
        if self.state != "idle":
            self.stop("Autocorreção parada")
            return
        if not self.supported or self.prepare is None:
            return
        error = self.prepare()
        if error or self.frame_at is None or monotonic() - self.frame_at > MAX_AGE:
            self.status.set(error or "Aguarde a análise de um quadro da câmera.")
            return
        if not self.apply():
            return
        self.reset_controllers()
        self.last_sent_at = None
        self.start_acknowledged = False
        self.fault_detail = None
        for panel in self.panels:
            panel.plot.clear()
        self.state = "pending"
        self.started_at = monotonic()
        limits = (value for cfg in reversed(self.config) for value in (cfg.course_min, cfg.course_max, cfg.pwm_max))
        self.send("BALLSTART " + " ".join(f"{value:.4f}" for value in limits))
        self.status.set("Aguardando confirmação do Pico")
        self.refresh_button()

    def stop(self, reason, send=True):
        was_active = self.state != "idle"
        self.state = "idle"
        if was_active:
            if send and self.send is not None:
                self.send("STOP")
            self.status.set(reason)
        self.refresh_button()

    def disconnect(self):
        self.supported = False
        self.connected_at = None
        self.compatible = None
        self.status_requested = False
        self.frame_at = None
        self.observation = None
        self.fault_detail = None
        self.connection.set("Conecte o Pico para liberar a autocorreção.")
        self.stop("Conexão interrompida", send=False)

    def connected(self, now, compatible=None):
        if self.connected_at is not None and self.compatible is compatible and not self.supported:
            return
        self.disconnect()
        self.connected_at = now
        self.compatible = compatible
        if compatible is False:
            self.connection.set("Firmware do Pico desatualizado para autocorreção.")
        else:
            self.connection.set("Aguardando confirmação da autocorreção pelo Pico.")

    def check_connection(self, now):
        if self.supported or self.connected_at is None or self.compatible is False:
            return
        elapsed = now - self.connected_at
        if elapsed >= 1 and self.compatible and not self.status_requested:
            self.status_requested = True
            self.send("BALLSTATUS")
        if elapsed >= 2:
            self.connection.set("Pico sem confirmação da autocorreção. Grave o UF2 atualizado e reconecte.")

    def reset_controllers(self):
        self.controllers = tuple(AxisController(config) for config in self.config)
        self.previous_time = None

    def record_sent(self, at, m1, m3):
        if self.started_at and at >= self.started_at:
            for panel, value in zip(self.panels, (m3, m1)):
                panel.plot.add(panel.plot.sent, at, value)

    def record_reference(self, at, sample):
        if self.state in ("centering", "running") and sample.mode == 3 and sample.calibrated == 3 and sample.span > 0:
            plot = self.panels[0 if sample.motor == 3 else 1].plot
            plot.add(plot.reference, at, 100 * sample.reference / sample.span)

    def receive_fault(self, line):
        try:
            _, motor, fault, position, target, duty, speed, threshold = line.split(b",")
            motor, fault = int(motor), int(fault)
            position, target, duty, speed, threshold = map(float, (position, target, duty, speed, threshold))
        except ValueError:
            return
        if (motor not in (1, 3) or not 0 < fault < len(FAULTS)
                or not all(map(isfinite, (position, target, duty, speed, threshold)))):
            return
        reason = FAULTS[fault]
        if threshold > 0:
            reason += f" ({speed:.1f} > {threshold:.1f} °/s)"
        self.fault_detail = (f"M{motor} · {reason} · posição {position:.1f}% → alvo {target:.1f}%"
                             f" · PWM {duty:+.1f}%")

    def receive(self, line):
        if self.compatible is False:
            return
        states = {b"!BALL,0": "Autocorreção parada no Pico", b"!BALL,2": "Pico parou por falta de comandos",
                  b"!BALL,3": "Pico parou por falha ou limite de curso"}
        started = self.state in ("running", "centering") or (self.state == "pending" and self.start_acknowledged)
        if line == b"!OK,BALLSTART":
            self.start_acknowledged = self.state == "pending"
        elif line.startswith(b"!BALLF,"):
            if started:
                self.receive_fault(line)
            return
        elif line in (b"!BALL,1", b"!BALL,4"):
            self.supported = True
            if self.state == "idle" and self.send is not None:
                self.send("STOP")
            elif started:
                self.state = "centering" if line == b"!BALL,4" else "running"
        elif line in states:
            self.supported = True
            if started:
                detail = self.fault_detail if line == b"!BALL,3" else None
                self.stop(detail or states[line], send=False)
        elif line.startswith(b"!ERR,BALLSTATUS,"):
            if not self.supported:
                self.connection.set("Firmware do Pico sem o comando de autocorreção. Grave o UF2 atualizado.")
        elif line.startswith((b"!ERR,BALLSTART,", b"!ERR,BALL,")):
            reason = line.rsplit(b",", 1)[-1].decode("ascii", errors="replace")
            if line.startswith(b"!ERR,BALL,") and reason in ("bola_fora_da_faixa", "bola_inativa"):
                # The Pico follows with a terminal state and any associated motor fault.
                return
            error = {
                "bola_requer_home": "Leve a mesa para Home (40%) antes de iniciar.",
                "bola_fora_do_curso": "Motor fora do curso calibrado.",
                "freio_necessario": "Aguarde os motores pararem antes de iniciar.",
                "home_requer_curso": "Calibre o curso dos dois motores.",
                "home_requer_encoder": "Aguarde as leituras dos encoders.",
                "faixa_bola_invalida": "Ajuste o curso mínimo e máximo dos dois motores em torno de 40%.",
                "pwm_bola_invalido": "Ajuste o PWM máximo da autocorreção entre 0,1% e 100%.",
            }.get(reason, reason.replace("_", " "))
            self.stop(f"Pico recusou a autocorreção ({error}).", send=self.state in ("running", "centering"))
        elif line == b"!OK,STOP":
            self.stop("Autocorreção parada", send=False)
        else:
            return
        if self.supported:
            self.connection.set("Autocorreção disponível no Pico")
        self.refresh_button()

    def update(self, now, result, setpoint):
        if now >= self.draw_at:
            for panel in self.panels:
                if panel.plot.changed:
                    panel.plot.draw()
            self.draw_at = now + .1
        if result is None:
            self.frame_at = self.observation = None
        elif self.frame_at != result.captured_at:
            self.frame_at = result.captured_at
            self.observation = None
            center = ball_center(result.boxes)
            if center is not None:
                height, width = result.frame.shape[:2]
                self.observation = (center[0] * width, center[1] * height, width, height)
        if self.state == "idle":
            return
        if self.frame_at is None or now - self.frame_at > MAX_AGE:
            self.stop("Autocorreção parada · sem quadros recentes da detecção")
            return
        if setpoint is None:
            self.stop("Autocorreção parada · setpoint removido")
            return
        if self.state == "pending":
            if now - self.started_at > .40:
                self.stop("O Pico não confirmou a autocorreção")
            return
        neutral = self.state == "centering" or self.observation is None
        stopped_tracking = neutral and self.previous_time is not None
        if not stopped_tracking and self.last_sent_at is not None and self.frame_at - self.last_sent_at < SEND_PERIOD:
            return
        self.last_sent_at = self.frame_at
        if neutral:
            if stopped_tracking:
                self.reset_controllers()
            self.status.set("Indo para 40% nos dois motores" if self.state == "centering"
                            else "Sem bola detectada · alvo de 40% nos dois motores")
            for panel in self.panels:
                panel.status.set("Alvo 40,00%")
            self.send(f"BALL {HOME_PERCENT:g} {HOME_PERCENT:g}")
            return
        self.status.set("Autocorreção ativa")
        timestamp = self.frame_at
        x, y, width, height = self.observation
        dt = 0 if self.previous_time is None else timestamp - self.previous_time
        self.previous_time = timestamp
        goals = (setpoint[0] * width, setpoint[1] * height)
        outputs = []
        for panel, controller, goal, measured in zip(self.panels, self.controllers, goals, (x, y)):
            command = controller.update(goal, measured, dt)
            limited = " · no limite" if controller.limited else ""
            panel.status.set(f"Erro {goal - measured:+.1f} px\nAlvo {command:.2f}%{limited}")
            outputs.append(command)
        self.send(f"BALL {outputs[1]:.4f} {outputs[0]:.4f}")
