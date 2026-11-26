"""Malha externa de posição da bola e ajustes locais do Tkinter."""

import csv
import json
from collections import deque
from math import ceil, copysign, floor, isfinite
from pathlib import Path
from threading import Event, Thread
from time import monotonic
from secrets import randbelow
import tkinter as tk
from tkinter import filedialog, ttk
from typing import NamedTuple

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from controle_motores import FAULTS


BALL_SETTINGS = Path(__file__).resolve().with_name("parametros_bola.txt")
LAWS = ("P", "PI", "PID", "PIDF")
HOME_PERCENT = 40.0
MAX_AGE = .32
MIN_DT = .001
MAX_DT = .32
SEND_PERIOD = .05
START_TIMEOUT = .40
COURSE_WINDOW = 8
MIN_CORRECTION_START_PX = 3
MIN_CORRECTION_FULL_PX = 8


class AxisConfig(NamedTuple):
    mode: str = "PIDF"
    kp: float = .02
    ki: float = 0
    kd: float = 0
    tf: float = .10
    limit: float = 2
    invert: bool = False
    course_min: float = 35
    course_max: float = 45
    pwm_max: float = 45
    correction_min: float = 0

    def gains(self):
        return self.kp, self.ki if self.mode != "P" else 0, self.kd if self.mode in ("PID", "PIDF") else 0

    def validate(self):
        if (self.mode not in LAWS or not all(isfinite(x) and 0 <= x <= 1000 for x in self[1:5])
                or not .1 <= self.limit <= 59 or not isinstance(self.invert, bool)):
            raise ValueError("Use ganhos finitos e correção máxima de 0,1 a 59 pontos percentuais.")
        if not (1 <= round(self.course_min, 4) < round(self.course_max, 4) <= 99):
            raise ValueError("Use curso mínimo ≥ 1%, máximo ≤ 99%.")
        if not .1 <= self.pwm_max <= 100:
            raise ValueError("Use PWM máx. de 0,1% a 100%.")
        if not 0 <= self.correction_min <= self.limit:
            raise ValueError("Use uma correção mínima entre zero e a correção máxima.")
        if self.mode == "PIDF" and self.kd > 0 and self.tf < .001:
            raise ValueError("Tf deve ser de pelo menos 0,001 s no PIDF com Kd ativo.")
        return self


def ball_center(boxes):
    if boxes:
        u0, v0, u1, v1 = max(boxes, key=lambda detection: detection.confidence).bounds
        return (u0 + u1) * .5, (v0 + v1) * .5
    return None


class AxisTerms(NamedTuple):
    error_px: float
    p: float
    i: float
    d: float
    raw: float
    saturated: float


class AxisController:
    """Pixels na entrada, pontos percentuais na saída."""

    def __init__(self, config):
        self.config = config.validate()
        self.reset()

    def reset(self):
        self.integral = self.derivative = 0.0
        self.previous = None

    def prepare(self, goal, measurement, dt):
        cfg = self.config
        sign = -1 if cfg.invert else 1
        error = goal - measurement
        kp, ki, kd = cfg.gains()
        if not isfinite(dt) or dt > MAX_DT:
            self.reset()
        step = 0.0
        if self.previous is None or dt >= MIN_DT or not isfinite(dt):
            if self.previous is not None:
                tf = cfg.tf if cfg.mode == "PIDF" else 0.0
                self.derivative = (tf * self.derivative - kd * sign * (measurement - self.previous)) / (tf + dt)
                step = ki * sign * error * dt
            self.previous = measurement
        return error, kp * sign * error, step

    def output(self, proportional, integral, error):
        cfg = self.config
        raw = proportional + integral + self.derivative
        # Apply optional breakaway only for P, away from the setpoint.
        correction = raw
        if cfg.mode == "P" and raw:
            ratio = min(1, max(0, (abs(error) - MIN_CORRECTION_START_PX)
                               / (MIN_CORRECTION_FULL_PX - MIN_CORRECTION_START_PX)))
            correction = copysign(max(abs(raw), cfg.correction_min * ratio), raw)
        return raw, clip(correction, -cfg.limit, cfg.limit)


def clip(value, low, high):
    return min(high, max(low, value))


class BallGeometry(NamedTuple):
    matrix: tuple = ((0.0, 1.0), (1.0, 0.0))
    neutral: tuple = (40.0, 40.0)
    loss_timeout_s: float = MAX_AGE
    ack_timeout_s: float = .15
    loss_behavior: str = "neutral_then_stop"

    def validate(self, configs):
        if (len(self.matrix) != 2 or any(len(row) != 2 for row in self.matrix)
                or any(not isfinite(value) or abs(value) > 100 for row in self.matrix for value in row)
                or not .05 <= self.loss_timeout_s <= MAX_AGE or not .02 <= self.ack_timeout_s <= .30
                or self.loss_behavior not in ("neutral_then_stop", "stop_on_missing") or len(self.neutral) != 2):
            raise ValueError("Matriz, neutros ou tempos de perda/ACK inválidos.")
        for neutral, cfg in zip(self.neutral, reversed(configs)):
            if not round(cfg.course_min, 4) < round(neutral, 4) < round(cfg.course_max, 4):
                raise ValueError("Cada neutro precisa ficar dentro da faixa do motor.")
        return self

    def mix(self, corrections):
        return tuple(neutral + sum(weight * value for weight, value in zip(row, corrections))
                     for neutral, row in zip(self.neutral, self.matrix))


class BallRegulator:
    def __init__(self, configs, geometry):
        self.geometry = geometry.validate(configs)
        self.axes = tuple(AxisController(cfg) for cfg in configs)
        self.bounds = tuple((round(cfg.course_min, 4), round(cfg.course_max, 4)) for cfg in reversed(configs))
        self.last_at = None

    def reset(self):
        for axis in self.axes:
            axis.reset()

    def update(self, captured_at, goal, measurement):
        if not isfinite(captured_at) or (self.last_at is not None and captured_at <= self.last_at):
            return None
        dt = 0 if self.last_at is None else captured_at - self.last_at
        self.last_at = captured_at
        if measurement is None or not all(map(isfinite, (*goal, *measurement))):
            self.reset()
            return None
        prepared = [axis.prepare(r, y, dt) for axis, r, y in zip(self.axes, goal, measurement)]
        integrals = [axis.integral for axis in self.axes]
        proposed = [clip(i + step, -axis.config.limit, axis.config.limit)
                    for axis, i, (_, _, step) in zip(self.axes, integrals, prepared)]

        def axis_outputs(values):
            return [axis.output(p, i, error) for axis, i, (error, p, _) in zip(self.axes, values, prepared)]

        candidate = axis_outputs(proposed)
        mixed = self.geometry.mix([saturated for _, saturated in candidate])
        for index, ((_, _, step), (raw, saturated)) in enumerate(zip(prepared, candidate)):
            if (raw - saturated) * step > 0 or any(
                (target - clip(target, *bounds)) * row[index] * step > 0
                for target, bounds, row in zip(mixed, self.bounds, self.geometry.matrix)
            ):
                proposed[index] = integrals[index]

        outputs = axis_outputs(proposed)
        terms = []
        for axis, integral, (error, p, _), (raw, saturated) in zip(self.axes, proposed, prepared, outputs):
            axis.integral = integral
            terms.append(AxisTerms(error, p, integral, axis.derivative, raw, saturated))
        targets = self.geometry.mix([saturated for _, saturated in outputs])
        return tuple(clip(target, *bounds) for target, bounds in zip(targets, self.bounds)), tuple(terms)


class BallTracking(Thread):
    """Consumes fresh YOLO results independently of Tk rendering."""

    def __init__(self, source, setpoint, regulator, session, send):
        super().__init__(name="ball-control", daemon=True)
        self.source = source
        self.setpoint = setpoint
        self.regulator = regulator
        self.session = session
        self.send = send
        self.running = Event()
        self.stopped = Event()
        self.samples = deque(maxlen=256)
        self.failure = None

    def run(self):
        last_capture = last_command = None
        last_seen = monotonic()
        neutral = self.regulator.geometry.neutral
        while not self.stopped.wait(.005):
            now = monotonic()
            result = self.source()
            fresh = result is not None and 0 <= now - result.captured_at <= MAX_AGE
            if fresh and (last_capture is None or result.captured_at > last_capture):
                center = ball_center(result.boxes)
                if center is not None:
                    last_seen = result.captured_at
            missing = fresh and not result.boxes and self.regulator.geometry.loss_behavior == "stop_on_missing"
            if missing or now - last_seen > self.regulator.geometry.loss_timeout_s:
                self.failure = "Autocorreção parada · tempo sem detecção da bola excedido"
                if not self.stopped.is_set():
                    self.send(f"BALLSTOP {self.session}")
                return
            if not fresh or (last_capture is not None and result.captured_at <= last_capture):
                continue
            if last_command is not None and now - last_command < SEND_PERIOD:
                if self.running.is_set() and center is None:
                    self.regulator.reset()
                continue
            last_command = now
            last_capture = result.captured_at
            targets, terms = neutral, ()
            if self.running.is_set():
                height, width = result.frame.shape[:2]
                goal = self.setpoint[0] * width, self.setpoint[1] * height
                measured = None if center is None else (center[0] * width, center[1] * height)
                calculated = self.regulator.update(last_capture, goal, measured)
                if calculated is not None:
                    targets, terms = calculated
            end = monotonic()
            if self.stopped.is_set():
                return
            sequence = self.send(f"TARGET {self.session} {targets[0]:.4f} {targets[1]:.4f}")
            self.samples.append((last_capture, end, sequence, targets, terms, result.duration_ms, end - now))


class CoursePlot:
    def __init__(self, parent, motor):
        frame = ttk.LabelFrame(parent, text=f"M{motor} (% do curso) · tempo no PC", padding=4)
        frame.pack(fill="x", pady=(6, 0))
        self.canvas = tk.Canvas(frame, width=202, height=176, bg="white", highlightthickness=0)
        self.canvas.pack(fill="x")
        self.calculated = deque(maxlen=256)
        self.accepted = deque(maxlen=256)
        self.measured = deque(maxlen=512)
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
        for series in self.series:
            series.clear()
        self.changed = True

    @property
    def series(self):
        return self.calculated, self.accepted, self.reference, self.measured

    def draw(self):
        self.changed = False
        canvas = self.canvas
        canvas.delete("all")
        left, right = 28, max(canvas.winfo_reqwidth(), canvas.winfo_width()) - 9
        top, bottom = 13, 112
        end = max((series[-1][0] for series in self.series if series), default=0)
        start = end - COURSE_WINDOW
        values = [value for series in self.series for at, value in series if at >= start]
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
        for series, color, label, legend_x, legend_y, steps in (
            (self.calculated, "#155ec0", "Calculado", 7, 143, True),
            (self.accepted, "#7642a8", "Confirmado", 103, 143, True),
            (self.reference, "#a65d05", "Ref. interna", 7, 160, False),
            (self.measured, "#00734a", "Encoder", 103, 160, False),
        ):
            canvas.create_line(legend_x, legend_y, legend_x + 14, legend_y, fill=color, width=2)
            canvas.create_text(legend_x + 18, legend_y, anchor="w", text=label, fill=color)
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
    def __init__(self, parent, title, config, motor):
        self.frame = ttk.LabelFrame(parent, text=title, padding=7)
        self.mode = tk.StringVar(value=config.mode)
        self.invert = tk.BooleanVar(value=config.invert)
        self.fields = []
        ttk.Label(self.frame, text="Modo").grid(row=0, column=0, sticky="w")
        ttk.Combobox(self.frame, textvariable=self.mode, values=LAWS, state="readonly",
                     width=7).grid(row=0, column=1, sticky="ew")
        labels = ("Kp (p.p./px)", "Ki (p.p./(px·s))", "Kd (p.p.·s/px)", "Tf (s)", "Correção ± p.p.", "Curso mín. %", "Curso máx. %", "PWM máx. %",
                  "Correção mín. p.p.")
        for row, (label, value) in enumerate(zip(labels, (*config[1:6], *config[7:])), start=1):
            var = tk.StringVar(value=f"{value:g}")
            self.fields.append(var)
            ttk.Label(self.frame, text=label).grid(row=row, column=0, sticky="w", pady=2, padx=(0, 4))
            ttk.Entry(self.frame, textvariable=var, width=8).grid(row=row, column=1, sticky="ew")
        ttk.Checkbutton(self.frame, text="Inverter sentido", variable=self.invert).grid(row=10, columnspan=2, sticky="w")
        self.status = tk.StringVar()
        ttk.Label(self.frame, text="Erro em px · saída em % do curso").grid(row=11, columnspan=2, sticky="w", pady=(7, 0))
        ttk.Label(self.frame, textvariable=self.status).grid(row=12, columnspan=2, sticky="w")
        self.frame.pack(fill="x")
        self.plot = CoursePlot(parent, motor)

    def values(self):
        values = [float(var.get().replace(",", ".")) for var in self.fields]
        return AxisConfig(self.mode.get(), *values[:5], self.invert.get(), *values[5:]).validate()


class BallControl:
    def __init__(self, left, right, toolbar, confidence):
        self.confidence = confidence
        self.config, self.geometry, saved_confidence, warning = self.load()
        self.confidence.set(saved_confidence * 100)
        self.panels = (AxisPanel(left, "Bola X · limites M3", self.config[0], 3),
                       AxisPanel(right, "Bola Y · limites M1", self.config[1], 1))
        self.send = None
        self.prepare = None
        self.status = tk.StringVar(value=warning)
        self.diagnostics = tk.StringVar()
        toolbar.master.after(100, self.refresh_diagnostics)
        self.log = deque(maxlen=60000)
        self.events = deque(maxlen=2048)
        self.term_history = deque(maxlen=256)
        self.term_window = None
        self.term_draw_at = 0.0
        self.counts = {}
        self.session = randbelow(2**31)
        self.worker = None
        self.source = lambda: None
        self.setpoint = None
        self.level_neutral = lambda: None
        self.latest_control = None
        self.camera_fps = self.yolo_fps = 0.0
        self.connection = tk.StringVar(value="Conecte o Pico para liberar a autocorreção.")
        self.toggle_text = tk.StringVar(value="Iniciar autocorreção")
        self.button = ttk.Button(toolbar, textvariable=self.toggle_text, command=self.toggle, state="disabled")
        self.button.pack(side="left", padx=(0, 5))
        self.apply_button = ttk.Button(toolbar, text="Aplicar PID da bola", command=self.apply)
        self.apply_button.pack(side="left", padx=(0, 5))
        self.save_button = ttk.Button(toolbar, text="Salvar PID da bola", command=self.save)
        self.save_button.pack(side="left", padx=(0, 6))
        ttk.Button(toolbar, text="Usar nivelamento", command=self.use_level_neutral).pack(side="left", padx=(0, 5))
        ttk.Button(toolbar, text="Rearmar", command=self.rearm).pack(side="left", padx=(0, 5))
        ttk.Button(toolbar, text="Curvas PID", command=self.show_terms).pack(side="left", padx=(0, 5))
        ttk.Button(toolbar, text="Exportar CSV", command=self.export).pack(side="left")
        details = ttk.Frame(toolbar.master, padding=(8, 0, 8, 6))
        details.pack(fill="x", after=toolbar)
        ttk.Label(details, textvariable=self.status).pack(side="left")
        ttk.Label(details, textvariable=self.connection).pack(side="right")
        ttk.Label(toolbar.master, textvariable=self.diagnostics, padding=(8, 0, 8, 4)).pack(anchor="w", after=details)
        self.state = "idle"
        self.supported = False
        self.connected_at = None
        self.compatible = None
        self.status_requested = False
        self.frame_at = None
        self.started_at = 0
        self.fault_detail = None

    @staticmethod
    def load():
        try:
            data = json.loads(BALL_SETTINGS.read_text(encoding="utf-8"))
            configs = tuple(AxisConfig(**data[axis]).validate() for axis in ("x", "y"))
            confidence = float(data["confidence"])
            if not .01 <= confidence <= .99:
                raise ValueError("Confiança inválida.")
            geometry = BallGeometry(tuple(tuple(row) for row in data.get("matrix", BallGeometry().matrix)),
                                    tuple(data.get("neutral", BallGeometry().neutral)),
                                    float(data.get("loss_timeout_s", MAX_AGE)),
                                    float(data.get("ack_timeout_s", .15)),
                                    data.get("loss_behavior", "neutral_then_stop")).validate(configs)
            return configs, geometry, confidence, ""
        except FileNotFoundError:
            return (AxisConfig(), AxisConfig()), BallGeometry(), .20, ""
        except (OSError, ValueError, TypeError, KeyError) as error:
            return (AxisConfig(), AxisConfig()), BallGeometry(), .20, f"Falha ao ler parâmetros ({error})."

    def refresh_button(self):
        self.button.state(["!disabled"] if self.supported else ["disabled"])
        self.toggle_text.set("Parar autocorreção" if self.state != "idle" else "Iniciar autocorreção")

    def apply(self):
        if self.state != "idle":
            self.status.set("Pare a autocorreção para aplicar outros ganhos.")
            return False
        try:
            configs = tuple(panel.values() for panel in self.panels)
            self.geometry.validate(configs)
            self.config = configs
        except (ValueError, tk.TclError) as error:
            self.status.set(f"Parâmetros da bola inválidos ({error}).")
            return False
        self.status.set("Ajustes aplicados")
        return True

    def save(self):
        if not self.apply():
            return
        settings = dict(zip(("x", "y"), (config._asdict() for config in self.config)))
        settings.update(self.geometry._asdict())
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
        for panel in self.panels:
            panel.plot.clear()
        self.events.clear()
        self.term_history.clear()
        self.log.clear()
        self.counts.clear()
        self.latest_control = None
        self.fault_detail = None
        self.session += 1
        self.state = "pending"
        self.started_at = monotonic()
        limits = (value for cfg, neutral in zip(reversed(self.config), self.geometry.neutral)
                  for value in (cfg.course_min, cfg.course_max, cfg.pwm_max, neutral))
        self.send(f"BALLSTART2 {self.session} " + " ".join(f"{value:.4f}" for value in limits))
        self.worker = BallTracking(self.source, self.setpoint, BallRegulator(self.config, self.geometry),
                                   self.session, self.send)
        self.status.set("Aguardando confirmação do Pico")
        self.refresh_button()

    def stop(self, reason, send=True):
        was_active = self.state != "idle"
        self.state = "idle"
        if self.worker is not None:
            self.worker.stopped.set()
            self.drain_control()
            self.worker = None
        if was_active:
            if send and self.send is not None:
                self.send(f"BALLSTOP {self.session}")
            self.status.set(reason)
        self.refresh_button()

    def close(self):
        if self.worker is not None:
            self.worker.stopped.set()

    def disconnect(self):
        self.supported = False
        self.connected_at = None
        self.compatible = None
        self.status_requested = False
        self.frame_at = None
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

    def rearm(self):
        if self.supported and self.state == "idle":
            self.send("REARM")

    def use_level_neutral(self):
        if self.state != "idle":
            return
        neutral = self.level_neutral()
        if neutral is None:
            self.status.set("Conclua o nivelamento com a IMU antes de capturar os neutros.")
            return
        try:
            geometry = self.geometry._replace(neutral=neutral).validate(tuple(panel.values() for panel in self.panels))
        except (ValueError, tk.TclError) as error:
            self.status.set(str(error))
            return
        self.geometry = geometry
        self.status.set(f"Neutros M1 {neutral[0]:.2f}% · M3 {neutral[1]:.2f}% · use Salvar PID para persistir")

    def show_terms(self):
        if self.term_window is not None and self.term_window.winfo_exists():
            self.term_window.lift()
            return
        self.term_window = tk.Toplevel(self.button)
        self.term_window.title("Controle da bola · X e Y")
        figure = Figure(figsize=(8, 5), layout="constrained")
        self.term_plots = []
        for name, axis in zip(("X", "Y"), figure.subplots(2, 1, sharex=True)):
            axis.set(ylabel=f"{name} · p.p.")
            error_axis = axis.twinx()
            error_axis.set(ylabel="Erro (px)")
            error, = error_axis.plot([], [], color="#475569", alpha=.65, label="Erro")
            traces = [axis.plot([], [], label=label, linestyle=style)[0]
                      for label, style in (("P", "-"), ("I", "-"), ("D", "-"), ("Bruta", "--"), ("Saturada", ":"))]
            axis.legend(handles=[error, *traces], loc="upper left", ncol=3, fontsize=8)
            axis.grid(alpha=.3)
            self.term_plots.append((axis, error_axis, [error, *traces]))
        axis.set_xlabel("Tempo desde a observação mais recente (s)")
        self.term_canvas = FigureCanvasTkAgg(figure, master=self.term_window)
        self.term_canvas.get_tk_widget().pack(fill="both", expand=True)
        self.draw_terms()

    def draw_terms(self):
        samples = list(self.term_history)
        latest = samples[-1][0] if samples else 0.0
        samples = [(at, terms) for at, terms in samples if latest - at <= COURSE_WINDOW]
        times = [at - latest for at, _ in samples]
        for index, (axis, error_axis, traces) in enumerate(self.term_plots):
            for term, trace in enumerate(traces):
                trace.set_data(times, [terms[index][term] if terms else float("nan") for _, terms in samples])
            axis.set_xlim(-COURSE_WINDOW, 0)
            for view in (axis, error_axis):
                view.relim()
                view.autoscale_view(scalex=False)
        self.term_canvas.draw_idle()

    def export(self):
        path = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV", "*.csv")])
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8", newline="") as output:
                writer = csv.writer(output)
                writer.writerow(("pc_monotonic_s", "session", "sequence", "event", "m1_pct", "m3_pct",
                                 "rtt_s", "pico_mutex_us", "pico_sample_us", "capture_s", "inference_ms",
                                 "command_generation_s", "x_error_px", "x_p_pp", "x_i_pp", "x_d_pp",
                                 "x_raw_pp", "x_sat_pp", "y_error_px", "y_p_pp", "y_i_pp", "y_d_pp",
                                 "y_raw_pp", "y_sat_pp"))
                writer.writerows(self.log)
        except OSError as error:
            self.status.set(f"Falha ao exportar CSV ({error}).")

    def record_event(self, event):
        if event.session != self.session and event.outcome != "invalid_reply":
            return
        self.log.append(event)
        self.events.append(event)
        key = event.outcome.split(":", 1)[0]
        self.counts[key] = self.counts.get(key, 0) + 1
        if event.outcome == "applied":
            for panel, value in zip(self.panels, (event.m3, event.m1)):
                panel.plot.add(panel.plot.accepted, event.at, value)

    def record_reference(self, at, sample):
        if self.state in ("centering", "running") and sample.mode == 3 and sample.calibrated == 3 and sample.span > 0:
            plot = self.panels[0 if sample.motor == 3 else 1].plot
            reference, position = (100 * value / sample.span for value in (sample.reference, sample.angle))
            plot.add(plot.reference, at, reference)
            plot.add(plot.measured, at, position)
            for kind, value in (("reference", reference), ("encoder", position)):
                self.log.append((at, self.session, None, kind, value if sample.motor == 1 else None,
                                 value if sample.motor == 3 else None, None, None, sample.timestamp))

    def drain_control(self):
        for _ in range(len(self.worker.samples)):
            captured, end, sequence, targets, terms, inference, generation = self.worker.samples.popleft()
            self.latest_control = captured, end, inference, generation
            self.term_history.append((captured, terms))
            self.log.append((end, self.session, sequence, "calculated", *targets, None, None, None,
                             captured, inference, generation, *(value for term in terms for value in term)))
            for panel, target in zip(self.panels, reversed(targets)):
                panel.plot.add(panel.plot.calculated, end, target)
                panel.status.set(f"Alvo {target:.2f}% · neutro")
            for panel, term, target in zip(self.panels, terms, reversed(targets)):
                panel.status.set(f"Erro {term.error_px:+.1f} px · alvo {target:.2f}%\n"
                                 f"P {term.p:+.2f} · I {term.i:+.2f} · D {term.d:+.2f} p.p.\n"
                                 f"Bruta {term.raw:+.2f} → {term.saturated:+.2f} p.p.")

    def refresh_diagnostics(self):
        if self.worker is not None:
            self.drain_control()
            if self.worker.failure:
                self.stop(self.worker.failure, send=False)
        if self.state == "pending" and monotonic() - self.started_at > START_TIMEOUT:
            self.stop("O Pico não confirmou a autocorreção")
        for panel in self.panels:
            if panel.plot.changed:
                panel.plot.draw()
        now = monotonic()
        if self.term_window is not None and self.term_window.winfo_exists() and now >= self.term_draw_at:
            self.draw_terms()
            self.term_draw_at = now + .25
        recent = [event for event in self.events if now - event.at <= 2]
        tx = sum(event.outcome == "sent" for event in recent) / 2
        rx = sum(event.rtt_s is not None for event in recent) / 2
        last_ack = next((event for event in reversed(self.events) if event.outcome == "applied"), None)
        rates = f"Câmera {self.camera_fps:.1f} FPS · YOLO {self.yolo_fps:.1f} FPS · TX {tx:.1f} Hz · ACK {rx:.1f} Hz"
        if last_ack:
            rates += f" · ACK #{last_ack.sequence} · RTT {last_ack.rtt_s * 1000:.1f} ms"
        rates += (f" · timeout {self.counts.get('timeout', 0)} · substituídos {self.counts.get('replaced', 0)}"
                  f" · rejeitados {self.counts.get('nack', 0)} · expirados {self.counts.get('expired', 0)}"
                  f" · inválidos {self.counts.get('invalid_reply', 0)}")
        if self.latest_control and self.state != "idle":
            captured, _, infer, generation = self.latest_control
            control_hz = sum(row[3] == "calculated" for row in list(self.log)[-512:] if now - row[0] <= 2) / 2
            rates += (f"\nControle {control_hz:.1f} Hz · idade {1000 * (now - captured):.0f} ms"
                      f" · inferência {infer:.1f} ms · geração {generation * 1000:.2f} ms")
        self.diagnostics.set(rates)
        self.button.after(100, self.refresh_diagnostics)

    def receive_fault(self, line):
        try:
            _, session, motor, fault, position, target, duty, speed, threshold = line.split(b",")
            if int(session) != self.session:
                return
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
        if self.compatible is not True:
            return
        if line == b"!OK,REARM":
            self.status.set("Pico rearmado · seleção de movimento disponível")
            return
        if line.startswith(b"!ERR,REARM,"):
            self.status.set("Rearme indisponível · confira freio, fins de curso e encoders")
            return
        if line.startswith((b"!COMMDIAG,", b"!MOTORDIAG,", b"!RXERR,")):
            if self.state != "idle":
                self.log.append((monotonic(), self.session, None, line.decode("ascii", errors="replace")))
            return
        fields = line.split(b",")
        kind = fields[0]
        if kind == b"!BALLLINK2" and len(fields) == 2:
            self.supported = True
            self.connection.set("Autocorreção disponível no Pico")
            self.refresh_button()
            return
        if kind == b"!BALLF2" and self.state != "idle":
            self.receive_fault(line)
            return
        if kind not in (b"!BALLSTATE2", b"!BALLSTARTERR") or len(fields) != 3:
            return
        try:
            session = int(fields[1])
            state = int(fields[2]) if kind == b"!BALLSTATE2" else None
        except ValueError:
            return
        if self.state != "idle" and session == self.session:
            if kind == b"!BALLSTARTERR":
                reason = fields[2].decode("ascii", errors="replace").replace("_", " ")
                self.stop(f"Pico recusou a autocorreção ({reason}).")
            elif state == 4 and self.state == "pending":
                self.state = "centering"
                self.status.set("Centralizando nos neutros M1/M3")
                self.worker.start()
            elif state == 1 and self.state in ("pending", "centering"):
                if self.state == "pending":
                    self.worker.start()
                self.state = "running"
                self.worker.running.set()
                self.status.set("Autocorreção ativa")
            elif state in (0, 2, 3):
                reason = {0: "Autocorreção parada no Pico", 2: "Pico parou por falta de comandos",
                          3: self.fault_detail or "Pico parou por falha ou limite de curso"}[state]
                self.stop(reason, send=False)
        self.refresh_button()

    def update_preview(self, result, setpoint):
        self.setpoint = setpoint
        self.frame_at = result.captured_at if result is not None and ball_center(result.boxes) else None
