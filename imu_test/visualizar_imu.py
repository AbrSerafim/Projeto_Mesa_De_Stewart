"""Exibe a telemetria do Pico e abre a interface de acionamento."""

from collections import deque
from math import cos, isfinite, nan, radians, sin
from time import monotonic
from typing import NamedTuple
import tkinter as tk
from tkinter import messagebox, ttk

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import serial
from serial.tools import list_ports
from controle_motores import LOCAL_GRAVITY, PROTOCOL_HELLO, MotorTab, SerialSession


HISTORY_SECONDS = 10
LOG_PERIOD = 0.02
LEGACY_LOG_PERIOD = 0.04
MAX_LINE_BYTES = 512
COLORS = ("tab:red", "tab:green", "tab:blue")
BOARD = np.array([[-1, -0.65, 0], [1, -0.65, 0], [1, 0.65, 0], [-1, 0.65, 0]])
IMU_MESSAGES = (
    "Inicializando a IMU",
    "Calibrando a IMU. Mantenha o sensor parado.",
    "IMU em leitura",
    "IMU indisponível. Verifique alimentação, SDA e SCL.",
)
TRAVEL_MESSAGES = (
    "Curso pendente. Use a aba Acionamento.",
    "Mínimo registrado. Buscando o máximo.",
    "Máximo registrado.",
    "Curso calibrado",
)


class MotorReading(NamedTuple):
    count: int
    pulses_a: int
    pulses_b: int
    angle: float
    rpm: float
    limits_captured: int | None = None
    travel_pct: float | None = None


class Packet(NamedTuple):
    imu: tuple[float, ...] | None
    imu_state: int
    motors: tuple[MotorReading, MotorReading] | None = None
    limits: tuple[bool, ...] = ()


def parse_motor(fields, travel):
    count, pulses_a, pulses_b = map(int, fields[:3])
    angle, rpm = map(float, fields[3:])
    if min(pulses_a, pulses_b) < 0 or not all(map(isfinite, (angle, rpm))):
        raise ValueError("Leitura de encoder inválida")
    if not travel:
        return MotorReading(count, pulses_a, pulses_b, angle, rpm)

    captured = int(travel[0])
    if captured not in range(len(TRAVEL_MESSAGES)):
        raise ValueError("Estado de calibração inválido")
    percent = None
    if captured == 3:
        percent = float(travel[1])
        if not 0 <= percent <= 100:
            raise ValueError("Posição relativa fora da escala")
    elif travel[1]:
        raise ValueError("Posição relativa sem calibração")
    return MotorReading(count, pulses_a, pulses_b, angle, rpm, captured, percent)


def parse_sample(line):
    fields = line.split(b",")
    try:
        if len(fields) == 9:  # Firmware anterior, apenas IMU.
            imu = tuple(map(float, fields))
            return Packet(imu, 2) if all(map(isfinite, imu)) else None
        if len(fields) not in (24, 28):
            return None
        state = int(fields[23])
        if state not in range(len(IMU_MESSAGES)):
            return None
        imu = None
        if state == 2:
            imu = tuple(map(float, fields[:9]))
            if not all(map(isfinite, imu)):
                return None
        elif any(fields[:9]):
            return None
        motors = (
            parse_motor(fields[9:14], fields[24:26]),
            parse_motor(fields[14:19], fields[26:28]),
        )
        limits = tuple(map(int, fields[19:23]))
        if any(value not in (0, 1) for value in limits):
            return None
        return Packet(imu, state, motors, tuple(map(bool, limits)))
    except ValueError:
        return None


class SerialLines:
    def __init__(self):
        self.pending = b""
        self.discard_until_newline = False

    def feed(self, chunk):
        parts = (self.pending + chunk).split(b"\n")
        if self.discard_until_newline:
            if len(parts) == 1:
                return []
            parts.pop(0)
            self.discard_until_newline = False
        self.pending = parts.pop()
        if len(self.pending) > MAX_LINE_BYTES:
            self.pending = b""
            self.discard_until_newline = True
        return [line.strip() for line in parts if len(line) <= MAX_LINE_BYTES]


def rotation_matrix(roll, pitch, yaw):
    """Converte Rz(yaw) @ Ry(pitch) @ Rx(roll) de graus para uma matriz 3D."""
    x, y, z = map(radians, (roll, pitch, yaw))
    cx, cy, cz = cos(x), cos(y), cos(z)
    sx, sy, sz = sin(x), sin(y), sin(z)
    return np.array([
        [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
        [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
        [-sy, cy * sx, cy * cx],
    ])


def add_traces(axis, title, units):
    axis.set(title=title, ylabel=units, xlim=(-HISTORY_SECONDS, 0))
    axis.grid(alpha=0.3)
    traces = [
        axis.plot([], [], label=name, color=color)[0]
        for name, color in zip("XYZ", COLORS)
    ]
    axis.legend(loc="upper right", ncols=3)
    return traces


def batch_times(length, now, period, previous):
    start = now - (length - 1) * period
    if previous is not None:
        start = max(start, previous + (now - previous) / length)
    return np.linspace(start, now, length) if length > 1 else [now]


class MotorPanel:
    def __init__(self, parent, motor):
        self.frame = ttk.LabelFrame(parent, text=f"Motor {motor}", padding=8)
        self.values = []
        for column, name in enumerate((
            "Quadratura", "Pulsos A", "Pulsos B", "Ângulo", "Velocidade", "Curso",
        )):
            self.frame.columnconfigure(column, weight=1)
            ttk.Label(self.frame, text=name).grid(row=0, column=column, padx=5)
            value = ttk.Label(self.frame)
            value.grid(row=1, column=column, padx=5, pady=(3, 0))
            self.values.append(value)
        indicators = ttk.Frame(self.frame)
        indicators.grid(row=2, columnspan=len(self.values), sticky="ew", pady=(5, 0))
        self.limit_labels = []
        for column, name in enumerate(("Mínimo", "Máximo")):
            indicators.columnconfigure(column, weight=1)
            ttk.Label(indicators, text=f"Fim de curso {name.lower()}").grid(row=0, column=column, pady=4)
            label = tk.Label(indicators, padx=10, pady=5)
            label.grid(row=1, column=column, sticky="ew", padx=3)
            self.limit_labels.append(label)
        self.travel_status = ttk.Label(self.frame, anchor="center")
        self.travel_status.grid(row=3, columnspan=len(self.values), sticky="ew", pady=(6, 0))
        self.update(None, (None, None))

    def update(self, reading, limits):
        values = ("—",) * len(self.values)
        status = "Aguardando dados"
        if reading is not None:
            percent = "—" if reading.travel_pct is None else f"{reading.travel_pct:.1f}%"
            values = (
                str(reading.count), str(reading.pulses_a), str(reading.pulses_b),
                f"{reading.angle:.3f}°", f"{reading.rpm:.3f} rpm", percent,
            )
            status = ("Firmware sem calibração de curso" if reading.limits_captured is None
                      else TRAVEL_MESSAGES[reading.limits_captured])
        self.travel_status.configure(text=status)
        for label, value in zip(self.values, values):
            label.configure(text=value)
        for label, pressed in zip(self.limit_labels, limits):
            if pressed is None:
                text, background, foreground = "Sem dados", "#e5e7eb", "#374151"
            elif pressed:
                text, background, foreground = "Acionado", "#b91c1c", "#ffffff"
            else:
                text, background, foreground = "Livre", "#166534", "#ffffff"
            label.configure(text=text, background=background, foreground=foreground)


class MotorPlots:
    def __init__(self, parent):
        figure = Figure(figsize=(11, 4.2), layout="constrained")
        axes = figure.subplots(2, 2, sharex=True)
        figure.supxlabel("Tempo relativo ao presente (s)", fontsize=10)
        self.canvas = FigureCanvasTkAgg(figure, master=parent)
        self.canvas.get_tk_widget().grid(row=1, column=0, columnspan=2, sticky="nsew", pady=(6, 0))
        self.traces = []
        for axis, title, units, field in (
            (axes[0, 0], "Contagem em quadratura", "contagens", "count"),
            (axes[0, 1], "Posição entre os fins de curso", "%", "travel_pct"),
            (axes[1, 0], "Posição angular estimada", "°", "angle"),
            (axes[1, 1], "Velocidade angular estimada", "RPM", "rpm"),
        ):
            axis.set(title=title, ylabel=units, xlim=(-HISTORY_SECONDS, 0))
            if field == "travel_pct":
                axis.set(ylim=(0, 100), yticks=(0, 25, 50, 75, 100))
            axis.grid(alpha=0.3)
            lines = []
            for motor, color in ((1, "tab:blue"), (3, "tab:orange")):
                line, = axis.plot([], [], color=color, label=f"M{motor}", marker=".", markersize=2)
                lines.append(line)
            axis.legend(loc="upper left", ncols=len(lines), fontsize=8)
            self.traces.append((axis, lines, field))

    def update(self, history, now):
        times = [timestamp - now for timestamp, _ in history]
        for axis, lines, field in self.traces:
            for index, line in enumerate(lines):
                values = [getattr(readings[index], field) for _, readings in history]
                line.set_data(times, [nan if value is None else value for value in values])
            if history:
                axis.relim()
                axis.autoscale_view(scalex=False)
        self.canvas.draw_idle()


def visualize():
    window = tk.Tk()
    window.title("Telemetria do Pico")
    controls = ttk.Frame(window, padding=(8, 6))
    controls.pack(side="top", fill="x")
    port_button = ttk.Button(controls, text="Selecionar porta")
    port_button.pack(side="left")
    port_menu = tk.Menu(port_button, tearoff=False)
    status = tk.StringVar(value="Selecione a porta serial do Pico")
    ttk.Label(controls, textvariable=status).pack(side="left", padx=(12, 0))

    tabs = ttk.Notebook(window)
    tabs.pack(fill="both", expand=True)
    imu_tab = ttk.Frame(tabs)
    motors_tab = ttk.Frame(tabs, padding=16)
    drive_tab = ttk.Frame(tabs, padding=8)
    tabs.add(imu_tab, text="IMU")
    tabs.add(motors_tab, text="Encoders e fins de curso")
    tabs.add(drive_tab, text="Acionamento")
    motor_panels = [MotorPanel(motors_tab, motor) for motor in (1, 3)]
    for column, panel in enumerate(motor_panels):
        motors_tab.columnconfigure(column, weight=1, uniform="motors")
        panel.frame.grid(row=0, column=column, sticky="new", padx=6)
    motors_tab.rowconfigure(1, weight=1)
    motor_plots = MotorPlots(motors_tab)
    motor_history = deque()

    imu_actions = ttk.Frame(imu_tab, padding=(8, 6))
    imu_actions.pack(side="top", fill="x")
    figure = Figure(figsize=(12, 6), layout="constrained")
    canvas = FigureCanvasTkAgg(figure, master=imu_tab)
    toolbar = NavigationToolbar2Tk(canvas, imu_tab, pack_toolbar=False)
    toolbar.pack(side="bottom", fill="x")
    canvas.get_tk_widget().pack(fill="both", expand=True)
    grid = figure.add_gridspec(2, 2)
    accel = figure.add_subplot(grid[0, 0])
    gyro = figure.add_subplot(grid[1, 0], sharex=accel)
    gyro.set_xlabel("Tempo relativo ao presente (s)")
    traces = add_traces(accel, "Acelerômetro", "m/s²")
    accel.axhline(LOCAL_GRAVITY, color="0.45", linestyle=":", linewidth=1)
    traces += add_traces(gyro, "Giroscópio", "°/s")

    orientation = figure.add_subplot(grid[:, 1], projection="3d")
    orientation.set(
        xlim=(-1.3, 1.3), ylim=(-1.3, 1.3), zlim=(-1.3, 1.3),
        xlabel="X", ylabel="Y", zlabel="Z", title="Orientação da IMU",
    )
    orientation.set_box_aspect((1, 1, 1))
    plane = Poly3DCollection([BOARD], facecolor="tab:cyan", edgecolor="teal", alpha=0.55)
    orientation.add_collection3d(plane)
    plane.set_visible(False)
    body_axes = [
        orientation.plot([], [], [], color=color, linewidth=2.5)[0]
        for color in COLORS
    ]

    history = deque(maxlen=2000)
    session = None
    drive = MotorTab(drive_tab, lambda command: session.send(command) if session else None)
    drive.add_level_controls(imu_actions)
    window.bind("<Escape>", lambda _event: session.send("STOP") if session else None)
    last_sample_at = None
    last_packet_at = None
    latest_imu = None
    acceleration_si = False
    message = "Aguardando dados"
    next_draw = 0.0
    timer = canvas.new_timer(interval=15)

    def reset_stream():
        nonlocal last_sample_at, last_packet_at, latest_imu, message, next_draw, acceleration_si
        history.clear()
        motor_history.clear()
        drive.reset()
        next_draw = 0
        last_sample_at = None
        last_packet_at = None
        latest_imu = None
        acceleration_si = False
        message = "Aguardando dados"
        for panel in motor_panels:
            panel.update(None, (None, None))
        for trace in traces:
            trace.set_data([], [])
        plane.set_visible(False)
        for axis in body_axes:
            axis.set_visible(False)
        orientation.set_title("Orientação da IMU")

    def connect(device):
        nonlocal session
        if session is not None and session.device == device:
            return
        try:
            new_port = serial.Serial(device, baudrate=115200, timeout=.01, write_timeout=.15)
        except OSError as error:
            details = f"Erro ao abrir {device}: {error}"
            if session is not None:
                details += f"\nA leitura continua em {session.device}."
            messagebox.showerror("Porta serial", details, parent=window)
            return

        if session is not None:
            session.close()
        session = SerialSession(new_port, SerialLines())
        reset_stream()
        status.set(f"{session.device} — {message}")

    def show_ports():
        port_menu.delete(0, "end")
        try:
            available = sorted(list_ports.comports())
        except OSError as error:
            messagebox.showerror("Porta serial", str(error), parent=window)
            return
        if not available:
            port_menu.add_command(label="Nenhuma porta disponível", state="disabled")
        for entry in available:
            active = session is not None and session.device == entry.device
            label = f"{entry.device} — {entry.description}"
            port_menu.add_command(
                label=f"{label} (conectada)" if active else label,
                state="disabled" if active else "normal",
                command=lambda device=entry.device: connect(device),
            )
        try:
            port_menu.tk_popup(
                port_button.winfo_rootx(), port_button.winfo_rooty() + port_button.winfo_height(),
            )
        finally:
            port_menu.grab_release()

    port_button.configure(command=show_ports)

    def refresh():
        nonlocal session, last_sample_at, last_packet_at, latest_imu, message, next_draw, acceleration_si
        if session is None:
            return
        now = monotonic()
        session.ui_at = now
        if session.error is not None:
            device = session.device
            error = session.error
            session.close()
            session = None
            reset_stream()
            status.set(f"Conexão com {device} interrompida ({error}).")
            redraw_selected_tab(None)
            return
        samples = []
        motor_samples = []
        latest = None
        for _ in range(min(len(session.incoming), 512)):
            line = session.incoming.popleft()
            if line in (PROTOCOL_HELLO, b"!HELLO,7") or line.startswith(b"accelx_mps2,"):
                acceleration_si = True
            if line.startswith((b"@", b"!")):
                drive.receive(line)
            elif line.startswith(b"#"):
                history.clear()
                last_sample_at = None
                latest_imu = None
                message = line[1:].strip().decode("utf-8", errors="replace")
                samples.clear()
            else:
                packet = parse_sample(line)
                if packet is None:
                    continue
                latest = packet
                if packet.motors is not None:
                    motor_samples.append(packet.motors)
                if packet.imu is None:
                    history.clear()
                    last_sample_at = None
                    latest_imu = None
                    samples.clear()
                else:
                    sample = packet.imu
                    if not acceleration_si:
                        sample = tuple(value * 9.80665 for value in sample[:3]) + sample[3:]
                    samples.append(sample)
        drive.check_sync(now)

        if motor_samples:
            previous = motor_history[-1][0] if motor_history else None
            period = LOG_PERIOD if drive.fast_telemetry else LEGACY_LOG_PERIOD
            times = batch_times(len(motor_samples), now, period, previous)
            motor_history.extend(zip(times, motor_samples))
        while motor_history and motor_history[0][0] < now - HISTORY_SECONDS:
            motor_history.popleft()
        if latest is not None:
            last_packet_at = now
            message = IMU_MESSAGES[latest.imu_state]
            if latest.motors is not None and tabs.select() == str(motors_tab):
                for index, (panel, reading) in enumerate(zip(motor_panels, latest.motors)):
                    panel.update(reading, latest.limits[2 * index:2 * index + 2])
            elif latest.motors is None:
                for panel in motor_panels:
                    panel.update(None, (None, None))
                message += " | Firmware com telemetria somente da IMU"
        elif last_packet_at is not None and now - last_packet_at > 1:
            last_packet_at = None
            message = "Sem dados recentes"
            for panel in motor_panels:
                panel.update(None, (None, None))

        if samples:
            period = LOG_PERIOD if drive.fast_telemetry else LEGACY_LOG_PERIOD
            if latest.motors is None:
                period = 0.01
            times = batch_times(len(samples), now, period, last_sample_at)
            history.extend((timestamp, *sample) for timestamp, sample in zip(times, samples))
            last_sample_at = now
            latest_imu = samples[-1]

        while history and history[0][0] < now - HISTORY_SECONDS:
            history.popleft()
        status.set(f"{session.device} — {message}")
        if now >= next_draw:
            next_draw = now + .12
            redraw_selected_tab(None)

    def redraw_selected_tab(_event):
        now = monotonic()
        selected = tabs.select()
        if selected == str(drive_tab):
            drive.draw()
            return
        if selected == str(motors_tab):
            motor_plots.update(motor_history, now)
            return
        if history:
            values = np.asarray(history)
            for column, trace in enumerate(traces, start=1):
                trace.set_data(values[:, 0] - now, values[:, column])
            for axis in (accel, gyro):
                axis.relim()
                axis.autoscale_view(scalex=False)
        else:
            for trace in traces:
                trace.set_data([], [])

        show_plane = last_sample_at is not None and now - last_sample_at <= 1
        plane.set_visible(show_plane)
        for axis in body_axes:
            axis.set_visible(show_plane)
        if show_plane and latest_imu is not None:
            roll, pitch, yaw = latest_imu[6:9]
            rotation = rotation_matrix(roll, pitch, yaw)
            plane.set_verts([BOARD @ rotation.T])
            for axis, (x, y, z) in zip(body_axes, rotation.T):
                axis.set_data_3d([0, x], [0, y], [0, z])
            orientation.set_title(f"X {roll:.1f}°    Y {pitch:.1f}°    Z relativo {yaw:.1f}°")
        else:
            orientation.set_title("Orientação da IMU")

        canvas.draw_idle()

    tabs.bind("<<NotebookTabChanged>>", redraw_selected_tab)
    timer.add_callback(refresh)
    timer.start()
    canvas.draw_idle()
    try:
        window.mainloop()
    finally:
        timer.stop()
        if session is not None:
            session.close()


if __name__ == "__main__":
    visualize()
