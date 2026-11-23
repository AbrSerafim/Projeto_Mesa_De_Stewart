"""Visualiza o CSV da MPU6050 recebido pela porta USB serial do Pico."""

from collections import deque
from math import cos, isfinite, radians, sin
from time import monotonic
import tkinter as tk
from tkinter import messagebox, ttk

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import serial
from serial.tools import list_ports


HISTORY_SECONDS = 10
SAMPLE_PERIOD = 0.01
MAX_LINE_BYTES = 128
COLORS = ("tab:red", "tab:green", "tab:blue")
BOARD = np.array([[-1, -0.65, 0], [1, -0.65, 0], [1, 0.65, 0], [-1, 0.65, 0]])


def parse_sample(line):
    fields = line.split(b",")
    if len(fields) != 9:
        return None
    try:
        values = tuple(float(field) for field in fields)
    except ValueError:
        return None
    return values if all(map(isfinite, values)) else None


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


def visualize():
    window = tk.Tk()
    window.title("MPU6050")
    controls = ttk.Frame(window, padding=(8, 6))
    controls.pack(side="top", fill="x")
    port_button = ttk.Button(controls, text="Selecionar porta")
    port_button.pack(side="left")
    port_menu = tk.Menu(port_button, tearoff=False)

    figure = Figure(figsize=(12, 6), layout="constrained")
    canvas = FigureCanvasTkAgg(figure, master=window)
    toolbar = NavigationToolbar2Tk(canvas, window, pack_toolbar=False)
    toolbar.pack(side="bottom", fill="x")
    canvas.get_tk_widget().pack(fill="both", expand=True)
    grid = figure.add_gridspec(2, 2)
    accel = figure.add_subplot(grid[0, 0])
    gyro = figure.add_subplot(grid[1, 0], sharex=accel)
    gyro.set_xlabel("Tempo relativo ao presente (s)")
    traces = add_traces(accel, "Acelerômetro", "g")
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

    status = figure.suptitle("Selecione a porta serial do Pico")
    history = deque(maxlen=2000)
    port = None
    lines = SerialLines()
    last_sample_at = None
    message = "Aguardando a MPU6050"
    timer = canvas.new_timer(interval=40)

    def reset_stream():
        nonlocal lines, last_sample_at, message
        history.clear()
        lines = SerialLines()
        last_sample_at = None
        message = "Aguardando a MPU6050"
        for trace in traces:
            trace.set_data([], [])
        plane.set_visible(False)
        for axis in body_axes:
            axis.set_visible(False)
        orientation.set_title("Orientação da IMU")

    def connect(device):
        nonlocal port
        if port is not None and port.port == device:
            return
        try:
            new_port = serial.Serial(device, baudrate=115200, timeout=0)
        except OSError as error:
            details = f"Erro ao abrir {device}: {error}"
            if port is not None:
                details += f"\nA leitura continua em {port.port}."
            messagebox.showerror("Porta serial", details, parent=window)
            return

        if port is not None:
            port.close()
        port = new_port
        reset_stream()
        status.set_text(f"{port.port} — {message}")
        canvas.draw_idle()

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
            active = port is not None and port.port == entry.device
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
        nonlocal port, last_sample_at, message
        if port is None:
            return
        samples = []
        try:
            for line in lines.feed(port.read(min(port.in_waiting, 16384))):
                if line.startswith(b"#"):
                    history.clear()
                    last_sample_at = None
                    message = line[1:].strip().decode("utf-8", errors="replace")
                    samples.clear()
                else:
                    sample = parse_sample(line)
                    if sample is not None:
                        samples.append(sample)
        except OSError:
            device = port.port
            port.close()
            port = None
            reset_stream()
            status.set_text(f"Conexão com {device} interrompida. Selecione uma porta para reconectar.")
            canvas.draw_idle()
            return

        now = monotonic()
        if samples:
            start = now - (len(samples) - 1) * SAMPLE_PERIOD
            if last_sample_at is not None:
                start = max(start, last_sample_at + (now - last_sample_at) / len(samples))
            times = np.linspace(start, now, len(samples)) if len(samples) > 1 else [now]
            history.extend((timestamp, *sample) for timestamp, sample in zip(times, samples))
            last_sample_at = now
            message = "Recebendo dados"
        elif last_sample_at is not None and now - last_sample_at > 1:
            message = "Sem dados recentes"

        while history and history[0][0] < now - HISTORY_SECONDS:
            history.popleft()
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
        if samples:
            roll, pitch, yaw = samples[-1][6:9]
            rotation = rotation_matrix(roll, pitch, yaw)
            plane.set_verts([BOARD @ rotation.T])
            for axis, (x, y, z) in zip(body_axes, rotation.T):
                axis.set_data_3d([0, x], [0, y], [0, z])
            orientation.set_title(f"X {roll:.1f}°    Y {pitch:.1f}°    Z relativo {yaw:.1f}°")
        elif not show_plane:
            orientation.set_title("Orientação da IMU")

        status.set_text(f"{port.port} — {message}")
        canvas.draw_idle()

    timer.add_callback(refresh)
    timer.start()
    canvas.draw_idle()
    try:
        window.mainloop()
    finally:
        timer.stop()
        if port is not None:
            port.close()


if __name__ == "__main__":
    visualize()
