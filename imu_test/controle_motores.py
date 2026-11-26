"""Comunicação e interface de acionamento dos motores da mesa."""

from collections import deque
import csv
import json
from math import isclose, isfinite
from pathlib import Path
from queue import Empty, Queue
from secrets import randbelow
from threading import Event, Thread
from time import monotonic
from typing import NamedTuple
from tkinter import BooleanVar, DoubleVar, StringVar, TclError, filedialog, messagebox, ttk

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


from ball_protocol import BallWindow


MOTORS = (1, 3)
PROTOCOL_HELLO = b"!HELLO,13"
LEGACY_HELLOS = tuple(f"!HELLO,{version}".encode("ascii") for version in range(1, 13))
FAST_TELEMETRY_HELLOS = (*LEGACY_HELLOS[5:], PROTOCOL_HELLO)
LOCAL_GRAVITY = 9.78627
IMU_REFERENCE_PATH = Path(__file__).with_name("referencia_imu.txt")
DEFAULT_IMU_REFERENCE = (0.0, 0.0, 1.0)
ENCODER_RESOLUTION_DEG = .28125
SETTINGS_PATH = Path(__file__).with_name("parametros_motores.txt")
FRICTION_PATH = Path(__file__).with_name("parametros_zona_morta.txt")
CONTROL_LAWS = ("P", "PI", "PID", "PIDF")
PIDF = CONTROL_LAWS.index("PIDF")
DEFAULTS = dict.fromkeys(MOTORS, (400, 20, 10, .105, 30, 90, 8, 4, 30, 1, PIDF))
PROFILE_HEADER = "# PIDF externo em graus, saida em graus/s. PI interno gera PWM."
MODES = ("Freio dinâmico", "Livre", "Calibração", "Posição", "Degrau", "Seno", "Cosseno", "Rampa", "Teste de PWM")
PHASES = ("busca rápida", "recuo", "busca lenta", "liberação da chave")
FAULTS = (
    "", "conexão perdida", "dois fins de curso ativos", "fim de curso inesperado na calibração",
    "tempo de calibração excedido", "leitura do encoder atrasada", "encoder sem movimento",
    "curso menor que 16 contagens", "fim de curso fora da posição calibrada",
    "velocidade excessiva", "encoder fora do curso calibrado",
    "fim de curso confirmado na autocorreção", "posição fora da faixa de autocorreção",
    "tempo excedido para chegar a 40%", "alvo de autocorreção fora da faixa",
)
STALL_FAULT = 6
OVERSPEED_FAULT = 9
ERRORS = {
    "teste_pwm_requer_curso": "Calibre o curso e deixe o motor a mais de 2° dos fins de curso.",
    "teste_pwm_em_andamento": "Pare o teste de PWM antes de usar outro acionamento.",
    "libere_os_fins_de_curso": "Libere os fins de curso antes de calibrar.",
    "freio_necessario": "Acione o freio antes de repetir a operação.",
    "calibracao_ou_freio_necessario": "Calibre o curso e acione o freio se o motor estiver livre.",
    "freio_e_motor_parado_necessarios": "Acione o freio e aguarde o motor parar.",
    "alvo_fora_do_curso": "O alvo ultrapassa o curso calibrado.",
    "teste_fora_da_faixa": "O teste ultrapassa o curso ou o tempo permitido.",
    "configuracao_fora_da_faixa": "Revise os parâmetros.",
    "calibracao_em_andamento": "A calibração está em andamento.",
    "fins_simultaneos": "Há dois fins de curso ativos no mesmo motor.",
    "sem_sessao": "A sessão serial foi interrompida.",
    "ganhos_ou_pwm_zerados": "Aplique pelo menos um ganho e um limite de PWM maior que zero.",
    "pwm_zerado": "Aplique um limite de PWM maior que zero nos dois motores para calibrar.",
    "nivelamento_em_andamento": "O nivelamento está em andamento. Use Parar para interromper.",
    "nivelamento_requer_limites": "Reative os limites V/A na aba Acionamento para nivelar.",
    "nivelamento_requer_curso": "Calibre o curso dos dois motores na aba Acionamento.",
    "nivelamento_fora_do_curso": "Reposicione os motores dentro do curso calibrado.",
    "imu_indisponivel": "Aguarde a IMU concluir a calibração inicial.",
    "imu_requer_mesa_parada": "Aguarde pelo menos 2 s com a mesa parada e tente novamente.",
    "referencia_imu_invalida": "Referência inválida. Confira a orientação da IMU.",
    "eixo_z_invertido": "O Z da mesa está medindo para baixo. Confira a montagem da IMU.",
    "home_requer_curso": "Calibre o curso dos dois motores para usar Home.",
    "home_requer_encoder": "Aguarde leituras dos dois encoders para usar Home.",
    "home_em_andamento": "A mesa está indo para Home. Use Parar para interromper.",
    "bola_em_andamento": "A autocorreção da bola está ativa. Use Parar para interromper.",
}

LEVEL_MESSAGES = (
    "Pronta para nivelar", "Medindo a inclinação inicial", "Medindo o efeito do M1",
    "Medindo o efeito do M3", "Corrigindo a inclinação", "Mesa nivelada", "Nivelamento interrompido",
    "A leitura da IMU foi interrompida", "Tempo excedido ao aguardar posição estável",
    "Inclinação inicial alta ou os motores não corrigem dois eixos independentes",
    "Limite de curso ou margem de movimento atingidos", "A inclinação deixou de melhorar",
    "Um motor parou por falha. Confira a aba Acionamento.",
)

FIELDS = (
    ("Kp", 0, 1000), ("Ki", 0, 5000), ("Kd", -100, 100), ("Tf (s)", 0, 2),
    ("V máx. °/s", 2, 360), ("A máx. °/s²", 1, 2000),
    ("Cal. rápida °/s", 2, 360), ("Cal. lenta °/s", 2, 360), ("PWM máx. %", 0, 100),
)


def settings_errors(values):
    if len(values) != 11 or not all(map(isfinite, values)):
        return ["Preencha todos os parâmetros com valores finitos."]
    errors = [f"{name} deve ficar entre {low:g} e {high:g} (informado {value:g})."
              for (name, low, high), value in zip(FIELDS, values) if not low <= value <= high]
    _, _, kd, tf, speed, _, fast, slow, _, polarity, law = values
    if law not in range(len(CONTROL_LAWS)):
        errors.append("Selecione P, PI, PID ou PIDF.")
    if law == PIDF and kd != 0 and tf < .0001:
        errors.append("Tf deve ser de pelo menos 0,0001 s quando Kd estiver ativo.")
    if fast > speed:
        errors.append(f"Cal. rápida ({fast:g} °/s) excede V máx. ({speed:g} °/s).")
    if slow > fast:
        errors.append(f"Cal. lenta ({slow:g} °/s) excede Cal. rápida ({fast:g} °/s).")
    if polarity not in (-1, 1):
        errors.append("Polaridade inválida.")
    return errors


def load_settings(path=SETTINGS_PATH):
    settings = DEFAULTS.copy()
    if not path.exists():
        return settings, ""
    try:
        seen = set()
        lines = path.read_text(encoding="utf-8").splitlines()
        legacy = PROFILE_HEADER not in lines
        for line in lines:
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
            motor, *raw = line.split()
            motor = int(motor)
            values = tuple(map(float, raw))
            if len(values) == 10:
                values = (*values, PIDF)
            if motor not in MOTORS or motor in seen or settings_errors(values):
                raise ValueError(f"parâmetros inválidos para o motor {motor}")
            if legacy:
                values = (*DEFAULTS[motor][:4], *values[4:10], PIDF)
            settings[motor] = values
            seen.add(motor)
        warning = "Perfil antigo carregado com os ganhos da malha em cascata." if legacy and seen else ""
        return settings, warning
    except (OSError, ValueError) as error:
        return DEFAULTS.copy(), f"Falha ao ler {path.name} ({error}). Padrões carregados."


def save_settings(settings, path=SETTINGS_PATH):
    content = (PROFILE_HEADER + "\n"
               "# Motor Kp Ki Kd Tf Vmax Amax Vcal_rapida Vcal_lenta PWMmax Polaridade Modo(0=P,1=PI,2=PID,3=PIDF)\n")
    content += "".join(f"{motor} " + " ".join(f"{x:.7g}" for x in settings[motor]) + "\n"
                       for motor in MOTORS)
    temporary = path.with_suffix(".tmp")
    temporary.write_text(content, encoding="utf-8")
    temporary.replace(path)


def same_settings(first, second):
    return all(isclose(a, b, rel_tol=1e-6, abs_tol=1e-8) for a, b in zip(first, second))


def parse_imu_reference(parts):
    values = tuple(map(float, parts))
    if len(values) != 3 or not all(map(isfinite, values)):
        raise ValueError("Vetor de referência inválido")
    norm_squared = sum(value * value for value in values)
    if not .99**2 <= norm_squared <= 1.01**2 or values[2] < .8 * norm_squared**.5:
        raise ValueError("Direção de referência inválida")
    return tuple(value / norm_squared**.5 for value in values)


def load_imu_reference():
    try:
        return parse_imu_reference(IMU_REFERENCE_PATH.read_text(encoding="utf-8").split()), ""
    except FileNotFoundError:
        return DEFAULT_IMU_REFERENCE, ""
    except (OSError, ValueError) as error:
        return DEFAULT_IMU_REFERENCE, f"Falha ao ler a referência da IMU ({error})."


def imu_reference_command(values):
    return "IMUCFG " + " ".join(f"{value:.8g}" for value in values)


class SerialSession:
    """Mantém RX, TX e heartbeat fora do thread do Tk."""

    def __init__(self, port, framer, ack_timeout=.15, ball_period=0):
        self.port = port
        self.device = port.port
        self.framer = framer
        self.incoming = deque(maxlen=4096)
        self.outgoing = Queue()
        self.ball = BallWindow(ack_timeout=ack_timeout, send_period=ball_period)
        self.link = 0
        self.stopping = Event()
        self.braking = Event()
        self.ui_at = monotonic()
        self.error = None
        self.thread = Thread(target=self.run, name="serial-pico", daemon=True)
        self.thread.start()

    def send(self, command):
        if command == "STOP":
            self.ball.clear(monotonic(), "cancelled")
            self.braking.set()
        elif command.startswith("BALLSTOP "):
            if self.ball.clear(monotonic(), "cancelled", int(command.split()[1])):
                self.braking.set()
        elif command.startswith(("BALLSTART2 ", "BALLTEST2 ")):
            kind, session, *values = command.split()
            self.ball.begin(int(session), monotonic())
            self.outgoing.put(" ".join((kind, session, str(self.link), *values)))
        elif command.startswith("TARGET "):
            _, session, m1, m3 = command.split()
            return self.ball.offer(int(session), (float(m1), float(m3)), monotonic())
        else:
            self.outgoing.put(command)

    def discard_commands(self):
        self.ball.clear(monotonic(), "cancelled")
        while True:
            try:
                self.outgoing.get_nowait()
            except Empty:
                return

    def write(self, command):
        payload = (("\x18" if command == "STOP" else "") + command + "\n").encode("ascii")
        if self.port.write(payload) != len(payload):
            raise OSError("Escrita USB incompleta")

    def run(self):
        hello = False
        ui_stopped = False
        next_ping = next_hello = 0
        try:
            while not self.stopping.is_set():
                now = monotonic()
                if self.braking.is_set():
                    self.braking.clear()
                    self.discard_commands()
                    self.write("STOP")
                if not hello and now >= next_hello:
                    self.write("HELLO")
                    next_hello = now + 1
                if hello and now >= next_ping:
                    self.write("PING")
                    next_ping = now + .15
                if now - self.ui_at < 1.25:
                    ui_stopped = False
                    if hello:
                        for _ in range(4):
                            if self.braking.is_set():
                                break
                            try:
                                command = self.outgoing.get_nowait()
                            except Empty:
                                break
                            self.write(command)
                        if not self.braking.is_set():
                            target = self.ball.next_command(monotonic())
                            if target is not None and not self.braking.is_set():
                                self.write(target)
                else:
                    self.discard_commands()
                    if hello and not ui_stopped:
                        self.write("STOP")
                        ui_stopped = True
                chunk = self.port.read(min(self.port.in_waiting or 1, 16384))
                for line in self.framer.feed(chunk):
                    if line == PROTOCOL_HELLO or line in LEGACY_HELLOS:
                        if not hello and line == PROTOCOL_HELLO:
                            self.link = randbelow(2**32 - 1) + 1
                            self.write(f"BALLLINK2 {self.link}")
                        hello = True
                        next_ping = 0
                    elif line == b"!ERR,PING,sem_sessao":
                        if not hello:
                            continue
                        hello = False
                        self.link = 0
                        next_hello = 0
                        self.discard_commands()
                    if line.startswith(b"!BALLLINK2,") and line != f"!BALLLINK2,{self.link}".encode("ascii"):
                        continue
                    received_at = monotonic()
                    self.ball.receive(line, received_at)
                    self.incoming.append((received_at, line))
        except OSError as error:
            self.error = str(error)
        finally:
            self.ball.clear(monotonic(), "disconnected")
            try:
                if self.stopping.is_set():
                    self.write("STOP")
            except OSError:
                pass
            finally:
                self.port.close()

    def close(self):
        self.stopping.set()
        self.thread.join(timeout=.35)


class MotorSample(NamedTuple):
    motor: int
    timestamp: int
    mode: int
    fault: int
    calibrated: int
    limits: int
    phase: int
    endpoint: int
    angle: float
    goal: float
    reference: float
    speed: float
    span: float
    pwm: float
    step: int
    elapsed: float
    enable: int = -1
    fault_speed: float = 0
    fault_threshold: float = 0
    speed_setpoint: float = 0
    step_origin: float | None = None
    bridge: int | None = None

    @classmethod
    def parse(cls, fields):
        try:
            if len(fields) not in (16, 17, 19, 20, 22):
                return None
            sample = cls(*map(int, fields[:8]), *map(float, fields[8:14]),
                         int(fields[14]), float(fields[15]), *map(int, fields[16:17]),
                         *map(float, fields[17:21]), *map(int, fields[21:]))
            if (sample.motor in MOTORS and 0 <= sample.mode < len(MODES)
                    and 0 <= sample.fault < len(FAULTS) and 0 <= sample.phase < len(PHASES)
                    and sample.enable in (-1, 0, 1) and sample.bridge in (None, 0, 1, 2, 3)
                    and (sample.step_origin is None or isfinite(sample.step_origin))
                    and all(map(isfinite, (*sample[8:14], sample.elapsed,
                                          sample.fault_speed, sample.fault_threshold, sample.speed_setpoint)))):
                return sample
        except (ValueError, IndexError, TypeError):
            pass
        return None


def step_metrics(samples, complete, resolution=ENCODER_RESOLUTION_DEG):
    if len(samples) < 2:
        return "Aguardando amostras"
    target = samples[0].goal
    initial = samples[0].step_origin if samples[0].step_origin is not None else target - 1
    amplitude = target - initial
    if amplitude == 0:
        return "Degrau sem deslocamento"

    def progress(sample):
        return (sample.angle - initial) / amplitude

    def crossing(level):
        if progress(samples[0]) >= level:
            return 0.0
        for before, after in zip(samples, samples[1:]):
            low, high = progress(before), progress(after)
            if low < level <= high:
                ratio = (level - low) / (high - low)
                return before.elapsed + ratio * (after.elapsed - before.elapsed)
        return None

    t10, t90 = crossing(.1), crossing(.9)
    rise = f"{t90 - t10:.2f} s" if t10 is not None and t90 is not None else "—"
    peak = max(samples, key=progress)
    overshoot = 100 * max(0, progress(peak) - 1)
    tail = [row.angle for row in samples if samples[-1].elapsed - row.elapsed <= .5]
    error = f"{target - sum(tail) / len(tail):+.3f}°" if complete else "—"
    settling = "—"
    if complete:
        last_outside = max((i for i, sample in enumerate(samples)
                            if abs(sample.angle - target) > resolution), default=-1)
        if last_outside + 1 < len(samples):
            first_inside = samples[last_outside + 1].elapsed
            if samples[-1].elapsed - first_inside >= .5:
                settling = f"{first_inside:.2f} s"
    return (f"Subida 10–90% {rise}  |  Sobressinal {overshoot:.1f}%  |  "
            f"Pico {peak.elapsed:.2f} s  |  Erro final {error}  |  "
            f"Acomodação (±1 cont.) {settling}")


class MotorPanel:
    def __init__(self, parent, motor, action, save):
        self.motor = motor
        self.action = action
        self.buttons = []
        self.frame = ttk.LabelFrame(parent, text=f"Motor {motor}", padding=7)
        self.status = StringVar(value="Aguardando firmware de acionamento")
        ttk.Label(self.frame, textvariable=self.status, wraplength=440).pack(anchor="w")
        gains = ttk.LabelFrame(self.frame, text="PIDF de posição · saída em °/s", padding=3)
        gains.pack(fill="x", pady=(4, 2))
        self.gains = [StringVar() for _ in range(4)]
        for index, (name, variable) in enumerate(zip(("Kp", "Ki", "Kd", "Tf (s)"), self.gains)):
            gains.columnconfigure(index, weight=1)
            ttk.Label(gains, text=name).grid(row=0, column=index, sticky="w")
            ttk.Entry(gains, textvariable=variable, width=8).grid(row=1, column=index, sticky="ew", padx=(0, 4))
        self.gain_status = StringVar()
        ttk.Label(gains, textvariable=self.gain_status).grid(row=2, columnspan=4, sticky="w")
        self.control_law = StringVar(value="PIDF")
        ttk.Label(gains, text="Modo").grid(row=3, column=0, sticky="w")
        ttk.Combobox(gains, textvariable=self.control_law, values=CONTROL_LAWS, state="readonly",
                     width=6).grid(row=3, column=1, sticky="w")
        sliders = ttk.Frame(self.frame)
        sliders.pack(fill="x")
        sliders.columnconfigure(1, weight=1)
        sliders.columnconfigure(4, weight=1)
        self.sliders = []
        for index, (name, lower, upper) in enumerate(FIELDS[4:]):
            row, column = divmod(index, 2)
            column *= 3
            var = DoubleVar()
            ttk.Label(sliders, text=name).grid(row=row, column=column, sticky="w")
            ttk.Scale(sliders, from_=lower, to=upper, variable=var, length=60,
                      command=lambda value, var=var: var.set(round(float(value), 2))).grid(
                row=row, column=column + 1, padx=2, sticky="ew")
            spinbox = ttk.Spinbox(sliders, from_=lower, to=upper, textvariable=var, width=6, increment=1)
            spinbox.grid(row=row, column=column + 2, padx=(0, 6))
            self.sliders.append((var, spinbox))
        settings = ttk.Frame(self.frame)
        settings.pack(fill="x", pady=3)
        self.active_limits = StringVar(value="Aguardando limites confirmados pelo Pico")
        ttk.Label(settings, textvariable=self.active_limits).pack(side="top", anchor="w", pady=(0, 3))
        self.invert = BooleanVar()
        ttk.Checkbutton(settings, text="Inverter sentido", variable=self.invert).pack(side="left")
        self.button(settings, "Salvar", lambda: save(motor), side="right", padx=(4, 0))
        self.button(settings, "Aplicar", lambda: action(self.apply), side="right", padx=(4, 0))
        ttk.Button(settings, text="Zerar PIDF", command=self.zero_gains).pack(side="right")

        go = ttk.Frame(self.frame)
        go.pack(fill="x", pady=2)
        self.target = StringVar(value="50")
        self.units = StringVar(value="% curso")
        ttk.Label(go, text="Alvo").pack(side="left")
        ttk.Entry(go, width=7, textvariable=self.target).pack(side="left", padx=4)
        ttk.Combobox(go, width=10, state="readonly", values=("% curso", "° desde mín."),
                     textvariable=self.units).pack(side="left")
        self.button(go, "Mover", lambda: action(self.go), side="left", padx=4)
        self.step_degrees = StringVar(value="1")
        self.button(go, "Degrau em °", lambda: action(self.step), side="right")
        ttk.Entry(go, width=6, textvariable=self.step_degrees).pack(side="right", padx=4)

        wave = ttk.Frame(self.frame)
        wave.pack(fill="x", pady=2)
        self.wave = StringVar(value="Seno")
        ttk.Combobox(wave, width=9, state="readonly", values=("Seno", "Cosseno", "Rampa"),
                     textvariable=self.wave).pack(side="left")
        self.wave_values = [StringVar(value=value) for value in ("3", "1", "4")]
        for label, var in zip(("Base °", "Δ °", "Tempo s"), self.wave_values):
            ttk.Label(wave, text=label).pack(side="left", padx=(4, 2))
            ttk.Entry(wave, width=5, textvariable=var).pack(side="left")
        self.button(wave, "Iniciar", lambda: action(self.run_wave), side="right")

    def add_pwm_test(self, toggle, use):
        row = ttk.LabelFrame(self.frame, text="Zona morta · teste de PWM", padding=3)
        row.pack(fill="x", pady=(3, 0))
        self.pwm_direction = StringVar(value="Subir curso (+)")
        self.pwm_direction_box = ttk.Combobox(row, textvariable=self.pwm_direction, width=15,
            values=("Subir curso (+)", "Descer curso (−)"), state="readonly")
        self.pwm_direction_box.pack(side="left")
        self.pwm_sustain = BooleanVar()
        ttk.Checkbutton(row, text="Sustentação", variable=self.pwm_sustain).pack(side="left", padx=3)
        self.pwm_target = DoubleVar()
        self.pwm_label = StringVar(value="0,00%")
        self.pwm_slider = ttk.Scale(row, from_=0, to=100, variable=self.pwm_target, state="disabled",
            command=lambda value: self.pwm_label.set(f"{float(value):.2f}%"))
        self.pwm_slider.pack(side="left", fill="x", expand=True, padx=4)
        ttk.Label(row, textvariable=self.pwm_label, width=8).pack(side="left")
        self.pwm_button = ttk.Button(row, text="Iniciar teste", command=lambda: toggle(self.motor))
        self.pwm_button.pack(side="left", padx=3)
        self.pwm_use = ttk.Button(row, text="Usar e salvar medição", command=lambda: use(self.motor))
        self.pwm_use.pack(side="left")
        self.pwm_message = StringVar()
        ttk.Label(self.frame, textvariable=self.pwm_message).pack(anchor="w")
        self.pwm_saved = StringVar()
        ttk.Label(self.frame, textvariable=self.pwm_saved).pack(anchor="w")

    def button(self, parent, label, callback, **pack):
        button = ttk.Button(parent, text=label, command=callback, state="disabled")
        button.pack(**pack)
        self.buttons.append(button)

    def set_values(self, values):
        for var, value in zip(self.gains, values[:4]):
            var.set(f"{value:.7g}")
        for (var, _), value in zip(self.sliders, values[4:9]):
            var.set(value)
        self.invert.set(values[9] == -1)
        self.control_law.set(CONTROL_LAWS[int(values[10])])

    def zero_gains(self):
        for variable in self.gains:
            variable.set("0")

    def describe_applied_gain(self, values, resolution):
        proportional = values[0] * resolution
        law = CONTROL_LAWS[int(values[10])]
        self.gain_status.set(f"No Pico · {law} · P por contagem {proportional:.2f} °/s")
        speed, accel, fast, slow, duty = values[4:9]
        self.active_limits.set(f"No Pico · V máx. {speed:g} °/s · A máx. {accel:g} °/s²"
                               f" · Cal. {fast:g}/{slow:g} °/s · PWM {duty:g}%")

    def values(self):
        values = []
        fields = self.gains + [spinbox for _, spinbox in self.sliders]
        for (name, _, _), entry in zip(FIELDS, fields):
            try:
                values.append(number(entry.get()))
            except ValueError as error:
                raise ValueError(f"{name} requer um número finito.") from error
        values = (*values, -1 if self.invert.get() else 1, CONTROL_LAWS.index(self.control_law.get()))
        errors = settings_errors(values)
        if errors:
            raise ValueError(" ".join(errors))
        return values

    def apply(self):
        return cfg_command(self.motor, self.values())

    def step(self):
        return f"STEP {self.motor} {number(self.step_degrees.get()):.7g}"

    def go(self):
        return f"GO {self.motor} {int(self.units.get() == '% curso')} {number(self.target.get()):.7g}"

    def run_wave(self):
        kind = ("Seno", "Cosseno", "Rampa").index(self.wave.get())
        args = " ".join(f"{number(var.get()):.7g}" for var in self.wave_values)
        return f"WAVE {self.motor} {kind} {args}"

    def update(self, sample):
        state = MODES[sample.mode]
        if sample.mode == 2:
            endpoint = "mínimo" if sample.endpoint == 1 else "máximo"
            state += f" ({endpoint}, {PHASES[sample.phase]})"
        if sample.fault:
            state += f" · {FAULTS[sample.fault]}"
        if sample.fault == STALL_FAULT and sample.fault_threshold > 0:
            if sample.calibrated != 3 and sample.endpoint in (1, 2):
                endpoint = "mínimo" if sample.endpoint == 1 else "máximo"
                state += f" · {PHASES[sample.phase]} do {endpoint}"
            state += f" a {sample.fault_speed:+.1f}% PWM (teto {sample.fault_threshold:.1f}%)"
        elif sample.fault == OVERSPEED_FAULT and sample.fault_threshold > 0:
            state += f" ({sample.fault_speed:.1f} > {sample.fault_threshold:.1f} °/s)"
        bounds = {0: "livres", 1: "mín.", 2: "máx.", 3: "ambos"}.get(sample.limits, "—")
        position = f"{sample.angle:.2f}° / {sample.span:.2f}°" if sample.calibrated == 3 else "curso pendente"
        enable = f"  |  EN na Pico: {sample.enable}" if sample.enable >= 0 else ""
        speed_reference = f"  |  Ref. de velocidade {sample.speed_setpoint:+.1f} °/s" if 3 <= sample.mode <= 7 else ""
        self.status.set(f"{state}  |  {position}  |  Fins {bounds}\n"
                        f"{sample.speed:+.1f} °/s{speed_reference}  |  PWM {sample.pwm:+.1f}%{enable}")


def number(raw):
    value = float(raw.replace(",", "."))
    if not isfinite(value):
        raise ValueError("Use um valor numérico finito.")
    return value


def cfg_command(motor, values):
    return f"CFG {motor} " + " ".join(f"{value:.7g}" for value in values)


class MotorTab:
    def __init__(self, parent, send):
        self.send = send
        self.settings, warning = load_settings()
        self.friction = dict.fromkeys(MOTORS, (0., 0., 0., 0.))
        try:
            data = json.loads(FRICTION_PATH.read_text(encoding="utf-8"))
            self.friction = {motor: tuple(map(float, data[str(motor)])) for motor in MOTORS}
            if any(len(pair) not in (2, 4) or any(not 0 <= value <= 100 for value in pair)
                   for pair in self.friction.values()):
                raise ValueError("limiar inválido")
            self.friction = {motor: (*pair, 0., 0.) if len(pair) == 2 else pair
                             for motor, pair in self.friction.items()}
        except FileNotFoundError:
            pass
        except (OSError, ValueError, KeyError, TypeError) as error:
            self.friction = dict.fromkeys(MOTORS, (0., 0., 0., 0.))
            warning = f"Falha ao carregar zona morta ({error})."
        self.configured = set()
        self.pending = {}
        self.applied = {}
        self.connected = False
        self.ready = False
        self.level_button = None
        self.level_stop_button = None
        self.level_message = StringVar()
        self.imu_reference, self.imu_file_warning = load_imu_reference()
        self.imu_message = StringVar()
        self.imu_buttons = []
        self.message = StringVar(value=warning or "Aguardando sessão de acionamento")
        self.warning = warning
        self.toolbar = ttk.Frame(parent)
        self.toolbar.pack(fill="x")
        self.buttons = []
        for text, command in (("Parar motores (Esc)", "STOP"), ("Soltar motores", "COAST"),
                              ("Calibrar M1 → M3", "CAL"), ("Calibrar só M3", "CAL 3")):
            button = ttk.Button(self.toolbar, text=text, command=lambda cmd=command: self.action(lambda: cmd),
                                state="disabled")
            button.pack(side="left", padx=(0, 5))
            self.buttons.append(button)
        self.limit_text = StringVar(value="Desativar limites V/A")
        self.limit_button = ttk.Button(self.toolbar, textvariable=self.limit_text,
                                       command=self.toggle_motion_limits, state="disabled")
        self.limit_button.pack(side="left", padx=(0, 5))
        self.limit_status = StringVar()
        ttk.Label(self.toolbar, textvariable=self.limit_status).pack(side="left", padx=(0, 5))
        self.circle = [StringVar(value="1"), StringVar(value="5")]
        for label, var in zip(("Amplitude °", "Período s"), self.circle):
            ttk.Label(self.toolbar, text=label).pack(side="left", padx=(4, 2))
            ttk.Entry(self.toolbar, width=5, textvariable=var).pack(side="left")
        button = ttk.Button(self.toolbar, text="Seno + cosseno", state="disabled",
                            command=lambda: self.action(self.run_circle))
        button.pack(side="left", padx=4)
        self.buttons.append(button)
        ttk.Label(parent, textvariable=self.message).pack(anchor="w", pady=4)
        layout = ttk.Frame(parent)
        layout.pack(fill="x")
        self.panels = {}
        for column, motor in enumerate(MOTORS):
            panel = MotorPanel(layout, motor, self.action, self.save_motor)
            panel.add_pwm_test(self.toggle_pwm, self.use_pwm_result)
            layout.columnconfigure(column, weight=1, uniform="controle")
            panel.frame.grid(row=0, column=column, sticky="nsew", padx=3)
            panel.set_values(self.settings[motor])
            self.panels[motor] = panel

        plot_bar = ttk.Frame(parent)
        plot_bar.pack(fill="x", pady=(6, 0))
        ttk.Button(plot_bar, text="Exportar motores CSV", command=self.export_samples).pack(side="right")
        self.plot_mode = StringVar(value="Degrau")
        ttk.Combobox(plot_bar, textvariable=self.plot_mode, state="readonly", width=16,
                     values=("Degrau", "Movimento ao vivo", "PWM ao vivo")).pack(side="left")
        self.plot_units = StringVar(value="Ângulo (°)")
        ttk.Combobox(plot_bar, textvariable=self.plot_units, state="readonly", width=12,
                     values=("Ângulo (°)", "Curso (%)")).pack(side="left", padx=(6, 0))
        self.scale_label = StringVar()
        ttk.Label(plot_bar, textvariable=self.scale_label).pack(side="left", padx=8)
        self.metrics = {motor: StringVar() for motor in MOTORS}
        for motor, metric in self.metrics.items():
            ttk.Label(parent, textvariable=metric).pack(anchor="w")
            metric.set(f"M{motor} · Aguardando degrau")
        figure = Figure(figsize=(9, 2.8), layout="constrained")
        self.axes = dict(zip(MOTORS, figure.subplots(1, 2)))
        self.traces = {}
        self.pwm_traces = {}
        self.speed_axes = {}
        self.showing_pwm = False
        for motor, axis in self.axes.items():
            axis.set(title=f"Motor {motor}", ylabel="Posição (°)", xlabel="Tempo (s)")
            axis.grid(alpha=.25)
            self.traces[motor] = [axis.plot([], [], style, label=label)[0] for style, label in (
                ("--", "Alvo"), (":", "Referência"), ("-", "Encoder"),
            )]
            self.pwm_traces[motor], = axis.plot([], [], color="tab:purple", label="PWM aplicado", visible=False)
            speed_axis = axis.twinx()
            speed_axis.set_ylabel("Velocidade (°/s)")
            speed_axis.set_visible(False)
            speed_axis.plot([], [], color="tab:green", label="Velocidade medida")
            speed_axis.plot([], [], ":", color="tab:orange", label="Ref. de velocidade")
            self.speed_axes[motor] = speed_axis
            axis.legend(handles=self.traces[motor], fontsize=8)
        self.canvas = FigureCanvasTkAgg(figure, master=parent)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self.reset()

    def set_buttons(self):
        for index, button in enumerate(self.buttons):
            enabled = self.connected if index == 0 else self.ready
            button.configure(state="normal" if enabled else "disabled")
        self.limit_button.configure(state="normal" if self.ready and self.limits_enabled is not None else "disabled")
        if self.level_button is not None:
            self.level_button.configure(
                state="normal" if self.ready and self.imu_ready and self.imu_pending is None
                and not self.level_running else "disabled")
            self.level_stop_button.configure(state="normal" if self.connected else "disabled")
        for button in self.imu_buttons:
            available = self.ready and self.imu_ready and not self.level_running and self.imu_pending is None
            button.configure(state="normal" if available else "disabled")
        for panel in self.panels.values():
            for button in panel.buttons:
                button.configure(state="normal" if self.ready else "disabled")
            selected = self.pwm_motor == panel.motor
            panel.pwm_button.configure(text="Parar teste" if selected else "Iniciar teste",
                state="normal" if self.ready and (selected or self.pwm_motor is None) else "disabled")
            panel.pwm_slider.configure(state="normal" if selected and self.pwm_ack else "disabled")
            panel.pwm_direction_box.configure(state="disabled" if selected else "readonly")
            panel.pwm_use.configure(state="normal" if self.ready and self.pwm_motor is None
                                    and panel.motor in self.pwm_results else "disabled")

    def reset(self):
        self.ready = self.connected = False
        self.compatible = False
        self.fast_telemetry = False
        self.pwm_motor = None
        self.pwm_ack = False
        self.pwm_results = {}
        self.pwm_next = 0
        self.retry_at = 0
        self.configured.clear()
        self.pending.clear()
        self.applied.clear()
        self.level_report = None
        self.level_running = False
        self.level_message.set("Aguardando o Pico")
        self.imu_ready = False
        self.imu_pending = None
        self.imu_retry_at = 0
        self.imu_message.set(self.imu_file_warning)
        self.set_motion_limits(None)
        self.samples = {motor: deque(maxlen=2000) for motor in MOTORS}
        self.steps = {motor: [] for motor in MOTORS}
        self.step_result = {motor: "" for motor in MOTORS}
        self.set_scale(ENCODER_RESOLUTION_DEG)
        self.message.set(self.warning or "Aguardando sessão de acionamento")
        for motor, panel in self.panels.items():
            panel.status.set("Aguardando firmware de acionamento")
            panel.gain_status.set("")
            panel.active_limits.set("Aguardando limites confirmados pelo Pico")
            self.metrics[motor].set(f"M{motor} · Aguardando degrau")
            panel.pwm_message.set("")
            self.show_friction(motor)
        self.set_buttons()

    def set_scale(self, resolution):
        self.resolution = resolution
        self.scale_label.set(f"Encoder {resolution:.5g}°/contagem · Acomodação em ±1 contagem")
        for motor, values in self.applied.items():
            self.panels[motor].describe_applied_gain(values, resolution)

    def add_level_controls(self, parent):
        level_row = ttk.Frame(parent)
        level_row.pack(fill="x")
        imu_row = ttk.Frame(parent)
        imu_row.pack(fill="x", pady=(4, 0))
        self.level_button = ttk.Button(level_row, text="Calibrar mesa", command=lambda: self.action(lambda: "LEVEL"))
        self.level_button.pack(side="left", padx=(0, 6))
        self.level_stop_button = ttk.Button(level_row, text="Parar (Esc)", command=lambda: self.action(lambda: "STOP"))
        self.level_stop_button.pack(side="left", padx=(0, 10))
        ttk.Label(level_row, textvariable=self.level_message).pack(side="left")
        for label, command in (("Zerar IMU no plano atual", "IMUZERO"), ("Zerar Z relativo", "IMUYAW"),
                               ("Limpar referência de inclinação", imu_reference_command(DEFAULT_IMU_REFERENCE))):
            button = ttk.Button(imu_row, text=label, command=lambda cmd=command: self.imu_action(cmd))
            button.pack(side="left", padx=(0, 6))
            self.imu_buttons.append(button)
        ttk.Label(imu_row, textvariable=self.imu_message).pack(side="left")
        self.set_buttons()

    def imu_action(self, command):
        if not (self.ready and self.imu_ready and self.imu_pending is None and not self.level_running):
            return
        self.imu_pending = command.split()[0]
        self.imu_command_deadline = monotonic() + 2
        self.imu_message.set("Aguardando confirmação do Pico")
        self.set_buttons()
        self.send(command)

    def set_motion_limits(self, enabled):
        self.limits_enabled = enabled
        self.limit_text.set("Reativar limites V/A" if enabled is False else "Desativar limites V/A")
        self.limit_status.set({None: "", True: "Limites V/A ativos", False: "Limites V/A desativados"}[enabled])

    def toggle_motion_limits(self):
        if self.ready and self.limits_enabled is not None:
            self.action(lambda: f"LIMITS {int(not self.limits_enabled)}")

    def run_circle(self):
        amplitude, period = [number(var.get()) for var in self.circle]
        return f"CIRCLE {amplitude:.7g} {period:.7g}"

    def check_sync(self, now):
        if self.compatible and not self.ready and now >= self.retry_at:
            self.retry_at = now + 2
            self.send("HELLO")
        if self.compatible and not self.imu_ready and now >= self.imu_retry_at:
            self.imu_retry_at = now + 2
            self.send("IMUGET")
        if self.imu_pending is not None and now >= self.imu_command_deadline:
            self.imu_pending = None
            self.imu_message.set("O Pico não confirmou o ajuste. Tente novamente.")
            self.set_buttons()

    def action(self, make_command):
        try:
            command = make_command()
        except (ValueError, ArithmeticError, TclError) as error:
            self.message.set(f"Entrada inválida. {error}")
            return
        if not (self.ready or self.connected and command == "STOP"):
            return
        if command == "COAST" and not messagebox.askyesno(
            "Soltar motores", "A mesa pode se mover com o próprio peso. Soltar os dois motores?",
            parent=self.toolbar,
        ):
            return
        if command.startswith("CFG "):
            motor = int(command.split()[1])
            if motor in self.pending:
                self.message.set(f"Motor {motor} · Aguarde a confirmação do Pico.")
                return
            self.pending[motor] = tuple(map(float, command.split()[2:]))
        elif command in ("CAL", "CIRCLE", "LEVEL") or command.startswith(("CAL ", "GO ", "STEP ", "WAVE ")):
            motors = MOTORS if command in ("CAL", "CIRCLE", "LEVEL") else (int(command.split()[1]),)
            if not self.check_applied(motors, calibration=command.split()[0] == "CAL"):
                if command == "LEVEL":
                    self.level_message.set(self.message.get())
                return
            if command == "LEVEL":
                self.level_message.set("Aguardando confirmação do Pico")
        if command.startswith("STEP "):
            self.plot_mode.set("Degrau")
        elif command.startswith(("GO ", "WAVE ", "CIRCLE ")):
            self.plot_mode.set("Movimento ao vivo")
        self.send(command)
        self.message.set(f"Enviado · {command.split()[0]}")

    def check_applied(self, motors, calibration=False):
        # Homing uses motion limits, duty and polarity. The position PIDF is independent.
        fields = slice(4, 10) if calibration else slice(None)
        for motor in motors:
            if motor in self.pending:
                self.message.set(f"Motor {motor} · Aguarde a confirmação do Pico.")
                return False
            try:
                edited = self.panels[motor].values()
            except (ValueError, TclError) as error:
                self.message.set(f"Motor {motor} · {error}")
                return False
            applied = self.applied.get(motor)
            if applied is None or not same_settings(edited[fields], applied[fields]):
                self.message.set(f"Motor {motor} · Aplique os ajustes e aguarde a confirmação para iniciar.")
                return False
        return True

    def show_friction(self, motor):
        plus, minus, running_plus, running_minus = (
            f"{value:.2f}%" if value else "a medir" for value in self.friction[motor]
        )
        self.panels[motor].pwm_saved.set(
            f"Partida + {plus} · − {minus}  |  Sustentação + {running_plus} · − {running_minus}"
        )

    def send_friction(self, motor):
        values = " ".join(f"{value:.2f}" for value in self.friction[motor])
        self.send(f"FRICTION {motor} {values}")

    def toggle_pwm(self, motor):
        if self.pwm_motor is not None:
            self.finish_pwm("Teste parado", brake=True)
            return
        if not self.ready or not self.check_applied((motor,), calibration=True):
            return
        panel = self.panels[motor]
        panel.pwm_slider.configure(to=self.applied[motor][8])
        panel.pwm_target.set(0)
        panel.pwm_label.set("0,00%")
        self.pwm_motor, self.pwm_ack = motor, False
        self.pwm_next = monotonic() + .35
        self.pwm_results.pop(motor, None)
        self.plot_mode.set("PWM ao vivo")
        panel.pwm_message.set("Aguardando confirmação do Pico")
        direction = 1 if panel.pwm_direction.get().endswith("(+)") else -1
        command = f"PWMSTART {motor} {direction}"
        self.send(command + " 1" if panel.pwm_sustain.get() else command)
        self.set_buttons()

    def finish_pwm(self, message, brake=False):
        if self.pwm_motor is None:
            return
        self.panels[self.pwm_motor].pwm_message.set(message)
        self.pwm_motor, self.pwm_ack = None, False
        if brake:
            self.send("STOP")
        self.set_buttons()

    def tick_pwm(self, now, visible):
        if self.pwm_motor is None:
            return
        if not visible:
            self.finish_pwm("Teste parado ao sair da aba", brake=True)
        elif now >= self.pwm_next:
            if not self.pwm_ack:
                self.finish_pwm("O Pico não confirmou o teste", brake=True)
                return
            motor = self.pwm_motor
            self.send(f"PWMVAL {motor} {self.panels[motor].pwm_target.get():.2f}")
            self.pwm_next = now + .08

    def receive_pwm(self, fields):
        try:
            motor, state, direction = map(int, fields[:3])
            duty = float(fields[3])
        except ValueError:
            return
        if motor != self.pwm_motor or not self.pwm_ack:
            return
        if state in (2, 4) and direction in (-1, 1) and 0 < duty <= 100:
            self.pwm_results[motor] = (direction, duty, state == 4)
            result = "Sustentou movimento" if state == 4 else "Saiu do repouso"
            self.finish_pwm(f"{result} a {duty:.2f}% PWM · {'subida' if direction > 0 else 'descida'}")
        elif state == 3:
            self.finish_pwm("Teste interrompido pelo Pico · confira o estado do motor")

    def use_pwm_result(self, motor):
        if not self.ready or self.pwm_motor is not None or motor not in self.pwm_results:
            return
        direction, duty, sustain = self.pwm_results[motor]
        measured = list(self.friction[motor])
        measured[2 * sustain + (direction < 0)] = duty
        values = {**self.friction, motor: tuple(measured)}
        temporary = FRICTION_PATH.with_suffix(".tmp")
        try:
            temporary.write_text(json.dumps(values, indent=2) + "\n", encoding="utf-8")
            temporary.replace(FRICTION_PATH)
        except OSError as error:
            self.panels[motor].pwm_message.set(f"Falha ao salvar ({error})")
            return
        self.friction = values
        self.send_friction(motor)
        self.show_friction(motor)
        self.panels[motor].pwm_message.set(f"Limiar de {duty:.2f}% salvo e enviado ao Pico")

    def receive(self, line):
        if line.startswith(b"@,"):
            sample = MotorSample.parse(line.split(b",")[1:])
            if sample is not None:
                self.record(sample)
            return sample
        fields = line.decode("ascii", errors="replace").split(",")
        if line in LEGACY_HELLOS:
            self.connected = True
            self.compatible = self.ready = False
            self.fast_telemetry = line in FAST_TELEMETRY_HELLOS
            self.message.set("Firmware anterior detectado. Grave o UF2 atualizado para liberar o acionamento.")
            self.level_message.set("Grave o firmware atualizado para nivelar")
            self.set_buttons()
        elif line == PROTOCOL_HELLO:
            self.finish_pwm("Teste interrompido pela reconexão")
            self.connected = True
            self.compatible = True
            self.fast_telemetry = True
            self.ready = False
            self.retry_at = monotonic() + 2
            self.configured.clear()
            self.pending.clear()
            self.applied.clear()
            self.imu_ready = False
            self.imu_pending = None
            self.imu_retry_at = monotonic() + 2
            self.set_motion_limits(None)
            self.message.set("Sincronizando parâmetros com o Pico")
            self.set_buttons()
            for motor in MOTORS:
                self.send_friction(motor)
        elif fields[0] == "!SCALE" and len(fields) == 2:
            try:
                resolution = number(fields[1])
            except ValueError:
                return
            if resolution > 0:
                self.set_scale(resolution)
        elif fields in (["!LIMITS", "0"], ["!LIMITS", "1"]) and self.compatible:
            self.set_motion_limits(fields[1] == "1")
            self.set_buttons()
        elif fields[0] == "!LEVEL" and len(fields) == 5 and self.compatible:
            self.receive_level(fields[1:])
        elif fields[0] == "!PWMTEST" and len(fields) == 5 and self.compatible:
            self.receive_pwm(fields[1:])
        elif fields[0] == "!IMUCFG" and len(fields) == 4 and self.compatible:
            self.receive_imu_reference(fields[1:])
        elif fields[0] == "!CFG" and len(fields) == 13 and self.compatible:
            try:
                motor = int(fields[1])
                values = tuple(map(float, fields[2:]))
            except ValueError:
                return
            if motor in MOTORS and not settings_errors(values):
                self.confirm_settings(motor, values)
        elif fields[0] == "!ERR" and len(fields) in (3, 4):
            command, reason = fields[1:3]
            if command == "PING":
                self.reset()
                return
            if command == "CFG":
                if len(fields) == 4 and fields[3] in ("1", "3"):
                    self.pending.pop(int(fields[3]), None)
                else:
                    self.pending.clear()
            self.message.set(f"{command} · {ERRORS.get(reason, reason.replace('_', ' '))}")
            if command == "LEVEL":
                self.level_message.set(ERRORS.get(reason, reason.replace("_", " ")))
            if command in ("PWMSTART", "PWMVAL"):
                self.finish_pwm(ERRORS.get(reason, reason.replace("_", " ")), brake=self.pwm_ack)
            if command in ("IMUZERO", "IMUYAW", "IMUCFG"):
                self.imu_pending = None
                self.imu_message.set(ERRORS.get(reason, reason.replace("_", " ")))
                self.set_buttons()
        elif fields[0] == "!OK" and len(fields) == 2:
            if fields[1] == "PWMSTART" and self.pwm_motor is not None:
                self.pwm_ack = True
                self.pwm_next = 0
                panel = self.panels[self.pwm_motor]
                panel.pwm_message.set("Inicie o movimento e reduza em passos, mantendo cada nível ≥ 300 ms"
                                      if panel.pwm_sustain.get() else
                                      "Suba em pequenos passos e aguarde o PWM aplicado")
                self.set_buttons()
            if fields[1] == "IMUYAW":
                self.imu_pending = None
                self.imu_message.set("Z relativo zerado")
                self.set_buttons()
            if fields[1] == "STOP":
                self.finish_pwm("Teste parado")
                self.pending.clear()
                if self.imu_pending is not None:
                    self.imu_pending = None
                    self.imu_message.set("Ajuste da IMU interrompido")
                    self.set_buttons()
                if self.compatible and not self.ready:
                    self.send("HELLO")
            self.message.set(f"Confirmado pelo Pico · {fields[1]}")

    def receive_imu_reference(self, fields):
        try:
            values = parse_imu_reference(fields)
        except ValueError:
            return
        pending = self.imu_pending
        if pending in ("IMUZERO", "IMUCFG"):
            self.imu_reference = values
            temporary = IMU_REFERENCE_PATH.with_suffix(".tmp")
            try:
                temporary.write_text(" ".join(f"{value:.8g}" for value in values) + "\n", encoding="utf-8")
                temporary.replace(IMU_REFERENCE_PATH)
                self.imu_file_warning = ""
                self.imu_message.set("Referência da IMU salva" if pending == "IMUZERO" else "Inclinação de referência restaurada")
            except OSError as error:
                self.imu_message.set(f"Referência aplicada. Falha ao salvar ({error}).")
        elif not self.imu_ready and not same_settings(values, self.imu_reference):
            self.send(imu_reference_command(self.imu_reference))
            return
        elif not self.imu_ready:
            self.imu_message.set(self.imu_file_warning or ("Referência da IMU carregada"
                                 if not same_settings(values, DEFAULT_IMU_REFERENCE) else "Montagem padrão"))
        self.imu_ready = True
        if pending != "IMUYAW":
            self.imu_pending = None
        self.set_buttons()

    def receive_level(self, fields):
        try:
            state = int(fields[0])
            tilt, m1, m3 = map(number, fields[1:])
        except ValueError:
            return
        report = (state, tilt, m1, m3)
        if state not in range(len(LEVEL_MESSAGES)) or report == self.level_report:
            return
        self.level_report = report
        self.level_running = state in range(1, 5)
        status = LEVEL_MESSAGES[state]
        if tilt >= 0:
            status += f" · Inclinação {tilt:.2f}°"
        if state == 5:
            status += f" · M1 {m1:.2f}° · M3 {m3:.2f}°"
        self.level_message.set(status)
        self.set_buttons()

    def confirm_settings(self, motor, values):
        pending = self.pending.pop(motor, None)
        if self.ready and pending is None:
            return
        if pending is None and not same_settings(values, self.settings[motor]):
            self.pending[motor] = self.settings[motor]
            self.send(cfg_command(motor, self.settings[motor]))
            return
        if pending is not None and not same_settings(values, pending):
            self.message.set(f"Parâmetros do motor {motor} diferem dos enviados.")
            return
        self.applied[motor] = values
        self.panels[motor].describe_applied_gain(values, self.resolution)
        self.configured.add(motor)
        if self.ready:
            self.message.set(f"Motor {motor} · Parâmetros aplicados ao Pico.")
        else:
            self.panels[motor].set_values(values)
        if not self.ready and self.configured == set(MOTORS):
            self.ready = True
            self.set_buttons()
            self.message.set(self.warning or "Acionamento conectado")

    def save_motor(self, motor):
        if not self.ready or motor in self.pending:
            self.message.set(f"Motor {motor} · Aguarde a confirmação do Pico.")
            return
        try:
            values = self.panels[motor].values()
            if not same_settings(values, self.applied[motor]):
                self.message.set(f"Motor {motor} · Aplique os valores antes de salvar.")
                return
            settings = {**self.settings, motor: self.applied[motor]}
            save_settings(settings)
        except (ValueError, TclError, OSError) as error:
            self.message.set(f"Motor {motor} · Falha ao salvar ({error}).")
            return
        self.settings = settings
        self.warning = ""
        self.message.set(f"Motor {motor} · Parâmetros salvos no arquivo.")

    def record(self, sample):
        motor = sample.motor
        stream = self.samples[motor]
        if stream and sample.timestamp < stream[-1].timestamp:
            stream.clear()
            self.steps[motor].clear()
        stream.append(sample)
        step = self.steps[motor]
        if sample.mode == 4:
            if step and step[0].step != sample.step:
                step.clear()
            if not step and sample.elapsed > 0:
                initial = sample.step_origin if sample.step_origin is not None else sample.goal - 1
                step.append(sample._replace(elapsed=0, angle=initial, reference=initial))
            step.append(sample)
            self.step_result[motor] = "Em andamento"
        elif step and self.step_result[motor] == "Em andamento":
            if sample.mode == 0 and not sample.fault and step[-1].elapsed >= 5.90:
                self.step_result[motor] = "Concluído"
            else:
                self.step_result[motor] = "Interrompido"

    def export_samples(self):
        path = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV", "*.csv")])
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8", newline="") as output:
                writer = csv.writer(output)
                writer.writerow(("motor", "pico_timestamp_us", "mode", "fault", "calibrated", "limits", "phase",
                                 "endpoint", "angle_deg", "goal_deg", "reference_deg", "speed_deg_s", "span_deg",
                                 "pwm_pct", "step", "elapsed_s", "enable", "fault_speed", "fault_threshold",
                                 "speed_setpoint_deg_s", "step_origin_deg", "bridge_state"))
                writer.writerows(sorted((*self.samples[1], *self.samples[3]), key=lambda row: row.timestamp))
        except OSError as error:
            self.message.set(f"Falha ao exportar CSV ({error})")

    def draw(self):
        plot_mode = self.plot_mode.get()
        show_step = plot_mode == "Degrau"
        show_pwm = plot_mode == "PWM ao vivo"
        show_travel = self.plot_units.get() == "Curso (%)"
        for motor, axis in self.axes.items():
            speed_axis = self.speed_axes[motor]
            stream = self.samples[motor]
            if stream:
                self.panels[motor].update(stream[-1])
            step = self.steps[motor]
            result = self.step_result[motor]
            if step:
                metrics = step_metrics(step, result == "Concluído", self.resolution)
                self.metrics[motor].set(f"M{motor} · {result} · {metrics}")
            rows = step if show_step else stream
            times = [row.elapsed for row in rows] if show_step else (
                [(row.timestamp - stream[-1].timestamp) * 1e-6 for row in rows] if stream else [])
            if show_pwm != self.showing_pwm:
                for trace in self.traces[motor]:
                    trace.set_visible(not show_pwm)
                self.pwm_traces[motor].set_visible(show_pwm)
                speed_axis.set_visible(show_pwm)
                handles = [self.pwm_traces[motor], *speed_axis.lines] if show_pwm else self.traces[motor]
                axis.legend(handles=handles, fontsize=8)
            axis.set_ylabel("PWM (%)" if show_pwm else "Curso (%)" if show_travel else "Posição (°)")
            if show_pwm:
                self.pwm_traces[motor].set_data(times, [row.pwm for row in rows])
                measured, reference = speed_axis.lines
                measured.set_data(times, [abs(row.speed) if row.mode == 2 else row.speed for row in rows])
                reference.set_data(times, [row.speed_setpoint for row in rows])
                speed_axis.relim()
                speed_axis.autoscale_view(scalex=False)
            else:
                for trace, field in zip(self.traces[motor], ("goal", "reference", "angle")):
                    data = [getattr(row, field) for row in rows]
                    if show_travel:
                        data = [100 * value / row.span if row.calibrated == 3 and row.span > 0 else float("nan")
                                for value, row in zip(data, rows)]
                    trace.set_data(times, data)
            axis.set_xlim((0, 6) if show_step else (-10, 0))
            axis.relim(visible_only=True)
            axis.autoscale_view(scalex=False)
        self.showing_pwm = show_pwm
        self.canvas.draw_idle()
