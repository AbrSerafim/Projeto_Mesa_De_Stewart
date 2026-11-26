"""Standalone ball and USB diagnostics."""

import argparse
from collections import Counter
import csv
import json
from math import ceil, isfinite
from secrets import randbelow
import sys
from time import monotonic, sleep

from ball_control import SEND_PERIOD, BallControl, BallRegulator
from controle_motores import PROTOCOL_HELLO, MotorSample, SerialSession
from visualizar_imu import SerialLines


def await_item(session, queue, predicate, timeout):
    deadline = monotonic() + timeout
    while monotonic() < deadline:
        session.ui_at = monotonic()
        if session.error:
            raise OSError(session.error)
        while queue:
            item = queue.popleft()
            if predicate(item):
                return item
        sleep(.001)
    return None


def serial_diagnostic(args):
    if args.mock:
        from pico_mock import PicoMock
        port = PicoMock()
    else:
        from serial import Serial
        port = Serial(args.port, baudrate=115200, timeout=.01, write_timeout=.15)
    connection = SerialSession(port, SerialLines())
    try:
        hello = await_item(connection, connection.incoming, lambda item: item[1] == PROTOCOL_HELLO, 3)
        if hello is None:
            raise OSError("Handshake v13 ausente")
        identifier = randbelow(2**32 - 1) + 1
        connection.send(f"TELEM {args.telemetry_hz}")
        connection.send(f"BALLTEST2 {identifier}")
        expected = f"!BALLSTATE2,{identifier},5".encode()
        if await_item(connection, connection.incoming, lambda item: item[1] == expected, .4) is None:
            raise OSError("Diagnóstico sem movimento recusado. Use STOP e aguarde os motores pararem.")
        outcomes = Counter()
        rtts = []
        next_at = monotonic()
        with open(args.output, "w", encoding="utf-8", newline="") as output:
            writer = csv.writer(output)
            writer.writerow(("pc_s", "session", "seq", "event", "m1_pct", "m3_pct", "rtt_s", "pico_mutex_us"))
            for index in range(args.count):
                if not args.mock:
                    sleep(max(0, next_at - monotonic()))
                    next_at = monotonic() + .05
                connection.ui_at = monotonic()
                for _ in range(len(connection.incoming)):
                    connection.incoming.popleft()
                m1, m3 = 38 + (index % 13) / 10, 42 - (index % 17) / 10
                connection.send(f"TARGET {identifier} {m1:.4f} {m3:.4f}")

                def record(event):
                    writer.writerow(event)
                    if event.outcome == "sent":
                        return False
                    outcomes[event.outcome] += 1
                    if event.rtt_s is not None:
                        rtts.append(event.rtt_s)
                    return True

                if await_item(connection, connection.ball.events, record, .35) is None:
                    raise OSError("Sem resultado para a referência pendente")
        if rtts:
            p95 = sorted(rtts)[ceil(.95 * len(rtts)) - 1]
            print(f"RTT p95 = {p95 * 1000:.2f} ms")
        print(f"{args.count} comandos: {dict(outcomes)}")
        return int(outcomes["diagnostic"] != args.count)
    finally:
        connection.close()


def ball_diagnostic(args):
    configs, geometry, _, warning = BallControl.load()
    if warning:
        raise ValueError(warning)
    controller = BallRegulator(configs, geometry)
    rows = [(index * .05, 320 - index * 10, 240 + index * 4) for index in range(11)]
    if args.input:
        with open(args.input, encoding="utf-8", newline="") as input_file:
            rows = [(float(row["capture_s"]), row["x_px"], row["y_px"]) for row in csv.DictReader(input_file)]
    with open(args.output, "w", encoding="utf-8", newline="") as output:
        writer = csv.writer(output)
        labels = tuple(f"{axis}_{term}" for axis in ("x", "y")
                       for term in ("error_px", "p_pp", "i_pp", "d_pp", "raw_pp", "sat_pp"))
        writer.writerow(("capture_s", "m1_pct", "m3_pct", *labels))
        for at, x, y in rows:
            measurement = None if x == "" or y == "" else (float(x), float(y))
            result = controller.update(at, (args.target_x, args.target_y), measurement)
            if result is None:
                continue
            targets, terms = result
            writer.writerow((at, *targets, *(value for axis in terms for value in axis)))
    return 0


def motor_replay(args):
    configs, geometry, _, warning = BallControl.load()
    if warning:
        raise ValueError(warning)
    with open(args.input, encoding="utf-8", newline="") as source:
        plan = [(float(row["t_s"]), float(row["m1_pct"]), float(row["m3_pct"]))
                for row in csv.DictReader(source)]
    if not plan or plan[0][0] != 0:
        raise ValueError("O ensaio exige uma referência no instante zero.")
    for index, (at, m1, m3) in enumerate(plan):
        if not all(map(isfinite, (at, m1, m3))) or (
            index and not .049999 <= at - plan[index - 1][0] <= .300001
        ):
            raise ValueError("Use instantes finitos e intervalos entre 50 e 300 ms.")
        if any(not cfg.course_min <= value <= cfg.course_max for cfg, value in zip(reversed(configs), (m1, m3))):
            raise ValueError(f"Alvo fora da faixa na linha {index + 2}.")

    if args.mock:
        from pico_mock import PicoMock
        port = PicoMock(telemetry=True)
    else:
        from serial import Serial
        port = Serial(args.port, baudrate=115200, timeout=.01, write_timeout=.15)
    connection = SerialSession(port, SerialLines(), ack_timeout=geometry.ack_timeout_s, ball_period=SEND_PERIOD)
    identifier = randbelow(2**32 - 1) + 1
    session_field = str(identifier).encode()

    try:
        if await_item(connection, connection.incoming, lambda item: item[1].startswith(b"!BALLLINK2,"), 3) is None:
            raise OSError("Handshake de autocorreção ausente")
        profile = (value for cfg, neutral in zip(reversed(configs), geometry.neutral)
                   for value in (cfg.course_min, cfg.course_max, cfg.pwm_max, neutral))
        with open(args.output, "w", encoding="utf-8", newline="") as output:
            log = csv.writer(output)
            log.writerow(("pc_monotonic_s", "record", "fields"))
            connection.send(f"TELEM {args.telemetry_hz}")
            connection.send(f"BALLSTART2 {identifier} " + " ".join(f"{value:.4f}" for value in profile))
            phase = None

            def drain():
                nonlocal phase
                connection.ui_at = monotonic()
                if connection.error:
                    raise OSError(connection.error)
                while connection.incoming:
                    at, line = connection.incoming.popleft()
                    parts = line.split(b",")
                    if parts[0] == b"!BALLSTARTERR" and parts[1] == session_field:
                        raise OSError(line.decode("ascii", "replace"))
                    if parts[0] == b"!BALLSTATE2" and parts[1] == session_field:
                        phase = int(parts[2])
                        if phase in (0, 2, 3):
                            raise OSError(line.decode("ascii", "replace"))
                    if parts[0] == b"@":
                        motor = MotorSample.parse(parts[1:])
                        if motor:
                            log.writerow((at, "motor", json.dumps(motor._asdict())))
                    elif parts[0].startswith((b"!BALLSTATE", b"!BALLF", b"!MOTORDIAG", b"!COMMDIAG")):
                        log.writerow((at, parts[0].decode(), json.dumps([part.decode("ascii", "replace") for part in parts[1:]])))
                while connection.ball.events:
                    event = connection.ball.events.popleft()
                    log.writerow((event.at, "target", json.dumps(event._asdict())))
                    if event.outcome.startswith("nack:"):
                        raise OSError(event.outcome)

            started = monotonic()
            next_neutral = started
            while phase != 1:
                drain()
                now = monotonic()
                if (phase is None and now - started >= .4) or now - started >= 46:
                    raise OSError("O Pico não confirmou a centralização.")
                if phase == 4 and now >= next_neutral:
                    connection.send(f"TARGET {identifier} {geometry.neutral[0]:.4f} {geometry.neutral[1]:.4f}")
                    next_neutral = now + SEND_PERIOD
                sleep(.004)
            start = monotonic()
            for at, m1, m3 in plan:
                while monotonic() < start + at:
                    drain()
                    sleep(.002)
                connection.send(f"TARGET {identifier} {m1:.4f} {m3:.4f}")
            end = monotonic() + geometry.ack_timeout_s + SEND_PERIOD
            while monotonic() < end:
                drain()
                sleep(.002)
            connection.send("STOP")
            await_item(connection, connection.incoming, lambda item: item[1] == b"!OK,STOP", .35)
        return 0
    finally:
        connection.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="mode", required=True)
    usb = commands.add_parser("serial", help="ACK/RTT com acionamento desabilitado")
    device = usb.add_mutually_exclusive_group(required=True)
    device.add_argument("--mock", action="store_true")
    device.add_argument("--port")
    usb.add_argument("--count", type=int, default=1000)
    usb.add_argument("--telemetry-hz", type=int, choices=(25, 200), default=25)
    usb.add_argument("--output", required=True)
    ball = commands.add_parser("bola", help="Controlador com posições sintéticas ou gravadas")
    ball.add_argument("--input")
    ball.add_argument("--output", required=True)
    ball.add_argument("--target-x", type=float, default=320)
    ball.add_argument("--target-y", type=float, default=240)
    motors = commands.add_parser("motores", help="Reproduzir uma sequência de curso absoluto M1/M3")
    device = motors.add_mutually_exclusive_group(required=True)
    device.add_argument("--mock", action="store_true")
    device.add_argument("--port")
    motors.add_argument("--autorizar-movimento", action="store_true")
    motors.add_argument("--input", required=True)
    motors.add_argument("--output", required=True)
    motors.add_argument("--telemetry-hz", type=int, choices=(25, 200), default=25)
    args = parser.parse_args()
    try:
        if args.mode == "motores":
            if not (args.mock or args.autorizar_movimento):
                parser.error("O ensaio físico requer --autorizar-movimento.")
            return motor_replay(args)
        if args.mode == "serial" and args.count <= 0:
            parser.error("--count precisa ser positivo")
        return serial_diagnostic(args) if args.mode == "serial" else ball_diagnostic(args)
    except (OSError, ValueError, KeyError) as error:
        print(error, file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
