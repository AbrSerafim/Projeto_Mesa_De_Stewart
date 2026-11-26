import math
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from ball_control import AxisConfig, BallControl, BallGeometry, BallRegulator, BallTracking
from ball_protocol import BallWindow
from controle_motores import MotorSample, SerialSession, step_metrics
from pico_mock import PicoMock
from visualizar_imu import SerialLines
from yolo_detection import Detection


def wait_for(predicate, timeout=1):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(.002)
    return False


class BallEquationTests(unittest.TestCase):
    def regulator(self, mode="PIDF", **kwargs):
        config = AxisConfig(mode=mode, **kwargs)
        return BallRegulator((config, config), BallGeometry())

    def test_backward_euler_modes(self):
        for mode, integral, derivative in (("P", 0, 0), ("PI", .08, 0),
                                           ("PID", .08, -.3), ("PIDF", .08, -.15)):
            with self.subTest(mode=mode):
                r = self.regulator(mode, kp=.02, ki=.1, kd=.03)
                r.update(0, (10, 10), (1, 1))
                _, terms = r.update(.1, (10, 10), (2, 2))
                self.assertAlmostEqual(terms[0].p, .16)
                self.assertAlmostEqual(terms[0].i, integral)
                self.assertAlmostEqual(terms[0].d, derivative)
                self.assertAlmostEqual(terms[0].raw, .16 + integral + derivative)

    def test_derivative_on_measurement_polarity_and_no_kick(self):
        r = self.regulator(kp=.02, ki=0, kd=.03, invert=True)
        r.update(0, (10, 10), (1, 1))
        self.assertAlmostEqual(r.update(.1, (10, 10), (2, 2))[1][0].d, .15)
        terms = r.update(.2, (20, 20), (2, 2))[1][0]
        self.assertAlmostEqual(terms.d, .075)
        self.assertAlmostEqual(terms.p, -.36)

    def test_age_repeat_and_reset(self):
        r = self.regulator(ki=.1, kd=.03)
        first = r.update(1, (10, 10), (2, 2))[1][0]
        self.assertEqual((first.i, first.d), (0, 0))
        self.assertIsNone(r.update(1, (10, 10), (9, 9)))
        self.assertIsNone(r.update(.9, (10, 10), (3, 3)))
        self.assertIsNone(r.update(math.nan, (10, 10), (3, 3)))
        self.assertEqual(r.update(1.0001, (10, 10), (4, 4))[1][0].i, 0)
        self.assertGreater(r.update(1.1, (10, 10), (4, 4))[1][0].i, 0)
        self.assertEqual(r.update(2, (10, 10), (3, 3))[1][0].i, 0)
        self.assertIsNone(r.update(2.1, (10, 10), None))
        recovered = r.update(2.2, (10, 10), (5, 5))[1][0]
        self.assertEqual((recovered.i, recovered.d), (0, 0))

    def test_axis_and_motor_anti_windup(self):
        r = self.regulator("PI", kp=1, ki=2)
        r.update(0, (50, 50), (0, 0))
        for index in range(1, 21):
            targets, terms = r.update(index * .1, (50, 50), (0, 0))
            self.assertEqual(targets, (42, 42))
            self.assertEqual((terms[0].i, terms[0].raw, terms[0].saturated), (0, 50, 2))
        x = AxisConfig(mode="PI", kp=.1, ki=.2)
        y = x._replace(course_max=40.3)
        r = BallRegulator((x, y), BallGeometry(matrix=((1, 1), (1, -1))))
        r.update(0, (2, 2), (0, 0))
        targets, terms = r.update(.1, (2, 2), (0, 0))
        self.assertEqual(targets, (40.3, 40))
        self.assertEqual((terms[0].i, terms[1].i), (0, 0))
        self.assertLess(r.update(.2, (-1, -1), (0, 0))[0][0], 40)

    def test_conservative_profile_and_motor_order(self):
        configs, geometry, _, warning = BallControl.load()
        self.assertFalse(warning)
        self.assertEqual(configs[0].kp, .02)
        r = BallRegulator(configs, geometry)
        targets, _ = r.update(0, (10, -20), (0, 0))
        self.assertAlmostEqual(targets[0], 40.4)
        self.assertAlmostEqual(targets[1], 39.8)
        self.assertEqual(BallGeometry(((2, -1), (0, 1)), (41, 38)).mix((1, .25)), (42.75, 38.25))

    def test_config_validation_and_optional_minimum(self):
        for options in (dict(tf=0, kd=1), dict(ki=math.nan), dict(limit=math.inf),
                        dict(invert=1), dict(correction_min=3)):
            with self.subTest(options=options), self.assertRaises(ValueError):
                AxisConfig(**options).validate()
        with self.assertRaises(ValueError):
            BallGeometry(neutral=(0, 40)).validate((AxisConfig(), AxisConfig()))
        r = self.regulator("P", kp=.02, correction_min=1)
        self.assertAlmostEqual(r.update(0, (2, 2), (0, 0))[1][0].saturated, .04)
        self.assertAlmostEqual(r.update(.1, (8, 8), (0, 0))[1][0].saturated, 1)
        pidf = self.regulator("PIDF", kp=.02, correction_min=1)
        self.assertAlmostEqual(pidf.update(0, (8, 8), (0, 0))[1][0].saturated, .16)

    def test_worker_repeated_result_and_loss(self):
        controls = BallRegulator((AxisConfig(), AxisConfig()), BallGeometry(loss_timeout_s=.08))
        frame = SimpleNamespace(shape=(480, 640, 3))
        current = [SimpleNamespace(frame=frame, captured_at=time.monotonic(), duration_ms=2,
                                   boxes=(Detection((.1, .1, .3, .3), .9),))]
        commands = []
        finished = threading.Event()

        def send(command):
            commands.append(command)
            if command == "BALLSTOP 7":
                finished.set()

        worker = BallTracking(lambda: current[0], (.5, .5), controls, 7, send)
        worker.running.set()
        worker.start()
        try:
            self.assertTrue(finished.wait(.6))
            self.assertEqual(len(commands), 2)
            self.assertTrue(commands[0].startswith("TARGET 7 "))
        finally:
            worker.stopped.set()
            worker.join(.5)


class SerialTests(unittest.TestCase):
    def test_serial_framing(self):
        framer = SerialLines()
        self.assertEqual(framer.feed(b"!BALLA"), [])
        self.assertEqual(framer.feed(b"CK,3,4\r\n!HOME,0\n!CFG"), [b"!BALLACK,3,4", b"!HOME,0"])
        self.assertEqual(framer.feed(b"x" * 600), [])
        self.assertEqual(framer.feed(b"dropped\nvalid\n"), [b"valid"])

    def test_backpressure_expiry_late_ack_and_stop(self):
        window = BallWindow()
        window.begin(7, 0)
        window.offer(7, (40, 41), 0)
        self.assertEqual(window.next_command(0), "BALL2 7 1 40.0000 41.0000")
        window.offer(7, (42, 40), .02)
        window.offer(7, (43, 40), .08)
        self.assertIsNone(window.next_command(.1))
        self.assertEqual(window.next_command(.16), "BALL2 7 3 43.0000 40.0000")
        window.receive(b"!BALLACK,7,1,1,40,41,0", .17)
        window.receive(b"!BALLACK,7,3,1,43,40,9", .18)
        self.assertEqual([event.outcome for event in window.events],
                         ["sent", "replaced", "timeout", "sent", "late_reply", "applied"])
        self.assertAlmostEqual(window.events[-1].rtt_s, .02)
        window.offer(7, (40, 41), .2)
        self.assertIsNone(window.next_command(.33))
        self.assertEqual(window.events[-1].outcome, "expired")
        window.offer(7, (40, 41), .35)
        self.assertTrue(window.clear(.35, "cancelled", 7))
        self.assertIsNone(window.next_command(.36))

    def test_ack_order_nonfinite_and_foreign_states(self):
        window = BallWindow()
        window.begin(8, 0)
        window.offer(8, (42, 40), .01)
        window.next_command(.01)
        window.receive(b"!BALLSTATE2,7,3", .02)
        window.receive(b"!BALLACK,8,1,1,nan,40,0", .02)
        window.receive(b"!BALLACK,8,1,1,40,42,0", .02)
        self.assertEqual(window.events[-1].outcome, "invalid_reply")
        self.assertIsNotNone(window.in_flight)
        window.receive(b"!BALLACK,8,1,4,42,40,0", .03)
        self.assertEqual(window.events[-1].outcome, "centering")
        window.receive(b"!BALLSTATE2,8,2", .04)
        self.assertEqual(window.session, 0)

    def test_worker_missing_ack_stop_reconnect(self):
        pico = PicoMock(drop=(1,), fragment=3)
        connection = SerialSession(pico, SerialLines())
        try:
            self.assertTrue(wait_for(lambda: pico.hello))
            connection.send("BALLTEST2 5")
            self.assertTrue(wait_for(lambda: pico.state == 5))
            connection.send("TARGET 5 39.2000 41.3000")
            self.assertTrue(wait_for(lambda: any(e.outcome == "timeout" for e in tuple(connection.ball.events))))
            pico.inject(b"!BALLACK,5,1,5,39.2000,41.3000,0")
            self.assertTrue(wait_for(lambda: any(e.outcome == "late_reply" for e in tuple(connection.ball.events))))
            connection.send("STOP")
            connection.send("TARGET 5 42 43")
            self.assertTrue(wait_for(lambda: "STOP" in pico.writes))
            self.assertEqual(pico.state, 0)
            self.assertEqual([cmd for cmd in pico.writes if cmd.startswith("BALL2 ")],
                             ["BALL2 5 1 39.2000 41.3000"])
            pico.close()
            self.assertTrue(wait_for(lambda: connection.error is not None))
        finally:
            connection.close()

        new_pico = PicoMock()
        new_connection = SerialSession(new_pico, SerialLines())
        try:
            self.assertTrue(wait_for(lambda: new_pico.hello))
            new_connection.send("TARGET 5 42 43")
            time.sleep(.022)
            self.assertFalse(any(cmd.startswith("BALL2 ") for cmd in new_pico.writes))
        finally:
            new_connection.close()


class MotorTelemetryTests(unittest.TestCase):
    def test_legacy_and_current_samples(self):
        line = b"1,5000,4,0,3,0,0,0,30,28,29,-4,54,-20,3,0.03,-1,0,0,-4,32,3"
        current = MotorSample.parse(line.split(b","))
        self.assertIsNotNone(current)
        self.assertEqual((current.bridge, current.step_origin), (3, 32))
        legacy = MotorSample.parse(line.split(b",")[:20])
        self.assertIsNone(legacy.bridge)
        self.assertEqual(current.angle / current.span * 100, 30 / 54 * 100)
        self.assertIsNone(MotorSample.parse(line.replace(b",32,3", b",inf,3").split(b",")))
        result = step_metrics([current._replace(angle=32, elapsed=0),
                               current._replace(angle=28, elapsed=1),
                               current._replace(angle=27, elapsed=2)], complete=True)
        self.assertIn("0.80 s", result)
        self.assertIn("25.0%", result)


class GuiTests(unittest.TestCase):
    def test_tk_mock_startup(self):
        import tkinter as tk
        from visualizar_imu import visualize

        try:
            root = tk.Tk()
        except tk.TclError as error:
            self.skipTest(str(error))
        root.destroy()

        original_mainloop = tk.Tk.mainloop
        callback_errors = []

        def smoke(window):
            window.withdraw()
            window.report_callback_exception = lambda kind, error, trace: callback_errors.append(error)
            window.after(350, window.quit)
            original_mainloop(window)
            window.destroy()

        with patch.object(tk.Tk, "mainloop", smoke):
            visualize(mock=True)
        self.assertEqual(callback_errors, [])


if __name__ == "__main__":
    unittest.main()
