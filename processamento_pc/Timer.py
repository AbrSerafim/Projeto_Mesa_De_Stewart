""" Class to measure processes time """

import time 
from contextlib import contextmanager

class PipelineTimer:
    def __init__(self):
        self.time_ms = {}

    @contextmanager # Allows to use "with"
    def measure(self, name: str):
        start = time.perf_counter_ns() # High resolution clock in nanoseconds

        try: 
            yield # Hands the control to the code inside of the with block
        finally: 
            end = time.perf_counter_ns()
            duration = (end - start) # ns

            if name not in self.time_ms:
                self.time_ms[name] = []

            self.time_ms[name].append(duration)

    def report(self):
        print("\n--- Pipeline timing ---")

        for name, durations in self.time_ms.items():
            print(f"\n{name}")

            for i, duration in enumerate(durations, start=1):
                print(f"  Run {i:<5} {duration:10.3f} ns")
        print("\n--- Average timings ---")

        for name, durations in self.time_ms.items():
            avg = sum(durations) / len(durations)
            print(f"{name:<30} {avg:10.3f} ns; {(avg*1e-6):10.3f} ms")