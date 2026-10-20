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
            self.time_ms[name] = (end - start) # ns

    def report(self):
        print("\n--- Pipeline timing ---")

        for name, duration in self.time_ms.items():
            print(f"{name:<30} {duration:10.3f}ns")