"""Build and run the firmware host harness with GCC, Clang or Zig cc."""

import argparse
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("compiler", nargs="+", help="Compiler executable and optional subcommand, e.g. zig cc")
    args = parser.parse_args()
    source = Path(__file__).resolve().with_name("firmware_tests.c")
    with TemporaryDirectory(prefix="stewart-test-") as path:
        directory = Path(path)
        for include in ("FreeRTOS.h", "semphr.h", "task.h", "hardware/clocks.h", "hardware/pwm.h",
                        "hardware/watchdog.h", "pico/stdlib.h"):
            header = directory / include
            header.parent.mkdir(parents=True, exist_ok=True)
            header.touch()
        binary = directory / "motor-host-test.exe"
        subprocess.run([*args.compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-I", path,
                        str(source), "-lm", "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
