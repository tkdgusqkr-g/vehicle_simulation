"""End-to-end check of the shared-memory link: vsim-core in real time,
vsim-link as the viewer (keys in, state and track mesh out)."""
import os
import re
import subprocess
import sys
import time


def main() -> int:
    core, link, scenario = sys.argv[1:4]
    name = f"vsim_e2e_{os.getpid()}"
    port = str(20000 + os.getpid() % 20000)
    sim = subprocess.Popen(
        [core, "--scenario", scenario, "--set", "track.template=acceleration", "--no-wheel",
         "--port", port, "--duration", "10", "--link-name", name],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        def tool(*args: str) -> str:
            result = subprocess.run([link, *args, "--name", name], capture_output=True, text=True, timeout=20)
            print(f"$ vsim-link {' '.join(args)}\n{result.stdout}{result.stderr}")
            if result.returncode != 0:
                raise RuntimeError(f"vsim-link {args[0]} failed")
            return result.stdout

        tool("wait", "--seconds", "10")
        out = tool("drive", "throttle", "--seconds", "1.5")
        speed = float(re.search(r"speed_mps ([0-9.]+)", out).group(1))
        assert speed > 5.0, f"the car did not accelerate on viewer keys ({speed} m/s)"
        assert re.search(r"track 'acceleration[^']*': \d+ sections", out), "no track mesh"
        latency = float(re.search(r"input -> state: mean ([0-9.]+) ms", out).group(1))
        assert latency < 10.0, f"input to state latency {latency} ms"

        tool("reset")
        time.sleep(0.3)
        out = tool("monitor", "--seconds", "0.2")
        t = float(re.search(r"^t ([0-9.]+) s", out, re.M).group(1))
        assert t < 1.0, f"reset did not restart the run (t = {t})"
        print("link end-to-end: ok")
        return 0
    finally:
        sim.terminate()
        try:
            output, _ = sim.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            sim.kill()
            output, _ = sim.communicate()
        print("--- vsim-core ---\n" + output)


if __name__ == "__main__":
    sys.exit(main())
