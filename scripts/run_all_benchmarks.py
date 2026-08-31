import os
import sys
import subprocess

presets = [
    ("orbit", False),
    ("multi_axis", False),
    ("hover", False),
    ("straight", False),
    ("stress", True)
]

for preset, dropout in presets:
    cmd = [sys.executable, "scripts/run_synthetic.py", "--preset", preset]
    if dropout:
        cmd.append("--dropout")
    print(f"\n>>> Running Benchmark: {preset.upper()} (dropout={dropout})...")
    res = subprocess.run(cmd, capture_output=False)
    if res.returncode != 0:
        print(f"Error running preset {preset}")

print("\n>>> Aggregating All Benchmark Metrics...")
subprocess.run([sys.executable, "scripts/evaluate.py"])
