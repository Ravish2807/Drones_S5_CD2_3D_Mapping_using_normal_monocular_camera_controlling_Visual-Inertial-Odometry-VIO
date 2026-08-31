import os
import sys
import glob
import json
import pandas as pd

# Ensure project root is in sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))


def main():
    metrics_dir = os.path.join("results", "metrics")
    json_files = glob.glob(os.path.join(metrics_dir, "*_metrics.json"))

    if not json_files:
        print(f"No metric JSON files found in {metrics_dir}.")
        print("Run scripts/run_synthetic.py or scripts/run_euroc.py first.")
        return

    records = []
    for jf in json_files:
        with open(jf, "r") as f:
            data = json.load(f)
            records.append(data)

    df = pd.DataFrame(records)
    print("=" * 85)
    print("                 PHASE 1 VIO ESTIMATION BENCHMARK RESULTS TABLE")
    print("=" * 85)
    print(df.to_string(index=False))
    print("=" * 85)

    summary_csv = os.path.join(metrics_dir, "benchmark_summary_table.csv")
    df.to_csv(summary_csv, index=False)
    print(f"Benchmark summary table exported to: {summary_csv}")


if __name__ == "__main__":
    main()
