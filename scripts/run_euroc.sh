#!/bin/bash
# Shell script to run EuRoC benchmark dataset sequences

SEQUENCE=${1:-"data/MH_01_easy"}

echo "Running EuRoC VIO on sequence: $SEQUENCE"
python scripts/run_euroc.py --sequence_path "$SEQUENCE" --config "config/euroc.yaml"

echo "Evaluating All Results..."
python scripts/evaluate.py
