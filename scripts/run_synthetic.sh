#!/bin/bash
# Shell script to run synthetic drone VIO benchmarks

echo "Running Baseline Orbit Trajectory..."
python scripts/run_synthetic.py --preset orbit

echo "Running Dynamic Multi-Axis Trajectory..."
python scripts/run_synthetic.py --preset multi_axis

echo "Running Stationary Hover Trajectory..."
python scripts/run_synthetic.py --preset hover

echo "Running Straight Flight Trajectory..."
python scripts/run_synthetic.py --preset straight

echo "Running Stress Test with Feature Dropout..."
python scripts/run_synthetic.py --preset stress --dropout

echo "Evaluating All Results..."
python scripts/evaluate.py
