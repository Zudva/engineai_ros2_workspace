#!/usr/bin/env python3
import sys
import os
import math
import argparse
import yaml

# Minimal validator for interface_example RL YAML configs

def flatten(list_of_lists):
    return [x for sub in list_of_lists for x in sub]

def main():
    p = argparse.ArgumentParser(description="Validate RL config YAML for interface_example")
    p.add_argument("config_dir", help="Directory containing rl_basic_param.yaml and policies/")
    args = p.parse_args()

    yaml_path = os.path.join(args.config_dir, "rl_basic_param.yaml")
    if not os.path.exists(yaml_path):
        print(f"ERROR: YAML not found: {yaml_path}")
        return 2

    with open(yaml_path, "r") as f:
        cfg = yaml.safe_load(f)

    errors = []
    warnings = []

    # Basic required fields
    required = [
        "policy_file",
        "num_observations",
        "num_commands",
        "num_clock_signal",
        "num_include_obs_steps",
        "active_joint_idx",
        "default_joint_q",
        "joint_kp",
        "joint_kd",
        "action_scale",
        "control_dt",
    ]
    for k in required:
        if k not in cfg:
            errors.append(f"Missing required field: {k}")

    if errors:
        for e in errors:
            print("ERROR:", e)
        return 2

    # Check policy file exists (relative to config dir)
    policy_path = os.path.join(args.config_dir, cfg["policy_file"])
    if not os.path.exists(policy_path):
        warnings.append(f"Policy file not found: {policy_path}")

    # Shapes
    try:
        num_obs = int(cfg["num_observations"])
        n_cmd = int(cfg["num_commands"])
        n_clk = int(cfg["num_clock_signal"])
        n_hist = int(cfg["num_include_obs_steps"])
        aj_idx = list(cfg["active_joint_idx"])  # list of ints
    except Exception as e:
        errors.append(f"Invalid numeric fields: {e}")
        aj_idx = []

    # Flatten grouped vectors and compare lengths
    def lensum(field):
        arr = cfg.get(field, [])
        if not isinstance(arr, list):
            errors.append(f"Field {field} must be a list of lists")
            return 0
        total = 0
        for i, group in enumerate(arr):
            if not isinstance(group, list):
                errors.append(f"Field {field}[{i}] must be a list")
                continue
            total += len(group)
        return total

    n_dof_default = lensum("default_joint_q")
    n_dof_kp = lensum("joint_kp")
    n_dof_kd = lensum("joint_kd")
    n_dof_scale = lensum("action_scale")

    if len(aj_idx) != n_dof_scale:
        errors.append(
            f"active_joint_idx length {len(aj_idx)} must equal action_scale dof {n_dof_scale}"
        )

    if not (n_dof_default == n_dof_kp == n_dof_kd):
        errors.append(
            f"default_joint_q({n_dof_default}) == joint_kp({n_dof_kp}) == joint_kd({n_dof_kd}) length mismatch"
        )

    # Observation input size expectation
    # The code constructs obs as: (num_observations * num_include_obs_steps) + num_clock_signal + num_commands
    expected_input = num_obs * n_hist + n_clk + n_cmd
    if expected_input <= 0:
        errors.append("Computed expected_input <= 0; check observation params")
    else:
        print(f"INFO: Expected network input size: {expected_input}")

    # control_dt sanity
    dt = float(cfg.get("control_dt", 0.0))
    if dt <= 0.0 or dt > 0.1:
        warnings.append(f"Suspicious control_dt={dt}; typical values around 0.005..0.02")

    # Report
    if errors:
        for e in errors:
            print("ERROR:", e)
        for w in warnings:
            print("WARN:", w)
        return 2

    for w in warnings:
        print("WARN:", w)

    print("OK: YAML config passed basic validation")
    return 0

if __name__ == "__main__":
    sys.exit(main())
