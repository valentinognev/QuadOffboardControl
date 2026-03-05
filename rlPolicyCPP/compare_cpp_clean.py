#!/usr/bin/env python3
"""
Compare C++ RLPolicyJSON with rl_policyClean using the same test data.

Runs both implementations on the same 30 observations and reports differences.

Usage:
  python compare_cpp_clean.py --test_data=<path> --ckpt=<path> [--json=<path>]

  # Using defaults (test data and checkpoint from swarm experiment):
  python compare_cpp_clean.py --test_data=train_dir/.../best_*_test_data.txt \\
      --ckpt=train_dir/.../best_*.pth
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _SCRIPT_DIR)


def load_test_data(filepath: str) -> tuple:
    """Load observations and optional reference actions from test data file."""
    observations = []
    reference_actions = []
    num_samples = obs_dim = action_dim = None

    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                if line.startswith("#") and not line.startswith("# obs"):
                    parts = line[1:].strip().split()
                    if len(parts) >= 2 and parts[0].isdigit():
                        num_samples = int(parts[0])
                        obs_dim = int(parts[1])
                        action_dim = int(parts[2]) if len(parts) >= 3 else None
                continue
            vals = [float(x) for x in line.split()]
            if obs_dim is not None:
                obs = vals[:obs_dim]
                observations.append(obs)
                if action_dim is not None and len(vals) >= obs_dim + action_dim:
                    ref_act = vals[obs_dim : obs_dim + action_dim]
                    reference_actions.append(ref_act)

    import numpy as np
    obs_arr = np.array(observations, dtype="float32")
    ref_arr = np.array(reference_actions, dtype="float32") if reference_actions else None
    return obs_arr, ref_arr


def run_rl_policy_clean(ckpt: str, observations, device: str = "cpu", normalized: bool = False):
    """Run rl_policyClean inference on observations."""
    from rl_policyClean import RLPolicyClean

    policy = RLPolicyClean.load_from_checkpoint(ckpt, device=device, nonlinearity="relu").eval()
    actions = []
    import torch
    with torch.no_grad():
        for obs in observations:
            policy.reset_hidden_state(batch_size=1)
            obs_t = torch.from_numpy(obs).float().unsqueeze(0)
            action_logits, _ = policy(obs_t, normalized=normalized)
            actions.append(action_logits[0].cpu().numpy())
    import numpy as np
    return np.array(actions)


def run_cpp_policy(json_path: str, test_data_path: str, normalized: bool = False) -> tuple:
    """Run C++ rlPolicyCPP with test data, return (observations, cpp_actions) from results file."""
    bin_path = os.path.join(_SCRIPT_DIR, "bin", "rlPolicyCPP")
    if not os.path.isfile(bin_path):
        raise FileNotFoundError(f"C++ binary not found: {bin_path}. Run 'make' first.")

    results_path = test_data_path.replace("_test_data.txt", "_test_data_results.txt")
    if os.path.isfile(results_path):
        os.remove(results_path)

    cmd = [
        bin_path,
        f"--json={json_path}",
        f"--test-data={test_data_path}",
        "--normalized=true" if normalized else "--normalized=false",
    ]
    result = subprocess.run(cmd, capture_output=True, text=True, cwd=_SCRIPT_DIR, timeout=30)
    if result.returncode != 0:
        raise RuntimeError(f"C++ run failed: {result.stderr}")

    if not os.path.isfile(results_path):
        raise FileNotFoundError(f"C++ did not produce results file: {results_path}")

    # Parse results file: # num obs_dim action_dim [ref_dim]
    # obs... ref... calc... diff...
    observations = []
    cpp_actions = []
    with open(results_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                if line.startswith("#") and not line.startswith("# obs"):
                    parts = line[1:].strip().split()
                    if len(parts) >= 3 and parts[0].isdigit():
                        obs_dim = int(parts[1])
                        action_dim = int(parts[2])
                continue
            vals = [float(x) for x in line.split()]
            if obs_dim is not None and len(vals) >= obs_dim + action_dim:
                obs = vals[:obs_dim]
                # calc columns come after obs + ref; ref has same action_dim
                calc_start = obs_dim + action_dim  # ref
                calc = vals[calc_start : calc_start + action_dim]
                observations.append(obs)
                cpp_actions.append(calc)

    import numpy as np
    return np.array(observations), np.array(cpp_actions)


def main() -> int:
    # Allow -ckpt= as typo for --ckpt=
    argv = []
    for a in sys.argv[1:]:
        if a.startswith("-ckpt="):
            argv.append("--ckpt=" + a[6:])
        else:
            argv.append(a)

    parser = argparse.ArgumentParser(
        description="Compare C++ RLPolicyJSON vs rl_policyClean",
        usage="%(prog)s --test_data=<path> --ckpt=<path> [--json=<path>] [--device=<cpu|cuda>] [--normalized]",
    )
    parser.add_argument("--test_data", type=str, required=True, help="Path to test data file")
    parser.add_argument("--ckpt", "-c", type=str, required=True, dest="ckpt", help="Path to checkpoint .pth")
    parser.add_argument("--json", type=str, default=None, help="Path to JSON (default: ckpt dir with .json)")
    parser.add_argument("--device", type=str, default="cpu", choices=["cpu", "cuda"])
    parser.add_argument("--normalized", action="store_true")
    args = parser.parse_args(argv)

    # Resolve paths
    test_data_path = os.path.abspath(args.test_data) if not os.path.isabs(args.test_data) else args.test_data
    ckpt_path = os.path.abspath(args.ckpt) if not os.path.isabs(args.ckpt) else args.ckpt

    if args.json:
        json_path = os.path.abspath(args.json) if not os.path.isabs(args.json) else args.json
    else:
        json_path = os.path.splitext(ckpt_path)[0] + ".json"

    if not os.path.isfile(test_data_path):
        print(f"Error: Test data not found: {test_data_path}", file=sys.stderr)
        return 1
    if not os.path.isfile(ckpt_path):
        print(f"Error: Checkpoint not found: {ckpt_path}", file=sys.stderr)
        return 1
    if not os.path.isfile(json_path):
        print(f"Error: JSON not found: {json_path}", file=sys.stderr)
        print("  Run: python pth2json.py --pth=<ckpt> <json_path>", file=sys.stderr)
        return 1

    # Ensure we're in rlPolicyCPP dir for C++ (paths may be relative)
    os.chdir(_SCRIPT_DIR)
    test_data_rel = os.path.relpath(test_data_path, _SCRIPT_DIR) if os.path.isabs(test_data_path) else test_data_path
    json_rel = os.path.relpath(json_path, _SCRIPT_DIR) if os.path.isabs(json_path) else json_path

    observations, ref_actions = load_test_data(test_data_path)
    n = len(observations)
    print(f"Loaded {n} observations from {test_data_path}")
    print(f"Observation shape: {observations.shape}")

    print("\nRunning rl_policyClean inference...")
    actions_clean = run_rl_policy_clean(ckpt_path, observations, device=args.device, normalized=args.normalized)
    print(f"  rl_policyClean output shape: {actions_clean.shape}")

    print("\nRunning C++ RLPolicyJSON inference...")
    try:
        _, actions_cpp = run_cpp_policy(json_rel, test_data_rel, normalized=args.normalized)
        print(f"  C++ output shape: {actions_cpp.shape}")
    except Exception as e:
        print(f"  Error: {e}", file=sys.stderr)
        return 1

    # Compare
    import numpy as np
    if actions_cpp.size == 0:
        print("  Error: C++ produced no actions. Check obs size mismatch (C++ expects obs_dim from JSON).", file=sys.stderr)
        return 1
    min_dim = min(actions_clean.shape[1], actions_cpp.shape[1])
    a_clean = actions_clean[:, :min_dim]
    a_cpp = actions_cpp[:, :min_dim]
    diff = np.abs(a_clean - a_cpp)
    max_diff = np.max(diff)
    mean_diff = np.mean(diff)
    max_per_sample = np.max(diff, axis=1)

    print("\n" + "=" * 60)
    print("COMPARISON REPORT: C++ RLPolicyJSON vs rl_policyClean")
    print("=" * 60)
    print(f"  Max absolute difference:  {max_diff:.2e}")
    print(f"  Mean absolute difference: {mean_diff:.2e}")
    print(f"  Samples with diff > 1e-5:  {np.sum(max_per_sample > 1e-5)} / {n}")
    print(f"  Samples with diff > 1e-4:  {np.sum(max_per_sample > 1e-4)} / {n}")

    if ref_actions is not None:
        ref = np.array(ref_actions)
        ref_dim = min(ref.shape[1], min_dim)
        diff_ref_clean = np.abs(ref[:, :ref_dim] - a_clean[:, :ref_dim])
        diff_ref_cpp = np.abs(ref[:, :ref_dim] - a_cpp[:, :ref_dim])
        print(f"\n  vs reference (from generate_test_data):")
        print(f"    Ref vs rl_policyClean max diff: {np.max(diff_ref_clean):.2e}")
        print(f"    Ref vs C++ max diff:             {np.max(diff_ref_cpp):.2e}")

    print("\n  First 3 samples - rl_policyClean vs C++:")
    for i in range(min(3, n)):
        print(f"    [{i}] clean: {a_clean[i].tolist()}")
        print(f"        cpp:   {a_cpp[i].tolist()}")
        print(f"        diff: {diff[i].tolist()}")

    if max_diff < 1e-5:
        print("\n  RESULT: Outputs match within 1e-5 tolerance.")
    elif max_diff < 1e-4:
        print("\n  RESULT: Small differences (< 1e-4). Acceptable floating-point variance.")
    elif max_diff < 1e-2:
        print("\n  RESULT: Moderate differences. May need investigation.")
    else:
        print("\n  RESULT: Significant differences detected. Investigate.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
