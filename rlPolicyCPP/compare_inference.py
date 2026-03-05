#!/usr/bin/env python3
"""
Compare action outputs from rl_policyClean vs inference_sf_reference (Sample Factory).

Loads test data (observations), runs both inference methods on each observation,
and reports differences.

Usage:
  python compare_inference.py --test_data=<path> --ckpt=<path> [options]

  # Using the generated test data:
  python compare_inference.py \\
      --test_data=train_dir/swarm_3d_v4_hyb/checkpoint_p0/best_*_test_data.txt \\
      --ckpt=train_dir/swarm_3d_v4_hyb/checkpoint_p0/best_*.pth
"""

from __future__ import annotations

import argparse
import os
import sys

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _SCRIPT_DIR)

# Sample Factory paths (same as inference_sf_reference)
_DEFAULT_SAMPLE_FACTORY = "/home/valentin/RL/CatSwarm/TrainingRoom/sample-factory"
_SF_DIR = os.environ.get("SF_REFERENCE_SAMPLE_FACTORY", _DEFAULT_SAMPLE_FACTORY)
_SWARM_TRAIN_DIR = os.path.join(_SCRIPT_DIR, "train_dir", "swarm_3d_v4_hyb")


def load_test_data(filepath: str) -> tuple:
    """Load observations (and optional reference actions) from test data file.

    Format: # num_samples obs_dim [action_dim]
            # col names...
            obs... [act...]
    Returns:
        (observations, reference_actions or None)
    """
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

    obs_arr = __import__("numpy").array(observations, dtype="float32")
    ref_arr = __import__("numpy").array(reference_actions, dtype="float32") if reference_actions else None
    return obs_arr, ref_arr


def run_rl_policy_clean(ckpt: str, observations, device: str = "cpu", normalized: bool = False):
    """Run rl_policyClean inference on observations."""
    from rl_policyClean import RLPolicyClean

    policy = RLPolicyClean.load_from_checkpoint(ckpt, device=device, nonlinearity="relu").eval()
    actions = []
    with __import__("torch").no_grad():
        for obs in observations:
            policy.reset_hidden_state(batch_size=1)
            obs_t = __import__("torch").from_numpy(obs).float().unsqueeze(0)
            action_logits, _ = policy(obs_t, normalized=normalized)
            actions.append(action_logits[0].cpu().numpy())
    return __import__("numpy").array(actions)


def _infer_experiment_dir_from_ckpt(ckpt: str) -> str | None:
    """Infer experiment dir from checkpoint path (e.g. .../swarm_3d_v4_hyb_new_obs/checkpoint_p0/xxx.pth -> .../swarm_3d_v4_hyb_new_obs)."""
    from pathlib import Path
    p = Path(ckpt).resolve()
    # Checkpoint is typically in experiment_name/checkpoint_p0/ or experiment_name/checkpoint_*/
    if p.parent.name.startswith("checkpoint_"):
        exp_dir = p.parent.parent
        if exp_dir.is_dir() and (exp_dir / "swarm_encoder.py").exists():
            return str(exp_dir)
    return None


def run_sf_reference(ckpt: str, observations, device: str = "cpu", normalized: bool = False,
                     train_dir: str | None = None, experiment: str = "swarm_3d_v4_hyb"):
    """Run Sample Factory inference on observations."""
    if _SF_DIR not in sys.path:
        sys.path.insert(0, _SF_DIR)
    # Prefer experiment dir inferred from checkpoint (has matching swarm_encoder for that checkpoint)
    exp_dir = _infer_experiment_dir_from_ckpt(ckpt)
    if exp_dir and exp_dir not in sys.path:
        sys.path.insert(0, exp_dir)
        print(f"  Using encoder from checkpoint dir: {exp_dir}")
    elif os.path.isdir(_SWARM_TRAIN_DIR) and _SWARM_TRAIN_DIR not in sys.path:
        sys.path.insert(0, _SWARM_TRAIN_DIR)

    import torch
    import gymnasium as gym
    import numpy as np
    from sample_factory.algo.utils.rl_utils import prepare_and_normalize_obs
    from sample_factory.model.actor_critic import create_actor_critic
    from sample_factory.model.model_utils import get_rnn_size
    from sample_factory.cfg.arguments import load_from_checkpoint

    # Register and get config (same as inference_sf_reference)
    # Infer train_dir/experiment from checkpoint path when experiment dir was found
    exp_dir = _infer_experiment_dir_from_ckpt(ckpt)
    if exp_dir and train_dir is None:
        from pathlib import Path
        train_dir = str(Path(exp_dir).parent)
        experiment = Path(exp_dir).name

    try:
        from train_swarm_envhyb import register_custom_components, parse_custom_args, make_swarm_circle_env_func
        register_custom_components()
    except ImportError:
        import json
        from sample_factory.algo.utils.context import global_model_factory
        from sample_factory.utils.attr_dict import AttrDict
        from swarm_encoder import make_swarm_encoder
        global_model_factory().register_encoder_factory(make_swarm_encoder)
        config_dir = exp_dir if exp_dir and os.path.isdir(exp_dir) else _SWARM_TRAIN_DIR
        config_path = os.path.join(config_dir, "config.json")
        cfg = AttrDict(json.load(open(config_path))) if os.path.isfile(config_path) else AttrDict(
            use_rnn=False, rnn_size=256, rnn_num_layers=1, rnn_type="gru",
            actor_critic_share_weights=False, adaptive_stddev=False, max_agents=4,
            nonlinearity="relu", obs_subtract_mean=None, obs_scale=None,
            normalize_input=False, normalize_returns=False,
        )
    else:
        td = train_dir or os.path.join(_SCRIPT_DIR, "train_dir")
        argv = ["--algo=APPO", "--env=swarm_circle_env", f"--train_dir={os.path.abspath(td)}", f"--experiment={experiment}"]
        cfg = parse_custom_args(argv)
        try:
            cfg = load_from_checkpoint(cfg)
        except Exception:
            pass

    obs_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(observations.shape[1],), dtype=np.float32)
    action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

    dev = torch.device("cpu" if device == "cpu" else "cuda")
    actor_critic = create_actor_critic(cfg, obs_space, action_space)
    actor_critic.eval()
    actor_critic.model_to_device(dev)

    extra = {"weights_only": False} if tuple(map(int, torch.__version__.split(".")[:2])) >= (2, 0) else {}
    ckpt_data = torch.load(os.path.abspath(ckpt), map_location=dev, **extra)
    actor_critic.load_state_dict(ckpt_data["model"] if "model" in ckpt_data else ckpt_data)

    rnn_size = get_rnn_size(cfg)
    actions = []
    with torch.no_grad():
        for obs in observations:
            rnn_states = torch.zeros(1, rnn_size, dtype=torch.float32, device=dev)
            obs_dict = {"obs": torch.from_numpy(obs).float().unsqueeze(0).to(dev)}
            if not normalized:
                obs_dict = prepare_and_normalize_obs(actor_critic, obs_dict)
            out = actor_critic(obs_dict, rnn_states, values_only=False)
            actions.append(out["action_logits"][0].cpu().numpy())
    return np.array(actions)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Compare rl_policyClean vs inference_sf_reference",
        usage="%(prog)s --test_data=<path> --ckpt=<path> [--device=<cpu|cuda>] [--normalized] [--train_dir=<path>] [--experiment=<name>]",
    )
    parser.add_argument("--test_data", type=str, required=True, help="Path to test data file")
    parser.add_argument("--ckpt", type=str, required=True, help="Path to checkpoint .pth")
    parser.add_argument("--device", type=str, default="cpu", choices=["cpu", "cuda"])
    parser.add_argument("--normalized", action="store_true")
    parser.add_argument("--train_dir", type=str, default=None)
    parser.add_argument("--experiment", type=str, default="swarm_3d_v4_hyb")
    args = parser.parse_args()

    if not os.path.isfile(args.test_data):
        print(f"Error: Test data file not found: {args.test_data}", file=sys.stderr)
        return 1
    if not os.path.isfile(args.ckpt):
        print(f"Error: Checkpoint not found: {args.ckpt}", file=sys.stderr)
        return 1

    observations, ref_actions = load_test_data(args.test_data)
    n = len(observations)
    print(f"Loaded {n} observations from {args.test_data}")
    print(f"Observation shape: {observations.shape}")

    print("\nRunning rl_policyClean inference...")
    actions_clean = run_rl_policy_clean(args.ckpt, observations, device=args.device, normalized=args.normalized)
    print(f"  rl_policyClean output shape: {actions_clean.shape}")

    print("\nRunning inference_sf_reference (Sample Factory) inference...")
    actions_sf = run_sf_reference(
        args.ckpt, observations, device=args.device, normalized=args.normalized,
        train_dir=args.train_dir, experiment=args.experiment,
    )
    print(f"  inference_sf_reference output shape: {actions_sf.shape}")

    # Compare (align dimensions if SF outputs more, e.g. [mean,logstd,mean2,logstd2])
    import numpy as np
    min_dim = min(actions_clean.shape[1], actions_sf.shape[1])
    if actions_clean.shape[1] != actions_sf.shape[1]:
        print(f"\n  Note: Output dim mismatch - rl_policyClean={actions_clean.shape[1]}, SF={actions_sf.shape[1]}. Comparing first {min_dim} values.")
    a_clean = actions_clean[:, :min_dim]
    a_sf = actions_sf[:, :min_dim]
    diff = np.abs(a_clean - a_sf)
    max_diff = np.max(diff)
    mean_diff = np.mean(diff)
    max_per_sample = np.max(diff, axis=1)

    print("\n" + "=" * 60)
    print("COMPARISON REPORT")
    print("=" * 60)
    print(f"  Max absolute difference:  {max_diff:.2e}")
    print(f"  Mean absolute difference: {mean_diff:.2e}")
    print(f"  Samples with diff > 1e-5:  {np.sum(max_per_sample > 1e-5)} / {n}")
    print(f"  Samples with diff > 1e-6:  {np.sum(max_per_sample > 1e-6)} / {n}")

    if ref_actions is not None:
        ref = np.array(ref_actions)
        ref_dim = min(ref.shape[1], min_dim)
        diff_ref_clean = np.abs(ref[:, :ref_dim] - a_clean[:, :ref_dim])
        diff_ref_sf = np.abs(ref[:, :ref_dim] - a_sf[:, :ref_dim])
        print(f"\n  vs reference (from generate_test_data / rl_policyClean):")
        print(f"    Ref vs rl_policyClean max diff: {np.max(diff_ref_clean):.2e}")
        print(f"    Ref vs SF max diff:             {np.max(diff_ref_sf):.2e}")

    # Sample outputs
    print("\n  First 3 samples - rl_policyClean vs inference_sf_reference:")
    for i in range(min(3, n)):
        print(f"    [{i}] clean: {a_clean[i].tolist()}")
        print(f"        sf:   {a_sf[i].tolist()}")
        print(f"        diff: {diff[i].tolist()}")

    if max_diff < 1e-5:
        print("\n  RESULT: Outputs match within 1e-5 tolerance.")
    elif max_diff < 1e-3:
        print("\n  RESULT: Small differences (< 1e-3). May be acceptable.")
    else:
        print("\n  RESULT: Significant differences detected. Investigate.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
