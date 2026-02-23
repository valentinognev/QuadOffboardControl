#!/usr/bin/env python3
"""
Reference inference script using full Sample Factory stack.

Performs the same type of inference as rl_policyClean.py (load checkpoint,
forward pass, output action logits [mean, logstd]) but using Sample Factory's
create_actor_critic, load_checkpoint, prepare_and_normalize_obs, and
actor_critic forward. Use this as the reference to verify rl_policyClean and
C++ RLPolicyJSON match SF's behavior.

Sample Factory path: set via --sample_factory_dir (default below) or
SF_REFERENCE_SAMPLE_FACTORY env var.

Usage:
  # From experiment dir with config.json (recommended)
  python inference_sf_reference.py --ckpt=path/to/best_xxx.pth \\
      --train_dir=/path/to/rlPolicyCPP/train_dir --experiment=swarm_3d_v4_hyb

  # With explicit SF path
  python inference_sf_reference.py --ckpt=path/to/best_xxx.pth \\
      --train_dir=... --experiment=... --sample_factory_dir=/path/to/sample-factory

  # Output format matches rl_policyClean.py for comparison
  python inference_sf_reference.py --ckpt=... --train_dir=... --experiment=...
  python rl_policyClean.py --ckpt=...

  # Without full env (e.g. no Quadcopter_SimCon): use dummy spaces
  python inference_sf_reference.py --ckpt=... --obs_dim=18 --action_dim=2
"""

from __future__ import annotations

import argparse
import os
import sys

# Default Sample Factory location (override with --sample_factory_dir or env)
_DEFAULT_SAMPLE_FACTORY = "/home/valentin/RL/CatSwarm/TrainingRoom/sample-factory"
_SF_DIR = os.environ.get("SF_REFERENCE_SAMPLE_FACTORY", _DEFAULT_SAMPLE_FACTORY)

# Script dir and swarm train dir (for register_custom_components)
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_SWARM_TRAIN_DIR = os.path.join(_SCRIPT_DIR, "train_dir", "swarm_3d_v4_hyb")


def _setup_paths(sample_factory_dir: str) -> None:
    if not os.path.isdir(sample_factory_dir):
        raise FileNotFoundError(
            f"Sample Factory dir not found: {sample_factory_dir}\n"
            "Set --sample_factory_dir or SF_REFERENCE_SAMPLE_FACTORY."
        )
    if sample_factory_dir not in sys.path:
        sys.path.insert(0, sample_factory_dir)
    if os.path.isdir(_SWARM_TRAIN_DIR) and _SWARM_TRAIN_DIR not in sys.path:
        sys.path.insert(0, _SWARM_TRAIN_DIR)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Reference inference using Sample Factory (same logic as rl_policyClean)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--ckpt", type=str, required=True, help="Path to checkpoint .pth (best_*.pth or checkpoint_*.pth)")
    parser.add_argument(
        "--train_dir",
        type=str,
        default=None,
        help="Training root (e.g. rlPolicyCPP/train_dir). With --experiment, load config from train_dir/experiment/config.json",
    )
    parser.add_argument(
        "--experiment",
        type=str,
        default="swarm_3d_v4_hyb",
        help="Experiment name (default: swarm_3d_v4_hyb). Used with --train_dir to load config.",
    )
    parser.add_argument(
        "--sample_factory_dir",
        type=str,
        default=_SF_DIR,
        help=f"Path to sample-factory repo (default: {_SF_DIR})",
    )
    parser.add_argument("--device", type=str, default="cpu", choices=["cpu", "cuda"])
    parser.add_argument("--normalized", action="store_true", help="Treat observations as already normalized")
    parser.add_argument(
        "--obs_dim",
        type=int,
        default=None,
        help="Override obs size (e.g. 18) if env cannot be created (dummy space used)",
    )
    parser.add_argument(
        "--action_dim",
        type=int,
        default=None,
        help="Override action size (e.g. 2) if env cannot be created",
    )
    args = parser.parse_args()

    _setup_paths(args.sample_factory_dir)

    import torch
    import gymnasium as gym
    import numpy as np
    from sample_factory.algo.learning.learner import Learner
    from sample_factory.algo.utils.rl_utils import prepare_and_normalize_obs
    from sample_factory.model.actor_critic import create_actor_critic
    from sample_factory.model.model_utils import get_rnn_size
    from sample_factory.cfg.arguments import load_from_checkpoint

    # Register swarm env and encoder (from train script, or encoder-only fallback)
    use_dummy_spaces = False
    cfg = None
    try:
        from train_swarm_envhyb import (
            register_custom_components,
            parse_custom_args,
            make_swarm_circle_env_func,
        )
        register_custom_components()
    except ImportError as e:
        if args.obs_dim is not None and args.action_dim is not None:
            use_dummy_spaces = True
            print(
                "Note: train_swarm_envhyb not available, using --obs_dim/--action_dim and encoder-only registration.",
                file=sys.stderr,
            )
            import json
            from sample_factory.algo.utils.context import global_model_factory
            from sample_factory.utils.attr_dict import AttrDict
            from swarm_encoder import make_swarm_encoder
            global_model_factory().register_encoder_factory(make_swarm_encoder)
            # Load config from experiment dir if present, else minimal defaults for swarm
            config_path = os.path.join(_SWARM_TRAIN_DIR, "config.json")
            if not os.path.isfile(config_path):
                config_path = os.path.join(os.path.dirname(_SWARM_TRAIN_DIR), args.experiment, "config.json")
            if os.path.isfile(config_path):
                with open(config_path) as f:
                    cfg = AttrDict(json.load(f))
            else:
                cfg = AttrDict(
                    use_rnn=False,
                    rnn_size=256,
                    rnn_num_layers=1,
                    rnn_type="gru",
                    actor_critic_share_weights=False,
                    adaptive_stddev=False,
                    max_agents=4,
                    nonlinearity="relu",
                    obs_subtract_mean=None,
                    obs_scale=None,
                    normalize_input=False,
                    normalize_returns=False,
                )
        else:
            print(
                "Error: Could not import train_swarm_envhyb.",
                "Set --obs_dim=18 --action_dim=2 to run with dummy spaces (encoder-only), or fix deps.",
                file=sys.stderr,
            )
            print(f"  {e}", file=sys.stderr)
            return 1

    # Build config: from experiment config.json if train_dir+experiment given, else defaults (skip if use_dummy_spaces)
    if not use_dummy_spaces:
        if args.train_dir and args.experiment:
            argv = [
                "--algo=APPO",
                "--env=swarm_circle_env",
                f"--train_dir={os.path.abspath(args.train_dir)}",
                f"--experiment={args.experiment}",
            ]
            cfg = parse_custom_args(argv)
            try:
                cfg = load_from_checkpoint(cfg)
            except Exception as e:
                print(f"Warning: Could not load config from experiment ({e}). Using defaults.", file=sys.stderr)
        else:
            default_train_dir = os.path.join(_SCRIPT_DIR, "train_dir")
            argv = [
                "--algo=APPO",
                "--env=swarm_circle_env",
                f"--train_dir={os.path.abspath(default_train_dir)}",
                f"--experiment={args.experiment}",
            ]
            cfg = parse_custom_args(argv)

    # Observation and action spaces: from env if possible, else dummy
    obs_space = None
    action_space = None
    if use_dummy_spaces or (args.obs_dim is not None and args.action_dim is not None):
        od = args.obs_dim if args.obs_dim is not None else 18
        ad = args.action_dim if args.action_dim is not None else 2
        obs_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(od,), dtype=np.float32
        )
        action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(ad,), dtype=np.float32
        )
    else:
        try:
            env = make_swarm_circle_env_func(
                "inference_ref", cfg, _env_config=None, render_mode=None
            )
            obs_space = env.observation_space
            action_space = env.action_space
        except Exception as e:
            print(
                f"Warning: Could not create env ({e}). Use --obs_dim=18 --action_dim=2 to skip.",
                file=sys.stderr,
            )
            obs_space = gym.spaces.Box(
                low=-np.inf, high=np.inf, shape=(18,), dtype=np.float32
            )
            action_space = gym.spaces.Box(
                low=-1.0, high=1.0, shape=(2,), dtype=np.float32
            )

    device = torch.device("cpu" if args.device == "cpu" else "cuda")
    actor_critic = create_actor_critic(cfg, obs_space, action_space)
    actor_critic.eval()
    actor_critic.model_to_device(device)

    # Load checkpoint (same as enjoy.py)
    checkpoint_path = os.path.abspath(args.ckpt)
    if not os.path.isfile(checkpoint_path):
        print(f"Error: Checkpoint not found: {checkpoint_path}", file=sys.stderr)
        return 1
    extra = {"weights_only": False} if tuple(map(int, torch.__version__.split(".")[:2])) >= (2, 0) else {}
    ckpt = torch.load(checkpoint_path, map_location=device, **extra)
    if "model" in ckpt:
        actor_critic.load_state_dict(ckpt["model"])
    else:
        actor_critic.load_state_dict(ckpt)

    obs_size = int(obs_space.shape[0]) if hasattr(obs_space, "shape") else 18
    rnn_size = get_rnn_size(cfg)
    rnn_states = torch.zeros(1, rnn_size, dtype=torch.float32, device=device)

    # Dummy observation (zeros), same as rl_policyClean __main__
    obs_np = np.zeros(obs_size, dtype=np.float32)
    obs_dict = {"obs": torch.from_numpy(obs_np).unsqueeze(0).to(device)}

    with torch.no_grad():
        if not args.normalized:
            obs_dict = prepare_and_normalize_obs(actor_critic, obs_dict)
        policy_outputs = actor_critic(obs_dict, rnn_states, values_only=False)

    action_logits = policy_outputs["action_logits"][0].cpu().numpy()
    new_rnn_states = policy_outputs["new_rnn_states"]

    # Match rl_policyClean output format
    print("action_logits shape:", tuple(action_logits.shape))
    print("hidden state shape:", tuple(new_rnn_states.shape))
    print("action_logits (first row):", list(action_logits.tolist()))

    return 0


if __name__ == "__main__":
    sys.exit(main())
