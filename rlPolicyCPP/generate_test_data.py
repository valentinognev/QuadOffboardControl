#!/usr/bin/env python3
"""
Generate test observations and actions for rlPolicyCPP validation.

This program:
1. Loads a PyTorch checkpoint (.pth) using RLPolicyClean
2. Loads an environment to get observation space
3. Generates test observations according to the observation space
4. Gets actions from the policy for each observation
5. Saves observations and actions to a file for rlPolicyCPP to use

Usage:
    python generate_test_data.py --pth=<checkpoint.pth> --env=<env_file.py> --output=<test_data.txt> [options]
"""

import sys
import os
import argparse
import numpy as np
import importlib.util
from pathlib import Path

# Add current directory to path to import rl_policyClean from same directory
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Import will be handled in main() to provide better error messages
RLPolicyClean = None
try:
    from rl_policyClean import RLPolicyClean
except ImportError as e:
    # Store the error to be handled in main()
    _import_error = e
    RLPolicyClean = None
else:
    _import_error = None


def load_environment(env_file_path):
    """Load environment from a Python file.
    
    Args:
        env_file_path: Path to the environment Python file
        
    Returns:
        Environment instance with observation_space and action_space
    """
    env_file_path = Path(env_file_path).resolve()
    
    if not env_file_path.exists():
        raise FileNotFoundError(f"Environment file not found: {env_file_path}")
    
    # Add necessary paths to sys.path for imports
    # Find the 'src' directory by navigating up from the environment file
    # This allows Quadcopter_SimCon and other modules to be imported
    current_path = env_file_path.parent
    src_dir = None
    max_depth = 10  # Prevent infinite loop
    depth = 0
    while depth < max_depth and current_path != current_path.parent:
        if current_path.name == 'src':
            src_dir = current_path
            break
        current_path = current_path.parent
        depth += 1
    
    if src_dir is not None:
        if str(src_dir) not in sys.path:
            sys.path.insert(0, str(src_dir))
    
    # Also add the environment file's parent directory
    if str(env_file_path.parent) not in sys.path:
        sys.path.insert(0, str(env_file_path.parent))
    
    # Load the module
    spec = importlib.util.spec_from_file_location("env_module", env_file_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Could not load environment module from {env_file_path}")
    
    env_module = importlib.util.module_from_spec(spec)
    
    try:
        spec.loader.exec_module(env_module)
    except Exception as e:
        import traceback
        error_msg = f"Error loading environment module: {e}\n"
        error_msg += f"Traceback:\n{''.join(traceback.format_exception(type(e), e, e.__traceback__))}"
        raise ImportError(error_msg)
    
    # Find the environment class (usually ends with 'Env' or inherits from gym.Env)
    env_class = None
    env_class_name = None
    last_error = None
    
    # Get all classes from the module
    candidate_classes = []
    all_classes_in_module = []
    for name in dir(env_module):
        obj = getattr(env_module, name)
        if isinstance(obj, type) and not name.startswith('_'):
            all_classes_in_module.append(name)
            if 'Env' in name:
                candidate_classes.append((name, obj))
    
    print(f"  Available classes in module: {', '.join(all_classes_in_module)}")
    if candidate_classes:
        print(f"  Candidate environment classes: {', '.join([n for n, _ in candidate_classes])}")
    
    # Sort by relevance: classes ending with 'Env' first
    candidate_classes.sort(key=lambda x: (not x[0].endswith('Env'), x[0]))
    
    # Try to instantiate each candidate (support multiple Sample Factory env signatures)
    _instantiation_tries = [
        ("full_env_name, cfg, env_config", lambda o: o(full_env_name="test_env", cfg=None, env_config=None)),
        ("full_env_name, cfg", lambda o: o(full_env_name="test_env", cfg=None)),
        ("full_env_name, cfg, render_mode", lambda o: o(full_env_name="test_env", cfg=None, render_mode=None)),
        ("no args", lambda o: o()),
    ]
    for name, obj in candidate_classes:
        for sig_name, try_fn in _instantiation_tries:
            try:
                test_env = try_fn(obj)
                if hasattr(test_env, 'observation_space') and hasattr(test_env, 'action_space'):
                    env_class = obj
                    env_class_name = name
                    break
            except TypeError:
                continue
            except Exception as e:
                last_error = e
                continue
        if env_class is not None:
            break
    
    if env_class is None:
        # List available classes for debugging
        all_classes = [name for name in dir(env_module) 
                      if isinstance(getattr(env_module, name), type) and not name.startswith('_')]
        error_msg = f"Could not find environment class in {env_file_path}\n"
        error_msg += f"Available classes: {', '.join(all_classes)}\n"
        if candidate_classes:
            error_msg += f"Classes with 'Env' in name: {', '.join([n for n, _ in candidate_classes])}\n"
        if last_error:
            error_msg += f"Last error when trying to instantiate: {last_error}\n"
        error_msg += f"\nLooking for a class that:\n"
        error_msg += f"  1. Has 'Env' in the name\n"
        error_msg += f"  2. Can be instantiated (with full_env_name or without parameters)\n"
        error_msg += f"  3. Has 'observation_space' and 'action_space' attributes after instantiation"
        raise ValueError(error_msg)
    
    print(f"  Found environment class: {env_class_name}")

    # Create environment instance (same signature set as above)
    for _sig_name, try_fn in _instantiation_tries:
        try:
            env = try_fn(env_class)
            break
        except (TypeError, Exception):
            continue
    else:
        raise ValueError(f"Could not instantiate {env_class_name} with any known signature")

    return env


def generate_test_observations(observation_space, num_samples=100, method='uniform'):
    """Generate test observations according to observation space.
    
    Args:
        observation_space: gymnasium.spaces.Box observation space
        num_samples: Number of test observations to generate
        method: 'uniform' (uniform random) or 'grid' (grid sampling)
        
    Returns:
        numpy array of shape (num_samples, obs_dim) with test observations
    """
    if not hasattr(observation_space, 'low') or not hasattr(observation_space, 'high'):
        raise ValueError("Observation space must be a Box space with low/high bounds")
    
    low = np.array(observation_space.low, dtype=np.float32)
    high = np.array(observation_space.high, dtype=np.float32)
    obs_dim = observation_space.shape[0] if hasattr(observation_space, 'shape') else len(low)
    # Handle unbounded Box: replace -inf/inf with finite fallback for sampling
    low = np.where(np.isfinite(low), low, -10.0)
    high = np.where(np.isfinite(high), high, 10.0)
    
    if method == 'uniform':
        # Generate uniform random samples within bounds
        obs = np.random.uniform(low=low, high=high, size=(num_samples, obs_dim)).astype(np.float32)
    elif method == 'grid':
        # Generate grid samples (for low-dimensional spaces)
        if obs_dim > 3:
            print(f"Warning: Grid sampling for {obs_dim}D space may be slow. Using uniform instead.")
            return generate_test_observations(observation_space, num_samples, 'uniform')
        
        # Create grid points
        points_per_dim = int(np.ceil(num_samples ** (1.0 / obs_dim)))
        grids = np.meshgrid(*[np.linspace(low[i], high[i], points_per_dim) for i in range(obs_dim)])
        obs = np.stack([g.flatten() for g in grids], axis=1).astype(np.float32)
        
        # Limit to num_samples
        if len(obs) > num_samples:
            indices = np.linspace(0, len(obs) - 1, num_samples, dtype=int)
            obs = obs[indices]
    else:
        raise ValueError(f"Unknown method: {method}")
    
    return obs


def get_actions_from_policy(policy, observations, normalized=False):
    """Get actions from policy for given observations.
    
    Args:
        policy: RLPolicyClean instance
        observations: numpy array of shape (num_samples, obs_dim)
        normalized: Whether observations are already normalized
        
    Returns:
        numpy array of actions (action_logits)
    """
    import torch
    
    policy.eval()
    actions = []
    
    # Reset hidden state for first observation
    policy.reset_hidden_state(batch_size=1)
    
    with torch.no_grad():
        for obs in observations:
            # Convert to tensor
            obs_tensor = torch.from_numpy(obs).float().unsqueeze(0)  # Add batch dimension
            
            # Get action
            action_logits, _ = policy(obs_tensor, normalized=normalized)
            
            # Convert to numpy
            action = action_logits[0].cpu().numpy()
            actions.append(action)
    
    return np.array(actions)


def save_test_data(observations, actions, output_file, include_actions=True):
    """Save test data to a file that rlPolicyCPP can read.
    
    Format:
        # num_samples obs_dim [action_dim]
        # obs_0 obs_1 ... obs_N [act_0 act_1 ... act_M]
        obs1_1 obs1_2 ... obs1_obs_dim [act1_1 ... act1_action_dim]
        obs2_1 obs2_2 ... obs2_obs_dim [act2_1 ... act2_action_dim]
        ...
    
    Args:
        observations: numpy array of shape (num_samples, obs_dim)
        actions: numpy array of shape (num_samples, action_dim) or None
        output_file: Path to output file
        include_actions: Whether to include actions in output
    """
    num_samples, obs_dim = observations.shape
    
    with open(output_file, 'w') as f:
        # Write header with dimensions
        if include_actions and actions is not None:
            action_dim = actions.shape[1] if len(actions.shape) > 1 else 1
            f.write(f"# {num_samples} {obs_dim} {action_dim}\n")
        else:
            f.write(f"# {num_samples} {obs_dim}\n")
        
        # Write column names
        col_names = [f"obs_{i}" for i in range(obs_dim)]
        if include_actions and actions is not None:
            action_dim = actions.shape[1] if len(actions.shape) > 1 else 1
            # Actions are [mean, logstd] concatenated, so action_dim = 2 * num_action_channels
            num_action_channels = action_dim // 2
            # Add mean columns
            col_names.extend([f"mean_{i}" for i in range(num_action_channels)])
            # Add logstd columns
            col_names.extend([f"logstd_{i}" for i in range(num_action_channels)])
        # Format column names with 14 character width
        f.write("# " + "".join(f"{name:14s}" for name in col_names) + "\n")
        
        # Write data with fixed column width of 14 characters
        for i in range(num_samples):
            # Write observation
            obs_str = ''.join(f'{x:14.8f}' for x in observations[i])
            f.write(obs_str)
            
            # Write action if included
            if include_actions and actions is not None:
                if len(actions.shape) > 1:
                    act_str = ''.join(f'{x:14.8f}' for x in actions[i])
                else:
                    act_str = f'{actions[i]:14.8f}'
                f.write(act_str)
            
            f.write('\n')
    
    print(f"Saved {num_samples} test samples to {output_file}")


def main():
    parser = argparse.ArgumentParser(
        description='Generate test observations and actions for rlPolicyCPP validation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Output file will be created in same directory as .pth file with name based on .pth file
  python generate_test_data.py --pth=checkpoint.pth --env=point_trajectory_env.py
  # Creates: checkpoint_test_data.txt in same directory as checkpoint.pth
  
  python generate_test_data.py --pth=checkpoint.pth --env=point_trajectory_env.py --samples=200 --method=grid
  
  # Specify custom extension (file name still based on .pth file)
  python generate_test_data.py --pth=checkpoint.pth --env=point_trajectory_env.py --output=.dat
  # Creates: checkpoint_test_data.dat in same directory as checkpoint.pth
        """
    )
    parser.add_argument('--samples', type=int, default=10,
                       help='Number of test observations to generate (default: 10)')
    parser.add_argument('--pth', type=str, required=True,
                       help='Path to PyTorch checkpoint (.pth file)')
    parser.add_argument('--env', type=str, required=True,
                       help='Path to environment Python file')
    parser.add_argument('--output', type=str, default=None,
                       help='Output file extension (optional). File name is always based on .pth file name and created in .pth file directory (default: .txt)')
    parser.add_argument('--method', type=str, default='uniform', choices=['uniform', 'grid'],
                       help='Sampling method: uniform (random) or grid (default: uniform)')
    parser.add_argument('--nonlinearity', type=str, default='relu', choices=['relu', 'elu', 'tanh'],
                       help='Activation function (must match training, default: relu)')
    parser.add_argument('--device', type=str, default='cpu', choices=['cpu', 'cuda'],
                       help='Device for policy inference (default: cpu)')
    parser.add_argument('--no-actions', action='store_true',
                       help='Do not include actions in output file (observations only)')
    parser.add_argument('--normalized', action='store_true',
                       help='Treat observations as already normalized')
    
    args = parser.parse_args()
    
    # Check if import was successful
    if RLPolicyClean is None:
        print("Error: Could not import RLPolicyClean.")
        print(f"  Make sure rl_policyClean.py is in the same directory as generate_test_data.py.")
        if _import_error:
            print(f"  Import error: {_import_error}")
        return 1
    
    # Validate files
    if not os.path.exists(args.pth):
        print(f"Error: Checkpoint file not found: {args.pth}")
        return 1
    
    if not os.path.exists(args.env):
        print(f"Error: Environment file not found: {args.env}")
        return 1
    
    # Determine output file path - always based on pth file and in pth file folder
    pth_path = Path(args.pth).resolve()
    pth_dir = pth_path.parent
    pth_stem = pth_path.stem  # filename without extension
    
    # Always use pth file name as base and create in pth file directory
    if args.output is None:
        # Default: use .txt extension
        args.output = str(pth_dir / f"{pth_stem}_test_data.txt")
        print(f"Output file not specified, using: {args.output}")
    else:
        # If output is specified, extract extension if provided, otherwise use .txt
        output_path_specified = Path(args.output)
        output_ext = output_path_specified.suffix if output_path_specified.suffix else '.txt'
        # Always use pth file name as base, place in pth file directory
        args.output = str(pth_dir / f"{pth_stem}_test_data{output_ext}")
        if output_path_specified.suffix:
            print(f"Output file specified with extension '{output_ext}', using: {args.output}")
        else:
            print(f"Output file will be based on pth file name: {args.output}")
    
    # Ensure output directory exists
    output_path = Path(args.output).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    print(f"Loading checkpoint from: {args.pth}")
    try:
        policy = RLPolicyClean.load_from_checkpoint(
            args.pth,
            device=args.device,
            nonlinearity=args.nonlinearity,
        ).eval()
        print(f"  ✓ Policy loaded successfully")
        print(f"  Observation size: {policy.obs_mean.shape[0]}")
    except Exception as e:
        print(f"  ✗ Error loading policy: {e}")
        return 1
    
    print(f"\nLoading environment from: {args.env}")
    try:
        env = load_environment(args.env)
        print(f"  ✓ Environment loaded successfully")
        print(f"  Observation space: {env.observation_space}")
        print(f"  Action space: {env.action_space}")
    except Exception as e:
        print(f"  ✗ Error loading environment: {e}")
        return 1
    
    # Verify observation space matches policy
    obs_dim = env.observation_space.shape[0] if hasattr(env.observation_space, 'shape') else len(env.observation_space.low)
    policy_obs_dim = policy.obs_mean.shape[0]
    if obs_dim != policy_obs_dim:
        print(f"\nWarning: Observation dimension mismatch!")
        print(f"  Environment: {obs_dim}, Policy: {policy_obs_dim}")
        print(f"  Using policy observation dimension: {policy_obs_dim}")
        # Adjust observation space bounds if needed
        if hasattr(env.observation_space, 'low'):
            # Create a compatible observation space
            low = env.observation_space.low[:policy_obs_dim] if len(env.observation_space.low) >= policy_obs_dim else np.pad(env.observation_space.low, (0, policy_obs_dim - len(env.observation_space.low)), constant_values=-1.0)
            high = env.observation_space.high[:policy_obs_dim] if len(env.observation_space.high) >= policy_obs_dim else np.pad(env.observation_space.high, (0, policy_obs_dim - len(env.observation_space.high)), constant_values=1.0)
            from gymnasium import spaces
            obs_space = spaces.Box(low=low, high=high, dtype=np.float32)
        else:
            print(f"  Error: Cannot adjust observation space")
            return 1
    else:
        obs_space = env.observation_space
    
    print(f"\nGenerating {args.samples} test observations (method: {args.method})...")
    try:
        observations = generate_test_observations(obs_space, num_samples=args.samples, method=args.method)
        print(f"  ✓ Generated {len(observations)} observations")
    except Exception as e:
        print(f"  ✗ Error generating observations: {e}")
        return 1
    
    actions = None
    if not args.no_actions:
        print(f"\nGetting actions from policy...")
        try:
            actions = get_actions_from_policy(policy, observations, normalized=args.normalized)
            print(f"  ✓ Generated {len(actions)} actions")
            print(f"  Action shape: {actions.shape}")
        except Exception as e:
            print(f"  ✗ Error getting actions: {e}")
            import traceback
            traceback.print_exc()
            return 1
    
    print(f"\nSaving test data to: {args.output}")
    try:
        save_test_data(observations, actions, args.output, include_actions=not args.no_actions)
        print(f"  ✓ Test data saved successfully")
    except Exception as e:
        print(f"  ✗ Error saving test data: {e}")
        return 1
    
    print(f"\n✓ Test data generation complete!")
    print(f"\nYou can now use this file with rlPolicyCPP for validation.")
    
    return 0


if __name__ == '__main__':
    exit_code = 0
    try:
        exit_code = main()
    except KeyboardInterrupt:
        print("\nInterrupted by user", file=sys.stderr)
        exit_code = 1
    except Exception as e:
        print(f"\nUnexpected error in main(): {e}", file=sys.stderr)
        import traceback
        traceback.print_exc(file=sys.stderr)
        exit_code = 1
    finally:
        # Flush all output before exiting
        sys.stdout.flush()
        sys.stderr.flush()
        if exit_code != 0:
            # Ensure error is visible - print to both stdout and stderr
            error_msg = f"ERROR: Script failed with exit code {exit_code}"
            print(error_msg, file=sys.stderr)
            print(error_msg)  # Also to stdout for visibility
            sys.stdout.flush()
            sys.stderr.flush()
    
    sys.exit(exit_code)
