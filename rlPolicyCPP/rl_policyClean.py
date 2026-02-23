import sys
# Fix for Raspberry Pi Zero: disable SVE detection to avoid prctl(PR_SVE_GET_VL) error
import os
# Force disable SVE detection before any imports
os.environ['CPUINFO_DISABLE_SVE'] = '1'
# Suppress cpuinfo errors by redirecting stderr during torch import
import warnings
warnings.filterwarnings('ignore', category=RuntimeWarning, message='.*cpuinfo.*')
warnings.filterwarnings('ignore', message='.*prctl.*')

# Redirect stderr at file descriptor level to suppress cpuinfo error messages
# This catches errors from C extensions that write directly to fd 2
_saved_stderr_fd = None
_devnull_fd = None
_original_stderr = None
_original_stderr_fd = None
try:
    _original_stderr_fd = sys.stderr.fileno()
    # Save a copy of the original stderr fd
    _saved_stderr_fd = os.dup(_original_stderr_fd)
    # Open /dev/null
    _devnull_fd = os.open(os.devnull, os.O_WRONLY)
    # Redirect stderr to /dev/null
    os.dup2(_devnull_fd, _original_stderr_fd)
except (AttributeError, OSError):
    # Fallback to Python-level redirection if fd manipulation fails
    _original_stderr = sys.stderr
    sys.stderr = open(os.devnull, 'w')

try:
    import torch
    import torch.nn as nn
    import torch.nn.functional as F
finally:
    # Restore stderr
    try:
        if _saved_stderr_fd is not None and _original_stderr_fd is not None:
            # Restore original stderr
            os.dup2(_saved_stderr_fd, _original_stderr_fd)
            os.close(_saved_stderr_fd)
            if _devnull_fd is not None:
                os.close(_devnull_fd)
        elif _original_stderr is not None:
            sys.stderr.close()
            sys.stderr = _original_stderr
    except Exception:
        pass  # Ignore errors during cleanup

# Ensure we can import Sample Factory utilities if needed
sys.path.insert(0, 'sample-factory')


EPS = 1e-5  # Match Sample Factory's _NORM_EPS from running_mean_std.py


class RLPolicyClean(nn.Module):
    """Minimal, inference-only policy that mirrors Sample Factory forward pass.

    Components:
      - Input normalization using checkpoint running mean/var
      - Encoder MLP (Linear/ReLU × 3) for vector obs
      - GRU core
      - Linear distribution head producing [mean, logstd]

    Forward returns (action_logits, new_hxs) where action_logits = [mean, logstd].
    """

    def __init__(self):
        super().__init__()
        self.obs_mean: torch.Tensor | None = None
        self.obs_var: torch.Tensor | None = None
        self.encoder: nn.Module | None = None
        self.core: nn.GRU | None = None
        self.decoder: nn.Module | None = None  # kept for parity; identity by default
        self.dist_linear: nn.Linear | None = None
        # Internal recurrent state (not exposed to user)
        self.hxs: torch.Tensor | None = None  # shape [num_layers, batch, hidden]

    def forward(self, obs: torch.Tensor, *, normalized: bool = False):
        """Forward pass through the policy network.
        
        Args:
            obs: Observation tensor [batch, features] or [features]
            normalized: If True, skip normalization (obs already normalized)
            
        Returns:
            action_logits: [batch, 2*action_dim] containing [mean, logstd] concatenated
            h_next: New hidden state [batch, hidden_size]
        """
        # Normalize observation
        if not isinstance(obs, torch.Tensor):
            obs = torch.tensor(obs, dtype=torch.float32)
        else:
            obs = obs.to(dtype=torch.float32)
        if obs.dim() == 1:
            obs = obs.unsqueeze(0)  # [features] -> [1, features]
        if not normalized and getattr(self, '_has_obs_normalizer', True):
            x = (obs - self.obs_mean) / torch.sqrt(self.obs_var + EPS)
            x = torch.clamp(x, -5.0, 5.0)
        else:
            x = obs

        # Encoder forward (standard PyTorch Sequential)
        x = self.encoder(x)  # [batch, encoder_out]

        # GRU forward (or pass-through when no RNN in checkpoint)
        # Core expects [seq_len, batch, features]; hxs is [num_layers, batch, hidden]
        x = x.unsqueeze(0)  # [batch, features] -> [1, batch, features]
        hidden_size = self._core_hidden_size()
        if self.hxs is None or self.hxs.shape[1] != x.shape[1]:
            self.hxs = torch.zeros(1, x.shape[1], hidden_size, device=x.device)

        # Standard PyTorch GRU forward (or pass-through core when use_rnn=False)
        x, h_next = self.core(x, self.hxs)
        x = x.squeeze(0)  # [1, batch, hidden_size] -> [batch, hidden_size]
        self.hxs = h_next

        # Decoder forward (standard PyTorch module)
        if self.decoder is not None:
            x = self.decoder(x)

        # Distribution linear forward (standard PyTorch Linear)
        params = self.dist_linear(x)  # [batch, action_dim] or [batch, 2*action_dim]
        
        # When learned_stddev exists (adaptive_stddev=False): params=mean only, logstd from learned_stddev
        if self.learned_stddev is not None:
            mean = params
            logstd = self.learned_stddev.unsqueeze(0).expand(mean.shape[0], -1)
            action_logits = torch.cat([mean, logstd], dim=-1)
        else:
            # Adaptive stddev: params = [mean, logstd] concatenated
            mean, logstd = params.chunk(2, dim=-1)
            action_logits = torch.cat([mean, logstd], dim=-1)
        
        # Return h_next in [batch, hidden_size] format for consistency with C++
        h_next = h_next.squeeze(0)  # [1, batch, hidden_size] -> [batch, hidden_size]
        return action_logits, h_next

    def _init_modules(self, obs_mean: torch.Tensor, obs_var: torch.Tensor,
                       encoder: nn.Module, core: nn.Module, dist_linear: nn.Linear,
                       decoder: nn.Module | None = None, learned_stddev: torch.Tensor | None = None):
        self.obs_mean = nn.Parameter(obs_mean.float(), requires_grad=False)
        self.obs_var = nn.Parameter(obs_var.float(), requires_grad=False)
        self.encoder = encoder
        self.core = core
        self.decoder = decoder
        self.dist_linear = dist_linear
        self.learned_stddev = nn.Parameter(learned_stddev.float(), requires_grad=False) if learned_stddev is not None else None
        self.hxs = None

    def _core_hidden_size(self) -> int:
        """Hidden size of the core (GRU or pass-through)."""
        return getattr(self.core, 'hidden_size', self.dist_linear.weight.shape[1])

    def reset_hidden_state(self, batch_size: int = 1, device: torch.device | None = None) -> None:
        """Reset internal RNN state to zeros for the given batch size."""
        if device is None:
            device = next(self.parameters()).device
        hidden_size = self._core_hidden_size()
        self.hxs = torch.zeros(1, batch_size, hidden_size, device=device)

    def set_hidden_state(self, hxs: torch.Tensor) -> None:
        """Set internal RNN state to match external state.
        
        Args:
            hxs: Hidden state tensor of shape [batch, hidden_size] or [1, batch, hidden_size]
        """
        if hxs.dim() == 2:
            # Convert [batch, hidden_size] to [1, batch, hidden_size]
            self.hxs = hxs.unsqueeze(0).clone()
        else:
            # Already [1, batch, hidden_size] or [num_layers, batch, hidden_size]
            self.hxs = hxs.clone()

    @staticmethod
    def _make_pass_through_core(hidden_size: int, device: str = 'cpu') -> nn.Module:
        """Create a pass-through 'core' when checkpoint has no RNN (use_rnn=False).
        Behaves like a GRU for API: forward(x, h) -> (x, h). hidden_size matches encoder output.
        """
        class _PassThroughCore(nn.Module):
            def __init__(self, size: int):
                super().__init__()
                self.hidden_size = size

            def forward(self, x: torch.Tensor, h: torch.Tensor):
                return x, h

        core = _PassThroughCore(hidden_size)
        return core.to(device)

    @staticmethod
    def _activation(nonlinearity: str) -> nn.Module:
        if nonlinearity == 'relu':
            return nn.ReLU(inplace=False)
        if nonlinearity == 'elu':
            return nn.ELU(inplace=False)
        if nonlinearity == 'tanh':
            return nn.Tanh()
        raise RuntimeError(f"Unsupported nonlinearity: {nonlinearity}")

    @staticmethod
    def _has_rnn(ckpt: dict) -> bool:
        """True if checkpoint contains GRU core weights."""
        return 'core.core.weight_hh_l0' in ckpt

    @staticmethod
    def _is_swarm_encoder_ckpt(ckpt: dict) -> bool:
        """True if checkpoint uses actor_encoder (swarm attention) instead of encoder.encoders.obs.mlp_head."""
        return 'actor_encoder.self_goal_encoder.0.weight' in ckpt

    @staticmethod
    def _build_swarm_encoder_from_ckpt(ckpt: dict, *, nonlinearity: str) -> nn.Module:
        """Build encoder from actor_encoder.* (SwarmAttentionEncoder) state dict.
        Obs layout: [self_obs (4)] [my_destination_rel (2)] [neighbor_obs ((max_agents-1)*4)].
        """
        act = RLPolicyClean._activation(nonlinearity)
        prefix = 'actor_encoder.'

        def load_sequential(ckpt: dict, key_prefix: str, layer_indices: list[int], last_has_act: bool = True) -> nn.Sequential:
            layers: list[nn.Module] = []
            for i, idx in enumerate(layer_indices):
                w = ckpt[f'{key_prefix}{idx}.weight']
                b = ckpt[f'{key_prefix}{idx}.bias']
                lin = nn.Linear(w.shape[1], w.shape[0])
                lin.weight.data.copy_(w)
                lin.bias.data.copy_(b)
                layers.append(lin)
                if i < len(layer_indices) - 1 or last_has_act:
                    layers.append(act)
            return nn.Sequential(*layers)

        # self_goal_encoder: 0, 2 (Linear layers)
        self_goal_encoder = load_sequential(ckpt, prefix + 'self_goal_encoder.', [0, 2])
        # neighbor: embedding_mlp 0,2; value_mlp 0,2; attention_mlp 0,2,4 (last layer is logits, no act)
        embedding_mlp = load_sequential(ckpt, prefix + 'neighbor_encoder.embedding_mlp.', [0, 2])
        value_mlp = load_sequential(ckpt, prefix + 'neighbor_encoder.value_mlp.', [0, 2])
        attention_mlp = load_sequential(ckpt, prefix + 'neighbor_encoder.attention_mlp.', [0, 2, 4], last_has_act=False)

        self_obs_dim = 4
        my_destination_rel_dim = 2
        single_neighbor_dim = 4
        hidden_size = ckpt[prefix + 'self_goal_encoder.0.weight'].shape[0]
        num_neighbors = ckpt[prefix + 'neighbor_encoder.embedding_mlp.0.weight'].shape[1] - self_obs_dim  # 8 - 4 = 4 -> wrong. Actually in_dim = self_obs_dim + single_neighbor_dim = 4+4=8. So we can't get num_neighbors from that. From attention_mlp.4.weight we have out 1. So we need num_neighbors from somewhere. It's (obs_dim - 6) // 4. So we'll get it from the wrapper which receives obs.

        class _SwarmNeighborEncoder(nn.Module):
            def __init__(self):
                super().__init__()
                self.embedding_mlp = embedding_mlp
                self.value_mlp = value_mlp
                self.attention_mlp = attention_mlp
                self.self_obs_dim = self_obs_dim
                self.single_neighbor_dim = single_neighbor_dim
                self.hidden_size = hidden_size

            def forward(self, self_obs: torch.Tensor, neighbor_obs_flat: torch.Tensor, batch_size: int) -> torch.Tensor:
                num_n = neighbor_obs_flat.shape[1] // self.single_neighbor_dim
                if num_n == 0:
                    return torch.zeros(batch_size, self.hidden_size, device=self_obs.device, dtype=self_obs.dtype)
                neighbor_obs = neighbor_obs_flat.reshape(-1, self.single_neighbor_dim)
                self_obs_rep = self_obs.repeat(num_n, 1)
                mlp_in = torch.cat((self_obs_rep, neighbor_obs), dim=1)
                embeddings = self.embedding_mlp(mlp_in)
                values = self.value_mlp(embeddings)
                embeddings_grp = embeddings.reshape(batch_size, -1, self.hidden_size)
                mean_emb = embeddings_grp.mean(dim=1)
                mean_emb_rep = mean_emb.repeat(num_n, 1)
                attn_in = torch.cat((embeddings, mean_emb_rep), dim=1)
                attn_scores = self.attention_mlp(attn_in).view(batch_size, -1)
                attn_weights = F.softmax(attn_scores, dim=1).view(-1, 1)
                weighted = (attn_weights * values).view(batch_size, -1, self.hidden_size)
                return weighted.sum(dim=1)

        neighbor_encoder = _SwarmNeighborEncoder()

        class _SwarmEncoderFlat(nn.Module):
            """Swarm encoder that takes flat obs [batch, 6 + num_neighbors*4] and outputs [batch, 512]."""

            def __init__(self):
                super().__init__()
                self.self_goal_encoder = self_goal_encoder
                self.neighbor_encoder = neighbor_encoder
                self.self_obs_dim = self_obs_dim
                self.my_destination_rel_dim = my_destination_rel_dim
                self.single_neighbor_dim = single_neighbor_dim

            def forward(self, obs: torch.Tensor) -> torch.Tensor:
                batch_size = obs.shape[0]
                idx = 0
                self_obs = obs[:, idx:idx + self.self_obs_dim]
                idx += self.self_obs_dim
                my_destination_rel = obs[:, idx:idx + self.my_destination_rel_dim]
                idx += self.my_destination_rel_dim
                neighbor_obs = obs[:, idx:]
                self_goal_in = torch.cat((self_obs, my_destination_rel), dim=1)
                self_goal_embed = self.self_goal_encoder(self_goal_in)
                neighbor_embed = self.neighbor_encoder(self_obs, neighbor_obs, batch_size)
                return torch.cat((self_goal_embed, neighbor_embed), dim=1)

        return _SwarmEncoderFlat()

    @staticmethod
    def _find_mlp_linear_indices(ckpt: dict) -> list[int]:
        """Find ordered Linear layer indices inside encoder.encoders.obs.mlp_head.*
        Sample Factory create_mlp uses indices 0,2,4,... for Linear layers (activations in between).
        """
        prefix = 'encoder.encoders.obs.mlp_head.'
        linear_indices: set[int] = set()
        for k in ckpt.keys():
            if not k.startswith(prefix):
                continue
            if k.endswith('.weight'):
                try:
                    idx = int(k[len(prefix):].split('.')[0])
                except Exception:
                    continue
                linear_indices.add(idx)
        return sorted(linear_indices)

    @classmethod
    def _build_encoder_from_ckpt(cls, ckpt: dict, *, nonlinearity: str, jit: bool) -> nn.Module:
        act = cls._activation(nonlinearity)
        indices = cls._find_mlp_linear_indices(ckpt)
        if len(indices) == 0:
            # No MLP layers configured
            enc = nn.Identity()
            return enc

        # Construct layers in the exact order of indices
        layers: list[nn.Module] = []
        for i, idx in enumerate(indices):
            w_key = f'encoder.encoders.obs.mlp_head.{idx}.weight'
            b_key = f'encoder.encoders.obs.mlp_head.{idx}.bias'
            weight = ckpt[w_key]
            bias = ckpt[b_key]
            in_f, out_f = weight.shape[1], weight.shape[0]
            lin = nn.Linear(in_f, out_f)
            # Load weights immediately to avoid dtype/device surprises later
            lin.weight.data.copy_(weight)
            lin.bias.data.copy_(bias)
            layers.append(lin)
            layers.append(act)

        enc = nn.Sequential(*layers)
        if jit:
            enc = torch.jit.script(enc)
        return enc

    @classmethod
    def load_from_checkpoint(
        cls,
        path: str,
        device: str = 'cpu',
        *,
        nonlinearity: str = 'relu',
        jit_encoder: bool = False,
    ) -> 'RLPolicyClean':
        """Create a policy instance from a Sample Factory checkpoint (.pth).

        Args:
            path: checkpoint path (best_*.pth or checkpoint_*.pth)
            device: 'cpu' or 'cuda'
            nonlinearity: 'relu' | 'elu' | 'tanh' — must match training config
            jit_encoder: if True, torch.jit.script the encoder MLP
        """
        ckpt = torch.load(path, map_location=device, weights_only=False)["model"]
        has_rnn = cls._has_rnn(ckpt)
        is_swarm = cls._is_swarm_encoder_ckpt(ckpt)

        # Observation normalizer parameters (RunningMeanStd)
        mean_key = 'obs_normalizer.running_mean_std.running_mean_std.obs.running_mean'
        var_key = 'obs_normalizer.running_mean_std.running_mean_std.obs.running_var'
        has_obs_normalizer = mean_key in ckpt and var_key in ckpt
        if has_obs_normalizer:
            obs_mean = ckpt[mean_key].detach()
            obs_var = ckpt[var_key].detach()
        else:
            in_size = 1
            if is_swarm:
                # Obs layout: self_obs (4) + my_destination_rel (2) + neighbor_obs ((max_agents-1)*4)
                in_size = 6 + 3 * 4  # default max_agents=4 -> 18
            else:
                try:
                    indices = cls._find_mlp_linear_indices(ckpt)
                    if len(indices) > 0:
                        first_w = ckpt[f'encoder.encoders.obs.mlp_head.{indices[0]}.weight']
                        in_size = first_w.shape[1]
                except Exception:
                    pass
            obs_mean = torch.zeros(in_size)
            obs_var = torch.ones(in_size)

        # Build encoder: swarm (actor_encoder) or default MLP (encoder.encoders.obs.mlp_head)
        if is_swarm:
            encoder = cls._build_swarm_encoder_from_ckpt(ckpt, nonlinearity=nonlinearity)
        else:
            encoder = cls._build_encoder_from_ckpt(ckpt, nonlinearity=nonlinearity, jit=jit_encoder)

        # Core: GRU when present, else pass-through (use_rnn=False)
        dist_in = ckpt['action_parameterization.distribution_linear.weight'].shape[1]
        dist_out = ckpt['action_parameterization.distribution_linear.weight'].shape[0]
        if has_rnn:
            gru_input_size = 512
            lin_indices = cls._find_mlp_linear_indices(ckpt)
            if len(lin_indices) > 0:
                last_idx = lin_indices[-1]
                last_w = ckpt[f'encoder.encoders.obs.mlp_head.{last_idx}.weight']
                gru_input_size = last_w.shape[0]
            rnn_size = ckpt['core.core.weight_hh_l0'].shape[1]
            core = nn.GRU(input_size=gru_input_size, hidden_size=rnn_size, batch_first=False)
            core.weight_ih_l0.data.copy_(ckpt['core.core.weight_ih_l0'])
            core.weight_hh_l0.data.copy_(ckpt['core.core.weight_hh_l0'])
            core.bias_ih_l0.data.copy_(ckpt['core.core.bias_ih_l0'])
            core.bias_hh_l0.data.copy_(ckpt['core.core.bias_hh_l0'])
        else:
            core = cls._make_pass_through_core(dist_in, device)

        # Identity decoder
        class _Identity(nn.Module):
            def forward(self, x):
                return x

        decoder = _Identity()

        # Distribution head: [mean, logstd] or mean + learned_stddev
        learned_stddev = None
        if 'action_parameterization.learned_stddev' in ckpt:
            # Non-adaptive: dist_linear outputs mean only, learned_stddev provides log(stddev)
            learned_stddev = ckpt['action_parameterization.learned_stddev'].detach()
        dist_linear = nn.Linear(dist_in, dist_out)
        dist_linear.weight.data.copy_(ckpt['action_parameterization.distribution_linear.weight'])
        dist_linear.bias.data.copy_(ckpt['action_parameterization.distribution_linear.bias'])

        policy = cls().to(device)
        policy._init_modules(obs_mean, obs_var, encoder, core, dist_linear, decoder, learned_stddev=learned_stddev)
        policy._has_obs_normalizer = has_obs_normalizer
        return policy


if __name__ == "__main__":
    import argparse
    import torch

    parser = argparse.ArgumentParser(description="RLPolicyClean one-step inference example")
    parser.add_argument("--ckpt", type=str, required=True, help="Path to Sample Factory checkpoint .pth")
    parser.add_argument("--device", type=str, default="cpu", choices=["cpu", "cuda"], help="Inference device")
    parser.add_argument("--nonlinearity", type=str, default="relu", choices=["relu", "elu", "tanh"], help="Activation to use (must match training)")
    parser.add_argument("--jit_encoder", action="store_true", help="Enable torch.jit.script for encoder MLP")
    parser.add_argument("--normalized", action="store_true", help="Treat provided obs as already normalized")
    args = parser.parse_args()

    # Load policy
    policy = RLPolicyClean.load_from_checkpoint(
        args.ckpt,
        device=args.device,
        nonlinearity=args.nonlinearity,
        jit_encoder=args.jit_encoder,
    ).eval()

    # Prepare a dummy observation (batch=1) matching checkpoint input size
    # If not normalized, obs will be normalized internally using checkpoint stats
    with torch.no_grad():
        in_size = policy.obs_mean.shape[0]
        obs = torch.zeros(1, in_size, device=args.device)
        policy.reset_hidden_state(batch_size=1, device=torch.device(args.device))

        action_logits, h_next = policy(obs, normalized=args.normalized)
        print("action_logits shape:", tuple(action_logits.shape))
        print("hidden state shape:", tuple(policy.hxs.shape))
        print("action_logits (first row):", action_logits[0].tolist())

