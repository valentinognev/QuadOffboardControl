from __future__ import annotations

import math
from dataclasses import dataclass

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Normal

LOG_STD_MIN = -5.0
LOG_STD_MAX = 1.5


class ConvBlock(nn.Module):
    def __init__(self, in_ch: int, out_ch: int, stride: int = 1):
        super().__init__()
        self.conv = nn.Conv2d(in_ch, out_ch, kernel_size=3, stride=stride, padding=1)
        self.norm = nn.GroupNorm(num_groups=max(1, min(8, out_ch // 8 or 1)), num_channels=out_ch)
        self.act = nn.GELU()

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.act(self.norm(self.conv(x)))


class MapEncoder(nn.Module):
    """Compact CNN encoder for local actor crops or coarse critic maps."""

    def __init__(self, in_channels: int, map_size: int, width: int = 32):
        super().__init__()
        self.net = nn.Sequential(
            ConvBlock(in_channels, width, stride=1),
            ConvBlock(width, width, stride=2),
            ConvBlock(width, width * 2, stride=1),
            ConvBlock(width * 2, width * 2, stride=2),
            ConvBlock(width * 2, width * 4, stride=1),
            nn.AdaptiveAvgPool2d((1, 1)),
            nn.Flatten(),
        )
        with torch.no_grad():
            dummy = torch.zeros(1, in_channels, map_size, map_size)
            self.out_dim = int(self.net(dummy).shape[-1])

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


class NeighborEncoder(nn.Module):
    """Encode variable-number neighbors using masked attention pooling."""

    def __init__(self, neighbor_dim: int, hidden: int = 96):
        super().__init__()
        self.pre = nn.Sequential(
            nn.Linear(neighbor_dim, hidden),
            nn.GELU(),
            nn.Linear(hidden, hidden),
            nn.GELU(),
        )
        self.score = nn.Linear(hidden, 1)
        self.out_dim = hidden

    def forward(self, neighbor_state: torch.Tensor, neighbor_mask: torch.Tensor) -> torch.Tensor:
        # neighbor_state: (B, K, D), neighbor_mask: (B, K)
        h = self.pre(neighbor_state)
        score = self.score(h).squeeze(-1)
        invalid = neighbor_mask <= 0.0
        score = score.masked_fill(invalid, -1e9)
        attn = torch.softmax(score, dim=1)
        attn = torch.where(invalid, torch.zeros_like(attn), attn)
        denom = attn.sum(dim=1, keepdim=True).clamp(min=1e-6)
        attn = attn / denom
        pooled = torch.sum(h * attn.unsqueeze(-1), dim=1)
        return pooled


class Actor(nn.Module):
    """Decentralized CNN actor: local map + self state + neighbor summary."""

    def __init__(self, map_channels: int, map_size: int, self_state_dim: int,
                 neighbor_state_dim: int, max_neighbors: int, action_dim: int = 3):
        super().__init__()
        _ = max_neighbors
        self.encoder = MapEncoder(map_channels, map_size, width=32)
        self.neighbor_enc = NeighborEncoder(neighbor_state_dim, hidden=96)
        total_in = self.encoder.out_dim + self_state_dim + self.neighbor_enc.out_dim
        self.backbone = nn.Sequential(
            nn.LayerNorm(total_in),
            nn.Linear(total_in, 256),
            nn.GELU(),
            nn.Linear(256, 256),
            nn.GELU(),
            nn.Linear(256, 192),
            nn.GELU(),
        )
        self.mean_head = nn.Linear(192, action_dim)
        self.log_std = nn.Parameter(torch.full((action_dim,), -0.7))

    def _features(self, maps: torch.Tensor, self_state: torch.Tensor,
                  neighbor_state: torch.Tensor, neighbor_mask: torch.Tensor) -> torch.Tensor:
        map_enc = self.encoder(maps)
        nbr_enc = self.neighbor_enc(neighbor_state, neighbor_mask)
        return self.backbone(torch.cat([map_enc, self_state, nbr_enc], dim=-1))

    def distribution(self, maps: torch.Tensor, self_state: torch.Tensor,
                     neighbor_state: torch.Tensor, neighbor_mask: torch.Tensor) -> Normal:
        z = self._features(maps, self_state, neighbor_state, neighbor_mask)
        mean = self.mean_head(z)
        log_std = torch.clamp(self.log_std, LOG_STD_MIN, LOG_STD_MAX)
        std = torch.exp(log_std).unsqueeze(0).expand_as(mean)
        return Normal(mean, std)

    @staticmethod
    def _tanh_squash(raw_action: torch.Tensor, dist: Normal):
        squashed = torch.tanh(raw_action)
        log_prob = dist.log_prob(raw_action).sum(dim=-1)
        correction = torch.log(1.0 - squashed.pow(2) + 1e-6).sum(dim=-1)
        return squashed, log_prob - correction

    def act(self, maps: torch.Tensor, self_state: torch.Tensor,
            neighbor_state: torch.Tensor, neighbor_mask: torch.Tensor,
            deterministic: bool = False):
        dist = self.distribution(maps, self_state, neighbor_state, neighbor_mask)
        raw = dist.mean if deterministic else dist.rsample()
        return self._tanh_squash(raw, dist)

    def evaluate_actions(self, maps: torch.Tensor, self_state: torch.Tensor,
                         neighbor_state: torch.Tensor, neighbor_mask: torch.Tensor,
                         actions: torch.Tensor):
        clipped = torch.clamp(actions, -0.999999, 0.999999)
        raw = 0.5 * (torch.log1p(clipped) - torch.log1p(-clipped))
        dist = self.distribution(maps, self_state, neighbor_state, neighbor_mask)
        log_prob = dist.log_prob(raw).sum(dim=-1)
        correction = torch.log(1.0 - clipped.pow(2) + 1e-6).sum(dim=-1)
        entropy = dist.entropy().sum(dim=-1)
        return log_prob - correction, entropy


class CentralizedCritic(nn.Module):
    """CNN critic over explicit coarse global maps plus global agent features.

    This is stronger than flattening the maps into an MLP because it preserves
    spatial locality in the uncovered/stale/global occupancy signals.
    """

    def __init__(self, map_channels: int, map_size: int, global_feature_dim: int,
                  self_state_dim: int, hidden_dim: int = 256):
        super().__init__()
        self.map_encoder = MapEncoder(map_channels, map_size, width=24)
        fused_dim = self.map_encoder.out_dim + global_feature_dim + self_state_dim
        self.backbone = nn.Sequential(
            nn.LayerNorm(fused_dim),
            nn.Linear(fused_dim, hidden_dim),
            nn.GELU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.GELU(),
        )
        self.value_head = nn.Linear(hidden_dim, 1)
        self.frontier_head = nn.Sequential(
            nn.Linear(hidden_dim, hidden_dim // 2),
            nn.GELU(),
            nn.Linear(hidden_dim // 2, 1),
            nn.Sigmoid(),
        )
        # Corner coverage auxiliary head: predicts per-corner coverage scalar(s).
        # The caller (ActorCritic) will pass `num_corner_feats` to indicate how
        # many scalar outputs are required (2 per corner in the env).
        self.corner_head = None  # created later by ActorCritic if needed
        # ── Guidance vector (CRITIC-ONLY, never seen by actor) ──
        # Predicts a 2D unit direction "where this drone SHOULD move" based on:
        #   - Voronoi frontier direction (go toward your assigned uncovered area)
        #   - Repulsion from nearby drones (spread out)
        #   - Corner attraction when coverage is high
        # The actor never sees this vector. It shapes the critic's shared backbone
        # to learn spatial awareness, which improves value estimation, which gives
        # the actor better advantage gradients.
        self.guidance_head = nn.Sequential(
            nn.Linear(hidden_dim, hidden_dim // 2),
            nn.GELU(),
            nn.Linear(hidden_dim // 2, 2),
            nn.Tanh(),
        )
        # ── Spatial value auxiliaries (learned, NOT hand-crafted waypoints) ──
        # Marginal contribution head: predicts how much NEW coverage this drone
        # will contribute in the next N steps from its current position [0,1]
        self.marginal_head = nn.Sequential(
            nn.Linear(hidden_dim, hidden_dim // 2),
            nn.GELU(),
            nn.Linear(hidden_dim // 2, 1),
            nn.Sigmoid(),
        )
        # Territorial value head: predicts fraction of uncovered cells that are
        # closest to THIS drone (Voronoi ownership fraction) [0,1]
        self.territorial_head = nn.Sequential(
            nn.Linear(hidden_dim, hidden_dim // 2),
            nn.GELU(),
            nn.Linear(hidden_dim // 2, 1),
            nn.Sigmoid(),
        )

    def _features(self, global_maps: torch.Tensor, global_features: torch.Tensor,
                  self_state: torch.Tensor) -> torch.Tensor:
        map_z = self.map_encoder(global_maps)
        return self.backbone(torch.cat([map_z, global_features, self_state], dim=-1))

    def forward(self, global_maps: torch.Tensor, global_features: torch.Tensor,
                self_state: torch.Tensor) -> torch.Tensor:
        h = self._features(global_maps, global_features, self_state)
        return self.value_head(h).squeeze(-1)

    def forward_with_frontier(self, global_maps: torch.Tensor, global_features: torch.Tensor,
                               self_state: torch.Tensor):
        h = self._features(global_maps, global_features, self_state)
        corner_out = self.corner_head(h).squeeze(-1) if self.corner_head is not None else None
        return (self.value_head(h).squeeze(-1),
                self.frontier_head(h).squeeze(-1),
                self.guidance_head(h),
                corner_out,
                self.marginal_head(h).squeeze(-1),
                self.territorial_head(h).squeeze(-1))


class ActorCritic(nn.Module):
    def __init__(self, actor_obs_spec: dict[str, int], critic_obs_spec: dict[str, int]):
        super().__init__()
        self.actor = Actor(
            map_channels=actor_obs_spec['map_channels'],
            map_size=actor_obs_spec['map_size'],
            self_state_dim=actor_obs_spec['self_state_dim'],
            neighbor_state_dim=actor_obs_spec['neighbor_state_dim'],
            max_neighbors=actor_obs_spec['max_neighbors'],
        )
        self.critic = CentralizedCritic(
            map_channels=critic_obs_spec['map_channels'],
            map_size=critic_obs_spec['map_size'],
            global_feature_dim=critic_obs_spec['global_feature_dim'],
            self_state_dim=actor_obs_spec['self_state_dim'],
        )
        # If critic_obs_spec provides corner dimensions, add corner head to critic
        num_corner_feats = critic_obs_spec.get('num_corners', 0) * critic_obs_spec.get('corner_feat_per', 0)
        if num_corner_feats > 0:
            hd = self.critic.backbone[-2].out_features if hasattr(self.critic.backbone[-2], 'out_features') else None
            # create a small head mapping hidden_dim -> num_corner_feats
            hidden_dim = 256
            self.critic.corner_head = nn.Sequential(
                nn.Linear(hidden_dim, max(64, hidden_dim // 2)),
                nn.GELU(),
                nn.Linear(max(64, hidden_dim // 2), num_corner_feats),
                nn.Sigmoid(),
            )
