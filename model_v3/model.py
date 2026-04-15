from __future__ import annotations

import torch
import torch.nn as nn
from torch.distributions import Normal


class MapEncoder(nn.Module):
    def __init__(self, in_channels: int, map_size: int):
        super().__init__()
        self.net = nn.Sequential(
            nn.Conv2d(in_channels, 32, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
            nn.Flatten(),
        )
        with torch.no_grad():
            dummy = torch.zeros(1, in_channels, map_size, map_size)
            self.out_dim = self.net(dummy).shape[-1]

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


class Actor(nn.Module):
    def __init__(self, map_channels: int, map_size: int, self_state_dim: int):
        super().__init__()
        self.encoder = MapEncoder(map_channels, map_size)
        self.mlp = nn.Sequential(
            nn.Linear(self.encoder.out_dim + self_state_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
        )
        self.mean_head = nn.Linear(256, 3)
        self.log_std = nn.Parameter(torch.zeros(3))

    def distribution(self, maps: torch.Tensor, self_state: torch.Tensor) -> Normal:
        enc = self.encoder(maps)
        z = self.mlp(torch.cat([enc, self_state], dim=-1))
        mean = self.mean_head(z)
        std = torch.exp(self.log_std).unsqueeze(0).expand_as(mean)
        return Normal(mean, std)

    def act(self, maps: torch.Tensor, self_state: torch.Tensor):
        dist = self.distribution(maps, self_state)
        raw = dist.rsample()
        squashed = torch.tanh(raw)
        log_prob = dist.log_prob(raw).sum(dim=-1)
        return squashed, log_prob

    def evaluate_actions(self, maps: torch.Tensor, self_state: torch.Tensor, actions: torch.Tensor):
        # approximate inverse tanh for stored squashed actions
        clipped = torch.clamp(actions, -0.999, 0.999)
        raw = 0.5 * torch.log((1 + clipped) / (1 - clipped))
        dist = self.distribution(maps, self_state)
        log_prob = dist.log_prob(raw).sum(dim=-1)
        entropy = dist.entropy().sum(dim=-1)
        return log_prob, entropy


class Critic(nn.Module):
    def __init__(self, input_dim: int):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, 1),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x).squeeze(-1)


class ActorCritic(nn.Module):
    def __init__(self, map_channels: int, map_size: int, self_state_dim: int, critic_dim: int):
        super().__init__()
        self.actor = Actor(map_channels, map_size, self_state_dim)
        self.critic = Critic(critic_dim)
