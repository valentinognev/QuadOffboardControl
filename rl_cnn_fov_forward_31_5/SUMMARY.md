# rl_cnn — Project Summary

## What the Model Does
A **multi-drone cooperative area coverage** system where 5 UAVs must efficiently scan a randomly-generated concave polygon (≈900 m²) within a 40×40 m world. Each drone has a narrow **60° forward-facing sensor** with 4 m range and must coordinate with teammates to achieve ≥98% coverage while keeping already-scanned areas "fresh" (not stale).

---

## Algorithm: PPO (Proximal Policy Optimization)
- **Vectorized training** across 8 parallel environments
- **Clipped surrogate objective** (clip_range=0.2)
- **GAE** (γ=0.995, λ=0.95) for advantage estimation
- **6 update epochs** per rollout, minibatch size 2048
- Gradient clipping at 0.8
- Entropy bonus (coef=0.02) for exploration
- **Tanh-squashed Gaussian** continuous actions

---

## Architecture (Actor-Critic with CTDE)

### Actor (Decentralized)
Each drone independently selects actions using only local + communicated info:
- **CNN MapEncoder**: 5-layer ConvNet (GroupNorm + GELU) with AdaptiveAvgPool → encodes 6-channel 32×32 local map
- **NeighborEncoder**: MLP + masked attention pooling over variable-number neighbors (up to 4)
- **MLP backbone**: [LayerNorm → 256 → 256 → 192] → Gaussian mean + learned log_std
- **Output**: 3 continuous actions ∈ [-1, 1] — forward speed, turn rate, strafe

### Critic (Centralized)
Has access to global state for training only:
- **CNN MapEncoder** on coarse 8×8 global maps (5 channels)
- Fused with global feature vector (45 dims: all agent states + coverage stats) + self_state
- **Auxiliary heads**: frontier potential prediction + frontier direction prediction (2D vector)

---

## Observations

### Actor Local Map (6 channels, 32×32)
| Ch | Content |
|----|---------|
| 0 | Coverage freshness (1=just scanned, 0=stale) |
| 1 | Never-visited mask (1=unexplored) |
| 2 | Other drones' positions |
| 3 | Active polygon area mask |
| 4 | Forward sensor FOV indicator |
| 5 | Shared trajectory heatmap (where all drones have been, decaying) |

### Self-State Vector (18 dims)
Normalized position, velocity, heading (sin/cos), wall distance, time progress, local coverage statistics (mean/min/max), local unexplored fraction, active area fraction, active agent ratio, active flag, previous action magnitude, corner proximity, local trajectory density.

### Neighbor State (10 dims per neighbor, up to 4)
Relative position, velocity, distance, active flag, heading (sin/cos), communicated trajectory direction (2D).
Aggregated via **learned attention pooling** (not fixed-order).

### Critic Global Obs
- 5-channel 8×8 downsampled maps (coverage, ever-seen, drone positions, active area, trajectory heatmap)
- Global feature vector: per-agent [x, y, vx, vy, heading_sin, heading_cos, active] × 5 agents + coverage stats (ever_seen, maintained, uncovered, stale fractions, time progress, etc.)

---

## Reward Design (Key Additions)

### Positive Rewards (main learning signal)
| Reward | Weight | Description |
|--------|--------|-------------|
| **First visit** | 600 | Team: new cells discovered (escalating — last 10% worth 4× more) |
| **Ever-seen gain** | 800 | Team: increase in ever-seen fraction (also escalating) |
| **Frontier align** | 150 | Per-drone: velocity aligned toward frontier direction — critical for 60° FOV |
| **New cells** | 200 | Per-drone: fraction of scan cone that was previously unseen |
| **Frontier progress** | 200 | Per-drone: moved closer to nearest unexplored cells |
| **Spread** | 60 | Per-drone: spatial separation from teammates (Voronoi-style) |
| **Heading spread** | 15 | Per-drone: heading diversity among drones |
| **Displacement** | 15 | Per-drone: rolling-window net displacement |
| **Milestones** | 50–1000 | One-time bonuses at 50/80/90/95/98% coverage |

### Negative Penalties (soft guards)
| Penalty | Weight | Description |
|---------|--------|-------------|
| Wall | 30 | Smooth penalty near polygon edges (0–3 range) |
| Collision | 20 | Hard + soft zones around 2m safety radius |
| Circling | 15 | Low straightness ratio over 3s rolling window |
| Stagnation | 10 | Not moving |
| Local staleness | 10 | Lingering where >95% cells already visited |
| Overlap | 5 | Scan IoU with other drones |
| Revisit | 1 | Scanning already-fresh cells |
| Control | 0.02 | Action smoothness |

### Coverage Decay Mechanism
- Scanned cells start at freshness=1.0, linearly decay to 0 over 120s
- "Maintained" = freshness ≥ 0.45 — drones must periodically revisit
- Two-phase reward: Phase 1 (explore) when ever_seen < 98%, Phase 2 (maintain) after

### Auxiliary Losses
- **Frontier potential**: critic predicts how much unexplored area is near each drone
- **Frontier direction**: critic predicts 2D unit vector toward each drone's frontier — helps representation learning

---

## Key Design Choices
1. **CTDE** (Centralized Training, Decentralized Execution): actor uses only local obs; critic sees everything
2. **Voronoi frontier waypoints**: each drone's frontier target is the centroid of its nearest unexplored cells — natural spatial partitioning
3. **Trajectory heatmap communication**: drones share where they've been (decaying heat) to avoid redundancy
4. **Attention-based neighbor encoding**: handles variable active drone count gracefully
5. **Wall sliding physics**: drones slide along walls instead of freezing, maintaining momentum
6. **Escalating first-visit reward**: finding the last 10% of cells is worth 4× more, preventing 90%-plateau stagnation

