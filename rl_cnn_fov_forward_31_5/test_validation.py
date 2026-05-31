import torch
import numpy as np
from env_swarm_polygon import SwarmSearchPolygonEnv
from train import make_env_cfg
from validation import CoverageValidator

cfg = make_env_cfg()
env = SwarmSearchPolygonEnv(cfg)
seed = 205
obs, info = env.reset(seed=seed)

validator = CoverageValidator(
    grid_shape=(env.grid_h, env.grid_w),
    n_drones=cfg.max_agents,
    grid_resolution=cfg.grid_resolution_m,
    drone_safety_radius=cfg.drone_safety_radius,
    wall_safety_margin=cfg.wall_safety_margin,
    decay_rate=cfg.trajectory_heatmap_decay,
    polygon_vertices=env.polygon_vertices,
    world_w=cfg.world_w,
    world_h=cfg.world_h,
    max_steps=env.max_steps,
)
validator._active_mask = env.active_area_mask.copy()

print(f"Start active cells: {validator.active_cells}")

for step in range(100):
    actions = np.random.randn(cfg.max_agents, 2)
    obs, reward, terminated, truncated, info = env.step(actions)
    done = terminated or truncated
    
    grid_positions = []
    for i in range(cfg.max_agents):
        gx = int((env.pos[i, 0] + cfg.world_w / 2) / cfg.grid_resolution_m)
        gy = int((env.pos[i, 1] + cfg.world_h / 2) / cfg.grid_resolution_m)
        gx = np.clip(gx, 0, env.grid_w - 1)
        gy = np.clip(gy, 0, env.grid_h - 1)
        grid_positions.append((gy, gx))
        
    validator.record_step(
        drone_positions=grid_positions,
        drone_world_positions=env.pos[:cfg.max_agents].tolist(),
        drone_headings=env.heading[:cfg.max_agents].tolist(),
    )
    validator.sync_coverage_from_fov(env.ever_visited, grid_positions)
    
    if done:
        break

env_coverage = float(np.mean(env.ever_visited[env.active_area_mask])) * 100
val_coverage = validator.get_metrics()['coverage_pct']
print(f"Env coverage: {env_coverage:.4f}%")
print(f"Val coverage: {val_coverage:.4f}%")
