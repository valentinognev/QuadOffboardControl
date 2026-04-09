# train_ppo.py
import os
import time
import numpy as np

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor

from env_swarm_coverage import SwarmCoverageEnv, SwarmConfig


def make_env(circle_radius: float):
    def _thunk():
        cfg = SwarmConfig(circle_radius=circle_radius)
        return SwarmCoverageEnv(cfg=cfg, render_mode=None)
    return _thunk


if __name__ == "__main__":
    os.makedirs("models", exist_ok=True)
    os.makedirs("logs", exist_ok=True)

    circle_radius = 4.0
    env = DummyVecEnv([make_env(circle_radius)])
    env = VecMonitor(env, filename="logs/monitor.csv")

    # PPO is a strong default for continuous control and is commonly used in swarm RL research.
    model = PPO(
        policy="MlpPolicy",
        env=env,
        device="cpu",
        verbose=1,
        n_steps=2048,
        batch_size=256,
        gamma=0.99,
        gae_lambda=0.95,
        learning_rate=3e-4,
        ent_coef=0.0,
        clip_range=0.2,
        tensorboard_log="logs/tensorboard",
    )

    start = time.time()
    model.learn(total_timesteps=1_000_000)  # start with 1e6, scale up
    dur = time.time() - start

    out_path = f"models/ppo_swarm_circle_r{circle_radius:.1f}.zip"
    model.save(out_path)
    print(f"Saved model to: {out_path}")
    print(f"Training time: {dur/60:.1f} minutes")
    print(model.policy)