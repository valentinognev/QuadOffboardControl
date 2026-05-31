import os

import matplotlib.pyplot as plt
import pandas as pd


def maybe_plot(csv_path: str, x: str, y: str, title: str):
    if not os.path.exists(csv_path):
        print(f'Missing: {csv_path}')
        return
    df = pd.read_csv(csv_path)
    if x not in df.columns or y not in df.columns:
        print(f'Missing columns {x}, {y} in {csv_path}')
        return
    plt.figure()
    plt.plot(df[x], df[y])
    plt.xlabel(x)
    plt.ylabel(y)
    plt.title(title)
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    train = 'outputs/logs/train_metrics.csv'
    loss = 'outputs/logs/ppo_losses.csv'

    maybe_plot(train, 'env_steps', 'percent_covered_at_least_once', 'Percent Covered At Least Once (%)')
    maybe_plot(train, 'env_steps', 'ever_seen_fraction', 'Ever-seen fraction over training')
    maybe_plot(train, 'env_steps', 'maintained_fraction', 'Maintained fraction over training')
    maybe_plot(train, 'env_steps', 'coverage_mean', 'Coverage mean over training')
    maybe_plot(train, 'env_steps', 'scan_efficiency_score', 'Scan efficiency score over training')
    maybe_plot(train, 'env_steps', 'overlap_ratio_mean', 'Overlap ratio over training')
    maybe_plot(train, 'env_steps', 'collision_penalty_mean', 'Collision penalty over training')
    maybe_plot(train, 'env_steps', 'min_inter_drone_dist', 'Min inter-drone distance over training')
    maybe_plot(loss, 'env_steps', 'total_loss', 'PPO total loss')
    maybe_plot(loss, 'env_steps', 'frontier_aux_loss', 'Frontier auxiliary loss')
