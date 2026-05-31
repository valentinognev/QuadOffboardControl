from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from polygon_utils import generate_concave_polygon, polygon_area

OUT_DIR = Path(__file__).resolve().parent / 'outputs' / 'maps'
OUT_DIR.mkdir(parents=True, exist_ok=True)


def main() -> None:
    rng = np.random.default_rng(123)
    for i in range(9):
        vertices = generate_concave_polygon(num_vertices=12, target_area=900.0, rng=rng)
        closed = np.vstack([vertices, vertices[0:1]])
        plt.figure(figsize=(5, 5))
        plt.plot(closed[:, 0], closed[:, 1], 'k-')
        plt.fill(closed[:, 0], closed[:, 1], alpha=0.25)
        plt.gca().set_aspect('equal')
        plt.grid(True, alpha=0.3)
        plt.title(f'Map {i} | area={polygon_area(vertices):.1f} m²')
        plt.tight_layout()
        plt.savefig(OUT_DIR / f'polygon_map_{i}.png', dpi=160)
        plt.close()
    print(f'Saved sample maps in {OUT_DIR}')


if __name__ == '__main__':
    main()
