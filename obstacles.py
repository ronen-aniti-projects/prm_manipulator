import numpy as np


class Obstacles:
    def __init__(self):
        self.obstacles = np.array(
            [
                [ 0.4, -0.5,  0.1,  0.7, -0.2,  0.5],
                [-0.6,  0.1,  0.2, -0.3,  0.4,  0.6],
                [ 0.2,  0.3,  0.0,  0.5,  0.6,  0.4],
                [-0.5, -0.5,  0.1, -0.2, -0.2,  0.5],
                #[-1.0, -2.0, -1,  1.0, 2.0,  -0.1],
            ],
            dtype=float,
        )