import numpy as np 

class Obstacles:
    def __init__(self):
        self.obstacles = np.array([
            [1, 1, 1, 2, 2, 2], 
            [1, 1, 1, 2, 2, 2], 
            [1, 1, 1, 2, 2, 2],
            [1, 1, 1, 2, 2, 2],
            [1, 1, 1, 2, 2, 2],
            [1, 1, 1, 2, 2, 2]],
            dtype=float
        ) * 10 # Modify with actual bb coords