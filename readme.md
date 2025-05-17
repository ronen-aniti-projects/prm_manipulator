# PRM Path Planner Demonstration for UR10e Manipulator

## Summary 
This repository implements a multi‑query Probabilistic Roadmap (PRM) path planner and A* query for a UR10e 6‑DOF manipulator. Running the demo will build a collision‑free roadmap in joint‑space, search for a feasible path between a hard‑coded start and goal, and visualize both the end‑effector roadmap and the full robot link configurations against workspace obstacles using Matplotlib.

## Dependencies 
Python, NumPy, SciPy, Matplotlib 

```
pip install numpy scipy matplotlib 
```

## How to Run 

First, clone the repository. 

```
git clone https://github.com/ronen-aniti-projects/prm_manipulator.git
```

Second, from the project root, execute the following:
```
python main.py
```

Third, examine the generated plots.
