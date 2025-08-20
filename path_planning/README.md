# Path Planning Algorithms

This directory contains implementations of various path planning algorithms in MATLAB.

## Algorithms

- **A* Search**:
  - `Astar_original.m`: A basic implementation of the A* algorithm.
  - `Astar_extend_map.m`: An A* variant that expands obstacles to create a safety margin around them.
  - `Astar_with_smooting.m`: An A* variant that includes a cost for turning, resulting in smoother paths.

- **D* Lite**:
  - `Dstar_original.m`: An implementation of the D* Lite algorithm, which is suitable for dynamic environments where obstacles can appear or change.

- **Hybrid A***:
  - `HybridAstar.m`: An implementation of the Hybrid A* algorithm, which considers vehicle kinematics to generate feasible paths for car-like robots.

- **RRT (Rapidly-exploring Random Tree)**:
  - `RRT_test.m`: An implementation of the RRT algorithm, a randomized algorithm that is efficient in high-dimensional spaces.

## How to Run

Each algorithm is implemented in its own `.m` file. To run a specific algorithm, open the corresponding file in MATLAB and run the `main` function. For example, to run the basic A* algorithm:

```matlab
Astar_original
```

## Map Format

The algorithms in this directory use a `.csv` file to represent the map. The map is a 2D grid where `0` represents free space and `1` represents an obstacle. The default map is `map_demo_1.csv`. You can use a different map by changing the `map_csv` variable in the `main` function of each script.
