# Autonomous Vehicle Path Planning and Control Simulation

This project provides a simulation environment for various path planning and control algorithms for autonomous vehicles. The simulation is built using MATLAB and Simulink.

## Project Goal

The main goal of this project is to implement, test, and visualize different path planning and control strategies for autonomous vehicles in a 2D environment with obstacles.

## Directory Structure

- `path_planning/`: Contains MATLAB implementations of various path planning algorithms.
- `controller/`: Contains MATLAB implementations of path tracking controllers.
- `maps/`: Contains map files used in the simulations.
- `simulink/`: Contains Simulink models for vehicle dynamics and control systems.
- `team/`: Contains team-related files and archives.
- `default_files/`: Contains default simulation files.
- `initial_sim_data/`: Contains initial data for simulations.
- `playground/`: A directory for temporary files and testing.

## Path Planning Algorithms

The following path planning algorithms are implemented in the `path_planning` directory:

- **A* Search**: A grid-based search algorithm that finds the shortest path between a start and a goal.
- **D* Lite**: A replanning algorithm that can efficiently update the path in dynamic environments.
- **Hybrid A***: A path planning algorithm that considers the vehicle's kinematic constraints.
- **RRT (Rapidly-exploring Random Tree)**: A sampling-based algorithm that is efficient for high-dimensional search spaces.

For more details, please refer to the `README.md` file in the `path_planning` directory.

## Path Tracking Controllers

The following path tracking controllers are implemented in the `controller` directory:

- **Pure Pursuit**: A geometric path tracking controller that calculates the steering angle to follow a given path.
- **PID Controller**: A Proportional-Integral-Derivative controller is used for longitudinal speed control.

## How to Run

1.  **Open MATLAB**: Make sure you have MATLAB and Simulink installed.
2.  **Navigate to the Project Directory**: Open the project's root directory in MATLAB.
3.  **Run a Simulation**: You can run a simulation by executing one of the main scripts in the `controller` or `path_planning` directories.

## Dependencies

- **MATLAB**: The project is developed and tested on MATLAB R2022b.
- **Simulink**: Simulink is required to run the vehicle dynamics models.
