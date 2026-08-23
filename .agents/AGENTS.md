# Project-Specific Rules for cpp_drone_ex3 (Assignment 3)

When working on this project, adhere to the following rules:

## General Ex2 Knowledge (Context)
- **Project Context:** This is a C++ project simulating a drone mapping a 3D voxel grid.
- **Units:** The project uses the `mp-units` library for strongly-typed physical units (e.g., `.numerical_value_in(cm)`).
- **Map Output:** Sparse 3D matrix in `.npy` format. Values: 0 (Empty), 1 (Occupied), -3 (Potentially occupied), -1 (Unmapped), -2 (Out of bounds). Output map resolution equals input map resolution.
- **Simulation Flow:** Includes `SimulationManager`, `SimulationRun`, `MissionControl`, `DroneControl`, `MappingAlgorithm`, `MockLidar`, `MapsComparison`.
- **Error Handling:** Do not use `exit()`. Errors result in a scenario score of `-1` and the simulation must continue. All errors must be logged.
- **Drone State:** 3D space position `{X, Y, Height}` and `XY-Angle` (0=east, 90=south, 180=west, 270=north).
- **Movement Commands:** `Rotate <Left/Right> by <angle>`, `Advance <distance amount>`, and `Elevate <distance amount>`.
- **Sensors:** Position Sensor and Lidar Sensor.

## Assignment 3 Requirements

### Project Structure (3 Separate Projects)
The assignment is separated into three independent parts, each built with its own makefile:
1. **Simulator:** Runs many drone missions in parallel. Executable: `simulator_<submitter_ids>`.
2. **Algorithm:** Represents drone algorithm. Compiled as a shared library: `Algorithm_<submitter_ids>.so`. Code must be in namespace `algorithm_<submitter_ids>`.
3. **MissionControl:** Runs a single drone mission. Compiled as a shared library: `MissionControl_<submitter_ids>.so`. Code must be in namespace `mission_control_<submitter_ids>`.

### Common Folders
- **`common`:** Includes common files published by course staff, used "as is" with no changes.
- **`UserCommon`:** Includes user's own common files, inside namespace `user_common_<submitter_ids>`.

### Simulator Modes
1. **Comparative run:** `simulator_<submitter_ids> -comparative simulation=<sim.yaml> mission_control_folder=<folder> algorithm=<algo.so> [num_threads=<num>] [-verbose]`
2. **Competition run:** `simulator_<submitter_ids> -competition simulation=<sim.yaml> mission_control=<mc.so> algorithms_folder=<folder> [num_threads=<num>] [-verbose]`

### Output & Threading
- Results go to `comparative_results_<time>` or `competition_<time>` under the given folders.
- Output includes `.npy` map files, error logs, and a unified `Simulation Result Output File` YAML (e.g., `Comparative Simulation Result Output File - YAML`).
- **Threading:** If `num_threads` >= 2, the simulator must use multithreading to run missions concurrently. 

### API and Registration
- Algorithms and MissionControls register themselves using macros: `REGISTER_MAPPING_ALGORITHM(<class_name>)` and `REGISTER_MISSION_CONTROL(<class_name>)`.
- The `.so` files must be loaded dynamically and unloaded (`dlclose`) when no longer in use, avoiding unloading if objects are still alive.
- Factories are used to create instances (do not cache instances, recreate them).

### Common Issues Handling (Mandatory)
- A valid algorithm should avoid errors it can detect. The valid algorithm will not create the error.
- If a faulty algorithm returns a movement resulting in a wall collision, the Mock Movement driver must throw an exception (Target: Drone Controller, SimulationRun).
- Optional bonus handling includes gracefully trying again, ignoring illegal movements, or splitting commands.

### Structuring Guidelines
- Mocks (e.g., `MockLidar`, `MockGPS`, `MockMovement`) are implemented in the Simulator's `src` folder, not `Common`. They simulate real APIs.
- The `Common` folder holds interfaces like `IDroneControl` and the mapping algorithm interfaces, since they are shared by the Simulator and the dynamically loaded components.
