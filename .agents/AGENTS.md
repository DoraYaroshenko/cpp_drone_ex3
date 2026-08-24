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

### Makefiles and Build
- **4 Makefiles/CMakeLists.txt total:** 1 inside each of the 3 project folders, plus 1 at the root (for building all 3 projects together).

### Common Folders
- **DO NOT CHANGE FILES IN ALL COMMON FOLDERS:** This includes `ex_3_skeleton\common`, `ex_3_skeleton\Simulator\common_simulator`, `ex_3_skeleton\MissionControl\common_mission_control`, and any other files published by course staff that are given "as is" and should not be changed.
- **`UserCommon`:** Includes user's own common files, inside namespace `user_common_<submitter_ids>`.

### Simulator Modes
1. **Comparative run:** `simulator_<submitter_ids> -comparative simulation=<sim.yaml> mission_control_folder=<folder> algorithm=<algo.so> [num_threads=<num>] [-verbose]`
2. **Competition run:** `simulator_<submitter_ids> -competition simulation=<sim.yaml> mission_control=<mc.so> algorithms_folder=<folder> [num_threads=<num>] [-verbose]`

### Command Line Arguments
- All args are mandatory except `num_threads`.
- Can appear in any order.
- The `=` sign should appear without spaces around it.
- **Errors:** If unsupported args are provided, print usage pointing out all unsupported args, then finish. If missing args, print usage detailing missing args. If a file/folder doesn't exist or a folder has zero relevant files, print a proper error and finish.

### Output
- Results go to `comparative_results_<time>` or `competition_<time>` directly under the given mission control or algorithms folders.
- Output includes `.npy` map files, error logs, and a unified `Simulation Result Output File` YAML (e.g., `Comparative Simulation Result Output File - YAML` or `Competitive Simulation Result Output File - YAML`).
- Each individual mission run will also generate its own Simulation Result YAML file in the same output folder, with the specific mission control or algorithm name added to the filename.

### Threading Model
- **Thread Count:** The total number of threads will **never be 2**. 
  - If `num_threads` is missing or `num_threads=1`, use a single thread (the main thread).
  - If `num_threads >= 2`, interpret this as the requested number of threads for running the actual simulation **in addition** to the main thread.
- Main thread can wait for all other threads in a join call (or any similar blocking wait).
- Do not open threads that cannot be utilized.
- **Locking:** It is better not to lock if you can avoid locking. Only use locks (like `std::mutex`) when absolutely necessary. For example, if a result table's size is known in advance, create it ahead of time to avoid needing a synchronized sparse matrix or dynamic resizing during concurrent execution. Always use `jthread` instead of `thread`, and lock_guard` when using locks.

### API and Registration
- Algorithms and MissionControls register themselves using macros: `REGISTER_MAPPING_ALGORITHM(<class_name>)` and `REGISTER_MISSION_CONTROL(<class_name>)`.
- The `.so` files must be loaded dynamically and unloaded (`dlclose`) before the program ends. Make sure not to call `dlclose` if there are objects related to the `.so` which are still alive.
- Factories are used to create instances. **Do not cache instances**, recreate them using the factories when needed.
- Example of registration algorithm in `assignments_materials/register_example.cpp`.

### Common Issues Handling (Mandatory)
- A valid algorithm should avoid errors it can detect. The valid algorithm will not create the error.
- If a faulty algorithm returns a movement resulting in a wall collision, the Mock Movement driver must throw an exception (Target: Drone Controller, SimulationRun).

### Structuring Guidelines
- Mocks (e.g., `MockLidar`, `MockGPS`, `MockMovement`) are implemented in the Simulator's `src` folder, not `Common`. They simulate real APIs.
- The `Common` folder holds interfaces like `IDroneControl` and the mapping algorithm interfaces, since they are shared by the Simulator and the dynamically loaded components.

### Additional Rules & Requirements
- **Memory Management:** You are not allowed to use `new` and `delete` in your code.
- **Smart Pointers:** Prefer using `std::unique_ptr` over `std::shared_ptr`. Use `std::shared_ptr` only if there is an actual need for sharing and the lifetime is unknown.
- **Algorithm Minimal Requirements:** 
  1. Do not fly the drone into walls.
  2. Try to map all the relevant surroundings in the configured boundaries.
  3. Try to be efficient and exact.
- **Submission:** A zip file `ex3_<student1_id>_<student2_id>.zip` containing the 5 folders, 4 makefiles, `students.txt`, and `README.md`. No binary files or external libraries (unless standard or explicitly approved).
