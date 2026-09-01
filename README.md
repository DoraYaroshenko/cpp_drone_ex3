# Assignment 3 - Drone Mapper

This project is a C++ 3D drone mapping simulation for Assignment 3. The simulation operates by running drone missions in parallel, with drones navigating a 3D voxel grid to map the environment while avoiding collisions.

## Submitter Details
- **Dora Yaroshenko**, 330371063
- **Lilac Gil-Ad**, 324976703

## Project Components
The project is split into three independent components, each with its own `CMakeLists.txt` and built individually or together from the root:
1. **Simulator**: Runs multiple drone missions in parallel. Output executable: `simulator_330371063_324976703`
2. **Algorithm**: The drone's mapping logic. Built as a shared library: `Algorithm_330371063_324976703.so`
3. **Mission Control**: Runs a single drone mission. Built as a shared library: `MissionControl_330371063_324976703.so`

## Namespaces
The implementation uses the following namespaces:
- `common`: Interfaces and types shared across all components (given by the course).
- `algorithm_330371063_324976703`: Code related to the algorithms.
- `mission_control_330371063_324976703`: Code related to mission control.
- `simulator_330371063_324976703`: Code related to the simulator and mocked drivers.
- `user_common_330371063_324976703`: Custom utilities, error handling, logging, and common features implemented for this project (found in the `UserCommon` folder).

## File Tree (Overview)

```text
.
├── Algorithm/                 # Implementation of mapping algorithms (e.g. Const, Cycling, Optimized)
│   ├── CMakeLists.txt
│   ├── include/Algorithm/
│   └── src/
├── MissionControl/            # Implementation of the drone mission control logic
│   ├── CMakeLists.txt
│   ├── common_mission_control/
│   ├── include/MissionControl/
│   └── src/
├── Simulator/                 # Simulation engine, multi-threading, and mocked hardware
│   ├── CMakeLists.txt
│   ├── common_simulator/
│   ├── include/Simulator/
│   └── src/
├── common/                    # Shared interfaces across the core skeleton
│   ├── CMakeLists.txt
│   └── include/Common/
├── UserCommon/                # Student-defined common utilities (logging, collisions, voxels)
│   ├── include/UserCommon/
│   └── src/
├── inputs/                    # YAML configurations and input maps (.npy, .cw)
│   ├── drone/
│   ├── lidar/
│   ├── map/
│   ├── mission/
│   └── simulation/
├── CMakeLists.txt             # Root CMake config
├── CMakePresets.json
├── README.md
├── students.txt               # Student details
├── vcpkg-configuration.json   # vcpkg dependencies
└── vcpkg.json
```

## Running the Simulator
The simulator supports two modes, which dynamically load the `MissionControl` and `Algorithm` `.so` plugins at runtime:

**Comparative Run**
```bash
./simulator_330371063_324976703 -comparative simulation=<sim.yaml> mission_control_folder=<folder> algorithm=<algo.so> [num_threads=<num>] [-verbose]
```

**Competition Run**
```bash
./simulator_330371063_324976703 -competition simulation=<sim.yaml> mission_control=<mc.so> algorithms_folder=<folder> [num_threads=<num>] [-verbose]
```

Output `.npy` maps and result YAML files will be generated in a dynamically created results folder.
