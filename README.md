## File Tree (Overview)

```text
.
├── Algorithm/                 # Implementation of mapping algorithm
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

The `-verbose` flag will cause the Mission Control to output drone movement log files named `drone_logs.jsonl` under each individual run directory next to the output map. These can be used with our visualizations (submitted in previous assignments).
