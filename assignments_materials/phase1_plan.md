# Phase 1: Pre-Split Scaffolding Implementation Plan

This plan details the exact steps to implement Phase 1 from `global_plan.md`. It covers scaffolding the project by porting the `cpp_drone_ex2` code into the main folder while respecting the provided common interfaces and assignment constraints.

## Clarifications & Adjustments

Based on your review, I have adjusted the plan and confirmed the following:
1. **Types & Units**: We will use the skeleton's versions entirely. `Types.h` and `Units.h` from the skeleton are sufficient (and `Units.h` even adds helpful operators). Furthermore, the skeleton already provides `SimulationTypes.h` in `Simulator/common_simulator/include/Simulator/SimulationTypes.h`, so we do NOT need to port your `SimulationTypes.h` to `UserCommon`.
2. **Simulator `main.cpp`**: You are absolutely right. The Ex3 main is completely different (handling `-comparative` and `-competition` args). The old `drone_mapper_simulation_main.cpp` and `maps_comparison_main.cpp` are obsolete. For Phase 1 (whose only goal is basic scaffolding and compilation), we will simply create a brand new, empty `main.cpp` (just returning 0) so the Simulator compiles. We will build the actual argument parsing and logic in Phase 2.
3. **`MapsComparison`**: While the standalone executable is gone, the `MapsComparison` class is still used internally by `SimulationRunImpl` to compute the `mission_score`. Thus, we still need to port the `MapsComparison.h/.cpp` files to the Simulator.
4. **Simulator Namespace**: The Simulator is the host executable, not a plugin. It does not require a custom namespace like the Algorithm or MissionControl.
5. **Compilation with REGISTRATION**: Yes, it will compile perfectly. On Linux (where `.so` shared libraries are built), unresolved symbols (like the `MappingAlgorithmRegistration` constructor) are permitted at compile time. They are resolved at runtime when `dlopen` is called. So Phase 1 will successfully compile. Implementing the Registrar and exporting the symbols from the Simulator is a runtime requirement that we will handle in Phase 2.

## Proposed Changes

### 1. UserCommon Component
We will create a new directory for shared utilities that are specific to your implementation, wrapping them in your namespace.

#### [NEW] `UserCommon/include/UserCommon/CollisionUtils.h`
#### [NEW] `UserCommon/src/CollisionUtils.cpp`
#### [NEW] `UserCommon/include/UserCommon/ScanResultToVoxels.h`
#### [NEW] `UserCommon/src/ScanResultToVoxels.cpp`
#### [NEW] `UserCommon/include/UserCommon/ConfigParserUtils.h`
#### [NEW] `UserCommon/src/ConfigParserUtils.cpp` (if exists, or header only)
#### [NEW] `UserCommon/include/UserCommon/Logger.h`
#### [NEW] `UserCommon/src/Logger.cpp`
- **Action**: Copy these from `ex2_submission` and wrap all declarations and definitions in `namespace user_common_330371063_324976703 { ... }`.

---

### 2. Algorithm Component
Move and adapt the drone algorithm logic to be compiled as a dynamically loaded shared library.

#### [NEW] `Algorithm/include/MappingAlgorithmImpl.h`
#### [NEW] `Algorithm/src/MappingAlgorithmImpl.cpp`
- **Action**: 
  - Copy from `ex2_submission`
  - Wrap in `namespace algorithm_330371063_324976703 { ... }`.
  - Rename the class to `MappingAlgorithmImpl_330371063_324976703`.
  - Add `REGISTER_MAPPING_ALGORITHM(MappingAlgorithmImpl_330371063_324976703)` in the global scope of the `.cpp` file.
#### [MODIFY] `Algorithm/CMakeLists.txt`
- **Action**: Update to include `../UserCommon/include`, compile the needed sources from `../UserCommon/src`, and build as a `SHARED` library named `Algorithm_330371063_324976703`.

---

### 3. MissionControl Component
Move and adapt the mission control logic to be compiled as a dynamically loaded shared library.

#### [NEW] `MissionControl/include/MissionControlImpl.h`
#### [NEW] `MissionControl/src/MissionControlImpl.cpp`
#### [NEW] `MissionControl/include/DroneControlImpl.h`
#### [NEW] `MissionControl/src/DroneControlImpl.cpp`
- **Action**: 
  - Copy from `ex2_submission`.
  - Wrap in `namespace mission_control_330371063_324976703 { ... }`.
  - Rename the class to `MissionControlImpl_330371063_324976703`.
  - Add `REGISTER_MISSION_CONTROL(MissionControlImpl_330371063_324976703)` in the global scope of `MissionControlImpl.cpp`.
#### [MODIFY] `MissionControl/CMakeLists.txt`
- **Action**: Update to include `../UserCommon/include`, compile needed sources from `../UserCommon/src`, and build as a `SHARED` library named `MissionControl_330371063_324976703`.

---

### 4. Simulator Component
Move the remaining simulation infrastructure, which acts as the host executable.

#### [NEW] `Simulator/include/MockLidar.h`
#### [NEW] `Simulator/src/MockLidar.cpp`
#### [NEW] `Simulator/include/MockGPS.h`
#### [NEW] `Simulator/src/MockGPS.cpp`
#### [NEW] `Simulator/include/MockMovement.h`
#### [NEW] `Simulator/src/MockMovement.cpp`
#### [NEW] `Simulator/include/Map3DImpl.h`
#### [NEW] `Simulator/src/Map3DImpl.cpp`
#### [NEW] `Simulator/include/SimulationManager.h`
#### [NEW] `Simulator/src/SimulationManager.cpp`
#### [NEW] `Simulator/include/SimulationRunImpl.h`
#### [NEW] `Simulator/src/SimulationRunImpl.cpp`
#### [NEW] `Simulator/include/SimulationRunFactoryImpl.h`
#### [NEW] `Simulator/src/SimulationRunFactoryImpl.cpp`
#### [NEW] `Simulator/include/MapsComparison.h`
#### [NEW] `Simulator/src/MapsComparison.cpp`
- **Action**: Copy these from `ex2_submission` into their respective `include` and `src` folders. Update their `#include` paths to correctly reference `UserCommon` or skeleton headers.

#### [NEW] `Simulator/src/main.cpp`
- **Action**: Create a new, minimal scaffold (e.g., `int main() { return 0; }`) just to satisfy Phase 1's compilation requirement. The actual parsing logic is scheduled for Phase 2.

#### [MODIFY] `Simulator/CMakeLists.txt`
- **Action**: Update to include `../UserCommon/include`, compile the needed sources, and build the executable named `simulator_330371063_324976703`.

## Verification Plan

### Automated Tests
- Run `cmake -B build` and `cmake --build build` from the root directory to ensure that all three components (Simulator, Algorithm, and MissionControl) compile correctly as per the new structure and namespace changes.

### Manual Verification
- Manually inspect the file tree to confirm the presence of `UserCommon` and the successful separation of components.
- Inspect the generated `.so` files for correct naming conventions.
