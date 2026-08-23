# Assignment 3 Implementation & Parallel Work Plan

This plan details how to adapt your `cpp_drone_ex2` code to the new repository structure, adhering to all the assignment requirements (shared libraries, multithreading, specific folder structure). It is optimized for two people working independently on parallel branches.

> [!NOTE]
> The student IDs `330371063` (Dora) and `324976703` (Lilac) have been integrated into this plan for namespaces and `.so` filenames.

> [!WARNING]
> You must strictly avoid using `new` and `delete` (use `std::make_unique` instead). Ensure all resources are properly released before `dlclose` is called on the shared libraries.

## Proposed Strategy

The work is split into three phases:
1. **Pre-Split (Together / 1 Person):** Scaffolding, copying files, and wrapping in namespaces. This should be done on the `main` branch before branching to avoid merge conflicts on file structure.
2. **Parallel Work (Person A & Person B):** Two independent branches working on separate components.
3. **Merge & Integration (Together):** Merging the parallel work and testing the fully integrated system.

---

### Phase 1: Pre-Split Scaffolding (Together)
*Goal: Port Ex2 code to Ex3 skeleton, setup namespaces, and ensure basic compilation before splitting work.*

#### 1. Setup `UserCommon` Folder
- [NEW] Create a new folder at the root: `UserCommon`.
- [NEW] Create `UserCommon/include/UserCommon` and `UserCommon/src`.
- Move all files from `ex2_submission` that are shared across Simulator, Algorithm, and MissionControl into `UserCommon`. This likely includes: `CollisionUtils`, `ScanResultToVoxels`, `ConfigParserUtils`, `Units`, `Types`, and `Logger`.
- **Namespace:** Wrap all code in `UserCommon` with `namespace user_common_330371063_324976703 { ... }`.
- *Note: As per instructions, `UserCommon` will not have its own CMakeLists.txt. Instead, the `Simulator`, `Algorithm`, and `MissionControl` Makefiles will include `UserCommon/include` and compile the needed source files directly.*

#### 2. Populate Component Folders
- **Algorithm:** Move `MappingAlgorithmImpl` to `Algorithm/src` and `Algorithm/include`. Wrap in `namespace algorithm_330371063_324976703`. Rename the class to `MappingAlgorithmImpl_330371063_324976703`. Add the macro `REGISTER_MAPPING_ALGORITHM(MappingAlgorithmImpl_330371063_324976703)` to the `.cpp` file in the global scope.
- **MissionControl:** Move `MissionControlImpl` and `DroneControlImpl` to `MissionControl/src` and `MissionControl/include`. Wrap in `namespace mission_control_330371063_324976703`. Rename the class to `MissionControlImpl_330371063_324976703`. Add `REGISTER_MISSION_CONTROL(MissionControlImpl_330371063_324976703)` to the `.cpp` file in the global scope.
- **Simulator:** Move the remaining Ex2 files (`MockLidar`, `MockGPS`, `MockMovement`, `Map3DImpl`, `SimulationManager`, `SimulationRunImpl`, `SimulationRunFactoryImpl`, `MapsComparison`) into `Simulator/src` and `Simulator/include`. Create a brand new, minimal `main.cpp` (just returning 0) for Phase 1 compilation. *(Mocks belong here because they simulate real APIs injected by the Simulator, as outlined in `Structuring the project.pdf` and `AGENTS.md`)*.

#### 3. Root and Component CMake Updates
- Update the component `CMakeLists.txt` (Algorithm, MissionControl, Simulator) to add `../UserCommon/include` to their include paths and compile the necessary `.cpp` files from `../UserCommon/src`.
- Make sure `Algorithm` and `MissionControl` are compiled as `SHARED` libraries (`.so`), with the specific name formatting: `Algorithm_330371063_324976703.so` and `MissionControl_330371063_324976703.so`.
- Ensure everything compiles sequentially before branching.

---

### Phase 2: Parallel Work (Independent Branches)

#### Person A: Shared Libraries & Mandatory Error Handling
*Goal: Ensure the Algorithm and MissionControl plugins are robust, independent, and handle the mandatory edge cases without crashing.*

1. **Algorithm Polish:**
   - Ensure the algorithm meets the minimal requirements: avoids walls, maps boundaries efficiently.
   - **Mandatory Error Avoidance:** The valid algorithm must avoid errors that it can detect. It must not generate movements that take the drone out of bounds or into walls.

2. **Mandatory Error Handling Implementation:**
   - If a faulty algorithm returns a movement resulting in a wall collision, the `MockMovement` driver (which holds the real map and detects this) must throw an exception.
   - This exception must be caught by the `DroneControl` / `SimulationRun` to prevent the Simulator from crashing. The run should gracefully record a scenario score of `-1` and the simulation must continue.

#### Person B: Simulator Core - Dynamic Loading, Threading & Output
*Goal: Implement the core Simulator executable to load plugins dynamically, run concurrently, and generate the proper reports.*

1. **Dynamic Loading:**
   - Implement a mechanism (e.g., a Registrar Singleton) using `<dlfcn.h>` (`dlopen`, `dlsym`, `dlclose`) to load `.so` files at runtime. (*`<dlfcn.h>` is a standard POSIX C library header required for dynamic linking on Unix/Linux, perfectly legal to use*).
   - Load the registration symbols to instantiate `IMappingAlgorithm` and `IMissionControl` via their factories.
   - Ensure `.so` files are closed (`dlclose`) strictly *after* all their objects are destroyed.

2. **Command Line Parsing & Output Setup:**
   - Parse all arguments: `-comparative`, `-competition`, `simulation=...`, `mission_control=...`, `algorithm=...`, `num_threads=...`, `-verbose`.
   - **YAML Parsing & Generation:** Extract the YAML parsing and output generation logic from the old ex2 `main.cpp`s into dedicated helper classes/files (e.g., `YamlParserUtils.h/cpp` in the Simulator). Adapt them to parse the new configuration formats and write the required reports.
   - Setup time-stamped output folders: `comparative_results_<time>` or `competition_<time>`.
   - Format the new YAML simulation summary reports exactly as specified in the assignment document.

3. **Multithreading Model:**
   - Parse `num_threads`. If missing or 1, run everything on the main thread.
   - If `>= 2`, launch a thread pool or spawn threads up to the `num_threads` limit to process `SimulationRun` instances concurrently.
   - Main thread should `join()` and wait for all simulation threads to finish before generating the final YAML report.
   - **Thread Safety (No Locks):** As emphasized by course staff, **avoid locks if possible**. Only use `std::mutex` when absolutely necessary. For example, pre-allocate result tables with a known size before running threads so you don't need to lock a sparse matrix or resize it dynamically while threads write to their designated indices.

---

### Phase 3: Integration & Final Polish (Together)
*Goal: Merge the branches and verify end-to-end functionality.*

1. **Merge & Resolve:** Merge Person A and Person B's branches into `main`.
2. **End-to-End Testing:**
   - Run a comparative mode test with 4+ threads. Verify output YAML and map generations are intact.
   - Run a competitive mode test. Verify output YAML.
   - Test deliberate failure scenarios (e.g., throwing mock exceptions) to ensure the `-1` score is recorded and the multithreading Simulator does not crash.
3. **Packaging:**
   - Ensure the directory has exactly 5 folders + 1 root Makefile (or CMake equivalent for root).
   - Add `students.txt` and `README.md`.
   - Package into `ex3_330371063_324976703.zip` strictly excluding binary files.

## Verification Plan

### Automated / Manual Verification
- Compile the code using the root `CMakeLists.txt` to verify all 3 projects build correctly.
- Execute `./simulator_330371063_324976703 -comparative ... num_threads=4` to verify concurrent runs and YAML generation.
- Execute `./simulator_330371063_324976703 -competition ... num_threads=4` to verify competitive mode.
- Inspect the generated `comparative_results_<time>` folder for map `.npy` files and correctly formatted YAML files.
- Inject a deliberate crash in an Algorithm and verify the Simulator catches it, assigns a `-1` score, logs it, and successfully continues other threads.
