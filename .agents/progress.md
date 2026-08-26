# Phase 2 Progress Track

This document tracks the current state of Phase 2 implementation to ensure context is maintained across sessions.

## Current State: Planning Phase Completed
- **Phase 1**: Completed. Scaffolded `UserCommon`, `Algorithm`, `MissionControl`, and `Simulator` namespaces and initial CMake setup.
- **Phase 2 Person A Plan**: Formulated and under review. Focuses on Algorithm safety and MockMovement error handling.
- **Phase 2 Person B Plan**: Pending. Focuses on Simulator dynamic loading, multithreading, and output generation.

## Person A Tasks (Pending Execution)
- `[x]` **Step 1:** Update `MockMovement` constructor to take hidden map and drone radius.
- `[x]` **Step 2:** Implement collision & bounds checking in `MockMovement`'s `advance` and `elevate`, throwing distinct `std::runtime_error`s on failure.
- `[x]` **Step 3:** Wrap movement calls in `DroneControlImpl::step` with `try/catch`, returning `DroneStepStatus::Error` on exceptions.
- `[x]` **Step 4:** Add safety clamps to `MappingAlgorithmImpl::handleFollowPath` to prevent overshooting the known safe path.
- `[ ]` **Step 5 (Optional):** Implement swept-path collision checking in `MockMovement`.

## Notes for Person B (Future)
- When Person A modifies the `MockMovement` constructor in Step 1, Person B will need to ensure they provide the `hidden_map` and `drone.radius` when instantiating `MockMovement` in `SimulationRunFactoryImpl`.
