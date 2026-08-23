add_executable(simulator_330371063_324976703 
    src/main.cpp
    src/MockLidar.cpp
    src/MockGPS.cpp
    src/MockMovement.cpp
    src/Map3DImpl.cpp
    src/SimulationManager.cpp
    src/SimulationRunImpl.cpp
    src/SimulationRunFactoryImpl.cpp

    src/MapsComparison.cpp
    ../UserCommon/src/CollisionUtils.cpp
    ../UserCommon/src/ScanResultToVoxels.cpp
    ../UserCommon/src/Logger.cpp
)

target_include_directories(simulator_330371063_324976703 PRIVATE
    include
    common_simulator/include
    ../UserCommon/include
    ../MissionControl/common_mission_control/include
)

find_package(TinyNPY CONFIG REQUIRED)
target_link_libraries(simulator_330371063_324976703 common::common ${CMAKE_DL_LIBS} TinyNPY::TinyNPY)

# Export symbols for dynamic plugins
set_target_properties(simulator_330371063_324976703 PROPERTIES ENABLE_EXPORTS TRUE)
