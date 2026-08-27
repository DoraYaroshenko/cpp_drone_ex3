add_library(Algorithm_330371063_324976703 SHARED 
    src/MappingAlgorithmImpl.cpp
    ../UserCommon/src/CollisionUtils.cpp
    ../UserCommon/src/ScanResultToVoxels.cpp
    ../UserCommon/src/Logger.cpp
)
target_include_directories(Algorithm_330371063_324976703 PRIVATE include ../UserCommon/include)
target_link_libraries(Algorithm_330371063_324976703 common::common)
set_target_properties(Algorithm_330371063_324976703 PROPERTIES PREFIX "")

add_library(Algorithm_Simple_330371063_324976703 SHARED 
    src/SimpleSweepAlgorithmImpl.cpp
    ../UserCommon/src/CollisionUtils.cpp
    ../UserCommon/src/ScanResultToVoxels.cpp
    ../UserCommon/src/Logger.cpp
)
target_include_directories(Algorithm_Simple_330371063_324976703 PRIVATE include src ../UserCommon/include)
target_link_libraries(Algorithm_Simple_330371063_324976703 common::common)
set_target_properties(Algorithm_Simple_330371063_324976703 PROPERTIES PREFIX "")

add_library(Algorithm_Recitation_330371063_324976703 SHARED 
    src/ConstAlgorithmImpl.cpp
    src/CyclingAlgorithmImpl.cpp
    src/SlowRandomAlgorithmImpl.cpp
    ../UserCommon/src/CollisionUtils.cpp
    ../UserCommon/src/ScanResultToVoxels.cpp
    ../UserCommon/src/Logger.cpp
)
target_include_directories(Algorithm_Recitation_330371063_324976703 PRIVATE include src ../UserCommon/include)
target_link_libraries(Algorithm_Recitation_330371063_324976703 common::common)
set_target_properties(Algorithm_Recitation_330371063_324976703 PROPERTIES PREFIX "")
