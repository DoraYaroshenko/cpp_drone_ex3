#add_library(Algorithm_330371063_324976703 SHARED 
#    src/MappingAlgorithmImpl.cpp
#    ../UserCommon/src/CollisionUtils.cpp
#    ../UserCommon/src/ScanResultToVoxels.cpp
#    ../UserCommon/src/Logger.cpp
#    ../UserCommon/src/TimeUtils.cpp
#    ../UserCommon/src/OrientationUtils.cpp
#)
#target_include_directories(Algorithm_330371063_324976703 PRIVATE include ../UserCommon/include)
#target_link_libraries(Algorithm_330371063_324976703 common::common)
#set_target_properties(Algorithm_330371063_324976703 PROPERTIES PREFIX "")

add_library(Algorithm_Optimized_330371063_324976703 SHARED 
    src/OptimizedAlgorithmImpl.cpp
    ../UserCommon/src/CollisionUtils.cpp
    ../UserCommon/src/ScanResultToVoxels.cpp
    ../UserCommon/src/Logger.cpp
    ../UserCommon/src/TimeUtils.cpp
    ../UserCommon/src/OrientationUtils.cpp
)
target_include_directories(Algorithm_Optimized_330371063_324976703 PRIVATE include src ../UserCommon/include)
target_link_libraries(Algorithm_Optimized_330371063_324976703 common::common)
set_target_properties(Algorithm_Optimized_330371063_324976703 PROPERTIES PREFIX "")

#add_library(Algorithm_Simple_330371063_324976703 SHARED 
#    src/SimpleSweepAlgorithmImpl.cpp
#    ../UserCommon/src/CollisionUtils.cpp
#    ../UserCommon/src/ScanResultToVoxels.cpp
#    ../UserCommon/src/Logger.cpp
#    ../UserCommon/src/TimeUtils.cpp
#    ../UserCommon/src/OrientationUtils.cpp
#)
#target_include_directories(Algorithm_Simple_330371063_324976703 PRIVATE include src ../UserCommon/include)
#target_link_libraries(Algorithm_Simple_330371063_324976703 common::common)
#set_target_properties(Algorithm_Simple_330371063_324976703 PROPERTIES PREFIX "")



#add_library(Algorithm_Cycling_330371063_324976703 SHARED 
#    src/CyclingAlgorithmImpl.cpp
#    ../UserCommon/src/CollisionUtils.cpp
#    ../UserCommon/src/ScanResultToVoxels.cpp
#    ../UserCommon/src/Logger.cpp
#    ../UserCommon/src/TimeUtils.cpp
#    ../UserCommon/src/OrientationUtils.cpp
#)
#target_include_directories(Algorithm_Cycling_330371063_324976703 PRIVATE include src ../UserCommon/include)
#target_link_libraries(Algorithm_Cycling_330371063_324976703 common::common)
#set_target_properties(Algorithm_Cycling_330371063_324976703 PROPERTIES PREFIX "")

add_library(Algorithm_Faulty_330371063_324976703 SHARED 
    src/FaultyAlgorithmImpl.cpp
    ../UserCommon/src/CollisionUtils.cpp
    ../UserCommon/src/ScanResultToVoxels.cpp
    ../UserCommon/src/Logger.cpp
    ../UserCommon/src/TimeUtils.cpp
    ../UserCommon/src/OrientationUtils.cpp
)
target_include_directories(Algorithm_Faulty_330371063_324976703 PRIVATE include src ../UserCommon/include)
target_link_libraries(Algorithm_Faulty_330371063_324976703 common::common)
set_target_properties(Algorithm_Faulty_330371063_324976703 PROPERTIES PREFIX "")

