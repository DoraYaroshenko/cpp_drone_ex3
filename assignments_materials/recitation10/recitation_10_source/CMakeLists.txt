cmake_minimum_required(VERSION 3.15)
project(recitation_9_dynamic_loading LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)

add_compile_options(-Wall -Wextra -Werror -pedantic)

# MODULE creates a runtime-loaded .so; algorithm_demo never links these targets.
add_library(const_algorithm MODULE algorithm1/ConstAlgorithm.cpp)
set_target_properties(const_algorithm PROPERTIES PREFIX "" OUTPUT_NAME "ConstAlgorithm")
target_include_directories(const_algorithm PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

add_library(cycling_algorithm MODULE algorithm2/CyclingAlgorithm.cpp)
set_target_properties(cycling_algorithm PROPERTIES PREFIX "" OUTPUT_NAME "CyclingAlgorithm")
target_include_directories(cycling_algorithm PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

add_library(slow_random_algorithm MODULE algorithm3/SlowRandomAlgorithm.cpp)
set_target_properties(slow_random_algorithm PROPERTIES PREFIX "" OUTPUT_NAME "SlowRandomAlgorithm")
target_include_directories(slow_random_algorithm PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(slow_random_algorithm PRIVATE Threads::Threads)

add_library(combined_algorithms MODULE
    algorithm1/ConstAlgorithm.cpp
    algorithm2/CyclingAlgorithm.cpp)
set_target_properties(combined_algorithms PROPERTIES PREFIX "" OUTPUT_NAME "CombinedAlgorithms")
target_include_directories(combined_algorithms PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(algorithm_demo main.cpp Registrar.cpp)
# Export AlgorithmRegistration's constructor so plugin static initialization can find it.
set_target_properties(algorithm_demo PROPERTIES ENABLE_EXPORTS ON)
target_include_directories(algorithm_demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(algorithm_demo PRIVATE ${CMAKE_DL_LIBS} Threads::Threads)
add_dependencies(algorithm_demo
    const_algorithm
    cycling_algorithm
    slow_random_algorithm
    combined_algorithms)

enable_testing()
add_test(NAME load_one_algorithm
    COMMAND algorithm_demo 1 3 $<TARGET_FILE:const_algorithm>)
add_test(NAME load_multiple_algorithms
    COMMAND algorithm_demo 1 3
        $<TARGET_FILE:const_algorithm>
        $<TARGET_FILE:cycling_algorithm>
        $<TARGET_FILE:slow_random_algorithm>)
add_test(NAME two_worker_threads
    COMMAND algorithm_demo 1 3 --threads
        $<TARGET_FILE:const_algorithm>
        $<TARGET_FILE:cycling_algorithm>
        $<TARGET_FILE:slow_random_algorithm>)
add_test(NAME load_multiple_from_one_library
    COMMAND algorithm_demo 1 3 $<TARGET_FILE:combined_algorithms>)
add_test(NAME missing_library
    COMMAND algorithm_demo 1 3
        ${CMAKE_CURRENT_BINARY_DIR}/MissingAlgorithm.so)
add_test(NAME duplicate_library
    COMMAND algorithm_demo 1 3
        $<TARGET_FILE:const_algorithm>
        $<TARGET_FILE:const_algorithm>)
set_tests_properties(missing_library duplicate_library PROPERTIES WILL_FAIL TRUE)
