#!/bin/bash

# executable
SIM=$(ls build/default/Simulator/simulator_* 2>/dev/null | head -n 1)
if [ -z "$SIM" ]; then
    echo "Simulator executable not found! Make sure you compiled the project."
    exit 1
fi
SIM="./$SIM"

echo "Using simulator: $SIM"

# Create dummy test environment
TEST_DIR="cmdline_test_env"
mkdir -p "$TEST_DIR"
touch "$TEST_DIR/sim.yaml"
mkdir -p "$TEST_DIR/mc_folder"
touch "$TEST_DIR/mc_folder/MissionControl_dummy.so"
mkdir -p "$TEST_DIR/algo_folder"
touch "$TEST_DIR/algo_folder/Algorithm_dummy.so"
touch "$TEST_DIR/algo.so"
touch "$TEST_DIR/mc.so"

mkdir -p "$TEST_DIR/empty_folder"

run_test() {
    echo -e "\n---> Running: $SIM $@"
    $SIM "$@"
    echo "Exit code: $?"
}

echo "========================================================"
echo "1. All command line arguments can appear in any order."
echo "========================================================"
run_test -comparative simulation=$TEST_DIR/sim.yaml mission_control_folder=$TEST_DIR/mc_folder algorithm=$TEST_DIR/algo.so
run_test algorithm=$TEST_DIR/algo.so -comparative mission_control_folder=$TEST_DIR/mc_folder simulation=$TEST_DIR/sim.yaml

echo "========================================================"
echo "2, 5. Missing command line arguments (mandatory missing)."
echo "========================================================"
run_test -comparative mission_control_folder=$TEST_DIR/mc_folder algorithm=$TEST_DIR/algo.so
run_test -comparative simulation=$TEST_DIR/sim.yaml mission_control_folder=$TEST_DIR/mc_folder
run_test simulation=$TEST_DIR/sim.yaml mission_control_folder=$TEST_DIR/mc_folder algorithm=$TEST_DIR/algo.so
run_test -competition simulation=$TEST_DIR/sim.yaml mission_control=$TEST_DIR/mc.so

echo "========================================================"
echo "3. Assume '=' sign appears without spaces around."
echo "========================================================"
echo "(This is assumed by the script structure above, no special test needed.)"

echo "========================================================"
echo "4. Unsupported command lines arguments."
echo "========================================================"
run_test -comparative simulation=$TEST_DIR/sim.yaml mission_control_folder=$TEST_DIR/mc_folder algorithm=$TEST_DIR/algo.so bad_arg=1 another_bad_arg=2 -weird_flag

echo "========================================================"
echo "6. Non-existing file or cannot be opened."
echo "========================================================"
run_test -comparative simulation=$TEST_DIR/non_existing_sim.yaml mission_control_folder=$TEST_DIR/mc_folder algorithm=$TEST_DIR/algo.so
run_test -comparative simulation=$TEST_DIR/sim.yaml mission_control_folder=$TEST_DIR/mc_folder algorithm=$TEST_DIR/non_existing_algo.so

echo "========================================================"
echo "7. Non-existing folder, cannot be traversed, or zero relevant files."
echo "========================================================"
run_test -comparative simulation=$TEST_DIR/sim.yaml mission_control_folder=$TEST_DIR/non_existing_folder algorithm=$TEST_DIR/algo.so
run_test -comparative simulation=$TEST_DIR/sim.yaml mission_control_folder=$TEST_DIR/empty_folder algorithm=$TEST_DIR/algo.so
run_test -competition simulation=$TEST_DIR/sim.yaml mission_control=$TEST_DIR/mc.so algorithms_folder=$TEST_DIR/empty_folder

echo "========================================================"
echo "8. Exact usage description and error message text are for your decision."
echo "========================================================"
echo "(Check the output of the tests above to ensure usage and messages are printed)"

echo "========================================================"
echo "9. num_threads is optional."
echo "========================================================"
run_test -comparative simulation=$TEST_DIR/sim.yaml mission_control_folder=$TEST_DIR/mc_folder algorithm=$TEST_DIR/algo.so num_threads=4
run_test -comparative simulation=$TEST_DIR/sim.yaml mission_control_folder=$TEST_DIR/mc_folder algorithm=$TEST_DIR/algo.so

echo "========================================================"
echo "Cleaning up test files..."
rm -rf "$TEST_DIR"
echo "Done."
