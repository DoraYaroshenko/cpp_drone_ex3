#!/bin/bash

# test_threading.sh
# This script sets up a test environment to verify that the simulator 
# doesn't open more threads than necessary when given a small number of tasks.

echo "Setting up test environment..."

# 1. Create a directory for our test algorithms
TEST_DIR="test_algos"
rm -rf $TEST_DIR
mkdir -p $TEST_DIR

# 2. Copy exactly 2 algorithms into this directory to simulate 2 tasks
# We use the existing built algorithms.
cp build/default/Algorithm/Algorithm_Simple_330371063_324976703.so $TEST_DIR/
cp build/default/Algorithm/Algorithm_Const_330371063_324976703.so $TEST_DIR/

echo "Copied 2 algorithms into $TEST_DIR."
echo ""
echo "Running the simulator with num_threads=10 on a simulation with 1 run..."
echo "(This creates exactly 2 tasks: 2 algorithms * 1 run each)"
echo "We will use strace to count the number of threads actually created (clone calls)."
echo ""

# 3. Run the simulator using the small simulation composition we created,
# with num_threads=10. We use strace to count how many clone/clone3 calls are made 
# to see exactly how many threads the OS spawned.
strace -f -e clone,clone3 ./build/default/Simulator/simulator_330371063_324976703 \
  -competition \
  simulation=inputs/test_threads_sim.yaml \
  mission_control=build/default/MissionControl/MissionControl_330371063_324976703.so \
  algorithms_folder=$TEST_DIR \
  num_threads=10 2>&1 | grep -E "clone|clone3"

echo ""
echo "--------------------------------------------------------"
echo "Check the strace output above."
echo "If it prints exactly 1 clone call, then 1 extra thread was opened (total 2 threads)."
echo "If it prints exactly 2 clone calls, then 2 extra threads were opened (total 3 threads)."
echo "If it prints 10 clone calls, then it's ignoring the task count and opening 10 threads."
echo "--------------------------------------------------------"

# Cleanup
rm -rf $TEST_DIR
