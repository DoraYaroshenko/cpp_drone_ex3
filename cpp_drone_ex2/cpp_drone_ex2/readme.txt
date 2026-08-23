-- IDS:
Lilac Gil-Ad: 324976703
Dora Yaroshenko: 330371063

-- OUTPUT DIRECTORY FORMAT:
The directory is called output_results, and is created in the given output path (or working directory).
Inside, there are indexed folders for each run (e.g. run_0, run_1 etc.).
Inside each run folder may include the following files:
 - map_output.npy - the output map.
 - drone_logs.jsonl - logs of the drone movement, scans performed and voxels changed.
 - error_log.txt - logs of the errors encountered.
The main output_results directory also includes the simulation_output.yaml file.

-- INPUTS:
We added some inputs from the previous exercise in the inputs directory.
To run the simulation:

./build/drone_mapper_simulation inputs/simulation.yaml output