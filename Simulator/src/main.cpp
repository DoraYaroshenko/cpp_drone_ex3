#include "PluginLoader.h"
#include "MappingAlgorithmRegistrar.h"
#include "MissionControlRegistrar.h"
#include <Simulator/SimulationManager.h>
#include <Simulator/SimulationRunFactoryImpl.h>
#include <Simulator/YamlParserUtils.h>

#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <map>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <cstdlib>
#include <algorithm>

void print_usage_and_exit(const std::string& error_msg) {
    std::cerr << "Error: " << error_msg << "\n";
    std::cerr << "Usage:\n";
    std::cerr << "  Comparative run: simulator_<submitter_ids> -comparative simulation=<sim.yaml> mission_control_folder=<folder> algorithm=<algo.so> [num_threads=<num>] [-verbose]\n";
    std::cerr << "  Competition run: simulator_<submitter_ids> -competition simulation=<sim.yaml> mission_control=<mc.so> algorithms_folder=<folder> [num_threads=<num>] [-verbose]\n";
    std::exit(1);
}

std::string generate_timestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm utc_tm{};
#if defined(_MSC_VER)
    gmtime_s(&utc_tm, &time_t_now);
#else
    gmtime_r(&time_t_now, &utc_tm);
#endif
    std::ostringstream time_ss;
    time_ss << std::put_time(&utc_tm, "%Y-%m-%dT%H:%M:%SZ");
    return time_ss.str();
}

std::string generate_folder_timestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm utc_tm{};
#if defined(_MSC_VER)
    gmtime_s(&utc_tm, &time_t_now);
#else
    gmtime_r(&time_t_now, &utc_tm);
#endif
    std::ostringstream time_ss;
    time_ss << std::put_time(&utc_tm, "%Y_%m_%d_%H_%M_%S");
    return time_ss.str();
}

int main(int argc, char** argv) {
    std::map<std::string, std::string> args;
    std::vector<std::string> unsupported_args;
    bool is_comparative = false;
    bool is_competition = false;
    bool is_verbose = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-comparative") {
            is_comparative = true;
        } else if (arg == "-competition") {
            is_competition = true;
        } else if (arg == "-verbose") {
            is_verbose = true;
        } else {
            auto pos = arg.find('=');
            if (pos != std::string::npos && pos > 0 && pos < arg.size() - 1) {
                std::string key = arg.substr(0, pos);
                std::string value = arg.substr(pos + 1);
                
                if (key == "simulation" || key == "mission_control_folder" || 
                    key == "algorithm" || key == "mission_control" || 
                    key == "algorithms_folder" || key == "num_threads") {
                    args[key] = value;
                } else {
                    unsupported_args.push_back(arg);
                }
            } else {
                unsupported_args.push_back(arg);
            }
        }
    }

    if (!unsupported_args.empty()) {
        std::string msg = "Unsupported arguments provided: ";
        for (const auto& a : unsupported_args) msg += a + " ";
        print_usage_and_exit(msg);
    }

    if (is_comparative && is_competition) {
        print_usage_and_exit("Cannot specify both -comparative and -competition modes.");
    }
    
    if (!is_comparative && !is_competition) {
        print_usage_and_exit("Missing mode argument (-comparative or -competition).");
    }

    if (args.find("simulation") == args.end()) {
        print_usage_and_exit("Missing mandatory argument: simulation=<...>");
    }

    std::filesystem::path sim_path = args["simulation"];
    if (!std::filesystem::exists(sim_path) || !std::filesystem::is_regular_file(sim_path)) {
        print_usage_and_exit("Simulation file does not exist or is not a regular file: " + sim_path.string());
    }

    // Number of threads
    int num_threads = 1;
    if (args.find("num_threads") != args.end()) {
        try {
            num_threads = std::stoi(args["num_threads"]);
        } catch (...) {
            print_usage_and_exit("Invalid value for num_threads: " + args["num_threads"]);
        }
    }

    std::vector<std::string> mc_plugins_to_load;
    std::vector<std::string> algo_plugins_to_load;
    std::filesystem::path output_base_dir;

    if (is_comparative) {
        if (args.find("mission_control_folder") == args.end()) print_usage_and_exit("Missing mandatory argument for comparative mode: mission_control_folder=<...>");
        if (args.find("algorithm") == args.end()) print_usage_and_exit("Missing mandatory argument for comparative mode: algorithm=<...>");

        std::filesystem::path mc_folder = args["mission_control_folder"];
        std::filesystem::path algo_file = args["algorithm"];

        if (!std::filesystem::exists(mc_folder) || !std::filesystem::is_directory(mc_folder)) {
            print_usage_and_exit("mission_control_folder does not exist or is not a directory: " + mc_folder.string());
        }
        if (!std::filesystem::exists(algo_file) || !std::filesystem::is_regular_file(algo_file)) {
            print_usage_and_exit("algorithm file does not exist or is not a regular file: " + algo_file.string());
        }

        algo_plugins_to_load.push_back(std::filesystem::canonical(algo_file).string());
        for (const auto& entry : std::filesystem::directory_iterator(mc_folder)) {
            if (entry.is_regular_file() && entry.path().extension() == ".so") {
                mc_plugins_to_load.push_back(std::filesystem::canonical(entry.path()).string());
            }
        }
        if (mc_plugins_to_load.empty()) {
            print_usage_and_exit("No .so files found in mission_control_folder: " + mc_folder.string());
        }
        
        output_base_dir = mc_folder / ("comparative_results_" + generate_folder_timestamp());
    } else {
        if (args.find("algorithms_folder") == args.end()) print_usage_and_exit("Missing mandatory argument for competition mode: algorithms_folder=<...>");
        if (args.find("mission_control") == args.end()) print_usage_and_exit("Missing mandatory argument for competition mode: mission_control=<...>");

        std::filesystem::path algo_folder = args["algorithms_folder"];
        std::filesystem::path mc_file = args["mission_control"];

        if (!std::filesystem::exists(algo_folder) || !std::filesystem::is_directory(algo_folder)) {
            print_usage_and_exit("algorithms_folder does not exist or is not a directory: " + algo_folder.string());
        }
        if (!std::filesystem::exists(mc_file) || !std::filesystem::is_regular_file(mc_file)) {
            print_usage_and_exit("mission_control file does not exist or is not a regular file: " + mc_file.string());
        }

        mc_plugins_to_load.push_back(std::filesystem::canonical(mc_file).string());
        for (const auto& entry : std::filesystem::directory_iterator(algo_folder)) {
            if (entry.is_regular_file() && entry.path().extension() == ".so") {
                algo_plugins_to_load.push_back(std::filesystem::canonical(entry.path()).string());
            }
        }
        if (algo_plugins_to_load.empty()) {
            print_usage_and_exit("No .so files found in algorithms_folder: " + algo_folder.string());
        }

        output_base_dir = algo_folder / ("competition_" + generate_folder_timestamp());
    }

    try {
        std::filesystem::create_directories(output_base_dir);
    } catch (const std::exception& e) {
        print_usage_and_exit("Failed to create output directory: " + output_base_dir.string() + " - " + e.what());
    }

    simulator::PluginLoader loader;

    // We will parse compositions once, for now we just verify it works
    simulator::types::SimulationCompositionData composition;
    try {
        composition = simulator::YamlParserUtils::parseCompositions(sim_path);
    } catch (const std::exception& e) {
        print_usage_and_exit("Failed to parse simulation configuration: " + std::string(e.what()));
    }

    // Load plugins... (Actual concurrent execution will happen in Step 3, we just prep the structure here)
    // Here we can simulate loading, creating simulation manager, and doing runs.
    
    // In Part 2 we simply log that we're set up successfully.
    std::cout << "Command line parsing successful.\n";
    std::cout << "Mode: " << (is_comparative ? "Comparative" : "Competitive") << "\n";
    std::cout << "Simulation file: " << sim_path << "\n";
    std::cout << "Num threads requested: " << num_threads << "\n";
    std::cout << "Algorithms to load: " << algo_plugins_to_load.size() << "\n";
    std::cout << "Mission controls to load: " << mc_plugins_to_load.size() << "\n";
    std::cout << "Output directory: " << output_base_dir << "\n";

    // Simulate loading libraries
    for (const auto& path : algo_plugins_to_load) {
        loader.loadLibrary(path);
    }
    for (const auto& path : mc_plugins_to_load) {
        loader.loadLibrary(path);
    }
    
    auto& algoFactories = simulator::MappingAlgorithmRegistrar::getInstance().getFactories();
    auto& mcFactories = simulator::MissionControlRegistrar::getInstance().getFactories();
    std::cout << "Successfully registered Algorithms: " << algoFactories.size() << std::endl;
    std::cout << "Successfully registered Mission Controls: " << mcFactories.size() << std::endl;

    // Cleanup: Clear registrars before unloading the libraries
    simulator::MappingAlgorithmRegistrar::getInstance().clear();
    simulator::MissionControlRegistrar::getInstance().clear();

    return 0;
}
