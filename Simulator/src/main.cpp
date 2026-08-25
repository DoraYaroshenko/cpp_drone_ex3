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

    // Load raw YAML for output generation
    YAML::Node config = YAML::LoadFile(sim_path.string());

    std::cout << "Starting Simulation...\n";
    std::cout << "Command line parsing successful.\n";
    std::cout << "Mode: " << (is_comparative ? "Comparative" : "Competitive") << "\n";
    std::cout << "Simulation file: " << sim_path << "\n";
    std::cout << "Num threads requested: " << num_threads << "\n";
    std::cout << "Algorithms to load: " << algo_plugins_to_load.size() << "\n";
    std::cout << "Mission controls to load: " << mc_plugins_to_load.size() << "\n";
    std::cout << "Output directory: " << output_base_dir << "\n";

    if (is_comparative) {
        std::map<std::string, simulator::types::SimulationManagerReport> mission_reports;
        
        simulator::PluginLoader algo_loader;
        algo_loader.loadLibrary(algo_plugins_to_load[0]);
        auto algoFactories = simulator::MappingAlgorithmRegistrar::getInstance().getFactories();
        if (algoFactories.empty()) print_usage_and_exit("Algorithm failed to register.");
        auto algoFactory = algoFactories[0];
        simulator::MappingAlgorithmRegistrar::getInstance().clear();
        std::string algo_name = std::filesystem::path(algo_plugins_to_load[0]).filename().stem().string();
        
        for (const auto& mc_path : mc_plugins_to_load) {
            simulator::PluginLoader mc_loader;
            mc_loader.loadLibrary(mc_path);
            auto mcFactories = simulator::MissionControlRegistrar::getInstance().getFactories();
            if (mcFactories.empty()) {
                std::cerr << "Warning: Mission Control " << mc_path << " failed to register.\n";
                simulator::MissionControlRegistrar::getInstance().clear();
                continue;
            }
            auto mcFactory = mcFactories[0];
            simulator::MissionControlRegistrar::getInstance().clear();
            std::string mc_name = std::filesystem::path(mc_path).filename().stem().string();
            
            auto run_factory = std::make_unique<simulator::SimulationRunFactoryImpl>(algoFactory, mcFactory);
            simulator::SimulationManager simulation{std::move(run_factory), num_threads};
            
            std::filesystem::path run_out_dir = output_base_dir / mc_name;
            std::filesystem::create_directories(run_out_dir);

            simulator::types::SimulationManagerReport report = simulation.run(composition, run_out_dir);
            simulator::YamlParserUtils::writeSimulationOutput(report, config, sim_path.filename().string(), run_out_dir, mc_name);
            mission_reports[mc_name] = std::move(report);
            
            // mc_loader goes out of scope here, so dlclose happens after simulation manager is destroyed.
        }
        
        simulator::YamlParserUtils::writeComparativeReport(sim_path.filename().string(), algo_name, mission_reports, output_base_dir);
    } else {
        std::map<std::string, simulator::types::SimulationManagerReport> algo_reports;
        
        simulator::PluginLoader mc_loader;
        mc_loader.loadLibrary(mc_plugins_to_load[0]);
        auto mcFactories = simulator::MissionControlRegistrar::getInstance().getFactories();
        if (mcFactories.empty()) print_usage_and_exit("Mission Control failed to register.");
        auto mcFactory = mcFactories[0];
        simulator::MissionControlRegistrar::getInstance().clear();
        std::string mc_name = std::filesystem::path(mc_plugins_to_load[0]).filename().stem().string();
        
        for (const auto& algo_path : algo_plugins_to_load) {
            simulator::PluginLoader algo_loader;
            algo_loader.loadLibrary(algo_path);
            auto algoFactories = simulator::MappingAlgorithmRegistrar::getInstance().getFactories();
            if (algoFactories.empty()) {
                std::cerr << "Warning: Algorithm " << algo_path << " failed to register.\n";
                simulator::MappingAlgorithmRegistrar::getInstance().clear();
                continue;
            }
            auto algoFactory = algoFactories[0];
            simulator::MappingAlgorithmRegistrar::getInstance().clear();
            std::string algo_name = std::filesystem::path(algo_path).filename().stem().string();
            
            auto run_factory = std::make_unique<simulator::SimulationRunFactoryImpl>(algoFactory, mcFactory);
            simulator::SimulationManager simulation{std::move(run_factory), num_threads};
            
            std::filesystem::path run_out_dir = output_base_dir / algo_name;
            std::filesystem::create_directories(run_out_dir);

            simulator::types::SimulationManagerReport report = simulation.run(composition, run_out_dir);
            simulator::YamlParserUtils::writeSimulationOutput(report, config, sim_path.filename().string(), run_out_dir, algo_name);
            algo_reports[algo_name] = std::move(report);
        }
        
        simulator::YamlParserUtils::writeCompetitiveReport(sim_path.filename().string(), mc_name, algo_reports, output_base_dir);
    }
    
    return 0;
}
