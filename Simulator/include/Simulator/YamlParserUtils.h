#pragma once

#include <Simulator/SimulationTypes.h>
#include <filesystem>
#include <yaml-cpp/yaml.h>
#include <string>

namespace simulator {

class YamlParserUtils {
public:
    static types::SimulationCompositionData parseCompositions(const std::filesystem::path& sim_path);
    
    // Writes output for a single simulation run (from Ex2)
    static void writeSimulationOutput(
        const types::SimulationManagerReport& report,
        const YAML::Node& comp_yaml,
        const std::string& composition_filename,
        const std::filesystem::path& output_path,
        const std::string& extra_suffix = ""
    );

    // Comparative Simulation Result Output File
    static void writeComparativeReport(
        const std::string& composition_filename,
        const std::string& algorithm_name,
        const std::map<std::string, types::SimulationManagerReport>& mission_reports,
        const std::filesystem::path& output_path
    );

    // Competitive Simulation Result Output File
    static void writeCompetitiveReport(
        const std::string& composition_filename,
        const std::string& mission_control_name,
        const std::map<std::string, types::SimulationManagerReport>& algo_reports,
        const std::filesystem::path& output_path
    );
};

} // namespace simulator
