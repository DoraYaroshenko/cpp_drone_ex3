#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>
#include <TinyNPY.h>
#include <drone_mapper/ConfigParserUtils.h>

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {

using namespace drone_mapper;
using namespace drone_mapper::types;

// Parse a drone config YAML file.
DroneConfigData parseDroneConfig(const std::filesystem::path& path) {
    YAML::Node config = YAML::LoadFile(path.string());
    if (!config["drone_config"]) throw std::invalid_argument("Missing drone_config");
    YAML::Node dc = config["drone_config"];

    DroneConfigData drone;
    drone.radius = (get_with_check<double>(dc, "dimensions_cm", 30.0, [](const double& v){ return v > 0.0; }, "dimensions_cm must be > 0") / 2.0) * cm;
    drone.max_rotate = get_with_check<double>(dc, "max_rotate_deg", 90.0, [](const double& v){ return v > 0.0 && v <= 360.0; }, "max_rotate_deg must be in (0, 360]") * horizontal_angle[deg];
    drone.max_advance = get_with_check<double>(dc, "max_advance_cm", 100.0, [](const double& v){ return v > 0.0; }, "max_advance_cm must be > 0") * cm;
    drone.max_elevate = get_with_check<double>(dc, "max_elevate_cm", 200.0, [](const double& v){ return v > 0.0; }, "max_elevate_cm must be > 0") * cm;
    return drone;
}

// Parse a lidar config YAML file.
LidarConfigData parseLidarConfig(const std::filesystem::path& path) {
    YAML::Node config = YAML::LoadFile(path.string());
    if (!config["lidar_config"]) throw std::invalid_argument("Missing lidar_config");
    YAML::Node lc = config["lidar_config"];

    LidarConfigData lidar;
    double z_max = get_with_default<double>(lc, "z_max_cm", 200.0);
    double z_min = get_with_check<double>(lc, "z_min_cm", 0.0, [](const double& v){ return v >= 0.0; }, "z_min_cm must be >= 0");
    if (z_min > z_max) throw std::invalid_argument("z_max_cm must be >= z_min_cm");
    
    lidar.z_min = z_min * cm;
    lidar.z_max = z_max * cm;
    lidar.d = get_with_check<double>(lc, "d_cm", 1.0, [](const double& v){ return v > 0.0; }, "d_cm must be > 0") * cm;
    lidar.fov_circles = get_with_check<std::size_t>(lc, "fov_circles", 2, [](const std::size_t& v){ return true; }, ""); // can be 0
    return lidar;
}

// Parse a mission config YAML file.
MissionConfigData parseMissionConfig(const std::filesystem::path& path, const MappingBounds& default_bounds) {
    YAML::Node config = YAML::LoadFile(path.string());
    if (!config["mission_config"]) throw std::invalid_argument("Missing mission_config");
    YAML::Node mc = config["mission_config"];

    MissionConfigData mission;
    mission.max_steps = get_with_check<std::size_t>(mc, "max_steps", 2500, [](const std::size_t& v){ return v > 0; }, "max_steps must be > 0");
    mission.gps_resolution = get_with_check<double>(mc, "gps_resolution_cm", 1.0, [](const double& v){ return v > 0.0; }, "gps_resolution_cm must be > 0") * cm;

    mission.output_mapping_resolution_factor = get_with_default<double>(mc, "output_mapping_resolution_factor", 1.0);

    mission.mission_bounds = default_bounds;
    if (mc["mission_boundaries"]) {
        auto b = mc["mission_boundaries"];
        if (b["x_boundary"]) {
            mission.mission_bounds.min_x = get_with_default<double>(b["x_boundary"], "min_cm", default_bounds.min_x.force_numerical_value_in(cm)) * x_extent[cm];
            mission.mission_bounds.max_x = get_with_default<double>(b["x_boundary"], "max_cm", default_bounds.max_x.force_numerical_value_in(cm)) * x_extent[cm];
        }
        if (b["y_boundary"]) {
            mission.mission_bounds.min_y = get_with_default<double>(b["y_boundary"], "min_cm", default_bounds.min_y.force_numerical_value_in(cm)) * y_extent[cm];
            mission.mission_bounds.max_y = get_with_default<double>(b["y_boundary"], "max_cm", default_bounds.max_y.force_numerical_value_in(cm)) * y_extent[cm];
        }
        if (b["height_boundary"]) {
            mission.mission_bounds.min_height = get_with_default<double>(b["height_boundary"], "min_cm", default_bounds.min_height.force_numerical_value_in(cm)) * z_extent[cm];
            mission.mission_bounds.max_height = get_with_default<double>(b["height_boundary"], "max_cm", default_bounds.max_height.force_numerical_value_in(cm)) * z_extent[cm];
        }
    }

    if (mission.mission_bounds.max_x < mission.mission_bounds.min_x) {
        throw std::invalid_argument("mission_boundaries x_boundary max must be >= min");
    }
    if (mission.mission_bounds.max_y < mission.mission_bounds.min_y) {
        throw std::invalid_argument("mission_boundaries y_boundary max must be >= min");
    }
    if (mission.mission_bounds.max_height < mission.mission_bounds.min_height) {
        throw std::invalid_argument("mission_boundaries height_boundary max must be >= min");
    }

    return mission;
}

// Parse a single simulation config YAML file.
SimulationConfigData parseSimulationConfig(const std::filesystem::path& path) {
    YAML::Node config = YAML::LoadFile(path.string());
    if (!config["simulation_config"]) throw std::invalid_argument("Missing simulation_config");
    YAML::Node sc = config["simulation_config"];

    SimulationConfigData sim;
    if (!sc["map_filename"]) throw std::invalid_argument("map_filename is required");
    sim.map_filename = sc["map_filename"].as<std::string>();
    if (sim.map_filename.empty()) throw std::invalid_argument("map_filename cannot be empty");
    
    sim.map_resolution = get_with_check<double>(sc, "map_resolution_cm", 10.0, [](const double& v){ return v > 0.0; }, "map_resolution_cm must be > 0") * cm;

    if (sc["initial_drone_position"]) {
        auto pos = sc["initial_drone_position"];
        sim.initial_drone_position = Position3D{
            XLength(get_with_default<double>(pos, "x_cm", 0.0) * x_extent[cm]),
            YLength(get_with_default<double>(pos, "y_cm", 0.0) * y_extent[cm]),
            ZLength(get_with_default<double>(pos, "height_cm", 0.0) * z_extent[cm])
        };
    } else {
        sim.initial_drone_position = Position3D{XLength(0.0*cm), YLength(0.0*cm), ZLength(0.0*cm)};
    }

    sim.initial_angle = get_with_default<double>(sc, "initial_angle_deg", 0.0) * horizontal_angle[deg];

    // Parse map axes offset.
    if (config["map_axes_offset"]) {
        auto offset = config["map_axes_offset"];
        sim.map_offset = Position3D{
            XLength(get_with_default<double>(offset, "x_offset", 0.0) * x_extent[cm]),
            YLength(get_with_default<double>(offset, "y_offset", 0.0) * y_extent[cm]),
            ZLength(get_with_default<double>(offset, "height_offset", 0.0) * z_extent[cm])
        };
    } else {
        sim.map_offset = Position3D{XLength(0.0*cm), YLength(0.0*cm), ZLength(0.0*cm)};
    }

    return sim;
}

// Parse the simulation compositions YAML file.
SimulationCompositionData parseCompositions(const std::filesystem::path& path) {
    YAML::Node config = YAML::LoadFile(path.string());
    YAML::Node comp = config["simulation_compositions"];

    SimulationCompositionData composition;
    composition.composition_file = path;

    // Base directory for resolving relative paths.
    std::filesystem::path base_dir = path.parent_path();
    if (base_dir.empty()) {
        base_dir = std::filesystem::current_path();
    }

    // Parse simulations (and their nested mission_configs).
    if (comp["simulations"]) {
        for (const auto& sim_node : comp["simulations"]) {
            std::filesystem::path sim_path = base_dir / sim_node["simulation_config"].as<std::string>();
            SimulationConfigData sim = parseSimulationConfig(sim_path);
            
            // Resolve map_filename relative to composition file's base_dir
            if (!sim.map_filename.is_absolute()) {
                sim.map_filename = std::filesystem::weakly_canonical(base_dir / sim.map_filename);
            }
            
            // Calculate default map boundaries from NPY file shape
            MappingBounds default_bounds;
            std::filesystem::path npy_path = sim.map_filename;
            try {
                auto arr = std::make_shared<NpyArray>();
                const char* err = arr->LoadNPY(npy_path.string().c_str());
                if (err == nullptr && arr->Shape().size() == 3) {
                    const auto& shape = arr->Shape(); // [X, Y, Z]
                    PhysicalLength res = sim.map_resolution;
                    double res_cm = res.numerical_value_in(cm);
                    
                    default_bounds.min_x = sim.map_offset.x;
                    default_bounds.max_x = sim.map_offset.x + (static_cast<double>(shape[0]) * res_cm * x_extent[cm]);
                    default_bounds.min_y = sim.map_offset.y;
                    default_bounds.max_y = sim.map_offset.y + (static_cast<double>(shape[1]) * res_cm * y_extent[cm]);
                    default_bounds.min_height = sim.map_offset.z;
                    default_bounds.max_height = sim.map_offset.z + (static_cast<double>(shape[2]) * res_cm * z_extent[cm]);
                }
            } catch (...) {
                // Ignore load errors here; they will be caught later in SimulationRunFactory.
            }

            std::vector<MissionConfigData> missions_for_sim;

            if (sim_node["mission_configs"]) {
                for (const auto& mission_path_node : sim_node["mission_configs"]) {
                    std::filesystem::path mission_path = base_dir / mission_path_node.as<std::string>();
                    MissionConfigData mission = parseMissionConfig(mission_path, default_bounds);
                    missions_for_sim.push_back(mission);
                }
            }
            composition.simulation_mission_groups.push_back({sim, missions_for_sim});
        }
    }

    // Parse drone configs.
    if (comp["drone_configs"]) {
        for (const auto& drone_path_node : comp["drone_configs"]) {
            std::filesystem::path drone_path = base_dir / drone_path_node.as<std::string>();
            DroneConfigData drone = parseDroneConfig(drone_path);
            composition.drones.push_back(drone);
        }
    }

    // Parse lidar configs.
    if (comp["lidar_configs"]) {
        for (const auto& lidar_path_node : comp["lidar_configs"]) {
            std::filesystem::path lidar_path = base_dir / lidar_path_node.as<std::string>();
            LidarConfigData lidar = parseLidarConfig(lidar_path);
            composition.lidars.push_back(lidar);
        }
    }

    return composition;
}

// Write the simulation output YAML report.
void writeSimulationOutput(const SimulationManagerReport& report,
                           const YAML::Node& comp_yaml,
                           const std::string& composition_file_name,
                           const std::filesystem::path& output_path) {
    YAML::Emitter out;
    out << YAML::BeginMap;

    out << YAML::Key << "score_report" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "composition_file" << YAML::Value << composition_file_name;
    out << YAML::Key << "generated_at_utc" << YAML::Value << report.generated_at_utc;
    out << YAML::Key << "metric" << YAML::Value << report.metric;
    out << YAML::Key << "score_range" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "min" << YAML::Value << std::get<0>(report.score_range);
    out << YAML::Key << "max" << YAML::Value << std::get<1>(report.score_range);
    out << YAML::EndMap;
    out << YAML::Key << "error_score" << YAML::Value << report.error_score;

    // Summary.
    int total = static_cast<int>(report.runs.size());
    int error_runs = 0;
    int scored_runs = 0;
    double total_score = 0.0;
    double min_score = 101.0;
    double max_score = -2.0;

    for (const auto& run : report.runs) {
        if (run.mission_score < 0) {
            ++error_runs;
        } else {
            ++scored_runs;
            total_score += run.mission_score;
            min_score = std::min(min_score, run.mission_score);
            max_score = std::max(max_score, run.mission_score);
        }
    }

    out << YAML::Key << "summary" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "total_runs" << YAML::Value << total;
    out << YAML::Key << "scored_runs" << YAML::Value << scored_runs;
    out << YAML::Key << "error_runs" << YAML::Value << error_runs;
    out << YAML::Key << "average_score" << YAML::Value
        << (scored_runs > 0 ? total_score / scored_runs : 0.0);
    out << YAML::Key << "min_score" << YAML::Value << (scored_runs > 0 ? min_score : 0.0);
    out << YAML::Key << "max_score" << YAML::Value << (scored_runs > 0 ? max_score : 0.0);
    out << YAML::EndMap;

    // Individual runs.
    out << YAML::Key << "simulations" << YAML::Value << YAML::BeginSeq;

    std::size_t run_idx = 0;
    if (comp_yaml["simulations"]) {
        for (const auto& sim_node : comp_yaml["simulations"]) {
            out << YAML::BeginMap;
            out << YAML::Key << "simulation_config" << YAML::Value << sim_node["simulation_config"].as<std::string>();
            out << YAML::Key << "missions" << YAML::Value << YAML::BeginSeq;

            if (sim_node["mission_configs"]) {
                for (const auto& mission_node : sim_node["mission_configs"]) {
                    out << YAML::BeginMap;
                    out << YAML::Key << "mission_config" << YAML::Value << mission_node.as<std::string>();

                    if (run_idx < report.runs.size()) {
                        const auto& first_run = report.runs[run_idx];
                        out << YAML::Key << "resolution_cm" << YAML::Value 
                            << first_run.output_map_config.resolution.numerical_value_in(cm);
                        
                        std::string res_status;
                        switch (first_run.resolution_request_status) {
                            case ResolutionRequestStatus::Accepted: res_status = "ACCEPTED"; break;
                            case ResolutionRequestStatus::Ignored: res_status = "IGNORED"; break;
                            case ResolutionRequestStatus::IgnoredTooSmall: res_status = "IGNORED TOO SMALL"; break;
                        }
                        out << YAML::Key << "resolution_request_status" << YAML::Value << res_status;
                    }

                    out << YAML::Key << "runs" << YAML::Value << YAML::BeginSeq;

                    if (comp_yaml["drone_configs"] && comp_yaml["lidar_configs"]) {
                        for (const auto& drone_node : comp_yaml["drone_configs"]) {
                            for (const auto& lidar_node : comp_yaml["lidar_configs"]) {
                                if (run_idx >= report.runs.size()) break;
                                const auto& run = report.runs[run_idx++];

                                out << YAML::BeginMap;
                                out << YAML::Key << "drone_config" << YAML::Value << drone_node.as<std::string>();
                                out << YAML::Key << "lidar_config" << YAML::Value << lidar_node.as<std::string>();

                                if (!run.mission_results.empty()) {
                                    const auto& mr = run.mission_results.front();
                                    std::string status;
                                    switch (mr.status) {
                                        case MissionRunStatus::Completed: status = "completed"; break;
                                        case MissionRunStatus::MaxSteps: status = "max_steps"; break;
                                        case MissionRunStatus::Error: status = "error"; break;
                                    }
                                    out << YAML::Key << "status" << YAML::Value << status;
                                    out << YAML::Key << "steps" << YAML::Value << static_cast<int>(mr.steps);
                                    
                                    out << YAML::Key << "score" << YAML::Value << run.mission_score;

                                    if (!mr.errors.empty()) {
                                        out << YAML::Key << "error_ref" << YAML::Value << YAML::BeginMap;
                                        out << YAML::Key << "code" << YAML::Value << mr.errors[0].code;
                                        out << YAML::EndMap;
                                    }
                                } else {
                                    out << YAML::Key << "status" << YAML::Value << "error";
                                    out << YAML::Key << "steps" << YAML::Value << 0;
                                    out << YAML::Key << "score" << YAML::Value << run.mission_score;
                                }

                                out << YAML::EndMap; // run
                            }
                        }
                    }
                    out << YAML::EndSeq; // runs
                    out << YAML::EndMap; // mission
                }
            }
            out << YAML::EndSeq; // missions
            out << YAML::EndMap; // simulation
        }
    }
    out << YAML::EndSeq; // simulations

    out << YAML::EndMap; // score_report
    out << YAML::EndMap; // root

    std::filesystem::path output_file = output_path / "simulation_output.yaml";
    std::ofstream file(output_file);
    if (!file) {
        std::cerr << "Error: Could not write simulation output to " << output_file << "\n";
        return;
    }
    file << out.c_str() << "\n";
}

} // namespace

int main(int argc, char** argv) {
    try {
        // Parse arguments.
        std::filesystem::path composition_file;
        if (argc >= 2) {
            composition_file = std::filesystem::path{argv[1]};
        } else {
            composition_file = std::filesystem::path{"simulation.yaml"};
        }

        // Resolve relative paths.
        if (!composition_file.is_absolute() && composition_file.is_relative()) {
            composition_file = std::filesystem::current_path() / composition_file;
        }

        const std::filesystem::path output_path =
            (argc >= 3) ? std::filesystem::path{argv[2]} : std::filesystem::current_path();

        // Parse composition YAML.
        SimulationCompositionData composition = parseCompositions(composition_file);
        
        // Load raw YAML for output generation
        YAML::Node config = YAML::LoadFile(composition_file.string());
        YAML::Node comp_yaml = config["simulation_compositions"];

        // Create and run simulation.
        auto run_factory = std::make_unique<SimulationRunFactoryImpl>();
        SimulationManager simulation{std::move(run_factory)};

        const SimulationManagerReport report = simulation.run(composition, output_path);

        // Write output YAML.
        writeSimulationOutput(report, comp_yaml, composition_file.filename().string(), output_path);

        std::cout << "Simulation completed: "
                  << report.runs.size() << " run(s).\n";

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
