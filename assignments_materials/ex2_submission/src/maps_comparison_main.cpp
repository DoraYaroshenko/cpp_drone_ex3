#include <drone_mapper/MapsComparison.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/ConfigParserUtils.h>

#include <yaml-cpp/yaml.h>

#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace {

using namespace drone_mapper;
using namespace drone_mapper::types;

// Load a map from an NPY file with optional config.
std::unique_ptr<Map3DImpl> loadMap(const std::filesystem::path& npy_path,
                                    double map_res_cm,
                                    Position3D offset,
                                    MappingBounds bounds) {
    auto arr = std::make_shared<NpyArray>();
    const char* err = arr->LoadNPY(npy_path.string().c_str());
    if (err != nullptr) {
        throw std::runtime_error(std::string("Failed to load NPY: ") + err);
    }
    if (arr->Shape().size() != 3) {
        throw std::runtime_error("Expected 3D NPY array.");
    }

    MapConfig config;
    config.resolution = map_res_cm * cm;
    config.offset = offset;
    config.boundaries = bounds;

    // If boundaries are all zero, compute from array shape.
    double bmin_x = bounds.min_x.force_numerical_value_in(cm);
    double bmax_x = bounds.max_x.force_numerical_value_in(cm);
    if (bmax_x <= bmin_x) {
        double off_x = offset.x.force_numerical_value_in(cm);
        double off_y = offset.y.force_numerical_value_in(cm);
        double off_z = offset.z.force_numerical_value_in(cm);
        const auto& shape = arr->Shape();

        config.boundaries.min_x = XLength(off_x * x_extent[cm]);
        config.boundaries.max_x = XLength((off_x + static_cast<double>(shape[0]) * map_res_cm) * x_extent[cm]);
        config.boundaries.min_y = YLength(off_y * y_extent[cm]);
        config.boundaries.max_y = YLength((off_y + static_cast<double>(shape[1]) * map_res_cm) * y_extent[cm]);
        config.boundaries.min_height = ZLength(off_z * z_extent[cm]);
        config.boundaries.max_height = ZLength((off_z + static_cast<double>(shape[2]) * map_res_cm) * z_extent[cm]);
    }

    return std::make_unique<Map3DImpl>(std::move(arr), config);
}

struct ComparisonMapConfig {
    double map_res_cm = 10.0;
    Position3D offset{};
    MappingBounds boundaries{};
};

ComparisonMapConfig parseMapSection(const YAML::Node& node) {
    ComparisonMapConfig cfg;
    cfg.map_res_cm = get_with_check<double>(node, "map_res_cm", 10.0, [](const double& v){ return v > 0.0; }, "map_res_cm must be > 0");

    if (node["map_offset"]) {
        auto off = node["map_offset"];
        cfg.offset = Position3D{
            XLength(get_with_default<double>(off, "x_offset", 0.0) * x_extent[cm]),
            YLength(get_with_default<double>(off, "y_offset", 0.0) * y_extent[cm]),
            ZLength(get_with_default<double>(off, "height_offset", 0.0) * z_extent[cm])
        };
    }
    if (node["map_boundaries"]) {
        auto b = node["map_boundaries"];
        if (b["x_boundary"]) {
            cfg.boundaries.min_x = XLength(get_with_default<double>(b["x_boundary"], "min_cm", 0.0) * x_extent[cm]);
            cfg.boundaries.max_x = XLength(get_with_default<double>(b["x_boundary"], "max_cm", 0.0) * x_extent[cm]);
        }
        if (b["y_boundary"]) {
            cfg.boundaries.min_y = YLength(get_with_default<double>(b["y_boundary"], "min_cm", 0.0) * y_extent[cm]);
            cfg.boundaries.max_y = YLength(get_with_default<double>(b["y_boundary"], "max_cm", 0.0) * y_extent[cm]);
        }
        if (b["height_boundary"]) {
            cfg.boundaries.min_height = ZLength(get_with_default<double>(b["height_boundary"], "min_cm", 0.0) * z_extent[cm]);
            cfg.boundaries.max_height = ZLength(get_with_default<double>(b["height_boundary"], "max_cm", 0.0) * z_extent[cm]);
        }
    }
    
    if (cfg.boundaries.max_x < cfg.boundaries.min_x) {
        throw std::invalid_argument("map_boundaries x_boundary max must be >= min");
    }
    if (cfg.boundaries.max_y < cfg.boundaries.min_y) {
        throw std::invalid_argument("map_boundaries y_boundary max must be >= min");
    }
    if (cfg.boundaries.max_height < cfg.boundaries.min_height) {
        throw std::invalid_argument("map_boundaries height_boundary max must be >= min");
    }

    return cfg;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 3 || argc > 4) {
        std::cout << "-1\n";
        std::cerr << "Usage: maps_comparison <origin_map> <target_map> [comparison_config=<path>]\n";
        return 1;
    }

    try {
        std::filesystem::path origin_path = argv[1];
        std::filesystem::path target_path = argv[2];

        ComparisonMapConfig origin_cfg;
        ComparisonMapConfig target_cfg;

        // Parse optional comparison config.
        if (argc == 4) {
            std::string arg3 = argv[3];
            std::string prefix = "comparison_config=";
            std::filesystem::path config_path;

            if (arg3.substr(0, prefix.size()) == prefix) {
                config_path = arg3.substr(prefix.size());
            } else {
                config_path = arg3;
            }

            YAML::Node config = YAML::LoadFile(config_path.string());
            YAML::Node comp = config["comparison_config"];

            if (comp["original"]) {
                origin_cfg = parseMapSection(comp["original"]);
            }
            if (comp["target"]) {
                target_cfg = parseMapSection(comp["target"]);
            }
        }

        // Load maps.
        // Apply boundaries inheritance fallback logic
        bool origin_has_bounds = (origin_cfg.boundaries.max_x > origin_cfg.boundaries.min_x);
        bool target_has_bounds = (target_cfg.boundaries.max_x > target_cfg.boundaries.min_x);

        if (!origin_has_bounds && target_has_bounds) {
            origin_cfg.boundaries = target_cfg.boundaries;
        } else if (origin_has_bounds && !target_has_bounds) {
            target_cfg.boundaries = origin_cfg.boundaries;
        }

        auto origin = loadMap(origin_path, origin_cfg.map_res_cm, origin_cfg.offset, origin_cfg.boundaries);
        auto target = loadMap(target_path, target_cfg.map_res_cm, target_cfg.offset, target_cfg.boundaries);

        // Compare.
        std::vector<IMap3D*> targets = {target.get()};
        std::vector<double> scores = MapsComparison::compare(*origin, targets);

        if (scores.empty()) {
            std::cout << "-1\n";
            std::cerr << "Comparison returned no scores.\n";
            return 1;
        }

        // Print just the score number (spec: floating point 0-100, no additional text).
        std::cout << scores[0] << "\n";
        return 0;

    } catch (const std::exception& e) {
        std::cout << "-1\n";
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
