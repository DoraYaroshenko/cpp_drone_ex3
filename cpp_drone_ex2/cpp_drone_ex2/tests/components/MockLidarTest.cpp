#include <gtest/gtest.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/Types.h>
#include "../mocks/Mocks.h"
#include "../TestHelpers.h"
#include <cmath>
#include <limits>

using namespace drone_mapper;
using namespace drone_mapper::types;
using namespace drone_mapper::tests;
using ::testing::_;
using ::testing::Return;
using ::testing::NiceMock;

namespace {
class MockLidar : public ::testing::Test {
protected:
    LidarConfigData lidar_config = makeTestLidarConfig();
    NiceMock<tests::MockMap3D> hidden_map;
    NiceMock<tests::MockGPS> gps;

    void SetUp() override {
        ON_CALL(gps, position()).WillByDefault(Return(Position3D{0*x_extent[cm], 0*y_extent[cm], 0*z_extent[cm]}));
        ON_CALL(gps, heading()).WillByDefault(Return(Orientation{0*deg, 0*deg}));
        ON_CALL(hidden_map, atVoxel(_)).WillByDefault(Return(VoxelOccupancy::Empty));
        ON_CALL(hidden_map, isInBounds(_)).WillByDefault(Return(true));
        
        MapConfig cfg = makeTestMapConfig();
        cfg.resolution = 1.0*cm; 
        ON_CALL(hidden_map, getMapConfig()).WillByDefault(Return(cfg));
    }
};

TEST_F(MockLidar, EmptyMap_AllMisses) {
    drone_mapper::MockLidar lidar(lidar_config, hidden_map, gps);
    auto res = lidar.scan(Orientation{0*deg, 0*deg});
    
    EXPECT_FALSE(res.empty());
    for(const auto& hit : res) {
        EXPECT_EQ(hit.distance.numerical_value_in(cm), std::numeric_limits<double>::max());
    }
}

TEST_F(MockLidar, OccupiedVoxelInRange_ReturnsDistance) {
    EXPECT_CALL(hidden_map, atVoxel(_)).WillRepeatedly([](const Position3D& pos) {
        if(pos.x.numerical_value_in(cm) > 40.0) {
            return VoxelOccupancy::Occupied;
        }
        return VoxelOccupancy::Empty;
    });

    drone_mapper::MockLidar lidar(lidar_config, hidden_map, gps);
    auto res = lidar.scan(Orientation{0*deg, 0*deg});
    
    EXPECT_NEAR(res[0].distance.numerical_value_in(cm), 40.0, 2.0); 
}

TEST_F(MockLidar, OccupiedVoxelBelowZmin_ReturnsZero) {
    EXPECT_CALL(hidden_map, atVoxel(_)).WillRepeatedly([](const Position3D& pos) {
        if(pos.x.numerical_value_in(cm) > 10.0) {
            return VoxelOccupancy::Occupied;
        }
        return VoxelOccupancy::Empty;
    });

    drone_mapper::MockLidar lidar(lidar_config, hidden_map, gps);
    auto res = lidar.scan(Orientation{0*deg, 0*deg});
    
    EXPECT_DOUBLE_EQ(res[0].distance.numerical_value_in(cm), 0.0);
}

TEST_F(MockLidar, OccupiedVoxelBeyondZmax_ReturnsMiss) {
    EXPECT_CALL(hidden_map, atVoxel(_)).WillRepeatedly([](const Position3D& pos) {
        if(pos.x.numerical_value_in(cm) > 150.0) {
            return VoxelOccupancy::Occupied;
        }
        return VoxelOccupancy::Empty;
    });

    drone_mapper::MockLidar lidar(lidar_config, hidden_map, gps);
    auto res = lidar.scan(Orientation{0*deg, 0*deg});
    
    EXPECT_EQ(res[0].distance.numerical_value_in(cm), std::numeric_limits<double>::max());
}

TEST_F(MockLidar, BeamCount_MatchesFOVCircles) {
    lidar_config.fov_circles = 5;
    drone_mapper::MockLidar lidar(lidar_config, hidden_map, gps);
    auto res = lidar.scan(Orientation{0*deg, 0*deg});
    
    EXPECT_EQ(res.size(), 341);
    
    lidar_config.fov_circles = 2;
    drone_mapper::MockLidar lidar2(lidar_config, hidden_map, gps);
    auto res2 = lidar2.scan(Orientation{0*deg, 0*deg});
    
    EXPECT_EQ(res2.size(), 5);
}

TEST_F(MockLidar, ComplexGeometryAndOrientations_CorrectHits) {
    lidar_config.fov_circles = 3;
    lidar_config.d = 10.0 * cm;
    lidar_config.z_min = 100.0 * cm;
    lidar_config.z_max = 200.0 * cm;
    
    EXPECT_CALL(gps, position()).WillRepeatedly(Return(Position3D{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]}));
    EXPECT_CALL(gps, heading()).WillRepeatedly(Return(Orientation{45.0 * deg, 0.0 * deg}));
    
    EXPECT_CALL(hidden_map, atVoxel(_)).WillRepeatedly([](const Position3D& pos) {
        double px = pos.x.numerical_value_in(cm);
        double py = pos.y.numerical_value_in(cm);
        double pz = pos.z.numerical_value_in(cm);
        
        auto is_near = [&](double x, double y, double z) {
            return std::abs(px - x) < 1.0 && std::abs(py - y) < 1.0 && std::abs(pz - z) < 1.0;
        };

        if (is_near(153.53, 109.77, 60.46)) return VoxelOccupancy::Occupied; // Center
        if (is_near(163.24, 131.41, 62.20)) return VoxelOccupancy::Occupied; // C1 B0
        if (is_near(118.08, 89.30, 64.87)) return VoxelOccupancy::Occupied;  // C1 B1 (too close)
        if (is_near(205.87, 139.99, 47.77)) return VoxelOccupancy::Occupied; // C1 B3
        if (is_near(132.31, 122.34, 59.59)) return VoxelOccupancy::Occupied; // C2 B0
        if (is_near(180.35, 112.52, 90.01)) return VoxelOccupancy::Occupied; // C2 B5
        if (is_near(194.54, 173.31, 52.07)) return VoxelOccupancy::Occupied; // C2 B15
        
        return VoxelOccupancy::Empty;
    });

    drone_mapper::MockLidar lidar(lidar_config, hidden_map, gps);
    auto res = lidar.scan(Orientation{-15.0 * deg, 5.0 * deg});
    
    ASSERT_EQ(res.size(), 21);
    
    // Assert Distances
    EXPECT_NEAR(res[0].distance.numerical_value_in(cm), 120.0, 1.5);
    EXPECT_NEAR(res[1].distance.numerical_value_in(cm), 140.0, 1.5);
    EXPECT_EQ(res[2].distance.numerical_value_in(cm), 0.0);
    EXPECT_EQ(res[3].distance.numerical_value_in(cm), std::numeric_limits<double>::max());
    EXPECT_NEAR(res[4].distance.numerical_value_in(cm), 180.0, 1.5);
    
    EXPECT_NEAR(res[5].distance.numerical_value_in(cm), 110.0, 1.5);
    EXPECT_NEAR(res[10].distance.numerical_value_in(cm), 150.0, 1.5);
    EXPECT_NEAR(res[20].distance.numerical_value_in(cm), 190.0, 1.5);
    
    // Assert Orientations (relative to sensor heading)
    EXPECT_NEAR(res[0].angle.horizontal.numerical_value_in(deg), -15.00, 0.1);
    EXPECT_NEAR(res[0].angle.altitude.numerical_value_in(deg), 5.00, 0.1);
    
    EXPECT_NEAR(res[1].angle.horizontal.numerical_value_in(deg), -9.29, 0.1);
    EXPECT_NEAR(res[1].angle.altitude.numerical_value_in(deg), 5.00, 0.1);
    
    EXPECT_NEAR(res[2].angle.horizontal.numerical_value_in(deg), -15.00, 0.1);
    EXPECT_NEAR(res[2].angle.altitude.numerical_value_in(deg), 10.71, 0.1);
    
    EXPECT_NEAR(res[4].angle.horizontal.numerical_value_in(deg), -15.00, 0.1);
    EXPECT_NEAR(res[4].angle.altitude.numerical_value_in(deg), -0.71, 0.1);
    
    EXPECT_NEAR(res[5].angle.horizontal.numerical_value_in(deg), -3.69, 0.1);
    EXPECT_NEAR(res[5].angle.altitude.numerical_value_in(deg), 5.00, 0.1);

    EXPECT_NEAR(res[10].angle.horizontal.numerical_value_in(deg), -19.38, 0.1);
    EXPECT_NEAR(res[10].angle.altitude.numerical_value_in(deg), 15.47, 0.1);
    
    EXPECT_NEAR(res[20].angle.horizontal.numerical_value_in(deg), -4.53, 0.1);
    EXPECT_NEAR(res[20].angle.altitude.numerical_value_in(deg), 0.62, 0.1);
}

} // namespace
