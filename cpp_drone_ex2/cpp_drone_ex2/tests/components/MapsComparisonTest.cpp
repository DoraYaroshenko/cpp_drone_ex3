#include <gtest/gtest.h>
#include <drone_mapper/MapsComparison.h>
#include <drone_mapper/Types.h>
#include "../mocks/Mocks.h"
#include "../TestHelpers.h"

using namespace drone_mapper;
using namespace drone_mapper::types;
using namespace drone_mapper::tests;
using ::testing::_;
using ::testing::Return;
using ::testing::NiceMock;

namespace {
class MapsComparison : public ::testing::Test {
protected:
    NiceMock<tests::MockMap3D> orig_map;
    NiceMock<tests::MockMap3D> target_map;
    MapConfig config = makeTestMapConfig();

    void SetUp() override {
        config.boundaries.min_x = 0 * x_extent[cm];
        config.boundaries.max_x = 100 * x_extent[cm];
        config.boundaries.min_y = 0 * y_extent[cm];
        config.boundaries.max_y = 100 * y_extent[cm];
        config.boundaries.min_height = 0 * z_extent[cm];
        config.boundaries.max_height = 100 * z_extent[cm];
        config.resolution = 10.0 * cm;

        ON_CALL(orig_map, getMapConfig()).WillByDefault(Return(config));
        ON_CALL(target_map, getMapConfig()).WillByDefault(Return(config));
    }
};

TEST_F(MapsComparison, IdenticalMaps_Score100) {
    EXPECT_CALL(orig_map, atVoxel(_)).WillRepeatedly(Return(VoxelOccupancy::Occupied));
    EXPECT_CALL(target_map, atVoxel(_)).WillRepeatedly(Return(VoxelOccupancy::Occupied));

    auto scores = drone_mapper::MapsComparison::compare(orig_map, {&target_map});
    EXPECT_EQ(scores.size(), 1);
    EXPECT_DOUBLE_EQ(scores[0], 100.0);
}

TEST_F(MapsComparison, AllMismatch_Score0) {
    EXPECT_CALL(orig_map, atVoxel(_)).WillRepeatedly(Return(VoxelOccupancy::Occupied));
    EXPECT_CALL(target_map, atVoxel(_)).WillRepeatedly(Return(VoxelOccupancy::Empty));

    auto scores = drone_mapper::MapsComparison::compare(orig_map, {&target_map});
    EXPECT_DOUBLE_EQ(scores[0], 0.0);
}

TEST_F(MapsComparison, AllUnmapped_PartialCredit) {
    EXPECT_CALL(orig_map, atVoxel(_)).WillRepeatedly(Return(VoxelOccupancy::Occupied));
    EXPECT_CALL(target_map, atVoxel(_)).WillRepeatedly(Return(VoxelOccupancy::Unmapped));

    auto scores = drone_mapper::MapsComparison::compare(orig_map, {&target_map});
    EXPECT_GT(scores[0], 0.0);
    EXPECT_LE(scores[0], 50.0); 
}

TEST_F(MapsComparison, AllPotentiallyOccupied_PartialCredit) {
    EXPECT_CALL(orig_map, atVoxel(_)).WillRepeatedly(Return(VoxelOccupancy::Occupied));
    EXPECT_CALL(target_map, atVoxel(_)).WillRepeatedly(Return(VoxelOccupancy::PotentiallyOccupied));

    auto scores = drone_mapper::MapsComparison::compare(orig_map, {&target_map});
    EXPECT_GT(scores[0], 0.0);
    EXPECT_LE(scores[0], 50.0);
}

TEST_F(MapsComparison, OutOfBounds_ReturnsMinusOne) {
    EXPECT_CALL(orig_map, atVoxel(_)).WillRepeatedly(Return(VoxelOccupancy::Occupied));
    EXPECT_CALL(target_map, atVoxel(_)).WillRepeatedly(Return(VoxelOccupancy::OutOfBounds));

    auto scores = drone_mapper::MapsComparison::compare(orig_map, {&target_map});
    EXPECT_DOUBLE_EQ(scores[0], -1.0);
}

TEST_F(MapsComparison, ZeroResolution_ReturnsMinusOne) {
    config.resolution = 0.0 * cm;
    EXPECT_CALL(orig_map, getMapConfig()).WillRepeatedly(Return(config));

    auto scores = drone_mapper::MapsComparison::compare(orig_map, {&target_map});
    EXPECT_DOUBLE_EQ(scores[0], -1.0);
}

TEST_F(MapsComparison, NullTarget_ReturnsMinusOne) {
    auto scores = drone_mapper::MapsComparison::compare(orig_map, {nullptr});
    EXPECT_DOUBLE_EQ(scores[0], -1.0);
}

TEST_F(MapsComparison, MultipleTargets_IndependentScores) {
    NiceMock<tests::MockMap3D> tgt2;
    ON_CALL(tgt2, getMapConfig()).WillByDefault(Return(config));

    EXPECT_CALL(orig_map, atVoxel(_)).WillRepeatedly(Return(VoxelOccupancy::Occupied));
    EXPECT_CALL(target_map, atVoxel(_)).WillRepeatedly(Return(VoxelOccupancy::Occupied)); 
    EXPECT_CALL(tgt2, atVoxel(_)).WillRepeatedly(Return(VoxelOccupancy::Empty)); 

    auto scores = drone_mapper::MapsComparison::compare(orig_map, {&target_map, &tgt2});
    EXPECT_EQ(scores.size(), 2);
    EXPECT_DOUBLE_EQ(scores[0], 100.0);
    EXPECT_DOUBLE_EQ(scores[1], 0.0);
}

TEST_F(MapsComparison, MixedValues_CorrectPercentage) {
    config.boundaries.max_x = 400.0 * x_extent[cm]; 
    EXPECT_CALL(orig_map, getMapConfig()).WillRepeatedly(Return(config));
    EXPECT_CALL(target_map, getMapConfig()).WillRepeatedly(Return(config));
    
    EXPECT_CALL(orig_map, atVoxel(_)).WillRepeatedly(Return(VoxelOccupancy::Occupied));
    EXPECT_CALL(target_map, atVoxel(_)).WillRepeatedly([](const Position3D& pos) {
        double x = pos.x.numerical_value_in(cm);
        if (x < 100.0) return VoxelOccupancy::Occupied;
        if (x < 200.0) return VoxelOccupancy::Unmapped;
        if (x < 300.0) return VoxelOccupancy::Empty;
        return VoxelOccupancy::Occupied;
    });

    auto scores = drone_mapper::MapsComparison::compare(orig_map, {&target_map});
    EXPECT_GT(scores[0], 40.0);
    EXPECT_LT(scores[0], 70.0);
}

TEST_F(MapsComparison, DifferentOffsets_Comparable_VoxelWise) {
    config.boundaries.min_x = 0 * x_extent[cm];
    config.boundaries.max_x = 5 * x_extent[cm];
    config.boundaries.min_y = 0 * y_extent[cm];
    config.boundaries.max_y = 5 * y_extent[cm];
    config.boundaries.min_height = 0 * z_extent[cm];
    config.boundaries.max_height = 5 * z_extent[cm];
    config.resolution = 1.0 * cm;
    EXPECT_CALL(orig_map, getMapConfig()).WillRepeatedly(Return(config));

    MapConfig target_config = config;
    target_config.boundaries.min_x = -6 * x_extent[cm];
    target_config.boundaries.max_x = -1 * x_extent[cm];
    target_config.boundaries.min_y = -6 * y_extent[cm];
    target_config.boundaries.max_y = -1 * y_extent[cm];
    target_config.boundaries.min_height = -9 * z_extent[cm];
    target_config.boundaries.max_height = -4 * z_extent[cm];
    EXPECT_CALL(target_map, getMapConfig()).WillRepeatedly(Return(target_config));

    EXPECT_CALL(orig_map, atVoxel(_)).WillRepeatedly(Return(VoxelOccupancy::Occupied));
    EXPECT_CALL(target_map, atVoxel(_)).WillRepeatedly([](const Position3D& pos) {
        double x = pos.x.numerical_value_in(cm);
        double y = pos.y.numerical_value_in(cm);
        double z = pos.z.numerical_value_in(cm);
        
        EXPECT_GE(x, -6.0);
        EXPECT_LT(x, -1.0);
        EXPECT_GE(y, -6.0);
        EXPECT_LT(y, -1.0);
        EXPECT_GE(z, -9.0);
        EXPECT_LT(z, -4.0);
        return VoxelOccupancy::Occupied;
    });

    auto scores = drone_mapper::MapsComparison::compare(orig_map, {&target_map});
    EXPECT_DOUBLE_EQ(scores[0], 100.0);
}

TEST_F(MapsComparison, DifferentOffsets_Larger_Comparable_VoxelWise) {
    config.boundaries.min_x = 0 * x_extent[cm];
    config.boundaries.max_x = 20 * x_extent[cm];
    config.boundaries.min_y = 0 * y_extent[cm];
    config.boundaries.max_y = 20 * y_extent[cm];
    config.boundaries.min_height = 0 * z_extent[cm];
    config.boundaries.max_height = 20 * z_extent[cm];
    config.resolution = 1.0 * cm;
    EXPECT_CALL(orig_map, getMapConfig()).WillRepeatedly(Return(config));

    MapConfig target_config = config;
    target_config.boundaries.min_x = -100 * x_extent[cm];
    target_config.boundaries.max_x = -80 * x_extent[cm];
    target_config.boundaries.min_y = -100 * y_extent[cm];
    target_config.boundaries.max_y = -80 * y_extent[cm];
    target_config.boundaries.min_height = -100 * z_extent[cm];
    target_config.boundaries.max_height = -80 * z_extent[cm];
    EXPECT_CALL(target_map, getMapConfig()).WillRepeatedly(Return(target_config));

    EXPECT_CALL(orig_map, atVoxel(_)).WillRepeatedly(Return(VoxelOccupancy::Occupied));
    EXPECT_CALL(target_map, atVoxel(_)).WillRepeatedly(Return(VoxelOccupancy::Occupied));

    auto scores = drone_mapper::MapsComparison::compare(orig_map, {&target_map});
    EXPECT_DOUBLE_EQ(scores[0], 100.0);
}

TEST_F(MapsComparison, MismatchedVoxelDimensions_ReturnsMinusOne) {
    EXPECT_CALL(orig_map, getMapConfig()).WillRepeatedly(Return(config));

    MapConfig target_config = config;
    target_config.boundaries.max_height = 50 * z_extent[cm];
    EXPECT_CALL(target_map, getMapConfig()).WillRepeatedly(Return(target_config));

    EXPECT_CALL(orig_map, atVoxel(_)).Times(0);
    EXPECT_CALL(target_map, atVoxel(_)).Times(0);

    auto scores = drone_mapper::MapsComparison::compare(orig_map, {&target_map});
    EXPECT_DOUBLE_EQ(scores[0], -1.0);
}

TEST_F(MapsComparison, DifferentMapSizes_MatchingBoundaries) {
    EXPECT_CALL(orig_map, getMapConfig()).WillRepeatedly(Return(config));
    EXPECT_CALL(target_map, getMapConfig()).WillRepeatedly(Return(config));

    EXPECT_CALL(orig_map, atVoxel(_)).WillRepeatedly(Return(VoxelOccupancy::Occupied));
    EXPECT_CALL(target_map, atVoxel(_)).WillRepeatedly(Return(VoxelOccupancy::Occupied));

    auto scores = drone_mapper::MapsComparison::compare(orig_map, {&target_map});
    EXPECT_DOUBLE_EQ(scores[0], 100.0);
}

} // namespace
