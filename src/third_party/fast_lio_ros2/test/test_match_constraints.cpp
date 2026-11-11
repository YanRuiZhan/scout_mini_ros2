#include "imu_gap_monitor.hpp"
#include "match_constraints.hpp"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>

namespace
{
struct Point
{
    float x, y, z;
};

const double MAP_VOXEL = 0.2;          // filter_size_map of the simulation config
const double MIN_SPREAD = 0.05;        // mapping.plane_min_spread of the simulation config
const double LASER_COV = 0.001;        // LASER_POINT_COV
const double DEG = M_PI / 180.0;

// Place 2-D pattern coordinates (u, v) on a vertical wall far from the map origin.
std::vector<Point> on_wall(const std::vector<Eigen::Vector2d> &pattern, const std::vector<double> &depth_noise = {})
{
    const Eigen::Vector3d origin(31.0, -17.0, 4.0);
    const Eigen::Vector3d along = Eigen::Vector3d(1.0, 2.0, 0.0).normalized();
    const Eigen::Vector3d up(0.0, 0.0, 1.0);
    const Eigen::Vector3d normal = along.cross(up);
    std::vector<Point> points;
    for (std::size_t i = 0; i < pattern.size(); i++)
    {
        const double noise = depth_noise.empty() ? 0.0 : depth_noise[i];
        const Eigen::Vector3d p = origin + pattern[i].x() * along + pattern[i].y() * up + noise * normal;
        points.push_back({static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z())});
    }
    return points;
}

Eigen::Matrix3d attitude(double roll, double pitch, double yaw)
{
    return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

TEST(NeighborSpread, OneScanLineHasNoCrossSpread)
{
    // Five map points left by one LiDAR ring on a wall, one per map voxel.
    const auto line = on_wall({{0.0, 0.0}, {0.2, 0.0}, {0.4, 0.0}, {0.6, 0.0}, {0.8, 0.0}});
    EXPECT_LT(neighbor_cross_spread(line, 5), 1e-3);
}

TEST(NeighborSpread, RangeNoiseDoesNotTurnALineIntoAPlane)
{
    // 1 cm range noise (the simulated sensor) is the only extent across the line.
    const auto line = on_wall({{0.0, 0.0}, {0.2, 0.0}, {0.4, 0.0}, {0.6, 0.0}, {0.8, 0.0}},
                              {0.012, -0.009, 0.015, -0.011, 0.004});
    const double spread = neighbor_cross_spread(line, 5);
    EXPECT_GT(spread, 0.005);
    EXPECT_LT(spread, MIN_SPREAD);
}

TEST(NeighborSpread, PatchAtMapResolutionIsAccepted)
{
    // The most compact real patch the voxel map can hold: 3 + 2 points on adjacent rows.
    const auto patch = on_wall({{0.0, 0.0}, {MAP_VOXEL, 0.0}, {2 * MAP_VOXEL, 0.0}, {0.0, MAP_VOXEL}, {MAP_VOXEL, MAP_VOXEL}});
    EXPECT_NEAR(neighbor_cross_spread(patch, 5), std::sqrt(0.008), 1e-3);
    EXPECT_GT(neighbor_cross_spread(patch, 5), MIN_SPREAD);
}

TEST(NeighborSpread, TwoAdjacentScanLinesAreAccepted)
{
    // Four points on one ring and one on the next (rings 0.35 m apart at ~10 m).
    const auto rows = on_wall({{0.0, 0.0}, {0.2, 0.0}, {0.4, 0.0}, {0.6, 0.0}, {0.3, 0.35}});
    EXPECT_GT(neighbor_cross_spread(rows, 5), MIN_SPREAD);
}

TEST(NeighborSpread, TooFewPointsNeverSpanAPlane)
{
    const auto two = on_wall({{0.0, 0.0}, {0.0, 1.0}});
    EXPECT_EQ(neighbor_cross_spread(two, 2), 0.0);
}

TEST(LevelConstraint, LevelBodyHasZeroResidualAtAnyYaw)
{
    for (double yaw : {0.0, 0.7, -2.5})
    {
        Eigen::MatrixXd h_x = Eigen::MatrixXd::Constant(5, 12, 9.0);
        Eigen::VectorXd h = Eigen::VectorXd::Constant(5, 9.0);
        write_level_constraint(attitude(0.0, 0.0, yaw), LASER_COV, 0.01 * DEG, 3, h_x, h);
        EXPECT_NEAR(h(3), 0.0, 1e-9);
        EXPECT_NEAR(h(4), 0.0, 1e-9);
        // Only the rotation columns are written, and the rows above are untouched.
        EXPECT_EQ(h_x.block(3, 0, 2, 3).norm(), 0.0);
        EXPECT_EQ(h_x.block(3, 6, 2, 6).norm(), 0.0);
        EXPECT_EQ(h_x(2, 4), 9.0);
        EXPECT_EQ(h(2), 9.0);
        // Rotating about the body z axis (yaw) is not constrained.
        EXPECT_EQ(h_x.block(3, 5, 2, 1).norm(), 0.0);
    }
}

TEST(LevelConstraint, JacobianMatchesFiniteDifferences)
{
    const Eigen::Matrix3d rot = attitude(0.03, -0.05, 1.1);
    const double meas_cov = 1.0, std_rad = 1.0;   // unit scale: compare raw model
    Eigen::MatrixXd h_x = Eigen::MatrixXd::Zero(2, 12);
    Eigen::VectorXd h(2);
    write_level_constraint(rot, meas_cov, std_rad, 0, h_x, h);

    const double eps = 1e-6;
    for (int axis = 0; axis < 3; axis++)
    {
        const Eigen::Vector3d delta = eps * Eigen::Vector3d::Unit(axis);
        const Eigen::Matrix3d perturbed = rot * Eigen::AngleAxisd(eps, Eigen::Vector3d::Unit(axis)).toRotationMatrix();
        Eigen::MatrixXd unused = Eigen::MatrixXd::Zero(2, 12);
        Eigen::VectorXd h_perturbed(2);
        write_level_constraint(perturbed, meas_cov, std_rad, 0, unused, h_perturbed);
        // h is the innovation z - h(x), so it changes by -h_x * delta.
        const Eigen::Vector2d predicted = -h_x.block(0, 3, 2, 3) * delta;
        EXPECT_NEAR((h_perturbed - h)(0), predicted(0), 1e-10);
        EXPECT_NEAR((h_perturbed - h)(1), predicted(1), 1e-10);
    }
}

TEST(LevelConstraint, RowsCarryTheirOwnStandardDeviationUnderTheSharedCovariance)
{
    const double std_rad = 0.005 * DEG;
    Eigen::MatrixXd h_x = Eigen::MatrixXd::Zero(2, 12);
    Eigen::VectorXd h(2);
    write_level_constraint(attitude(0.0, 0.0, 0.4), LASER_COV, std_rad, 0, h_x, h);
    // Information the filter sees: H^T H / R. Tilt axes get 1 / std^2, yaw gets none.
    const Eigen::Matrix3d info = h_x.block(0, 3, 2, 3).transpose() * h_x.block(0, 3, 2, 3) / LASER_COV;
    EXPECT_NEAR(info(0, 0) * std_rad * std_rad, 1.0, 1e-9);
    EXPECT_NEAR(info(1, 1) * std_rad * std_rad, 1.0, 1e-9);
    EXPECT_NEAR(info(2, 2), 0.0, 1e-9);
    EXPECT_NEAR(info(0, 1), 0.0, 1e-3);
}

TEST(LevelConstraint, GaussNewtonRemovesTiltAndKeepsYaw)
{
    Eigen::Matrix3d rot = attitude(2.0 * DEG, -3.5 * DEG, 0.9);
    const double yaw_before = std::atan2(rot(1, 0), rot(0, 0));
    for (int iteration = 0; iteration < 4; iteration++)
    {
        Eigen::MatrixXd h_x = Eigen::MatrixXd::Zero(2, 12);
        Eigen::VectorXd h(2);
        write_level_constraint(rot, LASER_COV, 0.005 * DEG, 0, h_x, h);
        const Eigen::MatrixXd jac = h_x.block(0, 3, 2, 3);
        // Minimum-norm step: the unobserved yaw direction stays at zero.
        const Eigen::Vector3d delta = jac.completeOrthogonalDecomposition().solve(h);
        rot = rot * Eigen::AngleAxisd(delta.norm(), delta.normalized()).toRotationMatrix();
    }
    EXPECT_NEAR(rot(0, 2), 0.0, 1e-9);
    EXPECT_NEAR(rot(1, 2), 0.0, 1e-9);
    EXPECT_NEAR(rot(2, 2), 1.0, 1e-9);
    EXPECT_NEAR(std::atan2(rot(1, 0), rot(0, 0)), yaw_before, 2e-3);
}

TEST(ImuGapMonitor, RegularStreamWithJitterHasNoGaps)
{
    ImuGapMonitor monitor;
    double stamp = 100.0;
    for (int i = 0; i < 2000; i++)
    {
        stamp += 0.005 * (i % 2 ? 1.1 : 0.9);
        EXPECT_EQ(monitor.update(stamp), 0.0);
    }
    EXPECT_EQ(monitor.gaps(), 0u);
    EXPECT_NEAR(monitor.nominal_period(), 0.005, 0.0005);
}

TEST(ImuGapMonitor, CountsDroppedSamples)
{
    ImuGapMonitor monitor;
    int reported = 0;
    for (int i = 0; i < 1000; i++)
    {
        if (i >= 400 && i < 410) continue;   // ten samples lost: one 55 ms interval
        if (i == 700) continue;              // one sample lost: one 10 ms interval
        const double gap = monitor.update(100.0 + i * 0.005);
        if (gap > 0.0) reported++;
    }
    EXPECT_EQ(reported, 2);
    EXPECT_EQ(monitor.gaps(), 2u);
    EXPECT_EQ(monitor.lost_samples(), 11u);
    EXPECT_NEAR(monitor.max_gap(), 0.055, 1e-9);
}

TEST(ImuGapMonitor, ClockResetAndDuplicatesAreNotGaps)
{
    ImuGapMonitor monitor;
    for (int i = 0; i < 100; i++) monitor.update(50.0 + i * 0.005);
    EXPECT_EQ(monitor.update(50.0 + 99 * 0.005), 0.0);   // duplicate
    EXPECT_EQ(monitor.update(1.0), 0.0);                 // simulation reset
    for (int i = 1; i < 100; i++) EXPECT_EQ(monitor.update(1.0 + i * 0.005), 0.0);
    EXPECT_EQ(monitor.gaps(), 0u);
}

TEST(ImuGapMonitor, RecoversWhenTheFirstIntervalIsNotNominal)
{
    ImuGapMonitor late;      // first interval is itself a gap
    late.update(10.0);
    late.update(10.2);
    for (int i = 1; i <= 100; i++) late.update(10.2 + i * 0.005);
    EXPECT_NEAR(late.nominal_period(), 0.005, 0.0005);
    EXPECT_GT(late.update(10.2 + 100 * 0.005 + 0.05), 0.0);

    ImuGapMonitor early;     // first interval is unusually short
    early.update(10.0);
    early.update(10.001);
    for (int i = 1; i <= 100; i++) early.update(10.001 + i * 0.005);
    EXPECT_NEAR(early.nominal_period(), 0.005, 0.0005);
    EXPECT_LE(early.gaps(), 4u);
}
}  // namespace
