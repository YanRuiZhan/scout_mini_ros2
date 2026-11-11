#include <rclcpp/rclcpp.hpp>
#include "IMU_Processing.hpp"
#include "map_accumulator.hpp"
#include "match_constraints.hpp"

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <unistd.h>

namespace
{
PointCloudXYZI scan(float x)
{
    PointCloudXYZI cloud;
    PointType point{};
    point.x = x;
    point.y = 2.0f;
    point.z = 3.0f;
    point.intensity = 4.0f;
    cloud.push_back(point);
    return cloud;
}

TEST(ImuProcessing, MissingImuClearsOldScanAndDoesNotChangeState)
{
    ImuProcess processor;
    esekfom::esekf<state_ikfom, 12, input_ikfom> filter;
    const auto before = filter.get_x();
    MeasureGroup measurement;
    *measurement.lidar = scan(10.0f);
    PointCloudXYZI::Ptr output(new PointCloudXYZI(scan(1.0f)));

    EXPECT_FALSE(processor.Process(measurement, filter, output));
    EXPECT_TRUE(output->empty());
    EXPECT_TRUE(filter.get_x().pos.isApprox(before.pos));
    EXPECT_TRUE(filter.get_x().vel.isApprox(before.vel));
    EXPECT_TRUE(filter.get_x().rot.matrix().isApprox(before.rot.matrix()));
}

TEST(ImuProcessing, InitializationAlsoClearsOldScanAndReportsNoOutput)
{
    ImuProcess processor;
    processor.set_extrinsic(V3D::Zero(), M3D::Identity());
    esekfom::esekf<state_ikfom, 12, input_ikfom> filter;
    MeasureGroup measurement;
    *measurement.lidar = scan(10.0f);
    auto imu = std::make_shared<sensor_msgs::msg::Imu>();
    imu->linear_acceleration.z = G_m_s2;
    measurement.imu.push_back(imu);
    PointCloudXYZI::Ptr output(new PointCloudXYZI(scan(1.0f)));

    EXPECT_FALSE(processor.Process(measurement, filter, output));
    EXPECT_TRUE(output->empty());
}

TEST(MapAccumulation, KeepsEveryFrameBetweenLowFrequencyPublications)
{
    MapAccumulator map;
    const auto copy = [](const PointType *in, PointType *out) { *out = *in; };
    // Ten 10 Hz scans must all be present before the first 1 Hz publication.
    for (int i = 1; i <= 10; ++i)
        map.append(scan(static_cast<float>(i)), i * 0.1, copy);

    ASSERT_EQ(map.cloud().size(), 10u);
    for (std::size_t i = 0; i < map.cloud().size(); ++i)
        EXPECT_FLOAT_EQ(map.cloud()[i].x, static_cast<float>(i + 1));
    EXPECT_DOUBLE_EQ(map.stamp(), 1.0);
}

TEST(MapAccumulation, SnapshotsKeepEachScansPoseAndNeverAppendCachedFrames)
{
    MapAccumulator map;
    float translation = 10.0f;
    const auto transform = [&translation](const PointType *in, PointType *out) {
        *out = *in;
        out->x += translation;
    };
    auto current_scan = scan(1.0f);
    map.append(current_scan, 1.0, transform);
    translation = 20.0f;
    map.append(current_scan, 1.1, transform);
    // Subsequent changes to the live pose/scan cannot change stored points.
    translation = 100.0f;
    current_scan[0].x = -1.0f;
    for (int publication = 0; publication < 3; ++publication)
    {
        ASSERT_EQ(map.cloud().size(), 2u);
        EXPECT_FLOAT_EQ(map.cloud()[0].x, 11.0f);
        EXPECT_FLOAT_EQ(map.cloud()[1].x, 21.0f);
        EXPECT_FLOAT_EQ(map.cloud()[1].intensity, 4.0f);
        EXPECT_DOUBLE_EQ(map.stamp(), 1.1);
    }
    map.append(PointCloudXYZI(), 2.0, transform);
    EXPECT_EQ(map.cloud().size(), 2u);
    EXPECT_DOUBLE_EQ(map.stamp(), 1.1);
}

TEST(MapAccumulation, PcdContainsNewFramesWithoutWaitingForPublication)
{
    MapAccumulator map;
    const auto copy = [](const PointType *in, PointType *out) { *out = *in; };
    map.append(scan(1.0f), 0.1, copy);
    map.append(scan(2.0f), 0.2, copy);
    char filename[] = "/tmp/fast_lio_map_test_XXXXXX";
    const int fd = mkstemp(filename);
    ASSERT_NE(fd, -1);
    close(fd);

    const int saved = pcl::io::savePCDFileBinary(filename, map.cloud());
    PointCloudXYZI loaded;
    const int read = pcl::io::loadPCDFile(filename, loaded);
    unlink(filename);
    ASSERT_EQ(saved, 0);
    ASSERT_EQ(read, 0);
    ASSERT_EQ(loaded.size(), 2u);
    EXPECT_FLOAT_EQ(loaded[0].x, 1.0f);
    EXPECT_FLOAT_EQ(loaded[1].x, 2.0f);
}

// Five map points left on a wall (x = 12 m) by one ring of a 16-line LiDAR,
// one per 0.2 m map voxel, with range noise displacing them along their rays.
PointVector scan_line_on_wall(double elevation_deg)
{
    const double wall_x = 12.0, sensor_z = 0.3;
    const double elevation = elevation_deg * M_PI / 180.0;
    const double range_noise[NUM_MATCH_POINTS] = {0.012, -0.009, 0.015, -0.011, 0.004};
    PointVector points;
    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        const double azimuth = std::atan2(0.2 * (j - 2), wall_x);
        const double range = wall_x / (std::cos(elevation) * std::cos(azimuth)) + range_noise[j];
        PointType point{};
        point.x = range * std::cos(elevation) * std::cos(azimuth);
        point.y = range * std::cos(elevation) * std::sin(azimuth);
        point.z = sensor_z + range * std::sin(elevation);
        points.push_back(point);
    }
    return points;
}

TEST(PlaneMatching, SingleScanLineOnAWallFitsANearHorizontalPlane)
{
    // Root cause of the map tilting on a flat floor: collinear neighbours let
    // range noise pick the plane, and the fit passes esti_plane's inlier test.
    const PointVector line = scan_line_on_wall(9.0);
    VF(4) plane;
    ASSERT_TRUE(esti_plane(plane, line, 0.1f));
    EXPECT_LT(std::fabs(plane(0)), 0.3f);   // the wall's true normal is +/-x
    EXPECT_GT(std::fabs(plane(2)), 0.9f);   // the fit is close to horizontal instead

    // The spread guard recognises that these neighbours do not define a plane.
    EXPECT_LT(neighbor_cross_spread(line, NUM_MATCH_POINTS), 0.05);
}

TEST(PlaneMatching, TwoScanLinesOnAWallFitTheWallAndPassTheGuard)
{
    PointVector patch = scan_line_on_wall(9.0);
    const PointVector upper = scan_line_on_wall(11.0);
    patch[0] = upper[1];
    patch[4] = upper[3];
    VF(4) plane;
    ASSERT_TRUE(esti_plane(plane, patch, 0.1f));
    EXPECT_GT(std::fabs(plane(0)), 0.99f);
    EXPECT_GT(neighbor_cross_spread(patch, NUM_MATCH_POINTS), 0.05);
}

const double TEST_LASER_COV = 0.001;
const double TEST_LEVEL_STD = 0.005 * M_PI / 180.0;

void level_only_measurement(state_ikfom &s, esekfom::dyn_share_datastruct<double> &data)
{
    data.h_x = Eigen::MatrixXd::Zero(LEVEL_CONSTRAINT_ROWS, 12);
    data.h.resize(LEVEL_CONSTRAINT_ROWS);
    write_level_constraint(s.rot.toRotationMatrix(), TEST_LASER_COV, TEST_LEVEL_STD, 0, data.h_x, data.h);
}

TEST(LevelConstraint, IteratedFilterUpdateLevelsTheBodyAndKeepsYawAndPosition)
{
    esekfom::esekf<state_ikfom, 12, input_ikfom> filter;
    double epsi[23];
    std::fill(epsi, epsi + 23, 0.001);
    filter.init_dyn_share(get_f, df_dx, df_dw, level_only_measurement, 5, epsi);

    state_ikfom state = filter.get_x();
    const Eigen::Quaterniond tilted = Eigen::AngleAxisd(0.8, Eigen::Vector3d::UnitZ()) *
                                      Eigen::AngleAxisd(-0.04, Eigen::Vector3d::UnitY()) *
                                      Eigen::AngleAxisd(0.03, Eigen::Vector3d::UnitX());
    state.rot = SO3(tilted);
    state.pos = vect3(Eigen::Vector3d(1.0, -2.0, 0.5));
    filter.change_x(state);
    auto covariance = filter.get_P();
    covariance.setIdentity();
    covariance *= 1e-4;   // 0.57 deg prior on attitude: far looser than the constraint
    filter.change_P(covariance);

    double solve_time = 0.0;
    filter.update_iterated_dyn_share_modified(TEST_LASER_COV, solve_time);

    const Eigen::Matrix3d rot = filter.get_x().rot.toRotationMatrix();
    EXPECT_NEAR(rot(0, 2), 0.0, 1e-5);
    EXPECT_NEAR(rot(1, 2), 0.0, 1e-5);
    EXPECT_NEAR(std::atan2(rot(1, 0), rot(0, 0)), 0.8, 2e-3);
    EXPECT_TRUE(filter.get_x().pos.isApprox(state.pos, 1e-9));
    // Tilt is now known to the constraint's accuracy; yaw kept its prior.
    const auto posterior = filter.get_P();
    EXPECT_LT(posterior(3, 3), 1e-6);
    EXPECT_LT(posterior(4, 4), 1e-6);
    EXPECT_NEAR(posterior(5, 5), 1e-4, 1e-5);
}
}  // namespace
