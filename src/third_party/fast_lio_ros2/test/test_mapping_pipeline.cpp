#include <rclcpp/rclcpp.hpp>
#include "IMU_Processing.hpp"
#include "map_accumulator.hpp"

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
}  // namespace
