#include "preprocess.h"

#include <gtest/gtest.h>
#include <pcl/filters/voxel_grid.h>

#include <cmath>
#include <limits>

namespace
{
velodyne_ros::Point point(float x, float y, float z, float time = 0.0f,
                         uint16_t ring = 0)
{
  velodyne_ros::Point p{};
  p.x = x;
  p.y = y;
  p.z = z;
  p.intensity = 1.0f;
  p.time = time;
  p.ring = ring;
  return p;
}

sensor_msgs::msg::PointCloud2::UniquePtr message(
    const pcl::PointCloud<velodyne_ros::Point> &cloud)
{
  auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(cloud, *msg);
  return msg;
}

class VelodynePreprocess : public ::testing::Test
{
protected:
  void SetUp() override
  {
    preprocess.set(false, VELO16, 0.3, 1);
    preprocess.N_SCANS = 16;
    preprocess.SCAN_RATE = 10;
    preprocess.time_unit = SEC;
    preprocess.instantaneous_scan = true;
  }

  PointCloudXYZI::Ptr process(const pcl::PointCloud<velodyne_ros::Point> &cloud)
  {
    auto msg = message(cloud);
    PointCloudXYZI::Ptr out(new PointCloudXYZI());
    preprocess.process(msg, out);
    return out;
  }

  Preprocess preprocess;
};

TEST_F(VelodynePreprocess, DropsNonfiniteXYZEvenWhenInputClaimsDense)
{
  const float inf = std::numeric_limits<float>::infinity();
  const float nan = std::numeric_limits<float>::quiet_NaN();
  pcl::PointCloud<velodyne_ros::Point> cloud;
  cloud.push_back(point(1, 2, 3));
  cloud.push_back(point(inf, 1, 1));
  cloud.push_back(point(1, -inf, 1));
  cloud.push_back(point(1, 1, inf));
  cloud.push_back(point(nan, 1, 1));
  cloud.push_back(point(1, nan, 1));
  cloud.push_back(point(1, 1, nan));
  cloud.push_back(point(2, 3, 4));
  cloud.is_dense = true;
  auto out = process(cloud);
  ASSERT_EQ(out->size(), 2u);
  EXPECT_FLOAT_EQ(out->front().x, 1.0f);
  EXPECT_FLOAT_EQ(out->back().x, 2.0f);
}

TEST_F(VelodynePreprocess, FeaturePathDropsNonfiniteBeforeBuffering)
{
  preprocess.feature_enabled = true;
  const float inf = std::numeric_limits<float>::infinity();
  const float nan = std::numeric_limits<float>::quiet_NaN();
  pcl::PointCloud<velodyne_ros::Point> cloud;
  for (uint16_t ring = 0; ring < 16; ++ring)
  {
    cloud.push_back(point(2, 1, 1, 0.0f, ring));
    cloud.push_back(point(inf, 1, 1, 0.0f, ring));
    cloud.push_back(point(1, nan, 1, 0.0f, ring));
    cloud.push_back(point(1, 1, -inf, 0.0f, ring));
  }
  auto out = process(cloud);
  for (int ring = 0; ring < 16; ++ring)
  {
    ASSERT_EQ(preprocess.pl_buff[ring].size(), 1u);
    EXPECT_TRUE(std::isfinite(preprocess.pl_buff[ring].front().x));
  }
  EXPECT_TRUE(out->empty());
}

TEST_F(VelodynePreprocess, InstantaneousScanIgnoresNonzeroPointTimes)
{
  pcl::PointCloud<velodyne_ros::Point> cloud;
  cloud.push_back(point(1, 2, 3, 0.01f));
  cloud.push_back(point(2, 3, 4, 0.02f));
  auto out = process(cloud);
  ASSERT_EQ(out->size(), 2u);
  for (const auto &p : *out)
    EXPECT_FLOAT_EQ(p.curvature, 0.0f);
}

TEST_F(VelodynePreprocess, SpinningScanPreservesProvidedPointTimes)
{
  preprocess.instantaneous_scan = false;
  pcl::PointCloud<velodyne_ros::Point> cloud;
  cloud.push_back(point(1, 2, 3, 0.01f));
  cloud.push_back(point(2, 3, 4, 0.02f));
  auto out = process(cloud);
  ASSERT_EQ(out->size(), 2u);
  EXPECT_FLOAT_EQ(out->front().curvature, 10.0f);
  EXPECT_FLOAT_EQ(out->back().curvature, 20.0f);
}

TEST_F(VelodynePreprocess, EmptyAndAllInvalidFramesDoNotRetainOldPoints)
{
  pcl::PointCloud<velodyne_ros::Point> cloud;
  cloud.push_back(point(1, 2, 3));
  ASSERT_EQ(process(cloud)->size(), 1u);
  cloud.clear();
  EXPECT_TRUE(process(cloud)->empty());
  cloud.push_back(point(std::numeric_limits<float>::infinity(), 1, 1));
  EXPECT_TRUE(process(cloud)->empty());
}

TEST_F(VelodynePreprocess, InvalidReturnDoesNotChangeVoxelDownsampling)
{
  pcl::PointCloud<velodyne_ros::Point> cloud;
  for (int x = 0; x < 80; ++x)
    for (int y = 0; y < 80; ++y)
      cloud.push_back(point(1.0f+x*0.05f, 1.0f+y*0.05f, 1.0f));
  auto valid = process(cloud);
  cloud.push_back(point(std::numeric_limits<float>::infinity(), 1, 1));
  auto filtered = process(cloud);
  ASSERT_EQ(filtered->size(), valid->size());

  pcl::VoxelGrid<PointType> grid;
  grid.setLeafSize(0.2f, 0.2f, 0.2f);
  PointCloudXYZI expected, actual;
  grid.setInputCloud(valid);
  grid.filter(expected);
  grid.setInputCloud(filtered);
  grid.filter(actual);
  ASSERT_GT(expected.size(), 300u);
  ASSERT_EQ(actual.size(), expected.size());
  for (size_t i = 0; i < actual.size(); ++i)
  {
    EXPECT_FLOAT_EQ(actual[i].x, expected[i].x);
    EXPECT_FLOAT_EQ(actual[i].y, expected[i].y);
    EXPECT_FLOAT_EQ(actual[i].z, expected[i].z);
  }
}
}  // namespace
