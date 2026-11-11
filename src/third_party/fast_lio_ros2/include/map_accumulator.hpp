#ifndef FAST_LIO_MAP_ACCUMULATOR_HPP
#define FAST_LIO_MAP_ACCUMULATOR_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Own world-frame points independently of the mutable current scan and pose.
// Reading the map for publication or saving never appends another scan.
class MapAccumulator
{
public:
    using Cloud = pcl::PointCloud<pcl::PointXYZINormal>;

    template<typename Transform>
    void append(const Cloud &scan, double stamp, Transform transform)
    {
        if (scan.empty()) return;
        Cloud world;
        world.resize(scan.size());
        for (std::size_t i = 0; i < scan.size(); ++i)
            transform(&scan.points[i], &world.points[i]);
        cloud_ += world;
        stamp_ = stamp;
    }

    const Cloud &cloud() const { return cloud_; }
    double stamp() const { return stamp_; }

private:
    Cloud cloud_;
    double stamp_ = 0.0;
};

#endif
