#ifndef FAST_LIO_MATCH_CONSTRAINTS_HPP
#define FAST_LIO_MATCH_CONSTRAINTS_HPP

// Guards for the point-to-plane measurement model. Eigen only, so they can be
// unit tested without ROS or PCL.

#include <algorithm>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

// RMS extent of the first `count` points along their second principal axis.
//
// esti_plane() fits a plane through the nearest map points. A sparse multi-line
// LiDAR leaves single scan lines on distant surfaces, so those neighbours are
// often collinear and every plane through the line fits them equally well. The
// least-squares answer is then decided by range noise, which displaces points
// along their rays: the result is the plane containing the line and the ray,
// not the surface. On a wall that plane is nearly horizontal, so the point
// wrongly constrains height and roll/pitch instead of the distance to the wall.
//
// A neighbourhood supports a plane only if it extends in two directions. This
// returns the smaller of the two in-plane extents; callers reject the match
// when it is below a threshold tied to the map resolution.
template<typename PointContainer>
double neighbor_cross_spread(const PointContainer &points, int count)
{
    if (count < 3) return 0.0;
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (int j = 0; j < count; j++)
        mean += Eigen::Vector3d(points[j].x, points[j].y, points[j].z);
    mean /= count;

    Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
    for (int j = 0; j < count; j++)
    {
        const Eigen::Vector3d d = Eigen::Vector3d(points[j].x, points[j].y, points[j].z) - mean;
        scatter += d * d.transpose();
    }
    scatter /= count;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
    solver.computeDirect(scatter, Eigen::EigenvaluesOnly);
    // Eigenvalues are sorted in increasing order: [normal, cross, along].
    return std::sqrt(std::max(0.0, solver.eigenvalues()(1)));
}

// Rows of the level pseudo-measurement appended after the LiDAR residuals.
constexpr int LEVEL_CONSTRAINT_ROWS = 2;

// Pseudo-measurement "the body z axis stays parallel to the map z axis", i.e.
// roll and pitch keep their initial value while yaw stays free. It is valid
// for a ground robot on one flat floor (the map frame is the first IMU frame).
//
// Writes LEVEL_CONSTRAINT_ROWS rows starting at `row` in FAST-LIO's
// convention: h is the innovation (z - h(x)) and h_x its Jacobian with respect
// to the error state, where the rotation error occupies columns 3..5 and
// perturbs on the right (R * Exp(delta)).
//
// The filter applies one scalar measurement covariance `meas_cov` to every
// row, so the rows are scaled by sqrt(meas_cov) / std_rad to give this
// constraint its own standard deviation `std_rad` (radians, per update).
template<typename JacobianMatrix, typename ResidualVector>
void write_level_constraint(const Eigen::Matrix3d &rot_world_body, double meas_cov, double std_rad,
                            int row, JacobianMatrix &h_x, ResidualVector &h)
{
    const double scale = std::sqrt(meas_cov) / std_rad;
    // Measured quantity: the x, y components of the body z axis in the map,
    // (R * e3)_xy, expected to be 0. d(R * Exp(delta) * e3) / d(delta) = -R * [e3]x,
    // whose columns are (-R.col(1), R.col(0), 0): yaw is unobserved.
    for (int k = 0; k < LEVEL_CONSTRAINT_ROWS; k++)
    {
        for (int c = 0; c < h_x.cols(); c++) h_x(row + k, c) = 0.0;
        h_x(row + k, 3) = -scale * rot_world_body(k, 1);
        h_x(row + k, 4) = scale * rot_world_body(k, 0);
        h(row + k) = -scale * rot_world_body(k, 2);
    }
}

#endif
