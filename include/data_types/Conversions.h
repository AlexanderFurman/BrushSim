#ifndef CONVERSIONS_H
#define CONVERSIONS_H

#include <Eigen/Core>
#include <exception>
namespace Conversions {

    // Note: Rename this whole thing to Utils

    // Function to convert Euler angles (roll, pitch, yaw) to a rotation matrix
    Eigen::Matrix3d eulerToRotationMatrix(double roll, double pitch, double yaw) {
        Eigen::Matrix3d R_x, R_y, R_z, R;

        // Define roll matrix
        R_x << 1, 0, 0,
            0, cos(roll), -sin(roll),
            0, sin(roll), cos(roll);

        // Define pitch matrix
        R_y << cos(pitch), 0, sin(pitch),
            0, 1, 0,
            -sin(pitch), 0, cos(pitch);

        // Define yaw matrix
        R_z << cos(yaw), -sin(yaw), 0,
            sin(yaw), cos(yaw), 0,
            0, 0, 1;

        // Compute the final rotation matrix
        R = R_z * R_y * R_x; // This uses the Yaw-Pitch-Roll computation order.

        return R;
    };

    // Function to convert Euler angles (roll, pitch, yaw) to a rotation matrix
    Eigen::Matrix3d eulerToRotationMatrix(Eigen::Vector3d orientation) {
        double roll = orientation[0];
        double pitch = orientation[1];
        double yaw = orientation[2];

        return eulerToRotationMatrix(roll, pitch, yaw);
    };

    std::vector<Eigen::Vector3d> ConvertNonZeroToVector3d(const Eigen::MatrixXd& matrix) {
        std::vector<Eigen::Vector3d> points;

        // Iterate through each element of the matrix
        for (int i = 0; i < matrix.rows(); ++i) {
            for (int j = 0; j < matrix.cols(); ++j) {
                if (matrix(i, j) != 0) { // Check if the matrix value is non-zero
                    // Create a Vector3d with x=i, y=j, z=0.0
                    points.emplace_back(i, j, 0.0);
                }
            }
        }

        return points;
    };

    // Function to generate linspace of integers
    std::vector<int> linspace(int start, int end, int num) {
        std::vector<int> result;
        if (num < 1){
            throw std::exception("Need at least 2 numbers in the linspace");
        }
        
        double step = static_cast<double>(end - start) / (num - 1);
        for (int i = 0; i < num; ++i) {
            result.push_back(start + static_cast<int>(i * step));
        }
        return result;
    };

    // Function to generate a ring of points around a center point given a normal vector
    std::vector<Eigen::Vector3d> generateRing(const Eigen::Vector3d& center, const Eigen::Vector3d& normal, double radius, int num_points) {
        std::vector<Eigen::Vector3d> ring_points;

        // Normalize the normal vector
        Eigen::Vector3d n = normal.normalized();

        // Find two vectors orthogonal to the normal vector
        Eigen::Vector3d arbitrary_vec(1, 0, 0);
        if (n.isApprox(arbitrary_vec)) {
            arbitrary_vec = Eigen::Vector3d(0, 1, 0);
        }
        Eigen::Vector3d u = n.cross(arbitrary_vec).normalized();
        Eigen::Vector3d v = n.cross(u).normalized();

        // Generate points in the ring
        for (int i = 0; i < num_points; ++i) {
            double angle = 2 * M_PI * i / num_points;
            Eigen::Vector3d point = center + radius * (std::cos(angle) * u + std::sin(angle) * v);
            ring_points.push_back(point);
        }

        return ring_points;
    };


} // namespace conversions

#endif // CONVERSIONS_H