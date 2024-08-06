#include "utils/Conversions.h"
#include <stdexcept>

namespace conversions {

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

    std::vector<Eigen::Vector2i> findNonZeroIndices(const Eigen::MatrixXd& matrix) {
        std::vector<Eigen::Vector2i> points;

        // Iterate through each element of the matrix
        for (int i = 0; i < matrix.rows(); ++i) {
            for (int j = 0; j < matrix.cols(); ++j) {
                if (matrix(i, j) != 0) { // Check if the matrix value is non-zero
                    // Create a Vector3d with x=i, y=j, z=0.0
                    points.emplace_back(i, j);
                }
            }
        }

        return points;
    };
    
}