#ifndef CONVERSIONS_H
#define CONVERSIONS_H

#include <Eigen/Core>
namespace Conversions {

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
        R = R_z * R_y * R_x;

        return R;
    }

    // Function to convert Euler angles (roll, pitch, yaw) to a rotation matrix
    Eigen::Matrix3d eulerToRotationMatrix(Eigen::Vector3d orientation) {
        double roll = orientation[0];
        double pitch = orientation[1];
        double yaw = orientation[2];

        return eulerToRotationMatrix(roll, pitch, yaw);
    }

} // namespace conversions

#endif // CONVERSIONS_H