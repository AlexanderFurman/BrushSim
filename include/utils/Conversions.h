#ifndef CONVERSIONS_H
#define CONVERSIONS_H

#include <Eigen/Core>

namespace conversions {

    // Function to convert Euler angles (roll, pitch, yaw) to a rotation matrix
    Eigen::Matrix3d eulerToRotationMatrix(double roll, double pitch, double yaw);

    // Function to convert Euler angles (roll, pitch, yaw) to a rotation matrix
    Eigen::Matrix3d eulerToRotationMatrix(Eigen::Vector3d orientation) ;

    // Function which finds all non-zero elements in a matrix and returns a vector of their indices
    std::vector<Eigen::Vector2i> findNonZeroIndices(const Eigen::MatrixXd& matrix);
    
} // namespace conversions

#endif // CONVERSIONS_H