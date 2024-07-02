#ifndef TWIST_H
#define TWIST_H

#include <Eigen/Core>

struct Twist{
    Eigen::Vector3d linearVelocity; // Linear velocity
    Eigen::Vector3d angularVelocity; // Angular Vecolity
};

#endif // TWIST_H