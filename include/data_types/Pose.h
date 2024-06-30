#ifndef POSE_H
#define POSE_H

#include <Eigen/Core>

//Struct which holds position and orientation information of an object
struct Pose{
    Eigen::Vector3d position; // position of the object (COM or some other reference point)
    Eigen::Vector3d orientation; // orientation using Euler angles in radians (roll, pitch, yaw)
};

#endif // POSE_H