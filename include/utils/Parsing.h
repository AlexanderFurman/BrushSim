#ifndef PARSING_H
#define PARSING_H

#include <Eigen/Core>
#include <stdexcept>
#include "data_types/Pose.h"
#include "data_types/Twist.h"

namespace parsing{
        
    // Helper function to parse a string to a double
    double parseDouble(const std::string& str);

    // Helper function to parse a string to an int
    int parseInt(const std::string& str);

    // Helper function to parse a string to an Eigen::Vector3d
    Eigen::Vector3d parseVector3d(const std::string& str);

    // Helper function to parse a string to a vector of Eigen::Vector3d
    std::vector<Eigen::Vector3d> parseVector3dList(const std::string& str);

    // Helper function to parse a string to a Pose
    Pose parsePose(const std::string& str);

    // Helper function to parse a string to a Twist
    Twist parseTwist(const std::string& str);
}

#endif // PARSING_H