#ifndef PARSING_H
#define PARSING_H

#include <Eigen/Core>

namespace parsing{
        
    // Helper function to parse a string to a double
    double parseDouble(const std::string& str) {
        std::stringstream ss(str);
        double value;
        ss >> value;
        if (ss.fail()) {
            throw std::runtime_error("Failed to parse double from string: " + str);
        }
        return value;
    }

    // Helper function to parse a string to an int
    int parseInt(const std::string& str) {
        std::stringstream ss(str);
        int value;
        ss >> value;
        if (ss.fail()) {
            throw std::runtime_error("Failed to parse int from string: " + str);
        }
        return value;
    }

    // Helper function to parse a string to an Eigen::Vector3d
    Eigen::Vector3d parseVector3d(const std::string& str) {
        Eigen::Vector3d vec;
        std::stringstream ss(str);
        char comma;
        ss >> vec[0] >> comma >> vec[1] >> comma >> vec[2];
        if (ss.fail() || comma != ',') {
            throw std::runtime_error("Failed to parse Eigen::Vector3d from string: " + str);
        }
        return vec;
    }

    // Helper function to parse a string to a vector of Eigen::Vector3d
    std::vector<Eigen::Vector3d> parseVector3dList(const std::string& str) {
        std::vector<Eigen::Vector3d> vecList;
        std::stringstream ss(str);
        std::string item;
        while (std::getline(ss, item, ';')) {
            vecList.push_back(parseVector3d(item));
        }
        return vecList;
    }
}


#endif // PARSING_H