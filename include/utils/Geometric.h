#ifndef GEOMETRIC_H
#define GEOMETRIC_H

#include <vector>
#include <Eigen/Core>

namespace geometric{
    
    // Function to generate linspace of integers
    std::vector<int> linspace(int start, int end, int num);

    // Function to generate a ring of points around a center point given a normal vector
    std::vector<Eigen::Vector3d> generateRing(const Eigen::Vector3d& center, const Eigen::Vector3d& normal, double radius, int num_points);
}

#endif // GEOMETRIC_H