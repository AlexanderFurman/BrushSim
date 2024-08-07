#ifndef GEOMETRIC_H
#define GEOMETRIC_H

#include <vector>
#include <Eigen/Core>

namespace geometric{
    
    // Function to generate linspace of integers
    std::vector<int> linspace(int start, int end, int num);

    // Function to generate a ring of points around a center point given a normal vector
    std::vector<Eigen::Vector3d> generateRing(const Eigen::Vector3d& center,
                                             const Eigen::Vector3d& normal,
                                             double radius,
                                             int num_points);

    std::vector<Eigen::Vector3d> generateEllipse(const Eigen::Vector3d& center,
                                                        const Eigen::Vector3d& normal,
                                                        double radiusX,
                                                        double radiusY,
                                                        int num_points);

    Eigen::Vector3d linePlaneIntersection(const Eigen::Vector3d& linePoint,
                                        const Eigen::Vector3d& lineDir,
                                        const Eigen::Vector3d& planePoint,
                                        const Eigen::Vector3d& planeNormal);
            }

#endif // GEOMETRIC_H