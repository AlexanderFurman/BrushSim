#ifndef GEOMETRIC_H
#define GEOMETRIC_H

#include <vector>
#include <Eigen/Core>

namespace geometric{
    
    // Function to generate linspace of values
    template <typename T>
    std::vector<T> linspace(T start, T end, int num);

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


        
    // Implementation in header file to ensure correct compilation with typename
    template <typename T>
    std::vector<T> linspace(T start, T end, int num) {
        static_assert(std::is_arithmetic<T>::value, "Template param T should be numeric type.");
        std::vector<T> result;
        if (num < 1){
            throw std::runtime_error("Need at least 2 numbers in the linspace");
        }
        
        double step = (end - start) / (num - 1);
        for (int i = 0; i < num; ++i) {
            result.push_back(static_cast<T>(start + i * step));
        }
        return result;
    };
}

#endif // GEOMETRIC_H