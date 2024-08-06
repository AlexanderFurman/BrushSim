#include "utils/Geometric.h"
#include <Eigen/Geometry>
#include <stdexcept>

namespace geometric{
    
    // Function to generate linspace of integers
    std::vector<int> linspace(int start, int end, int num) {
        std::vector<int> result;
        if (num < 1){
            throw std::runtime_error("Need at least 2 numbers in the linspace");
        }
        
        double step = static_cast<double>(end - start) / (num - 1);
        for (int i = 0; i < num; ++i) {
            result.push_back(start + static_cast<int>(i * step));
        }
        return result;
    };

    // Function to generate a ring of points around a center point given a normal vector
    std::vector<Eigen::Vector3d> generateRing(const Eigen::Vector3d& center, const Eigen::Vector3d& normal, double radius, int num_points) {
        std::vector<Eigen::Vector3d> ring_points;

        // Normalize the normal vector
        Eigen::Vector3d n = normal.normalized();

        // Find two vectors orthogonal to the normal vector
        Eigen::Vector3d arbitrary_vec(1, 0, 0);
        if (n.isApprox(arbitrary_vec)) {
            arbitrary_vec = Eigen::Vector3d(0, 1, 0);
        }
        Eigen::Vector3d u = n.cross(arbitrary_vec).normalized();
        Eigen::Vector3d v = n.cross(u).normalized();

        // Generate points in the ring
        for (int i = 0; i < num_points; ++i) {
            double angle = 2 * M_PI * i / num_points;
            Eigen::Vector3d point = center + radius * (std::cos(angle) * u + std::sin(angle) * v);
            ring_points.push_back(point);
        }

        return ring_points;
    };
}
