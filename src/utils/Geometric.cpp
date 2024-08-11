#include "utils/Geometric.h"
#include <Eigen/Geometry>
#include <stdexcept>

namespace geometric{
    
    // // Function to generate linspace of values
    // template <typename T>
    // std::vector<T> linspace(T start, T end, int num) {
    //     static_assert(std::is_arithmetic<T>::value, "Template param T should be numeric type.");
    //     std::vector<T> result;
    //     if (num < 1){
    //         throw std::runtime_error("Need at least 2 numbers in the linspace");
    //     }
        
    //     double step = (end - start) / (num - 1);
    //     for (int i = 0; i < num; ++i) {
    //         result.push_back(static_cast<T>(start + i * step));
    //     }
    //     return result;
    // };

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

    // Function to generate points on a 3D ellipse
    std::vector<Eigen::Vector3d> generateEllipse(const Eigen::Vector3d& center,
                                                        const Eigen::Vector3d& normal,
                                                        double radiusX,
                                                        double radiusY,
                                                        int num_points) {
        std::vector<Eigen::Vector3d> points(num_points);

        // Create an orthonormal basis from the normal vector
        Eigen::Vector3d x_axis(1, 0, 0);
        Eigen::Vector3d y_axis(0, 1, 0);
        Eigen::Vector3d u = normal.normalized();

        // Create two orthogonal vectors in the plane of the ellipse
        Eigen::Vector3d v1 = x_axis - u.dot(x_axis) * u;
        v1.normalize();
        Eigen::Vector3d v2 = u.cross(v1);

        // Generate points on the ellipse
        for (int i = 0; i < num_points; ++i) {
            double angle = 2 * M_PI * i / num_points;
            Eigen::Vector3d point = center + radiusX * std::cos(angle) * v1 + radiusY * std::sin(angle) * v2;
            points[i] = point;
        }

        return points;
    }

    // Function to find the intersection of a line and a plane
    Eigen::Vector3d linePlaneIntersection(const Eigen::Vector3d& linePoint,
                            const Eigen::Vector3d& lineDir,
                            const Eigen::Vector3d& planePoint,
                            const Eigen::Vector3d& planeNormal) {
        double denom = planeNormal.dot(lineDir);
        if (std::fabs(denom) < 1e-6) { // Line is parallel to the plane
            throw std::runtime_error("The line is parallel to the plane");
        }

        double t = (planeNormal.dot(planePoint - linePoint)) / denom;
        Eigen::Vector3d intersection = linePoint + t * lineDir;
        return intersection;
    }
}
