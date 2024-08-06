#ifndef OPEN3DVISUALIZER_H
#define OPEN3DVISUALIZER_H

#include "interfaces/IVisualizer.h"
#include <open3d/Open3D.h>


// This version of the visualizer shows the user a physical model of the brush, and the stroke created by the brush.
class Open3DVisualizer: IVisualizer{
    public:
    void visualize(const SimResult& result) override;
    void initialize(const IConfig& config) override;

    private:
    std::shared_ptr<open3d::geometry::TriangleMesh> m_canvas;
    std::shared_ptr<open3d::geometry::PointCloud> generatePointCloud(const std::vector<Eigen::Vector3d>& points) const;
    std::shared_ptr<open3d::geometry::TriangleMesh> generateMesh(const std::shared_ptr<open3d::geometry::PointCloud>& pointCloud) const;
    std::shared_ptr<open3d::geometry::LineSet> generateLineSet(const std::vector<Eigen::Vector3d>& points) const;
};

#endif // OPEN3DVISUALIZER_H