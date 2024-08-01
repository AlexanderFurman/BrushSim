#ifndef OPEN3DVISUALIZER_H
#define OPEN3DVISUALIZER_H

#include "IVisualizer.h"
#include <open3d/Open3D.h>


// This version of the visualizer shows the user a physical model of the brush, and the stroke created by the brush.
class Open3DVisualizer: IVisualizer{
    using namespace open3d;

    private:
    std::shared_ptr<geometry::TriangleMesh> m_canvas;

    public:
    void visualize(const ISimResult& result) override;
    void initialize(const IConfig& config) override;
}

#endif // OPEN3DVISUALIZER_H