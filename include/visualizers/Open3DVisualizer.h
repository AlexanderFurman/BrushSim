#ifndef OPEN3DVISUALIZER_H
#define OPEN3DVISUALIZER_H

#include "interfaces/IVisualizer.h"
#include <open3d/Open3D.h>


// This version of the visualizer shows the user a physical model of the brush, and the stroke created by the brush.
class Open3DVisualizer: public IVisualizer{
    public:
    void visualize(const SimStep& step, const SimResult& result) override;
    void initialize(const IConfig& config) override;

    
    private:
    open3d::visualization::VisualizerWithKeyCallback m_vis; 
    std::shared_ptr<open3d::geometry::TriangleMesh> m_canvas;
    std::shared_ptr<open3d::geometry::TriangleMesh> m_worldFrame;
    std::shared_ptr<open3d::geometry::TriangleMesh> m_brushStemFrame;
    std::shared_ptr<open3d::geometry::PointCloud> m_brushStrokePcd;
    std::shared_ptr<open3d::geometry::TriangleMesh> m_brush;
    Pose m_brushStemPose;
    bool is_paused = false;

    //TODO: Add simulation clock which ensures that the brush moves according to the timestamps provided by simstep,
    // and not simply by completing the stroke according to compute time.


    void generateCanvas(int numRows, int numCols);
    void updateBrush(const std::vector<Eigen::Vector3d>& vertices, const Pose& pose);
    void updateBrushStroke(const Eigen::MatrixXd& brushStroke);
    void initializeObjects(const std::map<std::string, std::string> visConfig);
    void initializeVisualizer(const std::map<std::string, std::string> visConfig);

    //Open3D helper functions
    std::shared_ptr<open3d::geometry::PointCloud> generatePointCloud(const std::vector<Eigen::Vector3d>& points) const;
    std::shared_ptr<open3d::geometry::TriangleMesh> generateMesh(const std::vector<Eigen::Vector3d>& points) const;
    std::shared_ptr<open3d::geometry::LineSet> generateLineSet(const std::vector<Eigen::Vector3d>& points) const;
    bool togglePause(open3d::visualization::Visualizer *vis);
};

#endif // OPEN3DVISUALIZER_H