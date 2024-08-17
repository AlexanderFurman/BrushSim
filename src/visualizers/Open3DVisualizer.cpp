#include "visualizers/Open3DVisualizer.h"
#include "utils/Conversions.h"
#include "utils/Parsing.h"
#include "utils/Geometric.h"

bool Open3DVisualizer::togglePause(open3d::visualization::Visualizer *vis) {
    is_paused = !is_paused;
    std::cout << (is_paused ? "Paused" : "Resumed") << std::endl;
    return true;
}

void Open3DVisualizer::initializeObjects(const std::map<std::string, std::string> visConfig){
    // Create canvas mesh
    int h = parsing::parseInt(visConfig.at("canvas height"));
    int w = parsing::parseInt(visConfig.at("canvas width"));
    generateCanvas(h, w);

    // Create brush stroke pointcloud
    m_brushStrokePcd = std::make_shared<open3d::geometry::PointCloud>();

    // Create brush mesh
    m_brush = std::make_shared<open3d::geometry::TriangleMesh>();

    //Create Brush Stem Frame
    m_brushStemFrame = open3d::geometry::TriangleMesh::CreateCoordinateFrame(h/20);
    const Pose& initialPose = parsing::parsePose(visConfig.at("brush initial pose"));
    auto rotation = conversions::eulerToRotationMatrix(initialPose.orientation);
    auto translation = initialPose.position;
    m_brushStemFrame->Translate(translation);
    m_brushStemFrame->Rotate(rotation, translation);

    //Initialize pose to the initial pose read in the config file
    m_brushStemPose = initialPose;
    
    //Create world frame
    m_worldFrame = open3d::geometry::TriangleMesh::CreateCoordinateFrame(h/20);
}

void Open3DVisualizer::initializeVisualizer(const std::map<std::string, std::string> visConfig){
    
    //Create window
    m_vis.CreateVisualizerWindow("Brush Simulation Visualization");

    // Add geometries
    m_vis.AddGeometry(m_canvas);
    m_vis.AddGeometry(m_worldFrame);
    m_vis.AddGeometry(m_brushStemFrame);
    m_vis.AddGeometry(m_brush);
    m_vis.AddGeometry(m_brushStrokePcd);

    // Register key callback to toggle pause on pressing the spacebar
    m_vis.RegisterKeyCallback(GLFW_KEY_SPACE, [this](open3d::visualization::Visualizer *vis) {
    this->togglePause(vis);
    return true;  // Return true if the event was handled, false otherwise
    });
}

void Open3DVisualizer::initialize(const IConfig& config)
{
    const std::map<std::string, std::string> visConfig = config.getVisualizerConfig();
    initializeObjects(visConfig);
    initializeVisualizer(visConfig);

 }

void Open3DVisualizer::generateCanvas(int numRows, int numCols){ 
    //initialize canvas
    m_canvas = std::make_shared<open3d::geometry::TriangleMesh>();

    //Generate edge points of the canvas
    std::vector<Eigen::Vector3d> canvasPoints;
    canvasPoints.emplace_back(Eigen::Vector3d(0,0,0));
    canvasPoints.emplace_back(Eigen::Vector3d(numRows,0,0));
    canvasPoints.emplace_back(Eigen::Vector3d(0,numCols,0));
    canvasPoints.emplace_back(Eigen::Vector3d(numRows,numCols,0));
    //Add points to the mesh
    for (const auto& vertex : canvasPoints) {
        m_canvas->vertices_.emplace_back(vertex);
    }

    // Create triangles of the mesh
    std::vector<Eigen::Vector3i> triangles = {
        Eigen::Vector3i(0, 2, 3),
        Eigen::Vector3i(0, 1, 3)
    };
    // Add triangles to the mesh
    for (const auto& triangle : triangles) {
        m_canvas->triangles_.emplace_back(triangle);
    }
    m_canvas->OrientTriangles();
}

void Open3DVisualizer::updateBrush(const std::vector<Eigen::Vector3d>& vertices, const Pose& pose){
    auto newBrush = generateMesh(vertices);
    m_brush->adjacency_list_ = newBrush->adjacency_list_;
    m_brush->vertices_ = newBrush->vertices_;
    m_brush->triangles_ = newBrush->triangles_;
    Eigen::Vector3d babyBlue(0.5, 0.5, 0.8);
    m_brush->PaintUniformColor(babyBlue);

    m_vis.UpdateGeometry(m_brush);

    //TODO: implement this later
    // if (m_brush->vertices_.size() == 0){
    //     m_brush = generateMesh(vertices);
    //     return;
    // }
    // //Rotate the current brush mesh
    // auto center = Eigen::Vector3d::Zero();
    // //NOTE: Currently assuming global rotation (this may need to change in future.)'
    // auto rotation = conversions::eulerToRotationMatrix(pose.orientation);
    // m_brush->Rotate(rotation);
    // //Check if the new vertices calculated 
    // auto centroid = geometric::computeCentroid(vertices);
    // std::vector<Eigen::Vector3d>& vertices_about_centroid;
    // for (const auto& vertex: vertices){
    //     vertices_about_centroid.emplace_back(vertex - centroid);
    // }
}

void Open3DVisualizer::updateBrushStroke(const Eigen::MatrixXd& brushStroke){
    //generate pointcloud of the brush stroke (should be a lineset in future)
    std::vector<Eigen::Vector3d> brushStrokeVectorized;
    const std::vector<Eigen::Vector2i> indices = conversions::findNonZeroIndices(brushStroke);
    for (const Eigen::Vector2i& idx : indices) {
        auto point = Eigen::Vector3d(static_cast<double>(idx.x()), static_cast<double>(idx.y()), 0.0);
        m_brushStrokePcd->points_.emplace_back(point);
    }
    Eigen::Vector3d red(1,0,0);
    m_brushStrokePcd->PaintUniformColor(red);
    m_vis.UpdateGeometry(m_brushStrokePcd);
}

void Open3DVisualizer::visualize(const SimStep& step, const SimResult& result){
    using namespace open3d;

    while (is_paused){
        m_vis.PollEvents();
    }

    //update Brush
    updateBrush(result.vertices, step.pose);

    //update brush stroke
    updateBrushStroke(result.brushStroke);

    //Undo previous transformation
    auto antiRotation = conversions::eulerToRotationMatrix(-m_brushStemPose.orientation);
    auto center = m_brushStemPose.position;
    m_brushStemFrame->Rotate(antiRotation, center);
    m_brushStemFrame->Translate(-center);
    //Add new transformation
    m_brushStemFrame->Translate(step.pose.position);
    auto newRotation = conversions::eulerToRotationMatrix(step.pose.orientation);
    m_brushStemFrame->Rotate(newRotation, step.pose.position);
    //update current pose
    m_brushStemPose = step.pose;
    m_vis.UpdateGeometry(m_brushStemFrame);
    
    
    // //generate line set showing normal vector of the brush
    // std::vector<Eigen::Vector3d> keyPoints;
    // keyPoints.emplace_back(step.pose.position);
    // // std::vector.emplace_back(result.brushTipPosition); // Note may remove the brushtip position from BrushStrokeResult
    // auto brushNormal = geometric::computeNormal(step.pose.orientation, Eigen::Vector3d(0,0,1)); // Magic vector (default axis for rotation calculations)
    // keyPoints.emplace_back(step.pose.position -  brushNormal * 50); //TODO: remove magic number
    // std::shared_ptr<geometry::LineSet> brushNormalLineSet = generateLineSet(keyPoints);

    // Eigen::Vector3d green(0,1,0);
    // // linesetPcd->PaintUniformColor(green);
    // brushNormalLineSet->PaintUniformColor(green);

    m_vis.PollEvents();

}

std::shared_ptr<open3d::geometry::PointCloud> Open3DVisualizer::generatePointCloud(const std::vector<Eigen::Vector3d>& points) const {
    using namespace open3d;
    // Create an Open3D PointCloud object
    auto pointCloud = std::make_shared<geometry::PointCloud>();
    pointCloud->points_ = points;
    return pointCloud;
} 

std::shared_ptr<open3d::geometry::TriangleMesh> Open3DVisualizer::generateMesh(const std::vector<Eigen::Vector3d>& points) const {
    auto pointCloud = generatePointCloud(points);

    // Estimate normals (required for Poisson reconstruction)
    pointCloud->EstimateNormals();
    pointCloud->OrientNormalsConsistentTangentPlane(1);

    auto result = pointCloud->ComputeConvexHull();

    auto mesh = std::get<0>(result);

    // // Perform Poisson surface reconstruction and get the mesh
    // auto result = open3d::geometry::TriangleMesh::CreateFromPointCloudPoisson(*pointCloud);

    // // Extract the TriangleMesh from the tuple
    // *mesh = *std::get<0>(result);

    return mesh;
}


std::shared_ptr<open3d::geometry::LineSet> Open3DVisualizer::generateLineSet(const std::vector<Eigen::Vector3d>& points) const {
    auto line_set = std::make_shared<open3d::geometry::LineSet>();

    //Add points to the line set
    line_set->points_ = points;

    // Add lines to the line set (note for now, we're just automatically connecting each 2 points neihbouring each other in the list)
    std::vector<Eigen::Vector2i> lines;
    for (size_t i = 0; i < points.size() - 1; ++i) {
        lines.push_back(Eigen::Vector2i(i, i + 1));
    }
    line_set->lines_ = lines;

    return line_set;
}


