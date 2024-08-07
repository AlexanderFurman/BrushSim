#include "visualizers/Open3DVisualizer.h"
#include "utils/Conversions.h"

void Open3DVisualizer::initialize(const IConfig& config)
{

}

void Open3DVisualizer::visualize(const SimResult& result){
    using namespace open3d;
    //// NOTE: I'm currently regenerating entire scene upon receiving result. this will definitely change
    //TODO: break this hideous method into multiple methods
    
    //Generate mesh of the canvas
    auto brushStroke = result.getBrushStroke();
    int numCols = brushStroke.cols();
    int numRows = brushStroke.rows();
    std::vector<Eigen::Vector3d> canvasPoints;
    //create points of all corners of the canvas
    canvasPoints.emplace_back(Eigen::Vector3d(0,0,0));
    canvasPoints.emplace_back(Eigen::Vector3d(numRows,0,0));
    canvasPoints.emplace_back(Eigen::Vector3d(0,numCols,0));
    canvasPoints.emplace_back(Eigen::Vector3d(numRows,numCols,0));
    auto canvasMesh = std::make_shared<open3d::geometry::TriangleMesh>();
    for (const auto& vertex : canvasPoints) {
        canvasMesh->vertices_.emplace_back(vertex);
    }
    std::vector<Eigen::Vector3i> triangles = {
        Eigen::Vector3i(0, 2, 3),
        Eigen::Vector3i(0, 1, 3)
    };
    // Add triangles to the mesh
    for (const auto& triangle : triangles) {
        canvasMesh->triangles_.emplace_back(triangle);
    }
    canvasMesh->OrientTriangles();
    bool hasVertexNormals = canvasMesh->HasVertexNormals();
    bool hasTriangleNormals = canvasMesh->HasTriangleNormals();
    canvasMesh->PaintUniformColor(Eigen::Vector3d(0.82, 0.71, 0.55));

    //generate mesh of the brush
    std::shared_ptr<geometry::PointCloud> brushPcd = generatePointCloud(result.getBrushVertices());
    std::shared_ptr<geometry::TriangleMesh> brushMesh = generateMesh(brushPcd);
    brushMesh->PaintUniformColor(Eigen::Vector3d(0.6, 0.6, 0.6));

    //generate pointcloud of the brush stroke (should be a lineset in future)
    std::vector<Eigen::Vector3d> brushStrokeVectorized;
    const std::vector<Eigen::Vector2i> indices = conversions::findNonZeroIndices(result.getBrushStroke());
    for (const Eigen::Vector2i& idx: indices)
    {
        brushStrokeVectorized.emplace_back(static_cast<double>(idx.x()), static_cast<double>(idx.y()), 0.0); // Construct Vector3d with z=0
    }
    std::shared_ptr<geometry::PointCloud> brushStrokePcd = generatePointCloud(brushStrokeVectorized);
    
    
    //generate line set showing normal vector of the brush
    std::vector<Eigen::Vector3d> keyPoints;
    keyPoints.emplace_back(result.getBrushPosition());
    // std::vector.emplace_back(result.brushTipPosition); // Note may remove the brushtip position from BrushStrokeResult
    keyPoints.emplace_back(result.getBrushPosition() - result.getBrushNormal() * 50); //TODO: remove magic number
    std::shared_ptr<geometry::LineSet> brushNormalLineSet = generateLineSet(keyPoints);

    Eigen::Vector3d green(0,1,0);
    // linesetPcd->PaintUniformColor(green);
    brushNormalLineSet->PaintUniformColor(green);
    
    //set brushStroke color:
    Eigen::Vector3d red(1, 0, 0);
    brushStrokePcd->PaintUniformColor(red);

    //set brushPcd color (testing only)
    Eigen::Vector3d blue(0,0,1);
    brushPcd->PaintUniformColor(blue);

    // Visualization
    open3d::visualization::Visualizer visualizer;
    visualizer.CreateVisualizerWindow("Brush Simulation Visualization");

    visualizer.AddGeometry(canvasMesh);
    visualizer.AddGeometry(brushMesh);
    visualizer.AddGeometry(brushPcd); //added for testing
    visualizer.AddGeometry(brushStrokePcd);
    visualizer.AddGeometry(brushNormalLineSet);

    // // Optionally set the view
    // visualizer.GetViewControl().SetFront(Eigen::Vector3d(0, 0, -1));
    // visualizer.GetViewControl().SetLookat(Eigen::Vector3d(numRows / 2, numCols / 2, 0));
    // visualizer.GetViewControl().SetUp(Eigen::Vector3d(0, 1, 0));
    visualizer.GetViewControl().SetZoom(0.3);

    // Run the visualizer
    visualizer.Run();

}

std::shared_ptr<open3d::geometry::PointCloud> Open3DVisualizer::generatePointCloud(const std::vector<Eigen::Vector3d>& points) const {
    using namespace open3d;
    // Create an Open3D PointCloud object
    auto pointCloud = std::make_shared<geometry::PointCloud>();
    pointCloud->points_ = points;
    return pointCloud;
} 

std::shared_ptr<open3d::geometry::TriangleMesh> Open3DVisualizer::generateMesh(const std::shared_ptr<open3d::geometry::PointCloud>& pointCloud) const {
    // Create a shared pointer for TriangleMesh
    auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();

    // Estimate normals (required for Poisson reconstruction)
    pointCloud->EstimateNormals();
    pointCloud->OrientNormalsConsistentTangentPlane(1);

    // Perform Poisson surface reconstruction and get the mesh
    auto result = open3d::geometry::TriangleMesh::CreateFromPointCloudPoisson(*pointCloud);

    // Extract the TriangleMesh from the tuple
    *mesh = *std::get<0>(result);

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


