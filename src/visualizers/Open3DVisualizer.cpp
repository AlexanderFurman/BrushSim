#include "Open3DVisualizer.h"


void Open3DVisualizer::visualize(const ISimResult& result){
    //// NOTE: I'm currently regenerating entire scene upon receiving result. this will definitely change
    
    //attempt to dynamic cast to BrushStrokeResult
    const BrushStrokeResult* result = dynamic_cast<const BrushStrokeResult*>(&result);
    if (!result) {
        throw std::exception("Unknown datatype passed to visualize method")
    }

    //Generate mesh of the canvas
    int numCols = result.brushStroke.cols();
    int numRows = result.brushStroke.rows();
    vector<Eigen::Vector3d> canvasPoints;
    //create points of all corners of the canvas
    canvasPoints.enplace_back(Eigen::Vector3d(0,0,0));
    canvasPoints.enplace_back(Eigen::Vector3d(numRows,0,0));
    canvasPoints.enplace_back(Eigen::Vector3d(0,numCols,0));
    canvasPoints.enplace_back(Eigen::Vector3d(numRows,numCols,0));
    std::shared_ptr<geometry::PointCloud> canvasPcd = generatePointCloud(canvasPoints);
    std::shared_ptr<geometry::TriangleMesh> canvasMesh = generateMesh(canvasPcd);


    //generate mesh of the brush
    std::shared_ptr<geometry::PointCloud> brushPcd = generatePointCloud(result.vertices);
    std::shared_ptr<geometry::TriangleMesh> brushMesh = generateMesh(brushPcd);

    //generate pointcloud of the brush stroke (should be a lineset in future)
    vector<Eigen::Vector3d> brushStrokeVectorized = Conversions::ConvertNonZeroToVector3d(result.brushStroke)
    std::shared_ptr<geometry::PointCloud> brushStrokePcd = generatePointCloud(brushStrokeVectorized)
    
    //generate line set showing normal vector of the brush
    vector<Eigen::Vector3d> keyPoints;
    vector.emplace_back(result.brushStemPosition);
    vector.emplace_back(result.brushTipPosition) // Note may remove the brushtip position from BrushStrokeResult
    std::shared_ptr<geometry::LineSet> brushNormalLineSet = generateLineSet

    //set brushMesh to transparent grey:
    Eigen::Vector3d grey_alpha_half(0.5, 0.5, 0.5, 0.5);
    brushMesh->vertex_colors_.resize(brushMesh->vertices_.size(), grey_alpha_half);

    //set canvasMesh to transparent white:
    Eigen::Vector3d white_alpha_half(1, 1, 1, 0.5);
    canvasMesh->vertex_colors_.resize(canvasMesh->vertices_.size(), white_alpha_half);

    //set line set color:
    Eigen::Vector3d green_alpha_1(0, 1, 0, 0.5);
    brushNormalLineSet->colors_.resize(brushNormalLineSet->lines_.size(), green_alpha_1);
    
    //set brushStroke color:
    Eigen::Vector3d red_alpha_1(1, 0, 0, 1);
    brushStrokePcd->colors_.resize(brushStrokePcd->points_.size(), red_alpha_1);

}

std::shared_ptr<geometry::PointCloud> generatePointCloud(const vector<Eigen::Vector3d>& points) const {
    // Create an Open3D PointCloud object
    auto pointCloud = std::make_shared<geometry::PointCloud>();
    pointCloud->points_ = open3d::utility::Vector3dVector(points);

    // Estimate normals (required for Poisson reconstruction)
    pointCloud->EstimateNormals();
    pointCloud->OrientNormalsConsistentTangentPlane(1);

    return pointCloud;
} 

std::shared_ptr<geometry::TriangleMesh> generateMesh(const std::shared_ptr<geometry::PointCloud>& pointCloud) const {
    // Create a shared pointer for TriangleMesh
    auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();

    // Perform Poisson surface reconstruction and get the mesh
    auto result = open3d::geometry::TriangleMesh::CreateFromPointCloudPoisson(*point_cloud);

    // Extract the TriangleMesh from the tuple
    *mesh = std::get<0>(result);

    return mesh;
}


std::shared_ptr<geometry::LineSet> generateLineSet(const vector<Eigen::Vector3d>& points) const {
    auto line_set = std::make_shared<open3d::geometry::LineSet>();

    //Add points to the line set
    line_set->points_ = points;

    // Add lines to the line set (note for now, we're just automatically connecting each 2 points neihbouring each other in the list)
    std::vector<Eigen::Vector2i> lines;
    for (size_t i = 0; i < points.size() - 1; ++i) {
        lines.push_back(Eigen::Vector2i(i, i + 1));
    }
    line_set->lines_ = lines;

    return line_set
}


