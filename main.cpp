#include <stdio.h>
#include "models/SimpleBrushModel.h"
#include "configs/JSONConfig.h"
#include "visualizers/Open3DVisualizer.h"
#include "simulation_data/SimulationData.h"


int main(int, char**){
    ////open3d check
    // printf("Hello, from BrushSim!\n");

    // // Path to the point cloud file
    // std::string filename = "/home/alex/Projects/bunny.ply";

    // // Initialize Open3D
    // std::cout << "Initializing Open3D..." << std::endl;

    // // Load the point cloud from the file
    // auto pcd = std::make_shared<open3d::geometry::PointCloud>();
    // if (open3d::io::ReadPointCloud(filename, *pcd)) {
    //     std::cout << "Point cloud successfully loaded from " << filename << std::endl;
    //     std::cout << "Number of points: " << pcd->points_.size() << std::endl;
    // } else {
    //     std::cerr << "Failed to load point cloud from " << filename << std::endl;
    //     return -1;
    // }

    // // Optional: Visualize the point cloud using Open3D's visualizer
    // auto vis = std::make_shared<open3d::visualization::Visualizer>();
    // vis->CreateVisualizerWindow("Open3D Point Cloud Viewer", 800, 600);
    // vis->AddGeometry(pcd);
    // vis->Run();
    // vis->DestroyVisualizerWindow();

    // return 0;

    auto config = JSONConfig("testing/config.json");
    auto model = SimpleBrushModel();
    auto visualizer = Open3DVisualizer();

    model.initialize(config);

    auto initialState = model.getResult();
    visualizer.visualize(initialState);

    // Pose p1 = Pose(Eigen::Vector3d(0,0,10), Eigen::Vector3d(0,0,0));
    // Twist t1 = Twist(Eigen::Vector3d(0,0,-1), Eigen::Vector3d(0,0,0));
    // double timestamp = 2;
    // auto simStep = SimStep(p1, t1, timestamp);

    // model.updateState(simStep);
    // auto result1 = model.getResult();
    // visualizer.visualize(result1);
    

    


}
