#include <stdio.h>
// #include <iostream>
// #include <open3d/Open3D.h>
#include "models/SimpleBrushModel.h"

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


    auto model = SimpleBrushModel();
    


}
