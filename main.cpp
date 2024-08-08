#include <stdio.h>
#include "models/SimpleBrushModel.h"
#include "configs/JSONConfig.h"
#include "visualizers/Open3DVisualizer.h"
#include "visualizers/OpenCVVisualizer.h"
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
    // auto visualizer = Open3DVisualizer();
    auto visualizer = OpenCVVisualizer();

    model.initialize(config);

    // auto initialState = model.getResult();
    // visualizer.visualize(initialState);

    // Pose p1 = Pose(Eigen::Vector3d(100,100,10), Eigen::Vector3d(0,0,0));
    // Twist t1 = Twist(Eigen::Vector3d(0,0,-1), Eigen::Vector3d(0,0,0));
    // double time1 = 2;
    // auto simStep1 = SimStep(p1, t1, time1);

    // Pose p2 = Pose(Eigen::Vector3d(110,100,10), Eigen::Vector3d(0,0,0));
    // Twist t2 = Twist(Eigen::Vector3d(3,0,0), Eigen::Vector3d(0,0,0));
    // double time2 = 4;
    // auto simStep2 = SimStep(p2, t2, time2);

    Pose p3 = Pose(Eigen::Vector3d(120,100,5), Eigen::Vector3d(0,0,0));
    Twist t3 = Twist(Eigen::Vector3d(4,0,0), Eigen::Vector3d(0,0,0));
    double time3 = 6;
    auto simStep3 = SimStep(p3, t3, time3);

    Pose p4 = Pose(Eigen::Vector3d(120,110,5), Eigen::Vector3d(0,0,0));
    Twist t4 = Twist(Eigen::Vector3d(0,2,0), Eigen::Vector3d(0,0,0));
    double time4 = 8;
    auto simStep4 = SimStep(p4, t4, time4);

    Pose p5 = Pose(Eigen::Vector3d(120,120,5), Eigen::Vector3d(0,0,0));
    Twist t5 = Twist(Eigen::Vector3d(0,3,0), Eigen::Vector3d(0,0,0));
    double time5 = 10;
    auto simStep5 = SimStep(p5, t5, time5);

    auto steps = std::vector<SimStep>();
    // steps.emplace_back(simStep1);
    // steps.emplace_back(simStep2);
    steps.emplace_back(simStep3);
    steps.emplace_back(simStep4);
    steps.emplace_back(simStep5);


    for (const auto step: steps){
        model.updateState(step);
        auto result1 = model.getResult();
        visualizer.visualize(result1);
    }
    

    


}
