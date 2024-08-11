#include <stdio.h>
#include "models/SimpleBrushModel.h"
#include "configs/JSONConfig.h"
#include "visualizers/Open3DVisualizer.h"
#include "visualizers/OpenCVVisualizer.h"
#include "simulation_data/SimulationData.h"
#include "utils/Geometric.h"


int main(int, char**){

    auto config = JSONConfig("testing/config.json");
    auto model = SimpleBrushModel();
    // auto visualizer = Open3DVisualizer();
    auto visualizer = OpenCVVisualizer();

    model.initialize(config);

    auto initialState = model.getResult();
    // visualizer.visualize(initialState);

    std::vector<int> timestamps = geometric::linspace<int>(0, 79, 80);
    std::vector<double> linspace_x = geometric::linspace<double>(20, 200, 80);
    std::vector<double> linspace_y_1 = geometric::linspace<double>(200, 200, 20);
    std::vector<double> linspace_y_2 = geometric::linspace<double>(200, 260, 60);
    std::vector<double> linspace_y = linspace_y_1;
    linspace_y.insert(linspace_y.end(), linspace_y_2.begin(), linspace_y_2.end());
    auto linspace_helper = geometric::linspace<double>(0, 2*M_PI, 80);
    std::vector<double> linspace_z;
    for (const double i: linspace_helper){
        // linspace_z.emplace_back(20 * (std::pow((std::sin(i/2)), 2)) + 1);
        linspace_z.emplace_back(1);
    }
    int n = timestamps.size();
    assert(n == linspace_y.size());

    auto steps = std::vector<SimStep>();
    double x, y, z;
    double v_x, v_y, v_z;
    double t = 0;
    double t_prev;
    Eigen::Vector3d orientation(0,0,0);
    Eigen::Vector3d angularVel(0,0,0);
    Eigen::Vector3d position, linearVel;
    Eigen::Vector3d position_prev;
    for (int i=0; i<n; i++){
        x = linspace_x[i];
        y = linspace_y[i];
        z = linspace_z[i];

        t = timestamps[i];

        if (t == t_prev){
            position = Eigen::Vector3d(x,y,z);
            linearVel = Eigen::Vector3d(1,0,0);
        }
        else{
            position = Eigen::Vector3d(x,y,z);
            linearVel = (position - position_prev)/(t - t_prev);
        }

        Pose pose = Pose(position, orientation);
        Twist twist = Twist(linearVel, angularVel);

        auto simStep = SimStep(pose, twist, t);
        steps.emplace_back(simStep);

        t_prev = t;
        position_prev = position;
    }

    double update, res, vis;
    using Clock = std::chrono::high_resolution_clock;
    using timePoint = std::chrono::_V2::system_clock::time_point;
    using duration = std::chrono::duration<double>;

    timePoint start;
    timePoint end;
    duration timeTaken;


    for (const auto step: steps){
        start = Clock::now();
        model.updateState(step);
        end = Clock::now();
        timeTaken = end - start;
        update = update + timeTaken.count();
        

        start = Clock::now();
        auto result = model.getResult();
        end = Clock::now();
        timeTaken = end - start;
        res = res + timeTaken.count();

        start = Clock::now();
        visualizer.visualize(result);
        end = Clock::now();
        timeTaken = end - start;
        vis = vis + timeTaken.count();
    }
    assert(n == steps.size());
    update = update/n;
    res = res/n;
    vis = vis/n;

    std::cout << "update: " << update << ", res: " << res << ", vis: " << vis << "." << std::endl;
    std::cout << ".";
}
