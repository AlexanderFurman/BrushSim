#ifndef SIMULATIONDATA_H
#define SIMULATIONDATA_H

#include "interfaces/ISimulationData.h"

struct SimStep {
    Pose pose;
    Twist twist;
    double timestamp;
};

struct SimResult {
    Eigen::MatrixXd brushStroke; // instantaneous brush stroke on the canvas (NOTE: replace this with some representation for 3 channeled color image)
    std::vector<Eigen::Vector3d> vertices; // vertices describing the brush's position in 3D
};


#endif // SIMULATIONDATA_H