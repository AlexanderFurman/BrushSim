#ifndef SIMULATIONDATA_H
#define SIMULATIONDATA_H

#include "ISimulationData.h"

struct SimStep: public ISimulationData {
    public:
    const Pose brushStemPose;
    const Twist brushStemTwist;
    const double timeStamp;
};

struct SimResult: public ISimulationData {
    public:
    const Eigen::MatrixXd& brushStroke; // instantaneous brush stroke on the canvas
    const Eigen::Vector3d& brushStemPosition; 
    const Eigen::Vector3d& brushTipPosition; // change this -- may not be able to use this for otehr more complex models
    const Eigen::Vector3d& strokeDirection;
    const vector<Eigen::Vector3d>& vertices; // vertices describing the brush's position in 3D
    const double timeStamp;
};


#endif // SIMULATIONDATA_H