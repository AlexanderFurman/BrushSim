#ifndef ISIMULATIONDATA_H
#define ISIMULATIONDATA_H

#include "data_types/Pose.h"
#include "data_types/Twist.h"

// Base struct representing simulation data
struct ISimulationData {
    virtual ~ISimulationData() = default;
    // virtual const double getTimeStamp() const = 0;
};

//Feel these are unnecessary and don't achieve the goal of data transfer with ease
// // Abstract struct holding data about which conditions have changed in current simulation step
// struct ISimStep: public ISimulationData{
//     virtual ~ISimStep() = default;
//     virtual const Pose& getPose() const = 0;
//     virtual const Twist& getTwist() const = 0;
//     //TODO: think if there is a better way for transferring data between simulator and model
// };

// // Abstract struct holding data from the result of the simulation step
// struct ISimResult: public ISimulationData{
//     virtual ~ISimResult() = default;
//     virtual const Pose& getBrushPose() const = 0;
// };





#endif