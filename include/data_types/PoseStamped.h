#ifndef POSESTAMPED_H
#define POSESTAMPED_H

#include "ISimulationData.h"
#include <vector>

struct PoseStamped: public ISimStep{
    public:
    double timestamp; // Time in seconds since a start point
    double x, y, z; // position
    double roll, pitch, yaw; // orientation using Euler angles
};


#endif // POSESTAMPED_H