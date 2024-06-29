#ifndef ISIMULATOR_H
#define ISIMULATOR_H

#include "IConfig.h"
#include <memory>
#include <vector>

using namespace std;

// Abstract class in charge of performing a simulation
class ISimulator{
    public:
    virtual void initialize(const IConfig& config) = 0;
    virtual void simulate() = 0;
    virtual void step(const ISimStep& step) = 0;
    virtual void reset() = 0;
    virtual ~ISimulator() = default;
};


#endif