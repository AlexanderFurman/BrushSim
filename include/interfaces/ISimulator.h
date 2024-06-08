#ifndef ISIMULATOR_H
#define ISIMULATOR_H

#include "IConfig.h"
#include <memory>
#include <vector>

using namespace std;

// Abstract class in charge of performing a simulation
class ISimulator{
    public:
    virtual void initialize(const IConfig& configRef);
    virtual void simulate();
    virtual void step(const ISimStep& step);
    virtual void reset();
    virtual ~ISimulator() = default;
};


#endif