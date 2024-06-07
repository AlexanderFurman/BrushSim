#ifndef ISIMULATOR_H
#define ISIMULATOR_H

#include "IConfig.h"

// Abstract class in charge of performing a simulation
class ISimulator{
    public:
    virtual void initialize(const IConfig& config);
    virtual void simulate() = 0;
    virtual void reset();
    virtual ~ISimulator() = default;

    private:
    const IConfig* config; //Internal Configuration of the simulation
};


#endif