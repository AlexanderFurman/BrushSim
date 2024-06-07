#ifndef ISIMULATIONDATA_H
#define ISIMULATIONDATA_H

// Base struct representing simulation data
struct ISimulationData {
    virtual ~ISimulationData() = default;
};

// Abstract struct holding data about which conditions have changed in current simulation step
struct ISimStep: public ISimulationData{
    virtual ~ISimStep() = default;
};

// Abstract struct holding data from the result of the simulation step
struct ISimResult: public ISimulationData{
    virtual ~ISimResult() = default;
};

#endif