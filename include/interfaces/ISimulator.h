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

    private:
    unique_ptr<IConfig> config; //Internal Configuration of the simulation
    unique_ptr<IModel> model; // Model used during the simulation
    vector<unique_ptr<ISimStep>> steps; // list of individual simulation steps
    std::vector<ISimResult> results; // list of results from the simulation steps
    unique_ptr<IVisualizer> visualizer; // visualizer object
};


#endif