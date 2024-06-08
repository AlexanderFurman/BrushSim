#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "ISimulator.h"
#include "ModelFactory.h"
//TODO: make a visualizer factory

// Abstract class in charge of performing a simulation
class Simulator: public ISimulator{
    public:
    virtual void initialize(const IConfig& configRef) override;
    virtual void simulate() override;
    virtual void step(const ISimStep& step) override;
    virtual void reset() override;

    private:
    unique_ptr<IConfig> config; //Internal Configuration of the simulation
    unique_ptr<IModel> model; // Model used during the simulation
    vector<unique_ptr<ISimStep>> steps; // list of individual simulation steps
    vector<ISimResult> results; // list of results from the simulation steps
    unique_ptr<IVisualizer> visualizer; // visualizer object
};


#endif // SIMULATOR_H


