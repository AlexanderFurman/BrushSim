#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "interfaces/ISimulator.h"
#include "interfaces/IVisualizer.h"
#include "interfaces/IModel.h"
//TODO: make a visualizer factory

// Abstract class in charge of performing a simulation
class Simulator: public ISimulator{
    public:
    virtual void initialize(const IConfig& configRef) override;
    virtual void simulate() override;
    virtual void step(const SimStep& step) override;
    virtual void reset() override;

    private:
    unique_ptr<IConfig> m_config; //Internal Configuration of the simulation
    unique_ptr<IModel> m_model; // Model used during the simulation
    vector<unique_ptr<SimStep>> m_steps; // list of individual simulation steps
    vector<SimResult> m_results; // list of results from the simulation steps
    unique_ptr<IVisualizer> m_visualizer; // visualizer object
};


#endif // SIMULATOR_H


