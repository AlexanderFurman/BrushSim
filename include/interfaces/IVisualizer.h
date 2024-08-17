#ifndef IVISUALIZER_H
#define IVISUALIZER_H

#include "simulation_data/SimulationData.h"
#include "interfaces/IConfig.h"

class IVisualizer{
    public:
    virtual ~IVisualizer() = default;
    virtual void visualize(const SimStep& step, const SimResult& result) = 0;
    virtual void initialize(const IConfig& config) = 0;
};


#endif