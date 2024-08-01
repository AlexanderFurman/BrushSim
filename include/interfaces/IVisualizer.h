#ifndef IVISUALIZER_H
#define IVISUALIZER_H

#include "ISimulationData.h"

class IVisualizer{
    public:
    virtual ~IVisualizer() = default;
    virtual void visualize(const ISimResult& data) = 0;
    virtual void initialize(const IConfig& config) = 0;
};


#endif