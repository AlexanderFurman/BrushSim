#ifndef IVISUALIZER_H
#define IVISUALIZER_H

#include "ISimulationData.h"

class IVisualizer{
    public:
    virtual ~IVisualizer() = default;
    virtual void visualize(const ISimulationData& data) = 0;
};


#endif