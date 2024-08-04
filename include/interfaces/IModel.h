#ifndef IMODEL_H
#define IMODEL_H

#include "ISimulationData.h"
#include "IConfig.h"

class IModel{
    public:
    virtual ~IModel() = default;
    virtual void initialize(const IConfig& config) = 0;
    virtual void updateState(const SimStep& simStep) = 0;
    virtual const SimResult& getResult() const = 0;
    virtual void reset() = 0;
};


#endif