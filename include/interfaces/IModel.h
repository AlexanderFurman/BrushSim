#ifndef IMODEL_H
#define IMODEL_H

#include "ISimulationData.h"
#include "IConfig.h"

class IModel{
    public:
    virtual ~IModel() = default;
    virtual void initialize(const IConfig& config) = 0;
    virtual void updateState(const ISimStep& simStep) = 0;
    virtual const ISimResult& getResult() = 0; //should I add const agter getResult()? will it destroy reference after leaving scope?
    virtual void reset() = 0;
};


#endif