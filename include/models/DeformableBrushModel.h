#ifndef DEFORMABLEBRUSHMODEL_H
#define DEFORMABLEBRUSHMODEL_H

#include "IModel.h"

class DeformableBrushModel: public IModel{
    void initialize(const IConfig& config) override;
    void updateState(const ISimStep& simStep) override;
    const ISimResult& getResult() const override;
    void reset() override;
};

#endif // DEFORMABLEBRUSHMODEL_H