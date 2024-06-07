#include "ISimulator.h"

void ISimulator::initialize(const IConfig& configRef){
    config = make_unique<IConfig>(configRef);
    // TODO: Add assignment to model and steps
}

void ISimulator::reset(){
    config = nullptr;
}

void ISimulator::simulate() {
    for (const auto& stepPtr: steps) {
        step(*stepPtr);
    }
}

void ISimulator::step(const ISimStep& step) {
    model->updateState(step);
    const ISimResult& result = model->getResult();
    results.emplace_back(result);
}