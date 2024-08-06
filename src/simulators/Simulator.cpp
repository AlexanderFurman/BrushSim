#include "simulators/Simulator.h"
#include "models/ModelFactory.h"

void Simulator::initialize(const IConfig& config){
    model = ModelFactory::createModel(config);
}

void Simulator::reset(){
    config = nullptr;
    model = nullptr;
}

void Simulator::simulate() {
    for (const auto& stepPtr: steps) {
        step(*stepPtr);
    }
}

void Simulator::step(const SimStep& step) {
    model->updateState(step);
    const SimResult& result = model->getResult();
    results.emplace_back(result);
}