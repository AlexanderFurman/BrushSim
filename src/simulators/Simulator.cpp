#include "Simulator.h"

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

void Simulator::step(const ISimStep& step) {
    model->updateState(step);
    const ISimResult& result = model->getResult();
    results.emplace_back(result);
}