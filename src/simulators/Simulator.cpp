#include "simulators/Simulator.h"
#include "models/ModelFactory.h"

void Simulator::initialize(const IConfig& config){
    m_model = ModelFactory::createModel(config);
}

void Simulator::reset(){
    m_config = nullptr;
    m_model = nullptr;
}

void Simulator::simulate() {
    for (const auto& stepPtr: m_steps) {
        step(*stepPtr);
    }
}

void Simulator::step(const SimStep& step) {
    m_model->updateState(step);
    const SimResult& result = m_model->getResult();
    // visualizer->visualize(result);
    m_results.emplace_back(result);
}