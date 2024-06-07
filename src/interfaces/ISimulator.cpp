#include "ISimulator.h"

void ISimulator::initialize(const IConfig& configRef){
    config = &configRef;
}

void ISimulator::reset(){
    config = nullptr;
}