#ifndef ICONFIG_H
#define ICONFIG_H

#include <string>
#include "IModel.h"
#include "IVisualizer.h"
#include <map>

using namespace std;

class IConfig {
public:
    // Constructors & Destructors
    IConfig(const std::string& filename);
    virtual ~IConfig() = default;
    IConfig(const IConfig& other);
    
    // Get configs
    virtual map<string, string> getSimulatorConfig() const = 0;
    virtual map<string, string> getModelConfig() const = 0;
    virtual map<string, string> getVisualizationConfig() const = 0;
};

#endif