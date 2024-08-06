#include "configs/JSONConfig.h"
#include <fstream>

JSONConfig::JSONConfig(const std::string& filename){
    // Load JSON configuration from file
    std::ifstream file(filename);
    json jsonDict;
    file >> jsonDict;

    // Convert JSON sub-dicts to std::map
    m_simulatorConfig = jsonToMap(jsonDict["simulator"]);
    m_modelConfig = jsonToMap(jsonDict["model"]);
    m_visualizerConfig = jsonToMap(jsonDict["visualizer"]);
};

const std::map<std::string, std::string>& JSONConfig::getModelConfig() const{
    return m_modelConfig;
};

const std::map<std::string, std::string>& JSONConfig::getSimulatorConfig() const{
    return m_simulatorConfig;
};

const std::map<std::string, std::string>& JSONConfig::getVisualizerConfig() const{
    return m_visualizerConfig;
};

std::map<std::string, std::string> JSONConfig::jsonToMap(const json& j) const{
    std::map<std::string, std::string> result;
    for (json::const_iterator it = j.begin(); it != j.end(); ++it) {
        result[it.key()] = it.value();
    }
    return result;
};