#include "configs/JSONConfig.h"
#include <fstream>
#include "json.hpp"

using json = nlohmann::json;

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

const map<string, string>& JSONConfig::getModelConfig() const{
    return m_modelConfig;
};

const map<string, string>& JSONConfig::getSimulatorConfig() const{
    return m_simulatorConfig;
};

const map<string, string>& JSONConfig::getVisualizerConfig() const{
    return m_visualizerConfig;
};

map<string, string> JSONConfig::jsonToMap(const json& j) const{
    map<string, string> result;
    for (json::const_iterator it = j.begin(); it != j.end(); ++it) {
        result[it.key()] = it.value();
    }
    return result;
};