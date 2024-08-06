#ifndef JSONCONFIG_H
#define JSONCONFIG_H

#include "interfaces/IConfig.h"
#include "json.hpp"


class JSONConfig: public IConfig{
    using json = nlohmann::json;
    public:
    // Constructors & Destructors
    JSONConfig(const std::string& filename);

    // Get configs
    const std::map<std::string, std::string>& getSimulatorConfig() const;
    const std::map<std::string, std::string>& getModelConfig() const;
    const std::map<std::string, std::string>& getVisualizerConfig() const;

    private:
    std::map<std::string, std::string> m_simulatorConfig;
    std::map<std::string, std::string> m_modelConfig;
    std::map<std::string, std::string> m_visualizerConfig;

    std::map<std::string, std::string> jsonToMap(const json& j) const;
};


#endif // JSONCONFIG_H