#ifndef ICONFIG_H
#define ICONFIG_H

#include <string>
#include <map>

class IConfig {
public:

    //virtual destructor
    virtual ~IConfig() = default;

    // Get configs
    virtual const std::map<std::string, std::string>& getSimulatorConfig() const = 0;
    virtual const std::map<std::string, std::string>& getModelConfig() const = 0;
    virtual const std::map<std::string, std::string>& getVisualizerConfig() const = 0;

    private:
    std::map<std::string, std::string> m_simulatorConfig;
    std::map<std::string, std::string> m_modelConfig;
    std::map<std::string, std::string> m_visualizerConfig;
};

#endif