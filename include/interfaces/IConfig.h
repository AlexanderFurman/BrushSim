#ifndef ICONFIG_H
#define ICONFIG_H

#include <string>

class IConfig {
public:
    virtual void loadConfig(const std::string& filename) = 0;
    virtual ~IConfig() = default;
};

#endif