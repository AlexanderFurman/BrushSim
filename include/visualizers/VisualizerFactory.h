#ifndef VISUALIZERFACTORY_H
#define VISUALIZERFACTORY_H

#include "interfaces/IVisualizer.h"
#include "interfaces/IConfig.h"
#include "visualizers/Open3DVisualizer.h"
#include "visualizers/OpenCVVisualizer.h"
#include <memory>

class VisualizerFactory{
    public:
    static std::unique_ptr<IVisualizer> createModel(const IConfig& config){
        const auto& visConfig = config.getVisualizerConfig();
        // Find model
        const auto& it = visConfig.find("visualizer type");
        if (it == visConfig.end())
            throw std::invalid_argument("Visualizer type not specified in config");
        const auto& modelType = it->second;
        
        // If model found, return pointer to new instance of the model
        IVisualizer* vis = createModelInstace(modelType);

        vis->initialize(config);

        return std::unique_ptr<IVisualizer>(vis);
    }


    private:
     //TODO: dont really like this here, find better place/method to store this mapping
    static IVisualizer* createModelInstace(const std::string& name){
        if (name == "open3d")
            return static_cast<IVisualizer*>(new Open3DVisualizer());
        else if (name == "opencv")
            return static_cast<IVisualizer*>(new OpenCVVisualizer());
        else
            throw std::invalid_argument("Unknown visualizer type");
    }
};

#endif // VISUALIZERFACTORY_H
