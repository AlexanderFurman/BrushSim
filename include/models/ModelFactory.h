#ifndef MODELFACTORY_H
#define MODELFACTORY_H

#include "interfaces/IModel.h"
#include <unordered_map>
#include <string>
#include <stdexcept>
#include <memory>

//include implemented Brush Models here:
#include "models/SimpleBrushModel.h"
#include "models/DeformableBrushModel.h"


class ModelFactory {
    public:
    
    static std::unique_ptr<IModel> createModel(const IConfig& config) {
        const auto& modelConfig = config.getModelConfig();
        // Find model
        const auto& it = modelConfig.find("model type");
        if (it == modelConfig.end())
            throw std::invalid_argument("Model type not specified in config");
        const auto& modelType = it->second;
        
        // If model found, return pointer to new instance of the model
        IModel* model = createModelInstace(modelType);

        model->initialize(config);

        return unique_ptr<IModel>(model);
    }


    private:

    //TODO: dont really like this here, find better place/method to store this mapping
    static IModel* createModelInstace(const std::string& name){
        // if (name == "DeformableBrushModel")
        //     return static_cast<IModel*>(new DeformableBrushModel()); //TODO: return this later
        if (name == "simple")
            return static_cast<IModel*>(new SimpleBrushModel());
        throw std::invalid_argument("Unknown model type");
    }

};

#endif // MODELFACTORY_H