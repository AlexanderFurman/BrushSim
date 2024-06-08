#ifndef MODELFACTORY_H
#define MODELFACTORY_H

#include "IModel.h"
#include <unordered_map>
#include <string>
#include <stdexcept>
#include <memory>

using namespace std;

//include implemented Brush Models here:
#include "SimpleBrushModel.h"
#include "DeformableBrushModel.h"

// map of string to creation of pointer to new model object -- If implement new model, add mapping here
static const unordered_map<string, IModel*> modelMap = {
    {"DeformableBrushModel", []() -> IModel* { return static_cast<IModel*>(new DeformableBrushModel()); }},
    {"SimpleBrushModel", []() -> IModel* { return static_cast<IModel*>(new SimpleBrushModel()); }}
};

class ModelFactory {
    public:
    
    static unique_ptr<IModel> createModel(const IConfig& config) {
        const auto& modelConfig = config.getModelConfig();
        // Find model
        const auto& it = modelConfig.find("modelType");
        if (it == modelConfig.end())
            throw std::invalid_argument("modelType not specified in config");
        const auto& modelType = it->second;

        // Check if model exists in mapping
        const auto& it = modelMap.find(modelType);

        //TODO: debug syntax from modelMap
        if (it == modelMap.end()) {
            throw std::invalid_argument("Unknown model type");
        
        // If model found, return pointer to new instance of the model
        unique_ptr<IModel> model = make_unique<IModel>(it->second);

        model->initialize(config);

        return model;
    }

};

#endif // MODELFACTORY_H