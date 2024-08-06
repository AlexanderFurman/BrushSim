#ifndef MODELFACTORY_H
#define MODELFACTORY_H

#include "interfaces/IModel.h"
#include <unordered_map>
#include <string>
#include <stdexcept>
#include <memory>

using namespace std;

//include implemented Brush Models here:
#include "models/SimpleBrushModel.h"
#include "models/DeformableBrushModel.h"

// // map of string to creation of pointer to new model object -- If implement new model, add mapping here
// static const unordered_map<string, IModel*> modelMap = {
//     {"DeformableBrushModel", []() -> IModel* { return static_cast<IModel*>(new DeformableBrushModel()); }},
//     {"SimpleBrushModel", []() -> IModel* { return static_cast<IModel*>(new SimpleBrushModel()); }}
// };

class ModelFactory {
    public:
    
    static unique_ptr<IModel> createModel(const IConfig& config) {
        const auto& modelConfig = config.getModelConfig();
        // Find model
        const auto& it = modelConfig.find("modelType");
        if (it == modelConfig.end())
            throw std::invalid_argument("modelType not specified in config");
        const auto& modelType = it->second;
        
        // If model found, return pointer to new instance of the model
        IModel* model = createModelInstace(modelType);

        model->initialize(config);

        return unique_ptr<IModel>(model);
    }


    private:

    //TODO: dont really like this here, find better place/method to store this mapping
    static IModel* createModelInstace(const string& name){
        // if (name == "DeformableBrushModel")
        //     return static_cast<IModel*>(new DeformableBrushModel()); //TODO: return this later
        if (name == "SimpleBrushModel")
            return static_cast<IModel*>(new SimpleBrushModel());
        throw std::invalid_argument("Unknown model type");
    }

};

#endif // MODELFACTORY_H