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
    
    static IModel* createModel(const string& modelType) {
        // Check if model exists in mapping
        auto it = modelMap.find(modelType);

        // If model found, return pointer to new instance of the model
        if (it != modelMap.end()) {
            return it->second();
        //Otherwise throw an error
        } else {
            throw std::invalid_argument("Unknown model type");
        }
    }

};

#endif // MODELFACTORY_H