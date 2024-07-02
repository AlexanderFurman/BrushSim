#include "SimpleBrushModel.h"

void SimpleBrushModel::updateState(const ISimStep& simStep){
    //Extract data from simulation step
    const Pose& pose = simStep.getPose(); // Pose of brush stem
    const Twist& twist = simStep.getTwist(); // Twist of brush stem
    const double timestamp = simStep.getTimeStamp(); // simulation timestamp

    //TODO:
    //-Find brush tip vector
    //-Find normal vectors of vt, wt using the velocity profile of the current stroke
    // (Maybe consider using a smoothing of a window of previous simsteps/footprints)
    // Calculate barebones [x,y] from equation in Wong paper.



}