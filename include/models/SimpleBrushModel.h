#ifndef SIMPLEBRUSHMODEL_H
#define SIMPLEBRUSHMODEL_H

#include "IModel.h"
#include "Pose.h"

// Definition of the brush footprint
// The footprint describes the contact area between the brush and the paper
// We assume that the paper is non-blocking, and the brush can pass through it
// When contact is made, an ellipse is formed
struct FootPrint{
    double u; // Major radius of the intersecting ellipse
    double v; // Minor radius of the intersecting ellipse
    double w; // angle of rotation of the intersecting ellipse
    double rho; // secondary angle of rotation of the intersecting ellipse (user defined?)
    double k_u; // modifying factor for u
    double k_v; // modifying factor for v
    double b;
};

// This brush model comes from the paper Virtual brush: a model-based synthesis of Chinese calligraphy (Wong et al.)
class SimpleBrushModel: public IModel{
    private:
    // Static Variables
    const double brush_radius; // length of radius of the brush stem
    const double brush_length; // length of brush stem to tip of brush hair
    const double hair_length; //length of each hair in the brush (note, for now we are assuming each hair is exactly brush_length long)
    const Pose world_frame; // World frame is defined as the bottom left corner of the paper/canvas. position=[0,0,0], orientation=[0,0,0]
    const Pose paper; // pose of center of the paper, relative to world frame.
    //TODO: If this (below) is too slow, replace using Eigen batch multiplication
    const vector<Pose> hair_base; // Collection of poses of all the bases of each hair of the brush, relative to the brush stem

    //Dynamic Variables
    Pose brush_stem; // pose of center of stem of the brush, relative to world frame.
    Pose brush_tip; //pose of the tip of the brush
    FootPrint footprint; // information about the footprint of the brush

};

#endif // SIMPLEBRUSHMODEL_H