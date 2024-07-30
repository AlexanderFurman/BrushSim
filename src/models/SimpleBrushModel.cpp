#include "SimpleBrushModel.h"
#include "Conversions.h"
#include <cmath>
void SimpleBrushModel::updateState(const ISimStep& simStep){
    //Extract const data from simulation step
    m_brushStemPose = simStep.getPose(); // Pose of brush stem
    const Twist& twist = simStep.getTwist(); // Twist of brush stem
    const double timestamp = simStep.getTimeStamp(); // simulation timestamp

    //TODO:
    //-Find brush tip vector
    //-Find normal vectors of vt, wt using the velocity profile of the current stroke
    // (Maybe consider using a smoothing of a window of previous simsteps/footprints)
    // Calculate barebones [x,y] from equation in Wong paper.


    // UPDATE BRUSH STEM POSE & BRUSH TIP POSE
    Eigen::Matrix3d brushRotation = Conversions::eulerToRotationMatrix(m_brushStemPose.orientation); // convert orientation of the brush to a rotation vector
    m_brushTipPose = m_brushStemInitialPose; // reset pose of the brush tip
    m_brushTipPose.position = brushRotation * ( m_brushStemPose.position + Eigen::Vector3d(0,0,m_brushLength) ); // update the position of the brush tip's Pose to its new position
    m_brushTipPose.orientation = m_brushStemPose.orientation; // TODO: is this neccessary?

    // UPDATE BRUSH STEM TWIST
    m_brushStemTwist = twist; // reset pose of the brush tip

    // ESTIMATE w -- angle of rotation of the ellipse w.r.t. the x-axis of the world frame.
    double v_x = m_brushStemTwist.linearVelocity[0];
    double v_y = m_brushStemTwist.linearVelocity[1];
    double w = std::atan2(v_y, v_x);

    // ESTIMATE INITIAL VALUES OF v, u. (As if the brush footprint was a perfect circle)
    double brushTipPosition_z = m_brushTipPose.position[2];
    double u = m_brushRadius / (m_brushLength * (m_brushLength - brushTipPosition_z));
    double v = u;

    // Modifying factors of v, u -- These are used to deform the footprint into an ellipse when moving the brush on the page
    // In the paper they seem to be numbers generated upon trial and error.
    // Will input constant values for now, but can dafinitely update these values according to speed of brush
    double k_u = 1.2;
    double k_v = 1.7;
    
    // ellipse rotation modifying factor -- seemingly used when "extra rotation" is needed in a brushstroke
    // For now, will set a constant value, however it seems that the angle can actually be determined by comparing the
    // angle between the brush stroke direction, and the direction of the brush
    double rho = 0; // The paper uses values -0.6 <= rho <= 0, mostly 0

    double b = 0; // Another modifying factor -- for bias

    //Find (xi, yi) representing the pixel location painted by brush hair i

    Eigen::Vector2d brushTipPosition_xy;
    brushTipPosition_xy << m_brushTipPose.position[0],
                           m_brushTipPose.position[1];

    // Note its not clear here whether they used a vector or a matrix (the notation changed between 2 equations)
    // Trying matrix first, and checking the results. If its wrng, switch this to vector and do cross product
    Eigen::Matrix2d rotationMat;
    rotationMat << std::cos(w + rho), - std::sin(w + rho),
                   std::sin(w + rho), std::cos(w + rho);
    

    for (const Eigen::Vector2d hairPosition: m_hairBase){
        // Define the terms in the equation
        Eigen::Vector2d pixel_xy;
        double hairPosition_x = hairPosition[0];
        double hairPosition_y = hairPosition[1];
        Eigen::Vector2d hairPositionScaled;
        hairPositionScaled << hairPosition_x * (v * k_v + b) / m_brushRadius,
                              hairPosition_y * (u * k_u + b) / m_brushRadius;

        
        pixel_xy = brushTipPosition_xy + rotationMat * hairPositionScaled;
    }




};