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
    double brushTipPosition_z = m_brushTipPose.position[3];
    double u = m_brushRadius / (m_brushLength * (m_brushLength - brushTipPosition_z));
    double v = u;

    









};