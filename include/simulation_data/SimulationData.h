#ifndef SIMULATIONDATA_H
#define SIMULATIONDATA_H

#include "interfaces/ISimulationData.h"

struct SimStep: public ISimulationData {
    public:
    // Constructor
    SimStep(const Pose& brushStemPose, const Twist& brushStemTwist, const double timeStamp)
    : m_brushStemPose(brushStemPose), m_brushStemTwist(brushStemTwist), m_timeStamp(timeStamp) {}

    Pose getBrushPose() const {return m_brushStemPose;}
    Twist getBrushTwist() const {return m_brushStemTwist;}
    double getTimeStamp() const {return m_timeStamp;}
    
    private:
    Pose m_brushStemPose;
    Twist m_brushStemTwist;
    double m_timeStamp;
};

class SimResult: public ISimulationData {
    public:
    SimResult(const Eigen::MatrixXd& brushStroke, const Eigen::Vector3d& brushStemPosition, const Eigen::Vector3d& strokeDirection, 
                const std::vector<Eigen::Vector3d>& vertices, const double timeStamp)
                : m_brushStroke(brushStroke), m_brushStemPosition(brushStemPosition), m_strokeDirection(strokeDirection),
                    m_vertices(vertices), m_timeStamp(timeStamp) {}
    
    Eigen::MatrixXd getBrushStroke() const {return m_brushStroke;}
    Eigen::Vector3d getBrushPosition() const {return m_brushStemPosition;}
    Eigen::Vector3d getBrushNormal() const {return m_brushNormal;}
    Eigen::Vector3d getBrushStrokeDirection() const {return m_strokeDirection;}
    std::vector<Eigen::Vector3d> getBrushVertices() const {return m_vertices;}
    double getTimeStamp() const {return m_timeStamp;}
    

    private:
    Eigen::MatrixXd m_brushStroke; // instantaneous brush stroke on the canvas
    Eigen::Vector3d m_brushStemPosition; 
    Eigen::Vector3d m_brushNormal;
    // Eigen::Vector3d& brushTipPosition; // change this -- may not be able to use this for otehr more complex models
    Eigen::Vector3d m_strokeDirection;
    std::vector<Eigen::Vector3d> m_vertices; // vertices describing the brush's position in 3D
    double m_timeStamp;
};


#endif // SIMULATIONDATA_H