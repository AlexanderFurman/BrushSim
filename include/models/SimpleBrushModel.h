#ifndef SIMPLEBRUSHMODEL_H
#define SIMPLEBRUSHMODEL_H

#include "interfaces/IModel.h"
#include "data_types/Pose.h"
#include "data_types/Twist.h"

// Definition of the brush footprint
// The footprint describes the contact area between the brush and the paper
// We assume that the paper is non-blocking, and the brush can pass through it
// When contact is made, an ellipse is formed
struct FootPrint{
    double u; // Major radius of the intersecting ellipse
    double v; // Minor radius of the intersecting ellipse
    double w; // angle of rotation of the intersecting ellipse -- 
    double rho; // secondary angle of rotation of the intersecting ellipse (user defined?)
    double k_u; // modifying factor for u
    double k_v; // modifying factor for v
    double b; // modifying factor for u, v -- bias

    //TODO: Possibly convert this to 3 chanel matrix for colors Eigen::Matrix<Eigen::Vector3d, Eigen::Dynamic, Eigen::Dynamic> image(rows, cols);
    Eigen::MatrixXd paintDeposited; // pixels painted by current brush stroke, with percent of paint deposited (value between 0-1)
    Eigen::MatrixXd boundary; // Boundary of the current footprint
};

// This brush model comes from the paper Virtual brush: a model-based synthesis of Chinese calligraphy (Wong et al.)
class SimpleBrushModel: public IModel{
    public:
    SimpleBrushModel(){};
    void updateState(const SimStep& simStep) override;
    void initialize(const IConfig& config) override;
    const SimResult getResult() const override;
    void reset() override;



    private:
    // Static Variables
    double m_brushRadius; // length of radius of the brush stem
    double m_brushLength; // length of brush stem to tip of brush hair
    double m_hairLength; //length of each hair in the brush (note, for now we are assuming each hair is exactly brush_length long)
    //TODO: If this (below) is too slow, replace using Eigen batch multiplication
    std::vector<Eigen::Vector3d> m_hairBase; // Collection of positions of all the bases of each hair of the brush, relative to the brush stem
    Pose m_brushStemInitialPose; // Initial Pose (position + orientation) of the brush, relative to the brush stem. 
                                        // We assume brush initially points directly downwards and is in the top left corner, with some elevation off the page (position = [0,0,z], orientation = [0,0,0])
    Twist m_brushStemInitialTwist; //Initial twist of the brush stem
    Eigen::Vector3d m_brushInitialNormal; // Initial normal of the brush [0,0,1] (out of the page)

    //Dynamic Variables
    Pose m_brushStemPose; // pose of center of stem of the brush, relative to world frame.
    Twist m_brushStemTwist; // twist of center of stem of the brush, relative to world frame.
    Eigen::Vector3d m_brushNormal;
    Eigen::Vector3d m_brushTipPosition; //position of the tip of the brush
    FootPrint m_footprint; // information about the footprint of the brush
    Eigen::MatrixXd m_canvas; // canvas with ink/paint deposited (will be used later for checking how much ink left in brush)
    double m_timeStamp; //current timestamp

    std::vector<Eigen::Vector3d> generateHandleVertices() const;
    std::vector<Eigen::Vector3d> generateBrushVertices() const;
    void updateBrushState(const SimStep& simStep);
    void updateFootPrintState();
    void generateBrushStroke();
    void generateFootPrintBoundary();
    void updateCanvasState();

    //TODO: Add function for updating the ink levels of each brush hair
};

#endif // SIMPLEBRUSHMODEL_H