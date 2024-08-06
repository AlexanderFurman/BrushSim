#include "models/SimpleBrushModel.h"
#include "utils/Conversions.h"
#include "utils/Geometric.h"
#include "simulation_data/SimulationData.h"
#include "utils/Parsing.h"
#include <cmath>
#include <random>

//create random seed for testing
unsigned int seed = 42;
std::mt19937 gen(seed);
std::uniform_real_distribution<> dis(0.0, 1.0);

void SimpleBrushModel::reset(){
    //TODO: find point for this 
}

void SimpleBrushModel::updateState(const SimStep& simStep){
    //TODO: Break this up into multiple methods

    //Extract const data from simulation step
    m_brushStemPose = simStep.getBrushPose(); // Pose of brush stem
    const Twist& twist = simStep.getBrushTwist(); // Twist of brush stem
    const double timestamp = simStep.getTimeStamp(); // simulation timestamp

    //UPDATE TIMESTAMP
    m_timeStamp = timestamp;

    // //Save the current simdata
    // m_currentStepData = simStep;

    // UPDATE BRUSH STEM POSE & BRUSH TIP POSE
    Eigen::Matrix3d brushRotation = conversions::eulerToRotationMatrix(m_brushStemPose.orientation); // convert orientation of the brush to a rotation vector
    m_brushTipPosition = brushRotation * ( m_brushStemPose.position + Eigen::Vector3d(0,0,-m_brushLength) ); // update the position of the brush tip's position

    // UPDATE BRUSH STEM TWIST
    m_brushStemTwist = twist; // reset pose of the brush tip

    // ESTIMATE w -- angle of rotation of the ellipse w.r.t. the x-axis of the world frame.
    double v_x = m_brushStemTwist.linearVelocity[0];
    double v_y = m_brushStemTwist.linearVelocity[1];
    double w = std::atan2(v_y, v_x);

    // ESTIMATE INITIAL VALUES OF v, u. (As if the brush footprint was a perfect circle)
    double brushTipPosition_z = m_brushTipPosition[2];
    m_footprint.u = m_brushRadius / (m_brushLength * (m_brushLength - brushTipPosition_z));
    m_footprint.v = m_footprint.u;

    // Modifying factors of v, u -- These are used to deform the footprint into an ellipse when moving the brush on the page
    // In the paper they seem to be numbers generated upon trial and error.
    // Will input constant values for now, but can dafinitely update these values according to speed of brush
    m_footprint.k_u = 1.2;
    m_footprint.k_v = 1.7;
    
    // ellipse rotation modifying factor -- seemingly used when "extra rotation" is needed in a brushstroke
    // For now, will set a constant value, however it seems that the angle can actually be determined by comparing the
    // angle between the brush stroke direction, and the direction of the brush
    double rho = 0; // The paper uses values -0.6 <= rho <= 0, mostly 0

    double b = 0; // Another modifying factor -- for bias

    //Find (xi, yi) representing the pixel location painted by brush hair i

    Eigen::Vector2d brushTipPosition_xy;
    brushTipPosition_xy << m_brushTipPosition[0],
                           m_brushTipPosition[1];

    // Note its not clear here whether they used a vector or a matrix (the notation changed between 2 equations)
    // Trying matrix first, and checking the results. If its wrng, switch this to vector and do cross product
    Eigen::Matrix2d rotationMat;
    rotationMat << std::cos(w + rho), - std::sin(w + rho),
                   std::sin(w + rho), std::cos(w + rho);
    
    for (const Eigen::Vector3d hairPosition: m_hairBase){
        // Define the terms in the equation
        Eigen::Vector2d pixel_xy;
        double hairPosition_x = hairPosition[0];
        double hairPosition_y = hairPosition[1];
        Eigen::Vector2d hairPositionScaled;
        hairPositionScaled << hairPosition_x * (m_footprint.v * m_footprint.k_v + b) / m_brushRadius,
                              hairPosition_y * (m_footprint.u * m_footprint.k_u + b) / m_brushRadius;

        
        pixel_xy = brushTipPosition_xy + rotationMat * hairPositionScaled;

        double x, y;
        x = std::round(pixel_xy[0]);
        y = std::round(pixel_xy[1]);

        // Note: we need to define some kind of ink model, which states firstly how much ink can be used by a brush before it needs to be redipped.
        // For now however, we will assume it will not run out.

        // Apply a random density (0-1) of ink to pixel (i,j):  (this should probably a more complex ink depositing mdoel, but lets keep this for testing)
        m_footprint.paintDeposited.coeffRef(x,y) = dis(gen);
    }

    // update the canvas with the accumulated ink from last stroke
    m_canvas = m_canvas + m_footprint.paintDeposited;
};

const SimResult SimpleBrushModel::getResult() const {
    //Once implemented, should return Pose of the brush stem, position of the brush tip
    std::vector<Eigen::Vector3d> handleVertices = generateHandleVertices();
    std::vector<Eigen::Vector3d> brushVertices = generateBrushVertices();
    std::vector<Eigen::Vector3d> vertices = handleVertices;
    vertices.insert(vertices.end(), brushVertices.begin(), brushVertices.end());

    Eigen::Matrix3d rotationMat = conversions::eulerToRotationMatrix(m_brushStemPose.orientation);
    Eigen::Vector3d brushNormal = rotationMat * m_brushInitialNormal;

    SimResult result = SimResult(m_footprint.paintDeposited, m_brushStemPose.position, brushNormal, 
                                    m_brushStemTwist.linearVelocity, vertices, m_timeStamp);
    return result;
}


std::vector<Eigen::Vector3d> SimpleBrushModel::generateHandleVertices() const{
    Eigen::Vector3d brushStemPosition = m_brushStemPose.position;
    Eigen::Matrix3d rotationMat = conversions::eulerToRotationMatrix(m_brushStemPose.orientation);
    Eigen::Vector3d brushNormal = rotationMat * m_brushInitialNormal;

    double handleLength = 5 * m_brushLength; // magic number, put somewhere else, maybe initialization
    std::vector<int> lengthLinspace = geometric::linspace(0, handleLength, 20); // another magic number

    std::vector<Eigen::Vector3d> brushHandleVertices;

    for (int i: lengthLinspace){
        Eigen::Vector3d ringCenter = brushStemPosition + i* brushNormal;
        std::vector<Eigen::Vector3d> ringPoints = geometric::generateRing(ringCenter, brushNormal, m_brushRadius, 10); // magic number.
        brushHandleVertices.insert(brushHandleVertices.end(), ringPoints.begin(), ringPoints.end());
    }
    return brushHandleVertices;
};


std::vector<Eigen::Vector3d> SimpleBrushModel::generateBrushVertices() const{
    Eigen::Vector3d brushStemPosition = m_brushStemPose.position;
    Eigen::Matrix3d rotationMat = conversions::eulerToRotationMatrix(m_brushStemPose.orientation);
    Eigen::Vector3d brushNormal = - rotationMat * m_brushInitialNormal; // Added negative sign as we will generate points going down towards the brush tip

    std::vector<int> lengthLinspace = geometric::linspace(0, m_brushLength, 10); // another magic number

    std::vector<Eigen::Vector3d> brushVertices;

    for (int i: lengthLinspace){
        Eigen::Vector3d ringCenter = brushStemPosition + i * brushNormal;
        double currentBrushRadius = m_brushRadius * (m_brushLength - i) / m_brushLength;
        std::vector<Eigen::Vector3d> ringPoints = geometric::generateRing(ringCenter, brushNormal, currentBrushRadius, 10); // magic number.
        brushVertices.insert(brushVertices.end(), ringPoints.begin(), ringPoints.end());
    }
    return brushVertices;
};

void SimpleBrushModel::initialize(const IConfig& config)
{
    //Note: Maybe would be better if initialize takes in a map instead of IConfig?
    const std::map<std::string, std::string> modelConfig = config.getModelConfig();
    m_brushRadius = parsing::parseDouble(modelConfig.at("brush radius"));
    m_brushLength = parsing::parseDouble(modelConfig.at("brush length"));
    m_hairLength = parsing::parseDouble(modelConfig.at("hair length"));
    int hairCount = parsing::parseInt(modelConfig.at("hair count"));

    //Generate the base of each brush hair w.r.t the base of the brush
    double radius, theta, x, y, rand1, rand2;
    for (int i=0; i<hairCount; i++){
        rand1 = dis(gen);
        rand2 = dis(gen);

        radius = m_brushRadius * rand1;
        theta = 2 * M_PI * rand2;

        x = radius * cos(theta);
        y = radius * sin(theta);

        Eigen::Vector3d hairBase(x, y, 0.);
        m_hairBase.emplace_back(hairBase);
    }

    m_brushStemInitialPose = parsing::parsePose(modelConfig.at("initial pose"));
    m_brushInitialNormal = parsing::parseVector3d(modelConfig.at("initial normal"));
    m_brushStemInitialTwist = parsing::parseTwist(modelConfig.at("initial twist"));
    m_brushStemPose = m_brushStemInitialPose;
    m_brushStemTwist = m_brushStemInitialTwist;

    Eigen::Matrix3d brushRotation = conversions::eulerToRotationMatrix(m_brushStemPose.orientation); // convert orientation of the brush to a rotation vector
    m_brushTipPosition = brushRotation * ( m_brushStemPose.position + Eigen::Vector3d(0,0,-m_brushLength) ); // update the position of the brush tip's position

    int h = parsing::parseInt(modelConfig.at("canvas height"));
    int w = parsing::parseInt(modelConfig.at("canvas width"));
    m_canvas.resize(h, w);
    m_footprint.paintDeposited.resize(h, w);

    m_timeStamp = parsing::parseDouble(modelConfig.at("initial timestamp"));
};
