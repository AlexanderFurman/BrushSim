#include "models/SimpleBrushModel.h"
#include "utils/Conversions.h"
#include "utils/Geometric.h"
#include "simulation_data/SimulationData.h"
#include "utils/Parsing.h"
#include <cmath>
#include <random>
#include <algorithm>
#include <iostream>
#include <chrono>

//create random seed for testing
unsigned int seed = 42;
std::mt19937 gen(seed);
std::uniform_real_distribution<> dis(0.0, 1.0);

void SimpleBrushModel::reset(){
    throw std::runtime_error("Not Implemented");
}

void SimpleBrushModel::updateState(const SimStep& simStep){
    updateBrushState(simStep);
    updateFootPrintState();
    generateBrushStroke();
    updateCanvasState();
    generateFootPrintBoundary();
};

const SimResult SimpleBrushModel::getResult() const {
    std::vector<Eigen::Vector3d> handleVertices = generateHandleVertices();
    std::vector<Eigen::Vector3d> brushVertices = generateBrushVertices();
    std::vector<Eigen::Vector3d> vertices = handleVertices;
    vertices.insert(vertices.end(), brushVertices.begin(), brushVertices.end());

    

    SimResult result = SimResult(m_footprint.paintDeposited, m_brushStemPose.position, m_brushNormal, 
                                    m_brushStemTwist.linearVelocity, vertices, m_timeStamp);
    return result;
}

void SimpleBrushModel::updateBrushState(const SimStep& simStep){
    m_brushStemPose = simStep.getBrushPose(); // Pose of brush stem
    m_brushStemTwist = simStep.getBrushTwist(); // Twist of brush stem
    m_timeStamp = simStep.getTimeStamp(); // simulation timestamp

    Eigen::Matrix3d brushRotation = conversions::eulerToRotationMatrix(m_brushStemPose.orientation); // convert orientation of the brush to a rotation vector
    m_brushTipPosition = brushRotation * ( m_brushStemPose.position + Eigen::Vector3d(0,0,-m_brushLength) ); // update the position of the brush tip's position

    m_brushNormal = brushRotation * m_brushInitialNormal;
}

void SimpleBrushModel::updateFootPrintState(){
    // ESTIMATE w -- angle of rotation of the ellipse w.r.t. the x-axis of the world frame.
    double v_x = m_brushStemTwist.linearVelocity[0];
    double v_y = m_brushStemTwist.linearVelocity[1];
    m_footprint.w = std::atan2(v_y, v_x); //angle between x axis (down the page) and the stroke direction (in radians)

    // ESTIMATE INITIAL VALUES OF v, u. (As if the brush footprint was a perfect circle)
    double brushTipPosition_z = m_brushTipPosition[2];
    auto max_abs_z = std::max(-brushTipPosition_z, 0.0);
    m_footprint.u = m_brushRadius * max_abs_z / m_brushLength;
    m_footprint.v = m_footprint.u;

    // Modifying factors of v, u -- These are used to deform the footprint into an ellipse when moving the brush on the page
    // In the paper they seem to be numbers generated upon trial and error.
    // Will input constant values for now, but can dafinitely update these values according to speed of brush
    m_footprint.k_u = 1.2;
    m_footprint.k_v = 0.7;
    
    // ellipse rotation modifying factor -- seemingly used when "extra rotation" is needed in a brushstroke
    // For now, will set a constant value, however it seems that the angle can actually be determined by comparing the
    // angle between the brush stroke direction, and the direction of the brush (not sure, but need to try)
    m_footprint.rho = 0; // The paper uses values -0.6 <= rho <= 0, mostly 0

    m_footprint.b = 0; // Another modifying factor -- for bias

}

void SimpleBrushModel::generateBrushStroke(){
    Eigen::Vector2d brushTipPosition_xy;
    brushTipPosition_xy << m_brushTipPosition[0],
                           m_brushTipPosition[1];

    Eigen::Matrix2d rotationMat;
    rotationMat << std::cos(m_footprint.w + m_footprint.rho), - std::sin(m_footprint.w + m_footprint.rho),
                   std::sin(m_footprint.w + m_footprint.rho), std::cos(m_footprint.w + m_footprint.rho);

    double x, y;
    Eigen::Vector2d xy_scaled;
    Eigen::Vector2d pixel_xy;
    int pixel_x, pixel_y;
    
    int numRows = m_footprint.paintDeposited.rows();
    int numCols = m_footprint.paintDeposited.cols();

    for (const Eigen::Vector3d& point: m_hairBase){
        x = point[0];
        y = point[1];
        // scale x, y according to the ellipse forming the boundary of the footprint
        auto x_scaled = x * (m_footprint.v * m_footprint.k_v + m_footprint.b) / m_brushRadius;
        auto y_scaled = y * (m_footprint.u * m_footprint.k_u + m_footprint.b) / m_brushRadius;
        xy_scaled[0] = x_scaled;
        xy_scaled[1] = y_scaled;

        // rotate the brush's footprint according to the value of w
        pixel_xy = brushTipPosition_xy + rotationMat * (xy_scaled);

        // round the values and covert to ints, so they can represent indices in the canvas
        pixel_x = (int) std::round(pixel_xy[0]);
        pixel_y = (int) std::round(pixel_xy[1]);

        // check if the indices found are within the bounds of the canvas
        if ((pixel_x >= 0 && pixel_x < numRows) && (pixel_y >= 0 && pixel_y < numCols))
        {
            //deposit paint onto the canvas. 
            //TODO: the paper implements this in a more complex way, taking into account the ink level in the brush.
            // implement this logic to the SimpleBrushModel.
            m_footprint.paintDeposited.coeffRef(pixel_x, pixel_y) = dis(gen);
        }
    }
}

void SimpleBrushModel::generateFootPrintBoundary(){
    Eigen::Vector2d brushTipPosition_xy;
    brushTipPosition_xy << m_brushTipPosition[0],
                           m_brushTipPosition[1];

    Eigen::Matrix2d rotationMat;
    rotationMat << std::cos(m_footprint.w + m_footprint.rho), - std::sin(m_footprint.w + m_footprint.rho),
                   std::sin(m_footprint.w + m_footprint.rho), std::cos(m_footprint.w + m_footprint.rho);

    auto ellipse = geometric::generateEllipse(Eigen::Vector3d::Zero(), Eigen::Vector3d(0,0,1), m_footprint.v * m_footprint.k_v + m_footprint.b, m_footprint.u * m_footprint.k_u + m_footprint.b, 200); 
    
    Eigen::Vector2d point_xy;
    Eigen::Vector2d pixel_xy;
    int pixel_x, pixel_y;
    
    int numRows = m_footprint.paintDeposited.rows();
    int numCols = m_footprint.paintDeposited.cols();

    for (const Eigen::Vector3d& point: ellipse){
        point_xy = Eigen::Vector2d(point[0], point[1]);
        pixel_xy = brushTipPosition_xy + rotationMat * (point_xy);
        pixel_x = (int) std::round(pixel_xy[0]);
        pixel_y = (int) std::round(pixel_xy[1]);

        if ((pixel_x >= 0 && pixel_x < numRows) && (pixel_y >= 0 && pixel_y < numCols))
        {
            m_footprint.boundary.coeffRef(pixel_x, pixel_y) = 1;
        }
    }
}

void SimpleBrushModel::updateCanvasState(){
    // update the canvas with the accumulated ink from last stroke
    m_canvas = m_canvas + m_footprint.paintDeposited;
}


std::vector<Eigen::Vector3d> SimpleBrushModel::generateHandleVertices() const{
    Eigen::Vector3d brushStemPosition = m_brushStemPose.position;
    Eigen::Matrix3d rotationMat = conversions::eulerToRotationMatrix(m_brushStemPose.orientation);
    Eigen::Vector3d brushNormal = rotationMat * m_brushInitialNormal;

    double handleLength = 5 * m_brushLength; // magic number, put somewhere else, maybe initialization
    std::vector<double> lengthLinspace = geometric::linspace<double>(0, handleLength, 20); // another magic number

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

    std::vector<double> lengthLinspace = geometric::linspace<double>(0, m_brushLength, 10); // another magic number

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
    m_footprint.boundary.resize(h, w);

    m_timeStamp = parsing::parseDouble(modelConfig.at("initial timestamp"));
};
