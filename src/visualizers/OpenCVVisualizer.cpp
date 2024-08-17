#include "visualizers/OpenCVVisualizer.h"
#include <opencv2/core/eigen.hpp>
#include "utils/Parsing.h"

void OpenCVVisualizer::initialize(const IConfig& config){
    const std::map<std::string, std::string> visConfig = config.getVisualizerConfig();
    int h = parsing::parseInt(visConfig.at("canvas height"));
    int w = parsing::parseInt(visConfig.at("canvas width"));
    m_canvas.resize(h, w);
}

void OpenCVVisualizer::visualize(const SimStep& step, const SimResult& result){
    //NOTE: through testing it was found that this function can currently run at roughly 22 fps


    //TODO: Add stroke direction vector, and lines connecting the strokes

    ///////// GET BRUSH STROKE /////////////////
    auto brushStroke = result.brushStroke;

    m_canvas = m_canvas + brushStroke;
    m_canvas = m_canvas.cwiseMin(1.0); // clamp values between 0 and 1

    Eigen::MatrixXd roundedMatrix = m_canvas * 255;
    roundedMatrix = roundedMatrix.array().round();
    Eigen::MatrixXi matrixI = roundedMatrix.cast<int>();
    cv::Mat image(matrixI.rows(), matrixI.cols(), CV_8UC1);
    cv::eigen2cv(matrixI, image);

    // Convert the image to type CV_8U (8-bit unsigned integer)
    cv::Mat image_8U;
    image.convertTo(image_8U, CV_8U);
    image_8U = 255 - image_8U;

    // Display the image
    cv::imshow("Canvas", image_8U);
    cv::waitKey(1);
}