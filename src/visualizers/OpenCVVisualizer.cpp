#include "visualizers/OpenCVVisualizer.h"
#include <opencv2/core/eigen.hpp>
void OpenCVVisualizer::initialize(const IConfig& config){
    throw std::runtime_error("Not implemented");
}

void OpenCVVisualizer::visualize(const SimResult& result){
    //TODO: Add stroke direction vector, and lines connecting the strokes

    ///////// GET BRUSH STROKE /////////////////
    auto brushStroke = result.getBrushStroke();
    double h = brushStroke.rows();
    double w = brushStroke.cols();
    if (m_canvas.rows() == 0 || m_canvas.cols() == 0){ //TODO: move this to initialization

        m_canvas.resize(h, w);
        m_canvas.setZero(); // Initialize canvas to zeros
    }

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

    // Display the image
    cv::imshow("Canvas", image_8U);
    cv::waitKey(1000);
}