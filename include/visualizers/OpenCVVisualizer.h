#ifndef OPENCVVISUALIZER_H
#define OPENCVVISUALIZER_H

#include "interfaces/IVisualizer.h"
#include <opencv2/opencv.hpp>

class OpenCVVisualizer: public IVisualizer{
    public:
    void visualize(const SimStep& step, const SimResult& result) override;
    void initialize(const IConfig& config) override;

    private:
    Eigen::MatrixXd m_canvas;
};

#endif // OPENCVVISUALIZER_H