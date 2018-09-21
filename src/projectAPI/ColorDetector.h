#ifndef COLOR_DETECTOR_H_INCLUDED
#define COLOR_DETECTOR_H_INCLUDED

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml/ml.hpp>

#include <algorithm>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <cstdio>
#include <ctime>
#include <string>

#include "Utils.h"

class ColorDetector
{
    public:
        std::string Detect(cv::Mat image);
};

#endif