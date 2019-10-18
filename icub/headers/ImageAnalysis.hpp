//
// Heriott Watt University, 2019
// Intelligent Robotics
// Authors: Dale Carter, Rohit Chakraborty, Lucas Dumy, Solène Navaro, Emanuele De Pellegrin
//

#ifndef INTELLIGENT_ROBITICS_ATTENTION_SYSTEM_IMAGEANALYSIS_HPP
#define INTELLIGENT_ROBITICS_ATTENTION_SYSTEM_IMAGEANALYSIS_HPP

#include <cstdio>
#include <utility>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/aruco.hpp>

#include <yarp/os/SystemClock.h>
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Property.h>

#include <yarp/sig/Image.h>

#include <yarp/dev/PolyDriver.h>

#include <yarp/cv/Cv.h>
#include <yarp/cv/Cv-inl.h>

#include "RobotMoves.hpp"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

class ImageAnalysis {
    public:
    ImageAnalysis();
    ~ImageAnalysis();

    int     initImageAnalysis();
    int     runAnalysis();

    enum ColorThreshold {
        RED,
        GREEN,
        BLUE
    };

    private:
    int     initGrabber(PolyDriver *polyDriver);
    void    locateRedColor(ImageOf<PixelRgb> *feedImage);
    void    colorThresholding(ColorThreshold color, cv::Mat &inputImage, ImageOf<PixelRgb> &outputImage);
    void    sobelDerivative(cv::Mat &inputImage, ImageOf<PixelRgb> &outputImage);
    void    faceDetection(cv::Mat &inputImage, ImageOf<PixelRgb> &outputImage, cv::CascadeClassifier &cascade, cv::CascadeClassifier &eyesCascade);
    void    markerDetection(cv::Mat &inputImage, ImageOf<PixelRgb> &outputImage);

    RobotMoves _robot;
    bool    _stop;
    bool    _faceRecognized;
    bool    _redTracked;
    std::pair<double, double>   _redPos;
    PolyDriver _grabber_dev;
    ColorThreshold _ct;
    cv::CascadeClassifier _cc;
    cv::CascadeClassifier _cce;
    cv::Ptr<cv::aruco::Dictionary> _dict;
    BufferedPort<ImageOf<PixelRgb>> _feedPort; //read input image
    BufferedPort<ImageOf<PixelRgb>> _thPort;   //threshold output port
    BufferedPort<ImageOf<PixelRgb>> _sdPort;   //sobel derivative port
    BufferedPort<ImageOf<PixelRgb>> _fdPort;   //face detection port
    BufferedPort<ImageOf<PixelRgb>> _mdPort;   //marker detection port
};

#endif //INTELLIGENT_ROBITICS_ATTENTION_SYSTEM_IMAGEANALYSIS_HPP
