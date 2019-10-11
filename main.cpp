#include <cstdio>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include <yarp/os/SystemClock.h>
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Property.h>

#include <yarp/sig/Image.h>

#include <yarp/dev/PolyDriver.h>

#include <yarp/cv/Cv.h>
#include <yarp/cv/Cv-inl.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

enum ColorThreshold
{
    RED,
    GREEN,
    BLUE
};

int init_grabber(PolyDriver *polyDriver)
{
    Property p;
    p.put("device", "grabber");
    p.put("subdevice", "opencv_grabber");
    p.put("name", "/grabber");
    polyDriver->open(p);
    if (!polyDriver->isValid())
    {
        return -1;
    }
    Network::connect("/grabber", "/icubSim/texture/screen");

    return 1;
}

void ColorThresholding(ColorThreshold color, cv::Mat &inputImage, ImageOf<PixelRgb> &outputImage, bool usingHSV = true, bool applyBlur = false)
{
    cv::Mat mask, out;
    if (usingHSV) //USING HSV
    {

        cv::Mat hsv_conv;
        cv::cvtColor(inputImage, hsv_conv, cv::COLOR_BGR2HSV);

        switch (color)
        {
        default:
        case ColorThreshold::RED:
            cv::inRange(hsv_conv, cv::Scalar(120, 120, 140), cv::Scalar(180, 250, 200), mask);
            break;
        case ColorThreshold::GREEN:
            cv::inRange(hsv_conv, cv::Scalar(60, 110, 110), cv::Scalar(100, 220, 250), mask);
            break;
        case ColorThreshold::BLUE:
            cv::inRange(hsv_conv, cv::Scalar(100, 100, 100), cv::Scalar(130, 255, 255), mask);
            break;
        }

        cv::bitwise_and(hsv_conv, hsv_conv, out, mask);

        cv::cvtColor(out, out, cv::COLOR_HSV2RGB);
    }
    else //USING BGR
    {
        switch (color)
        {
        default:
        case ColorThreshold::RED:
            cv::inRange(inputImage, cv::Scalar(0, 0, 170), cv::Scalar(80, 80, 225), mask);
            break;
        case ColorThreshold::GREEN:
            cv::inRange(inputImage, cv::Scalar(0, 0, 170), cv::Scalar(80, 80, 225), mask);
            break;
        case ColorThreshold::BLUE:
            cv::inRange(inputImage, cv::Scalar(0, 0, 170), cv::Scalar(80, 80, 225), mask);
            break;
        }
        cv::bitwise_and(inputImage, inputImage, out, mask);

        cv::cvtColor(out, out, cv::COLOR_BGR2RGB);
    }

    if (applyBlur)
    {
        cv::GaussianBlur(out, out, cv::Size(3, 3), 0);
    }
    memcpy(outputImage.getRawImage(), out.data, sizeof(unsigned char) * inputImage.rows * inputImage.cols * inputImage.channels());
}

void SobelDerivative(cv::Mat &inputImage, ImageOf<PixelRgb> &outputImage)
{
    cv::Mat grayInput, gradient_x, gradient_y, abs_gradient_x, abs_gradient_y, out;

    cv::cvtColor(inputImage, grayInput, CV_BGR2GRAY);

    //Gradient X
    cv::Sobel(grayInput, gradient_x, CV_16S, 1, 0);
    cv::convertScaleAbs(gradient_x, abs_gradient_x);

    //Gradient Y
    cv::Sobel(grayInput, gradient_y, CV_16S, 0, 1);
    cv::convertScaleAbs(gradient_y, abs_gradient_y);

    //Total Gradient
    cv::addWeighted(abs_gradient_x, 0.5, abs_gradient_y, 0.5, 0, out);
    cv::cvtColor(out, out, CV_GRAY2RGB);
    memcpy(outputImage.getRawImage(), out.data, sizeof(unsigned char) * inputImage.rows * inputImage.cols * inputImage.channels());
}

int main()
{

    Network::init();
    /* INITIALIZE CAMERA AND CONNECT TO TEXTURE */
    PolyDriver grabber_dev;
    if (init_grabber(&grabber_dev) == -1)
    {
        printf("Failed to Initialize Webcam\n");
        return -1;
    }

    /* CREATE PORTS FOR IMG PROCESSING */
    BufferedPort<ImageOf<PixelRgb>> feedPort; //read input image
    BufferedPort<ImageOf<PixelRgb>> thPort;   //thresold output port
    BufferedPort<ImageOf<PixelRgb>> sdPort;   //sobel derivative port
    feedPort.open("/img_proc/feed/in");
    thPort.open("/img_proc/threshold");
    sdPort.open("/img_proc/sobel");

    Network::connect("/icubSim/cam/left", "/img_proc/feed/in");
    while (1)
    {
        ImageOf<PixelRgb> *feedImage = feedPort.read();
        ImageOf<PixelRgb> &thImage = thPort.prepare();
        ImageOf<PixelRgb> &sdImage = sdPort.prepare();
        thImage = *feedImage;
        sdImage = *feedImage;
        cv::Mat cvFeedImage = yarp::cv::toCvMat(*feedImage);
        ColorThresholding(ColorThreshold::BLUE, cvFeedImage, thImage, true, true);
        SobelDerivative(cvFeedImage, sdImage);
        thPort.write();
        sdPort.write();
    }
    return 0;
}