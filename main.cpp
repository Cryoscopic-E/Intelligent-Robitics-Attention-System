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

void ColorThresholding(ColorThreshold color, ImageOf<PixelRgb> &inputImage, ImageOf<PixelRgb> &outputImage, bool usingHSV = true, bool applyBlur = false)
{
    cv::Mat cvFeedImage, mask, out;
    cvFeedImage = yarp::cv::toCvMat(inputImage);
    if (usingHSV) //USING HSV
    {

        cv::Mat hsv_conv;
        cv::cvtColor(cvFeedImage, hsv_conv, cv::COLOR_BGR2HSV);

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
            cv::inRange(cvFeedImage, cv::Scalar(0, 0, 170), cv::Scalar(80, 80, 225), mask);
            break;
        case ColorThreshold::GREEN:
            cv::inRange(cvFeedImage, cv::Scalar(0, 0, 170), cv::Scalar(80, 80, 225), mask);
            break;
        case ColorThreshold::BLUE:
            cv::inRange(cvFeedImage, cv::Scalar(0, 0, 170), cv::Scalar(80, 80, 225), mask);
            break;
        }
        cv::bitwise_and(cvFeedImage, cvFeedImage, out, mask);

        cv::cvtColor(out, out, cv::COLOR_BGR2RGB);
    }

    if (applyBlur)
    {
        cv::GaussianBlur(out, out, cv::Size(3, 3), 0);
    }
    memcpy(outputImage.getRawImage(), out.data, sizeof(unsigned char) * cvFeedImage.rows * cvFeedImage.cols * cvFeedImage.channels());
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

    feedPort.open("/img_proc/feed/in");
    thPort.open("/img_proc/threshold/red");

    Network::connect("/icubSim/cam/left", "/img_proc/feed/in");
    while (1)
    {
        ImageOf<PixelRgb> *feedImage = feedPort.read();
        ImageOf<PixelRgb> &thImage = thPort.prepare();
        thImage = *feedImage;
        ColorThresholding(ColorThreshold::BLUE, *feedImage, thImage, true, true);

        thPort.write();
    }
    return 0;
}