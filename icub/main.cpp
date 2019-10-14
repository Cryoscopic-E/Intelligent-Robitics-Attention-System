#include <cstdio>

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

void ColorThresholding(ColorThreshold color, cv::Mat &inputImage, ImageOf<PixelRgb> &outputImage, bool applyBlur = false)
{
    cv::Mat mask, out, hsv_conv;
    cv::cvtColor(inputImage, hsv_conv, cv::COLOR_BGR2HSV);

    switch (color)
    {
    default:
    case ColorThreshold::RED:
        cv::inRange(hsv_conv, cv::Scalar(0, 50, 50), cv::Scalar(10, 255, 255), mask);
        break;
    case ColorThreshold::GREEN:
        cv::inRange(hsv_conv, cv::Scalar(50, 50, 50), cv::Scalar(70, 255, 255), mask);
        break;
    case ColorThreshold::BLUE:
        cv::inRange(hsv_conv, cv::Scalar(90, 100, 0), cv::Scalar(180, 255, 255), mask);
        break;
    }

    cv::bitwise_and(hsv_conv, hsv_conv, out, mask);

    cv::cvtColor(out, out, cv::COLOR_HSV2RGB);

    if (applyBlur)
    {
        cv::GaussianBlur(out, out, cv::Size(3, 3), 0);
    }
    memcpy(outputImage.getRawImage(), out.data, sizeof(unsigned char) * inputImage.rows * inputImage.cols * inputImage.channels());
}

void SobelDerivative(cv::Mat &inputImage, ImageOf<PixelRgb> &outputImage)
{
    cv::Mat grayConv, gradient_x, gradient_y, abs_gradient_x, abs_gradient_y, out;

    cv::cvtColor(inputImage, grayConv, CV_BGR2GRAY);

    //Gradient X
    cv::Sobel(grayConv, gradient_x, CV_16S, 1, 0);
    cv::convertScaleAbs(gradient_x, abs_gradient_x);

    //Gradient Y
    cv::Sobel(grayConv, gradient_y, CV_16S, 0, 1);
    cv::convertScaleAbs(gradient_y, abs_gradient_y);

    //Total Gradient
    cv::addWeighted(abs_gradient_x, 0.5, abs_gradient_y, 0.5, 0, out);
    cv::cvtColor(out, out, CV_GRAY2RGB);
    memcpy(outputImage.getRawImage(), out.data, sizeof(unsigned char) * inputImage.rows * inputImage.cols * inputImage.channels());
}

void FaceDetection(cv::Mat &inputImage, ImageOf<PixelRgb> &outputImage, cv::CascadeClassifier &cascade)
{
    vector<cv::Rect> faces;
    cv::Mat grayConv, out;
    cv::cvtColor(inputImage, grayConv, CV_BGR2GRAY);
    cascade.detectMultiScale(grayConv, faces, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));
    cv::cvtColor(inputImage, out, CV_BGR2RGB);
    //Draw boxes
    for (size_t i = 0; i < faces.size(); i++)
    {
        cv::Rect r = faces[i];
        cv::Scalar color = (0, 255, 0);
        cv::rectangle(out, cvPoint(cvRound(r.x), cvRound(r.y)), cvPoint(cvRound(r.x + r.width - 1), cvRound(r.y + r.height - 1)), color, 3);
    }
    memcpy(outputImage.getRawImage(), out.data, sizeof(unsigned char) * inputImage.rows * inputImage.cols * inputImage.channels());
}

void MarkerDetection(cv::Mat &inputImage, ImageOf<PixelRgb> &outputImage, cv::Ptr<cv::aruco::Dictionary> dict)
{
    cv::Mat out;
    inputImage.copyTo(out);
    vector<int> markersId;
    vector<vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(out, dict, corners, markersId);
    if (markersId.size() > 0)
    {
        cv::aruco::drawDetectedMarkers(out, corners, markersId);
    }
    cv::cvtColor(out, out, CV_BGR2RGB);
    memcpy(outputImage.getRawImage(), out.data, sizeof(unsigned char) * inputImage.rows * inputImage.cols * inputImage.channels());
}

int main()
{
    Network::init();

    /* STOP PROGRAM */
    bool stop = false;

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
    BufferedPort<ImageOf<PixelRgb>> fdPort;   //face detection port
    BufferedPort<ImageOf<PixelRgb>> mdPort;   //marker detection port
    feedPort.open("/img_proc/feed/in");
    thPort.open("/img_proc/threshold");
    sdPort.open("/img_proc/sobel");
    fdPort.open("/img_proc/face");
    mdPort.open("/img_proc/marker");

    ColorThreshold ct = ColorThreshold::BLUE;

    /*CASCADE CLASSIFIER*/
    cv::CascadeClassifier cc;
    if (!cc.load("haarcascade_frontalface_alt.xml"))
    {
        printf("Error loading face cascade classifier\n");
    }

    /*MARKERS DICTIONARY*/
    cv::Ptr<cv::aruco::Dictionary> dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);

    Network::connect("/icubSim/cam/left", "/img_proc/feed/in");
    cv::namedWindow("Feed iCub", cv::WINDOW_AUTOSIZE);
    while (!stop)
    {
        ImageOf<PixelRgb> *feedImage = feedPort.read();
        ImageOf<PixelRgb> &thImage = thPort.prepare();
        ImageOf<PixelRgb> &sdImage = sdPort.prepare();
        ImageOf<PixelRgb> &fdImage = fdPort.prepare();
        ImageOf<PixelRgb> &mdImage = mdPort.prepare();
        thImage = sdImage = fdImage = mdImage = *feedImage;
        cv::Mat cvFeedImage = yarp::cv::toCvMat(*feedImage);
        cv::imshow("Feed iCub", cvFeedImage);
        ColorThresholding(ct, cvFeedImage, thImage, true);
        SobelDerivative(cvFeedImage, sdImage);
        FaceDetection(cvFeedImage, fdImage, cc);
        MarkerDetection(cvFeedImage, mdImage, dict);
        thPort.write();
        sdPort.write();
        fdPort.write();
        mdPort.write();

        int key = cv::waitKey(3);
        switch (key)
        {
        case 27:
        case 'q':
            stop = true;
            break;
        case 'r':
            ct = ColorThreshold::RED;
            break;
        case 'g':
            ct = ColorThreshold::GREEN;
            break;
        case 'b':
            ct = ColorThreshold::BLUE;
            break;
        default:
            break;
        }
    }
    feedPort.close();
    thPort.close();
    sdPort.close();
    fdPort.close();
    mdPort.close();
    grabber_dev.close();
    return 0;
}