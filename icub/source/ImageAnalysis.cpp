/*
** Heriott Watt University, 2019
** Intelligent Robotics
** Authors: Dale Carter, Rohit Chakraborty, Lucas Dumy, Solène Navaro, Emanuele De Pellegrin
*/

#include "ImageAnalysis.hpp"

ImageAnalysis::ImageAnalysis()
{
}

ImageAnalysis::~ImageAnalysis()
{
    _feedPort.close();
    _thPort.close();
    _sdPort.close();
    _fdPort.close();
    _mdPort.close();
    _grabber_dev.close();
}

int ImageAnalysis::initGrabber(PolyDriver *polyDriver)
{
    Property p;

    p.put("device", "grabber");
    p.put("subdevice", "opencv_grabber");
    p.put("name", "/grabber");
    polyDriver->open(p);
    if (!polyDriver->isValid())
    {
        return (-1);
    }
    Network::connect("/grabber", "/icubSim/texture/screen");
    return (1);
}

void ImageAnalysis::colorThresholding(ImageAnalysis::ColorThreshold color, cv::Mat &inputImage, ImageOf<PixelRgb> &outputImage)
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

    cv::GaussianBlur(out, out, cv::Size(3, 3), 0);
    memcpy(outputImage.getRawImage(), out.data, sizeof(unsigned char) * inputImage.rows * inputImage.cols * inputImage.channels());
}

void ImageAnalysis::sobelDerivative(cv::Mat &inputImage, ImageOf<PixelRgb> &outputImage)
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

void ImageAnalysis::faceDetection(cv::Mat &inputImage, ImageOf<PixelRgb> &outputImage, cv::CascadeClassifier &cascade, cv::CascadeClassifier &eyesCascade)
{
    vector<cv::Rect> faces;
    cv::Mat grayConv, out;
    cv::cvtColor(inputImage, grayConv, CV_BGR2GRAY);
    cascade.detectMultiScale(grayConv, faces, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));
    cv::cvtColor(inputImage, out, CV_BGR2RGB);
    //Draw boxes

    _faceRecognized = false;
    for (size_t i = 0; i < faces.size(); i++)
    {
        cv::Rect r = faces[i];
        cv::Scalar color = (0, 255, 0);
        cv::rectangle(out, cvPoint(cvRound(r.x), cvRound(r.y)), cvPoint(cvRound(r.x + r.width - 1), cvRound(r.y + r.height - 1)), color, 3);

        cv::Mat faceROI = out(faces[i]);
        vector<cv::Rect> eyes;

        // Detect eyes in each face
        eyesCascade.detectMultiScale(faceROI, eyes, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));
        for (size_t j = 0; j < eyes.size(); j++)
        {
            cv::Point centreEye(faces[i].x + eyes[j].x + eyes[j].width / 2, faces[i].y + eyes[j].y + eyes[j].height / 2);
            int radius = cvRound((eyes[j].width + eyes[j].height) * 0.25);
            cv::circle( out, centreEye, radius, cv::Scalar( 255, 0, 0 ), 4, 8, 0 );
            _faceRecognized = true;
        }
    }
    memcpy(outputImage.getRawImage(), out.data, sizeof(unsigned char) * inputImage.rows * inputImage.cols * inputImage.channels());
}

void ImageAnalysis::markerDetection(cv::Mat &inputImage, ImageOf<PixelRgb> &outputImage)
{
    cv::Mat out;
    inputImage.copyTo(out);
    vector<int> markersId;
    vector<vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(out, _dict, corners, markersId);

    if (markersId.size() > 0)
    {
        cv::aruco::drawDetectedMarkers(out, corners, markersId);
    }
    cv::cvtColor(out, out, CV_BGR2RGB);
    memcpy(outputImage.getRawImage(), out.data, sizeof(unsigned char) * inputImage.rows * inputImage.cols * inputImage.channels());
}

void    ImageAnalysis::locateRedColor(ImageOf<PixelRgb> *feedImage)
{
    if (feedImage != NULL) {
        double xMean = 0;
        double yMean = 0;
        int colourThreshold = 0;

        for (int x = 0; x < feedImage->width(); x++) {
            for (int y = 0; y < feedImage->height(); y++) {
                PixelRgb &pixel = feedImage->pixel(x, y);

                if (pixel.b > pixel.r * 2 + 10 && pixel.b > pixel.g * 2 + 10) {
                    xMean += x;
                    yMean += y;
                    colourThreshold++;
                }
            }
        }
        if (colourThreshold > 0) {
            xMean /= colourThreshold;
            yMean /= colourThreshold;
        }

        _redPos.first = xMean;
        _redPos.second = yMean;
        _redPos.first -= 320 / 2;
        _redPos.second -= 240 / 2;
        _redPos.first *= 0.2;
        _redPos.second *= -0.2;

        if (colourThreshold >
            (feedImage->width() / 20) * (feedImage->height() / 20)) {
            _redTracked = true;
        } else {
            _redTracked = false;
            _redPos.first = 0;
            _redPos.second = 0;
        }
    }
}

int ImageAnalysis::runAnalysis()
{
    /* INITIALIZE CAMERA AND CONNECT TO TEXTURE */
    PolyDriver grabber_dev;
    if (initGrabber(&grabber_dev) == -1) {
        printf("Failed to Initialize Webcam\n");
        return (-1);
    }

    _robot.initRobot();

    while (!_stop) {
        ImageOf<PixelRgb> *feedImage = _feedPort.read();
        ImageOf<PixelRgb> &thImage = _thPort.prepare();
        ImageOf<PixelRgb> &sdImage = _sdPort.prepare();
        ImageOf<PixelRgb> &fdImage = _fdPort.prepare();
        ImageOf<PixelRgb> &mdImage = _mdPort.prepare();
        thImage = sdImage = fdImage = mdImage = *feedImage;
        cv::Mat cvFeedImage = yarp::cv::toCvMat(*feedImage);
        cv::imshow("Feed iCub", cvFeedImage);
        colorThresholding(_ct, cvFeedImage, thImage);
        sobelDerivative(cvFeedImage, sdImage);
        faceDetection(cvFeedImage, fdImage, _cc, _cce);
        markerDetection(cvFeedImage, mdImage);
        _thPort.write();
        _sdPort.write();
        _fdPort.write();
        _mdPort.write();

        if (_faceRecognized) {
            _robot.fuckYou();
        } else {
            _robot.resetRightArm();
        }

        locateRedColor(feedImage);
        _robot.lookAt(_redPos);

        /*int key = cv::waitKey(3);
        switch (key) {
        case 'a':
            _robot.fuckYou();
            break;
        case 'z':
            _robot.resetRightArm();
            break;
        case 'e':
            _robot.riseRightArm();
            break;
        case 'q':
            _stop = true;
            break;
        case 'r':
            _ct = ColorThreshold::RED;
            break;
        case 'g':
            _ct = ColorThreshold::GREEN;
            break;
        case 'b':
            _ct = ColorThreshold::BLUE;
            break;
        default:
            break;
        }*/
    }
    return (0);
}

int ImageAnalysis::initImageAnalysis()
{
    /* STOP PROGRAM */
    _stop = false;

    Network::init();

    /* INITIALIZE CAMERA AND CONNECT TO TEXTURE */
    PolyDriver grabber_dev;
    if (initGrabber(&grabber_dev) == -1) {
        printf("Failed to Initialize Webcam\n");
        return (-1);
    }

    /* CREATE PORTS FOR IMG PROCESSING */
    _feedPort.open("/img_proc/feed/in");
    _thPort.open("/img_proc/threshold");
    _sdPort.open("/img_proc/sobel");
    _fdPort.open("/img_proc/face");
    _mdPort.open("/img_proc/marker");

    _ct = ColorThreshold::BLUE;

    /*CASCADE CLASSIFIER*/
    if (!_cc.load("../haarcascade_frontalface_alt.xml"))
    {
        printf("Error loading face cascade classifier\n");
    }

    if (!_cce.load("../haarcascade_eye_tree_eyeglasses.xml"))
    {
        printf("Error loading eye cascade classifier\n");
    }

    /*MARKERS DICTIONARY*/
    _dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);

    Network::connect("/icubSim/cam/left", "/img_proc/feed/in");
    cv::namedWindow("Feed iCub", cv::WINDOW_AUTOSIZE);
    return (0);
}