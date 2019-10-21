/*
** Heriott Watt University, 2019
** Intelligent Robotics
** Authors: Dale Carter, Rohit Chakraborty, Lucas Dumy, Solène Navaro, Emanuele De Pellegrin
*/

#include "ImageAnalysis.hpp"
#include "Transition.hpp"

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
    _cdPort.close();
    _grabber_dev.close();
}

int ImageAnalysis::initGrabber(PolyDriver *polyDriver)
{
    Property p;
    /*
        Initialize a new opencv grabber
    */
    p.put("device", "grabber");
    p.put("subdevice", "opencv_grabber");
    p.put("name", "/grabber");
    polyDriver->open(p);
    if (!polyDriver->isValid())
    {
        return (-1);
    }
    // connect the grabber to the icub simulation texture
    Network::connect("/grabber", "/icubSim/texture/screen");
    return (1);
}

void ImageAnalysis::colorThresholding(ImageAnalysis::ColorThreshold color, cv::Mat &inputImage, ImageOf<PixelRgb> &outputImage)
{
    // create the mask for the color, the output image and the conversion to hsv temporary matrices
    cv::Mat mask, out, hsv_conv;
    // convert bgr to hsv
    cv::cvtColor(inputImage, hsv_conv, cv::COLOR_BGR2HSV);

    // create a color mask based on the color argument, the ranges were defined with an external program
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
    // apply bitwise & using the color mask created
    cv::bitwise_and(hsv_conv, hsv_conv, out, mask);
    // convert the image back to rgb
    cv::cvtColor(out, out, cv::COLOR_HSV2RGB);
    // memory copy the output matrice data in the yarp Imageof<PixelRgb>
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
    cv::cvtColor(inputImage, grayConv, CV_BGR2GRAY); // Convert to gray scale
    // Detect faces using cascade classifier
    // Input image, array, scale factor, minimum neighbours, flags, minumum size
    cascade.detectMultiScale(grayConv, faces, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));
    cv::cvtColor(inputImage, out, CV_BGR2RGB); // Convert the input image to RGB

    // 1+ faces detected
    if (faces.size() > 0)
    {
        // set the face center location as target for the robot head
        std::pair<double, double> pos;
        pos.first = faces[0].x + faces[0].width / 2.0;
        pos.second = faces[0].y + faces[0].height / 2.0;
        pos.first -= 320 / 2;
        pos.second -= 240 / 2;
        pos.first *= 0.2;
        pos.second *= -0.2;
        _robot.setFaceLastPos(pos);
        _stateMachine.OnEvent(Transition::Event::FACE_DETECTED);
    }
    // Loop through each face
    for (size_t i = 0; i < faces.size(); i++)
    {
        cv::Rect r = faces[i];
        cv::Scalar color = (0, 255, 0);
        // Draw rectangle around the face - x and y axis starts at top-left corner
        cv::rectangle(out, cvPoint(cvRound(r.x), cvRound(r.y)), cvPoint(cvRound(r.x + r.width - 1), cvRound(r.y + r.height - 1)), color, 3);

        cv::Mat roi = grayConv(r);
        vector<cv::Rect> eyes;
        // Detect eyes in each face using cascade classifer
        eyesCascade.detectMultiScale(roi, eyes, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));
        for (size_t j = 0; j < eyes.size(); j++)
        {
            cv::Rect e = eyes[j];
            // Find the centre of the circles - x and y axis starts at top left corner
            cv::Point centre(cvRound(r.x) + cvRound(e.x) + cvRound(e.width / 2), cvRound(r.y) + cvRound(e.y) + cvRound(e.height / 2));
            // Calculate the radius of the circles
            int radius = cvRound(cvRound((e.width + e.height) * 0.25));
            // Draw circles around the eyes
            cv::circle(out, centre, radius, cv::Scalar(0, 0, 255), 4, 8, 0);
        }
    }
    memcpy(outputImage.getRawImage(), out.data, sizeof(unsigned char) * inputImage.rows * inputImage.cols * inputImage.channels());
}

void ImageAnalysis::markerDetection(cv::Mat &inputImage, ImageOf<PixelRgb> &outputImage)
{
    cv::Mat out;
    // Create a copy of the image feeded in
    inputImage.copyTo(out);
    // Create stl vector that holds the id of the detected markers
    vector<int> markersId;
    // Create stl vector that holds the 2D coord
    vector<vector<cv::Point2f>> corners;
    // detect markers and if detected add the corners location inside the corners list
    cv::aruco::detectMarkers(out, _dict, corners, markersId);

    if (markersId.size() > 0)
    {
        // set the marker top left location as target for the robot head
        std::pair<double, double> pos;
        pos.first = corners[0][0].x;
        pos.second = corners[0][0].y;
        pos.first -= 320 / 2;
        pos.second -= 240 / 2;
        pos.first *= 0.2;
        pos.second *= -0.2;
        _robot.setMarkerLastPos(pos);
        _stateMachine.OnEvent(Transition::Event::MARKER_DETECTED);
        // draw marker rectangle with id as label
        cv::aruco::drawDetectedMarkers(out, corners, markersId);
    }
    cv::cvtColor(out, out, CV_BGR2RGB);
    memcpy(outputImage.getRawImage(), out.data, sizeof(unsigned char) * inputImage.rows * inputImage.cols * inputImage.channels());
}

void ImageAnalysis::circleDetection(cv::Mat &inputImage, ImageOf<PixelRgb> &outputImage)
{
    cv::Mat gryImg, out;
    // Convert it to gray - needed for this function but shouldn't when integrated
    cv::cvtColor(inputImage, gryImg, cv::COLOR_RGB2GRAY);
    // Reduce the noise so we avoid false circle detection
    cv::medianBlur(gryImg, gryImg, 5);
    // Create a vector of size 3 for the circle definition
    vector<cv::Vec3f> circles;

    cv::HoughCircles(gryImg, circles, CV_HOUGH_GRADIENT, 2, (gryImg.rows / 4), 200, 100); // (min_radius & max_radius) to detect large circles
    // 1+ circles detected
    if (circles.size() > 0)
    {
        _stateMachine.OnEvent(Transition::Event::CIRCLE_DETECTED);
    }
    cv::cvtColor(gryImg, out, cv::COLOR_GRAY2RGB);
    for (size_t i = 0; i < circles.size(); i++)
    {
        //Creates vector of integers of size 3
        cv::Vec3i c = circles[i];
        // Round the floating points to a point value
        cv::Point center = cv::Point(c[0], c[1]);
        // Draws a dot for the centre location
        cv::circle(out, center, 1, cv::Scalar(0, 100, 100), 3, cv::LINE_AA); //img, center, radius, colour, thickness, lineType, shift
        // Draws a circle outline around the detected item
        int radius = (c[2]);
        cv::circle(out, center, radius, cv::Scalar(255, 0, 255), 3, cv::LINE_AA); // same as above
    }

    memcpy(outputImage.getRawImage(), out.data, sizeof(unsigned char) * inputImage.rows * inputImage.cols * inputImage.channels());
}

int ImageAnalysis::runAnalysis()
{
    /* INITIALIZE CAMERA AND CONNECT TO TEXTURE */
    PolyDriver grabber_dev, head_dev, arm_dev;
    if (initGrabber(&grabber_dev) == -1)
    {
        printf("Failed to Initialize Webcam\n");
        return (-1);
    }
    // initialize robot head and arm drivers
    _robot.initRobot(&head_dev, &arm_dev);
    // set the robot as context for the state machine
    _stateMachine.SetRobot(_robot);
    while (!_stop)
    {
        // read icub left camera
        ImageOf<PixelRgb> *feedImage = _feedPort.read();
        // prepare all output ports
        ImageOf<PixelRgb> &thImage = _thPort.prepare();
        ImageOf<PixelRgb> &sdImage = _sdPort.prepare();
        ImageOf<PixelRgb> &fdImage = _fdPort.prepare();
        ImageOf<PixelRgb> &mdImage = _mdPort.prepare();
        ImageOf<PixelRgb> &cdImage = _cdPort.prepare();
        thImage = sdImage = fdImage = mdImage = cdImage = *feedImage;
        // convert the yarp image to cv Matrix to be the inputs of all the image processing functions
        cv::Mat cvFeedImage = yarp::cv::toCvMat(*feedImage);
        // default: no event detected
        _stateMachine.OnEvent(Transition::Event::NO_EVENT);
        // image processing functions
        colorThresholding(_ct, cvFeedImage, thImage);
        sobelDerivative(cvFeedImage, sdImage);
        circleDetection(cvFeedImage, cdImage);
        faceDetection(cvFeedImage, fdImage, _cc, _cce);
        markerDetection(cvFeedImage, mdImage);
        // write the yarp image to the output port
        _thPort.write();
        _sdPort.write();
        _fdPort.write();
        _mdPort.write();
        _cdPort.write();
        // execute the transition
        _stateMachine.Execute();
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
    if (initGrabber(&grabber_dev) == -1)
    {
        printf("Failed to Initialize Webcam\n");
        return (-1);
    }

    /* CREATE PORTS FOR IMG PROCESSING */
    _feedPort.open("/img_proc/feed/in");
    _thPort.open("/img_proc/threshold");
    _sdPort.open("/img_proc/sobel");
    _fdPort.open("/img_proc/face");
    _mdPort.open("/img_proc/marker");
    _cdPort.open("/img_proc/circle");

    _ct = ColorThreshold::GREEN;

    // CASCADE CLASSIFIERS FILES
    if (!_cc.load("../haarcascade_frontalface_alt.xml"))
    {
        printf("Error loading face cascade classifier\n");
    }

    if (!_cce.load("../haarcascade_eye_tree_eyeglasses.xml"))
    {
        printf("Error loading eye cascade classifier\n");
    }

    // MARKERS DICTIONARY LOADING
    _dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);

    // connect the icub left cam to the image feed port
    Network::connect("/icubSim/cam/left", "/img_proc/feed/in");
    return (0);
}