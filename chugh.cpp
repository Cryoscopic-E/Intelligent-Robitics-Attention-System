#include <stdio.h>
#include <string.h>
#include <cstdio>
#include <math.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Property.h>
#include <yarp/sig/Image.h>
#include <thread>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <yarp/cv/Cv.h>
#include <yarp/cv/Cv-inl.h>
#include <vector>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace cv;
using namespace std;


// function prototype declaration
int detectCircles(cv::Mat &inputImage, ImageOf<PixelRgb> &outputImage);
//int imageCheck(cv::Mat &inputImage);

int main() {
    // Initialize network
    Network yarp; // set up yarp
    // setting up the connection
    BufferedPort<ImageOf<PixelRgb> > imagePort;  // make a port for reading images
    BufferedPort<ImageOf<PixelRgb> > port;   // make a port for displaying
    imagePort.open("/img_proc/target/in");      // Opening port for incoming images
    Network::connect("/icubSim/cam/left", "/img_proc/target/in"); // Connects the iCub's left view to the bring in the images
    port.open("/img_proc/circleDetection");

    while (1) { // repeat forever
        ImageOf<PixelRgb> *feedImage = imagePort.read();
        ImageOf<PixelRgb> &decImage = port.prepare(); //preparing the port
        decImage = *feedImage; // setting up output image
        cv::Mat cvFeedImage = yarp::cv::toCvMat(*feedImage);
        detectCircles(cvFeedImage, decImage);
        port.write();
    }
    return 0;
}
// Definition of Hough Transform function
int detectCircles(cv::Mat &inputImage,  ImageOf<PixelRgb> &outputImage)
{
    //inputImage = 1./255;
    Mat gryImg;
    // Convert it to gray - needed for this function but shouldn't when integrated
    cvtColor(inputImage, gryImg, cv::COLOR_RGB2GRAY);
    medianBlur(gryImg, gryImg, 5);
    // Reduce the noise so we avoid false circle detection - needed for the function but shouldn't when integrated
    //GaussianBlur(gryImg, gryImg, Size(9, 9), 2, 2 );
    vector<Vec3f> circles; // Create a vector of size 3 for the circle definition

    HoughCircles(gryImg, circles, CV_HOUGH_GRADIENT, 2, (gryImg.rows/4), 200, 100);                                                                               // (min_radius & max_radius) to detect large circles
    for(size_t i =0; i < circles.size(); i++)
    {
        Vec3i c = circles[i]; //Creates vector of integers of size 3
        Point center = Point(c[0],c[1]); // Round the floating points to a point value
        // Draws a dot for the centre location
        circle(inputImage, center, 1, Scalar(0,100,100), 3, LINE_AA); //img, center, radius, colour, thickness, lineType, shift
        // Draws a circle outline around the detected item
        int radius = (c[2]);
        circle(inputImage, center, radius, Scalar(255,0,255), 3, LINE_AA); // same as above
    }
    memcpy(outputImage.getRawImage(), inputImage.data, sizeof(unsigned char) * inputImage.rows * inputImage.cols * inputImage.channels());
}


