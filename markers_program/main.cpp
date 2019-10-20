#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <sstream>
#include <string>
#include <iostream>

const int maxNumber = 249;
int numberSlider = 0;

cv::Mat markerImage;
cv::Ptr<cv::aruco::Dictionary> dic;

static void onTrackbar(int, void *)
{
    cv::aruco::drawMarker(dic, numberSlider, 200, markerImage);
    cv::imshow("marker", markerImage);
}
int main(int argc, char *argv[])
{
    if (argc == 1)
    {
        std::cout << "Please enter 1 of the modes: ./aruco_markers [-generate, -detect]." << std::endl;
        return 0;
    }
    else if (argc > 2)
    {
        std::cout << "Too many arguments. Select between [-generate, -detect]." << std::endl;
        return 0;
    }
    std::string argument = argv[1];
    if (argument.compare("-generate") == 0)
    {
        dic = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
        cv::namedWindow("marker");
        cv::createTrackbar("markerId", "marker", &numberSlider, maxNumber, onTrackbar);
        onTrackbar(0, 0);
        int key = cv::waitKey(0);
        if (key == 's')
        {
            std::ostringstream ss;
            ss << numberSlider;
            printf("Saving as 'marker%d'\n", numberSlider);
            cv::imwrite("./marker" + ss.str() + ".jpg", markerImage);
        }
        return 0;
    }
    else if (argument.compare("-detect") == 0)
    {
        cv::VideoCapture inputVideo;
        inputVideo.open(0);
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);

        while (inputVideo.grab())
        {
            cv::Mat image, imageCopy;
            inputVideo.retrieve(image);
            image.copyTo(imageCopy);
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            cv::aruco::detectMarkers(image, dictionary, corners, ids);
            // if at least one marker detected
            if (ids.size() > 0)
                cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
            cv::imshow("out", imageCopy);
            char key = (char)cv::waitKey(10);
            if (key == 27)
                break;
        }
    }
    else
    {
        std::cout << "Invalid argument. Select between [-generate, -detect]." << std::endl;
        return 0;
    }
}