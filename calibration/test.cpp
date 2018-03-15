#include <iostream>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

int main(int argc, char** argv){

    std::string filename = "../videos/LifeCam_asymmetric_circles.avi";
    VideoCapture capture(filename);
    Mat frame;

    if( !capture.isOpened() )
        throw "Error when reading steam_avi";

    namedWindow( "w", 1);

    for( ; ; )
    {
        capture >> frame;
        if(frame.empty())
            break;
        imshow("w", frame);
        waitKey(20); // waits to display frame
    }
    waitKey(0); // key press to close window
    // releases and window destroy are automatic in C++ interface
}