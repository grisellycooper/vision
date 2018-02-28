/**
g++ -std=c++11 main.cpp addfunctions.cpp `pkg-config opencv --cflags --libs` -o main && ./main
**/

#include "includes.h"

//#define video_path "videos/calibration_mslifecam.avi"
//#define video_path "../videos/calibration_ps3eyecam.avi"
//#define video_path "videos/Kinect2_rgb.avi"
//#define video_path "videos/realsense_Depth.avi"
//#define video_path "videos/realsense_RGB.avi"

#define windowName "video"
enum Pattern{CHESSBOARD,CIRCLES_GRID,ASYMMETRIC_CIRCLES_GRID,RING_GRID};

/** Global Variables **/
cv::Mat frame;
int PatternType;

std::vector<cv::Point2f> imgPoints; // 2D Points en la Imagen(Pixels)
std::vector<cv::Point2f> worldPoints; // 3D Points en World Space


int main(){

	//Terminos de fin
	cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::MAX_ITER, 30, 0.001);

#ifdef video_path
    cv::VideoCapture cap(video_path);
#else
    cv::VideoCapture cap(0); // --> For video Capture
    cap.set(cv::CAP_PROP_FPS,60); // ---> Cantidad de FPS caputrados por la camara
#endif

    if(!cap.isOpened()){
    	cout << "Cannot open the video file!" << endl;
    	return -1;
    }

    namedWindow(windowName,0);
    resizeWindow(windowName,1000,1000);

    while(true){

    	cap >> frame;

    	bool found = false;

    	std::vector<cv::Point2f> PointBuffer;

    	switch(PatternType){
    		case CHESSBOARD:
    			found = cv::findChessboardCorners(frame, Size(6,9),PointBuffer);
    			break;
    		case RING_GRID:
    			found = findRingGridPattern(frame,Size(6,9),PointBuffer);
    			break;
    		default:
    			found = false;
    			break;
    	}

    	if(found){
    		if(PatternType == CHESSBOARD){
    			cv::Mat gray;
    			cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    			cv::cornerSubPix(gray, PointBuffer, Size(11,11), Size(-1,-1),criteria);
    			cv::drawChessboardCorners(frame, Size(6,9), PointBuffer,found);
    		}
    	}

    	




    	cv::imshow(windowName,frame);

    	int key = cv::waitKey(1);
    	if (key == 27)
    		break;

    }

    //terminando el programa
    cap.release();
    cv::destroyAllWindows();	
}