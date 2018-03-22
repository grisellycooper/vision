#include "addFunctions.h"

#define RGBWindowName "RGB"
#define DEPTHWindowName "Depth"
#define NormalWindowName "Normals"

int main(){

	cv::VideoCapture capture( CV_CAP_OPENNI );
	capture.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ );
	capture.set(CV_CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION,1); 

	for(;;)
	{
	    Mat depthMap; // Depth Map con formato CV_16UC1
	    Mat PointCloudMap; // Point cloud CV_32FC3
	    Mat bgrImage; // Color Image CV_8UC3

	    capture.grab();

	    capture.retrieve( depthMap, CV_CAP_OPENNI_DEPTH_MAP );
	    capture.retrieve( PointCloudMap, CV_CAP_OPENNI_POINT_CLOUD_MAP );
	    capture.retrieve( bgrImage, CV_CAP_OPENNI_BGR_IMAGE );

	    cv::Mat FalseColorDepthMap = displayDepth(depthMap);

	    cv::Mat recDepthMap = RectifyDepthMap(depthMap);

	    cv::Mat NormalMap = estimateNormals(recDepthMap);

	    cv::Mat FalseColorNormalMap = displayNormal(NormalMap);

	    cv::rectangle(FalseColorDepthMap, cv::Point(200,200), cv::Point(300,300),Scalar(1,250 ,0), 2.0f);
	    cv::rectangle(NormalMap, cv::Point(200,200), cv::Point(300,300),Scalar(0,1,0), 2.0f);

	    cv::circle(FalseColorNormalMap,cv::Point(280,220), 3, Scalar(255,255,255));

	    cv::circle(NormalMap,cv::Point(320,240), 3, Scalar(255,255,255));
	    cv::circle(FalseColorDepthMap,cv::Point(320,240), 3, Scalar(255,255,255));

	    cv::namedWindow(RGBWindowName);
	    cv::namedWindow(DEPTHWindowName);
	    cv::namedWindow(NormalWindowName);

	    cv::imshow(RGBWindowName,bgrImage);
	    cv::imshow(DEPTHWindowName,FalseColorDepthMap);
	    cv::imshow(NormalWindowName,NormalMap);

	    int key = waitKey(10000000);

	    if(key == 27)
	        break;
	}
}
