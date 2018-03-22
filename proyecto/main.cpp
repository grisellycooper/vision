#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
//#include <opencv2/videoio.hpp>

#include <iostream>

using namespace cv;
using namespace std;


int main(){

	VideoCapture capture(0); // or CV_CAP_OPENNI
	for(;;)
	{
	    Mat depthMap;
	    Mat bgrImage;

	    capture.grab();

	    capture.retrieve( depthMap, CV_CAP_OPENNI_DEPTH_MAP );
	    capture.retrieve( bgrImage, CV_CAP_OPENNI_BGR_IMAGE );


	    // Codigo para visualizar el Depth Image Correspondiente
	    double min,max;
	    cv::minMaxIdx(depthMap,&min,&max);
	    cout << "max:" << max << "min:" << min << endl;
	    cv::Mat adjMap;
	    //src.convertTo(adjMap,CV_8UC1,(double)255/(256*100),-min); // Coloramiento Uniforme
	    depthMap.convertTo(adjMap,CV_8UC1,255/(max-min),-min); // Coloramiento de acuerdo a valores maximos y minimos

	    cv::Mat FalseColorMap;
	    cv::applyColorMap(adjMap,FalseColorMap,cv::COLORMAP_BONE);
	    cv::cvtColor(FalseColorMap,FalseColorMap,CV_BGR2RGB);

	    
	    cv::imshow("d",FalseColorMap);
	    cv::imshow("r",bgrImage);

	    if( waitKey( 30 ) >= 0 )
	        break;
	}
}
