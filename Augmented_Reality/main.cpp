/**
Codigo para Compilar
g++ -std=c++11 main.cpp `pkg-config opencv --cflags --libs` -o app '-Wl,-rpath,/home/maxito911/Qt/5.9.1/gcc_64/lib' && ./app
**/

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

#define WinName "frame"

int main(int argc, char const *argv[])
{
	
	VideoCapture cap(0);

	if( !(cap.isOpened()) ){
		cout << "No se pudo leer\n";
		return 0;
	}

	for(;;){
		cv::Mat frame;
		cap >> frame;

		if(frame.empty()) break;

		cv::imshow(WinName,frame);
		if(waitKey(10) == 27) break;
	}
	return 0;
}