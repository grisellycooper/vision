/**
Solo usaremos el patron de anillos

g++ -std=c++11 -O3 main.cpp addFunctions.cpp `pkg-config opencv --cflags --libs` -o main && ./main
**/

#include "includes.h"

int patternType = RINGS_GRID;
int noImages = 6; // Numero de imagenes para la Calibración
float squareSize = 0.04540;
cv::Size patternSize = cv::Size(5,4);

cv::Mat frame;
std::vector< std::vector<cv::Point3f> > objPoints; // Puntos de nuestro objeto(Patron de calibracion)
// Suponemos que el patron se encuentra de forma paralela a la camara, y a una misma altura
std::vector< std::vector<cv::Point2f> > imgPoints; // 2D Points en la Imagen(Pixels)

int main(){
	// Inital Calibration
	objPoints.resize(1);
	calcBoardCornerPositions(cv::Size(5,4),squareSize,objPoints[0],patternType);
	objPoints.resize(noImages,objPoints[0]);

	bool isTracking = false; // Variable para ayudar a la función FindRingGridPattern
	std::vector<cv::Point2f> oldPoints; // Punto usados para el Tracking en RingGrid

	cv::namedWindow(windowName,0);
	cv::resizeWindow(windowName,1000,1000);


	FOR(i,noImages){
		
		string filename = "../img/" + std::to_string(i)  +  ".jpg";

		frame = cv::imread(filename,CV_LOAD_IMAGE_COLOR);

		std::vector<cv::Point2f> PointBuffer;

		bool found = findRingsGridPattern(frame,patternSize, PointBuffer, isTracking,oldPoints);
		if(isTracking)
			oldPoints = PointBuffer;

		if(found)
			cv::drawChessboardCorners(frame,patternSize, PointBuffer,found);


		cv::imshow(windowName,frame)

		if(key == 27) break;

	}

	//terminando el programa
    cv::destroyAllWindows();


	return 0;
}