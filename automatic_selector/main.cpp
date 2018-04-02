/**
Codigo para Compilar
g++ -std=c++11 main.cpp addFunctions.cpp `pkg-config opencv --cflags --libs` -o app && ./app
**/
#include "includes.h"

#define WinName "frame"

// videos de test
//#define video_path "../videos/test/LifeCam_chess.avi"
//#define video_path "../videos/test/LifeCam_asymmetric_circles.avi"
#define video_path "../videos/test/LifeCam_rings.avi"

//#define video_path "../videos/test/PS3_chess.avi"
//#define video_path "../videos/test/PS3_asymmetric_circles.avi"
//#define video_path "../videos/test/PS3_rings.avi"


int patternType = RINGS_GRID;
int noImages = 30; // Numero de imagenes para la Calibración
//int noIterations = 30;
float squareSize = 0.04540;//meters
cv::Size imgPixelSize = Size(640,480); // Tamaño de la imagen
cv::Size patternSize = Size(5,4);
cv::Size detectionGridSize = Size(4,3);
int noOutputImages = 30;


int main(int argc, char const *argv[])
{
	
	VideoCapture cap(video_path);

	if( !(cap.isOpened()) ){
		cout << "No se pudo leer\n";
		return 0;
	}

	cv::namedWindow(WinName,0);
	cv::resizeWindow(WinName,640,480);

	bool isTracking; // Variable para ayudar a la función FindRingGridPattern
	std::vector<cv::Point2f> oldPoints; // Punto usados para el Tracking en RingGrid

	std::vector< cv::Mat > voMats;
	// Guarda los cuadrantes que fueron contabilizados en cada uno de los frames capturados
	std::vector< std::vector<bool> > voAffections; 

	for(;;){
		cv::Mat frame;
		cap >> frame;

		if(frame.empty()) break; // Verificación de que hayamos capturado un frame

		std::vector<cv::Point2f> PointBuffer;

		isTracking = false; // Para que busque en todas las imagenes
		bool found = findRingsGridPattern(frame,patternSize, PointBuffer, isTracking,oldPoints);

		if(found){
			//cv::drawChessboardCorners(frame,patternSize, PointBuffer,found);
			// Registramos el frame
			voMats.push_back(frame);

			// Registrar los cuadrantes donde afecto la detección
			voAffections.push_back( calc_affection(PointBuffer, imgPixelSize, detectionGridSize) );

		}


		cv::imshow(WinName,frame);
		if(waitKey(10) == 27) break;


	}

	// Numero total de frames donde se detecto el patron
	cout << "Numero Total de Frames Capturados: " << voMats.size() << endl;

	// FOR(i,voMats.size()){
	// 	cv::imshow(WinName,voMats[i]);
	// 	if(waitKey(10) == 27) break;		
	// }

	// Obtenemos la mejor combinación de frames
	std::vector<int> bestFrames = calc_BestFrameCombination(voAffections,noOutputImages);

	// Escribimos los frames correspondientes
	FOR(i, bestFrames.size()){
		std::string str = "../img/30/" + std::to_string(i)+".jpg";
		bool captured = cv::imwrite(str,voMats[ bestFrames[i] ]);

		if(!captured) cout << "No se pudo Capturar Imagen " << i << endl;
	}

	return 0;
}