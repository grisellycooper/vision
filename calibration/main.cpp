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

/** Global Variables **/
cv::Mat frame;
int patternType = CHESSBOARD; // Tipo de Patrón empleado (Ver Enum en includes)
cv::Size imgPixelSize = Size(480,640); // Tamaño de la imagen
cv::Size patternSize = Size(6,9); // Varia dependiendo del tipo de patron que usamos
float squareSize = 0.03; // Separacion Real(en m) entre los puntos detectados
// el Size(x,y) .. x: numero de filas, y: numero de columnas
int noImages = 12; // Number of Images used for calibration
int mode = 0;

std::vector< std::vector<cv::Point3f> > objPoints; // Puntos de nuestro objeto(Patron de calibracion)
// Suponemos que el patron se encuentra de forma paralela a la camara, y a una misma altura
std::vector< std::vector<cv::Point2f> > imgPoints; // 2D Points en la Imagen(Pixels)



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

    double rms;
    cv::Mat cameraMatrix = cv::Mat::eye(3,3,CV_64F); // Matriz para guardar la camera Intrinsics
    cv::Mat distCoeffs = cv::Mat::zeros(8, 1,CV_64F); // Aqui guardamos los coeficientes de Distorsion
    std::vector<cv::Mat> rvecs,tvecs; //Vectores de rotacion y de traslacion para para frame

    int key;

    int counter = 0;
    while(true){

    	//Capturamos un frame O.o!
    	cap >> frame;

    	// El programa correra haciendo uso de diferentes modos

    	// Detect_mode: captura n images donde se capturo el patron de calibracion
    	// de forma correcta. Tambien arma los vectores objPoints e imgPoints.
    	// la captura de esta imagenes debera respetar cierto timming puese ser una
    	// toma cada segundo.

    	// Calibration_mode: corre la funcion de opencv, calibrate camera. Devuelve una
    	// matriz con los parametros y el error de reprojeccion.

    	// Undistortion_mode: Captura nuevos frames en la camara pero usa la matriz
    	// obtenida para rectificar la imagen, eliminando las distorsiones.
    	// (Podría agregarse una nueva metrica aqui)
    	switch(mode){

    		case DETECT_MODE:{
    			if(counter % 10 == 0){

	    			//Esta funcion llena nuestro objPoints vector of vectors, suponiendo una superficie plana
	    			objPoints.resize(1);
					calcBoardCornerPositions(patternSize,squareSize,objPoints[0],patternType);
					objPoints.resize(noImages,objPoints[0]);

	    			bool found = false;

			    	std::vector<cv::Point2f> PointBuffer;

			    	switch(patternType){
			    		case CHESSBOARD:
			    			found = cv::findChessboardCorners(frame, patternSize,PointBuffer);
			    			break;
			    		case RINGS_GRID:
			    			found = findRingGridPattern(frame,patternSize,PointBuffer);
			    			break;
			    		default:
			    			found = false;
			    			break;
			    	}

			    	// Si encontramos el patron --> lo dibujamos en el frame
			    	if(found){
			    		// Paso adicional para refinar esquina en el caso de Chessboard
			    		if(patternType == CHESSBOARD){
			    			cv::Mat gray;
			    			cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
			    			cv::cornerSubPix(gray, PointBuffer, Size(11,11), Size(-1,-1),criteria);
			    		}
			    		cv::drawChessboardCorners(frame, patternSize, PointBuffer,found);

			    		//Agregamos el patrón encontrado a nuestros imgPoints
			    		imgPoints.push_back(PointBuffer);

			    	}


	    			// verificamos si ya encontramos suficientes imagenes
	    			// caso contrario esperamos una cantidad de tiempo para tratar de capturar otra imagen
	    			if(imgPoints.size() == noImages)
	    				mode++;
	    			else
	    				key = cv::waitKey(1); // Tiempo de espera 1000ms
    			}
    			break;
    		}
    		case CALIBRATION_MODE:{
    			rms = calibrateCamera(objPoints, imgPoints, imgPixelSize, cameraMatrix, distCoeffs, rvecs, tvecs);
    			cout << "El error de reproyeccion obtenido fue de " << rms << endl;
    			mode++;
    			break;
    		}
    		case UNDISTORTION_MODE:{
    			cout << "Empezando la InDistorsion \n";
    			key = waitKey(1);
    			break;
    		}
    	}


    	cv::imshow(windowName,frame);
    	counter++;

    	if (key == 27)
    		break;

    }

    //terminando el programa
    cap.release();
    cv::destroyAllWindows();
}