/**
g++ -std=c++11 -O3 main.cpp addfunctions.cpp `pkg-config opencv --cflags --libs` -o main && ./main
**/

#include "includes.h"

//#define video_path "videos/calibration_mslifecam.avi"
//#define video_path "../videos/calibration_ps3eyecam.avi"


//#define video_path "../videos/chessboard_livecam.webm"
//#define video_path "../videos/rings_livecam.webm"

// videos de test
//#define video_path "../videos/test/LifeCam_chess.avi"
//#define video_path "../videos/test/LifeCam_asymmetric_circles.avi"
//#define video_path "../videos/test/LifeCam_rings.avi"

//#define video_path "../videos/test/PS3_chess.avi"
//#define video_path "../videos/test/PS3_asymmetric_circles.avi"
#define video_path "../videos/test/PS3_rings.avi"

/** Global Variables **/
cv::Mat frame;
//int patternType = CHESSBOARD; // Tipo de Patrón empleado (Ver Enum en includes)
//int patternType = ASYMMETRIC_CIRCLES_GRID; // Tipo de Patrón empleado (Ver Enum en includes)
int patternType = RINGS_GRID; // Tipo de Patrón empleado (Ver Enum en includes)
cv::Size imgPixelSize = Size(640,480); // Tamaño de la imagen
cv::Size patternSize[] = {Size(6,9),Size(4,5),Size(4,11),Size(5,4)}; // Varia dependiendo del tipo de patron que usamos
float squareSizes[] = {0.02315,0.02315,0.02315,0.04540}; // Separacion Real(en m) entre los puntos detectados
// el Size(x,y) .. x: numero de filas, y: numero de columnas
int noImages = 75; // Number of Images used for calibration
int sample_FPS = 20; // trata de tomar una muestra cierto FPS
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
    cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);
#endif

    if(!cap.isOpened()){
    	cout << "Cannot open the video file!" << endl;
    	return -1;
    }

    namedWindow(windowName,0);
    resizeWindow(windowName,1000,1000);

    double rms, totAvgErr;
    cv::Mat cameraMatrix = cv::Mat::eye(3,3,CV_64F); // Matriz para guardar la camera Intrinsics
    cv::Mat distCoeffs = cv::Mat::zeros(8, 1,CV_64F); // Aqui guardamos los coeficientes de Distorsion
    std::vector<cv::Mat> rvecs,tvecs; //Vectores de rotacion y de traslacion para para frame
    std::vector<float> reprojErrors; // Vector que calcula los errores para cada punto en todos los frames

    bool isTracking = false; // Variable que ayuda a FindRingPatterns
    std::vector<cv::Point2f> oldPoints; // Punto usados para el Tracking en RingGrid
    std::vector<float> avgColinearityPreCalibrated;
    std::vector<float> avgColinearity;

    double elapsed_seconds = 0.0; // Para medir el tiempo
    double time = 0.0;
    cout << "Empezando Captura\n";
    auto start = std::chrono::system_clock::now();

    int key;
    int counter = 0;
    while(true){

    	//Capturamos un frame O.o!
    	cap >> frame;
        if(frame.empty())
            break;

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
    			//Esta funcion llena nuestro objPoints vector of vectors, suponiendo una superficie plana
	    		objPoints.resize(1);
				calcBoardCornerPositions(patternSize[patternType],squareSizes[patternType],objPoints[0],patternType);
				objPoints.resize(noImages,objPoints[0]);

	    		bool found = false;

			    std::vector<cv::Point2f> PointBuffer;

			    switch(patternType){
			    	case CHESSBOARD:
			    		found = cv::findChessboardCorners(frame, patternSize[CHESSBOARD],PointBuffer);
			    		break;
                    case CIRCLES_GRID:
                        found = cv::findCirclesGrid(frame,patternSize[CIRCLES_GRID],PointBuffer);
                        break;
                    case ASYMMETRIC_CIRCLES_GRID:
                        found = cv::findCirclesGrid(frame,patternSize[ASYMMETRIC_CIRCLES_GRID],PointBuffer,CALIB_CB_ASYMMETRIC_GRID);
                        break;
			    	case RINGS_GRID:
			    		found = findRingsGridPattern(frame,patternSize[RINGS_GRID],PointBuffer,isTracking,oldPoints);
                        if(isTracking)
                            oldPoints = PointBuffer;

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

                    avgColinearityPreCalibrated.push_back( getAvgColinearityFromVector(PointBuffer, patternSize[patternType]));

		    		cv::drawChessboardCorners(frame, patternSize[patternType], PointBuffer,found);
			    		//Agregamos el patrón encontrado a nuestros imgPoints
                    if(counter % sample_FPS == 0){
		    		    imgPoints.push_back(PointBuffer);
                        cout << "Imagen Capturada. Imagenes Totales: " << imgPoints.size() << endl;
                    }
		    	}

    			// verificamos si ya encontramos suficientes imagenes
    			// caso contrario esperamos una cantidad de tiempo para tratar de capturar otra imagen
    			if(imgPoints.size() == noImages)
    				mode++;
    			else
    				key = cv::waitKey(1); // Tiempo de espera 1000ms
    			break;
    		}

    		case CALIBRATION_MODE:{
                auto end = std::chrono::system_clock::now();
                elapsed_seconds += std::chrono::duration_cast<std::chrono::duration<double> >(end - start).count();
                cout << "Captura Finalizada. Tiempo ===> " << elapsed_seconds * 1000.0 << " ms"<< endl;

    			rms = calibrateCamera(objPoints, imgPoints, imgPixelSize, cameraMatrix, distCoeffs, rvecs, tvecs);
    			cout << "El error de reproyeccion obtenido fue de " << rms << endl;
                cout << "Matriz Intrinseca" << endl << cameraMatrix << endl;
                cout << "Coeficientes de distorsion" << endl << distCoeffs << endl;

                totAvgErr = computeReprojectionErrors(objPoints,imgPoints,rvecs, tvecs,cameraMatrix,distCoeffs, reprojErrors);
                cout << "Average Reprojection Error: " << totAvgErr << endl;

    			cout << "Empezando la InDistorsion \n";
                mode++;
                break;
            }

            case UNDISTORTION_MODE:{
                cv::Mat temp = frame.clone();
                cv::undistort(temp,frame,cameraMatrix,distCoeffs);

                bool found = false;
                std::vector<cv::Point2f> PointBuffer;

                //Calculando la colinearidad
                switch(patternType){
                    case CHESSBOARD:
                        found = cv::findChessboardCorners(frame, patternSize[CHESSBOARD],PointBuffer);
                        break;
                    case CIRCLES_GRID:
                        found = cv::findCirclesGrid(frame,patternSize[CIRCLES_GRID],PointBuffer);
                        break;
                    case ASYMMETRIC_CIRCLES_GRID:
                        found = cv::findCirclesGrid(frame,patternSize[ASYMMETRIC_CIRCLES_GRID],PointBuffer,CALIB_CB_ASYMMETRIC_GRID);
                        break;
                    case RINGS_GRID:
                        found = findRingsGridPattern(frame,patternSize[RINGS_GRID],PointBuffer,isTracking,oldPoints);
                        if(isTracking)
                            oldPoints = PointBuffer;

                        break;
                    default:
                        found = false;
                        break;
                }

                if(found){
                    avgColinearity.push_back( getAvgColinearityFromVector(PointBuffer, patternSize[patternType]));
                    cv::drawChessboardCorners(frame, patternSize[patternType], PointBuffer,found);
                    if(counter % sample_FPS == 0){
                        printAvgColinearity(avgColinearity);
                    }
                }

    			key = waitKey(1);
    			break;
    		}
    	}


    	cv::imshow(windowName,frame);
    	counter++;

    	if (key == 27)
    		break;

    }

    //Cuando termina antes de tiempo
    if(imgPoints.size() < noImages){
        objPoints.resize(imgPoints.size(),objPoints[0]);
        auto end = std::chrono::system_clock::now();
        elapsed_seconds += std::chrono::duration_cast<std::chrono::duration<double> >(end - start).count();
        cout << "Captura Finalizada. Tiempo ===> " << elapsed_seconds * 1000.0 << " ms"<< endl;

        rms = calibrateCamera(objPoints, imgPoints, imgPixelSize, cameraMatrix, distCoeffs, rvecs, tvecs);
        cout << "El error de reproyeccion obtenido fue de " << rms << endl;
        cout << "Matriz Intrinseca" << endl << cameraMatrix << endl;
        cout << "Coeficientes de distorsion" << endl << distCoeffs << endl;

        totAvgErr = computeReprojectionErrors(objPoints,imgPoints,rvecs, tvecs,cameraMatrix,distCoeffs, reprojErrors);
        cout << "Average Reprojection Error: " << totAvgErr << endl;
    }
    cout << "========================\n";
    cout << "Colinearidad Pre Calibracion: " << endl;
    cout << "========================\n";
    printAvgColinearity(avgColinearityPreCalibrated);
    cout << "========================\n";
    cout << "Colinearidad Calibrada: " << endl;
    cout << "========================\n";
    printAvgColinearity(avgColinearity);

    //terminando el programa
    cap.release();
    cv::destroyAllWindows();
}