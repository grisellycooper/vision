/**
Solo usaremos el patron de anillos

g++ -std=c++11 -O3 main.cpp addFunctions.cpp `pkg-config opencv --cflags --libs` -o main && ./main
**/

#include "includes.h"

#define path "../img/30/20/"

int patternType = RINGS_GRID;
int noImages = 20; // Numero de imagenes para la Calibración
int noIterations = 30;
float squareSize = 0.04540;//meters
cv::Size imgPixelSize = Size(640,480); // Tamaño de la imagen
cv::Size patternSize = cv::Size(5,4);

// FUNCION MAIN

int main(){
	cv::Mat frame;
	std::vector< std::vector<cv::Point3f> > objPoints; // Puntos de nuestro objeto(Patron de calibracion)
	// Suponemos que el patron se encuentra de forma paralela a la camara, y a una misma altura
	std::vector< std::vector<cv::Point2f> > imgPoints; // 2D Points en la Imagen(Pixels)


	// Inital Calibration
	objPoints.resize(1);
	calcBoardCornerPositions(cv::Size(5,4),squareSize,objPoints[0],patternType);
	objPoints.resize(noImages,objPoints[0]);

	bool isTracking; // Variable para ayudar a la función FindRingGridPattern
	std::vector<cv::Point2f> oldPoints; // Punto usados para el Tracking en RingGrid

	cv::namedWindow(windowName,0);
	cv::resizeWindow(windowName,1000,1000);


	//Variables para guardar los Valores de Correccion
	double rms;
    cv::Mat cameraMatrix = cv::Mat::eye(3,3,CV_64F); // Matriz para guardar la camera Intrinsics
    cv::Mat distCoeffs = cv::Mat::zeros(8, 1,CV_64F); // Aqui guardamos los coeficientes de Distorsion
    std::vector<cv::Mat> rvecs,tvecs; //Vectores de rotacion y de traslacion para cada frame

    //Capturamos las matrices
	FOR(i,noImages){
		
		string filename = path + std::to_string(i)  +  ".jpg";

		frame = cv::imread(filename,CV_LOAD_IMAGE_COLOR);

		std::vector<cv::Point2f> PointBuffer;

		isTracking = false; // Para que busque en todas las imagenes
		bool found = findRingsGridPattern(frame,patternSize, PointBuffer, isTracking,oldPoints);

		//PrintSTDVector(PointBuffer);

		if(found){
			imgPoints.push_back(PointBuffer);
			cv::drawChessboardCorners(frame,patternSize, PointBuffer,found);
		}
		else{
			cout << "Patron no encontrado\n";
		}


		cv::imshow(windowName,frame);

		int key = cv::waitKey(100000);

		bool c = true;
		switch(key){
			case 27:{
				c = false;
				break;
			}
			case 'c': //Pasar al siguiente frame
				break;
			default:
				break;
		}
		if(c) continue;
		else break;
	}

	// Calibracion Iterativa
	vector<float> rms_set;
	std::vector<cv::Point2f> fronto_corners = getFrontoParallelCorners(imgPixelSize,patternSize);
	
	FOR(it,noIterations)
	{
		// cout << "=================================\n";
		// cout << "           Iteracion " << it << endl;
		// cout << "=================================\n";
		// Limpiamosc variables
		rvecs.clear(); tvecs.clear();
		// cout << imgPoints.size() << endl;

		// Comenzamos la Calibracion
		rms = cv::calibrateCamera(objPoints,imgPoints, imgPixelSize,cameraMatrix,distCoeffs,rvecs,tvecs);
		cout << it << " " << cameraMatrix.at<double>(0,0) << " " << cameraMatrix.at<double>(1,1) <<
		" " << cameraMatrix.at<double>(0,2) << " " << cameraMatrix.at<double>(1,2) << " " << rms << " ";
		// cout << "El error de reproyeccion obtenido fue de " << rms << endl;
		// cout << "Matriz Intrinseca:" << endl << cameraMatrix << endl;
		// cout << "Coeficientes de Distorsion: " << endl << distCoeffs << endl;

		rms_set.push_back(rms);

		std::vector< std::vector<cv::Point2f> > imgPoints2;

		vector<float> v; // Para sacar un promedio de las colinealidades del vector

		// Mostrar imagens sin Distorsion
		FOR(i,noImages){
			string filename = path + std::to_string(i)  +  ".jpg";
			frame = cv::imread(filename,CV_LOAD_IMAGE_COLOR);



			//getAvgColinearityFromVector( PointBuffer, patternSize );

			cv::Mat temp = frame.clone();
			cv::Mat OptimalMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, Size(640,480), 1.0);
			cv::undistort(temp,frame,cameraMatrix,distCoeffs,OptimalMatrix);


			std::vector<cv::Point2f> PointBuffer;

			// Buffer de puntos hallados usando el algoritmo
			// // isTracking = false; // Para que busque en todas las imagenes
			// // bool found = findRingsGridPattern(frame,patternSize, PointBuffer, isTracking,oldPoints);
			// //cv::imshow(windowName, frame);
			// //waitKey(100000);
			//
			// if(found){
			// 	//imgPoints2.push_back(PointBuffer);
			// 	//cv::drawChessboardCorners(frame,patternSize, PointBuffer,found);
			// }
			// else{
			// 	cout << "Patron no encontrado\n";
			// }

			// Buffer hallado usando solo los coeficientes de calibrate camera
			// undistorting points only
			//PrintSTDVector(imgPoints[i]);
			cv::undistortPoints(imgPoints[i], PointBuffer, cameraMatrix, distCoeffs, cv::noArray(),OptimalMatrix);

			//PrintSTDVector(PointBuffer);
			//cv::drawChessboardCorners(frame,patternSize, PointBuffer,true);


			float m = getAvgColinearityFromVector( PointBuffer, patternSize );
			v.push_back(m);


			// Almacenamos solo cuatro esquinas
			std::vector<cv::Point2f> corners1 = extractCorners(PointBuffer,patternSize);
			//PrintSTDVector(corners1);
			//Verificación de los puntos obtenidos

			

			/**
	        for(int i = 0; i < corners1.size(); i++){
	        	putText(frame,to_string(i),Point(corners1[i].x,corners1[i].y),FONT_HERSHEY_SIMPLEX,0.4,Scalar(255,0,0),1,CV_AA);
	        	putText(frame,to_string(i),Point(corners2[i].x,corners2[i].y),FONT_HERSHEY_SIMPLEX,0.4,Scalar(255,0,0),1,CV_AA);
	        }
	        **/

	        cv::Mat H = cv::findHomography(corners1,fronto_corners);
	        //cout << "H:\n" << H << endl;
	        //cout << "H.inv:\n" << H.inv() << endl;

	        //Transformacion Fronto Parallel
	        cv::Mat imgWarp;
	        cv::warpPerspective(frame,imgWarp,H,Size(320,240));

	        /**

	        cv::Mat imgWarp_gray;
	        cv::cvtColor(imgWarp,imgWarp_gray,CV_BGR2GRAY);
	        adaptiveThreshold(imgWarp_gray,imgWarp_gray,255,ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY,45,4);

	        int kernel_size = 5;
			int scale = 1;
			int delta = 0;
			int ddepth = CV_16S;

			cv::Mat imgWarp_dst;
	        cv::Laplacian(imgWarp_gray, imgWarp_dst, ddepth,kernel_size,scale,delta,cv::BORDER_DEFAULT);

	        cv::Mat abs_imgWarp_dst;
	        cv::convertScaleAbs(imgWarp_dst,abs_imgWarp_dst);

	        //cv::cvtColor(imgWarp,imgWarp,CV_GRAY2BGR);

	        cv::Mat newFrame;
	        //cv::hconcat(frame,imgWarp,newFrame); **/


	        PointBuffer.clear();
			isTracking = false; // Para que busque en todas las imagenes
			bool found2 = findRingsGridPattern(imgWarp,patternSize, PointBuffer, isTracking,oldPoints);

			cv::imshow("a", imgWarp);
			waitKey(100000);



			if(!found2){
				//cv::drawChessboardCorners(imgWarp,patternSize, PointBuffer,found);
				cout << "no se pudo enconrtar el patron en la proyeccion FrontoParallel\n";
				//return 0;
			}

			//Transformacion Fronto Parallel Inversa
			cv::Mat imgWarp_inv;
	        cv::warpPerspective(imgWarp,imgWarp_inv,H.inv(),frame.size());

	        vector<Point2f> points_buffer2;

	        cv::perspectiveTransform( PointBuffer, points_buffer2, H.inv() );

	        //PrintSTDVector(points_buffer2);

	        /*
	        for(int i = 0; i < points_buffer2.size(); i++){
	        	putText(imgWarp_inv,to_string(i),Point(points_buffer2[i].x,points_buffer2[i].y),FONT_HERSHEY_SIMPLEX,0.4,Scalar(255,0,0),1,CV_AA);
	        }
	        */


	        //cout << "Intrinsics: " << endl <<  cameraMatrix <<  endl;
	        //cout <<  cameraMatrix.at<double>(0,0) << endl; 
	        //cout << "coeff dist: " << distCoeffs  << endl;

	        //vector<Point2f> corrected_points  = distortion(points_buffer2,cameraMatrix,distCoeffs);
	        //std::vector<cv::Point3f> corrected_points_tmp;
	        std::vector<cv::Point2f> corrected_points;
	        //cv::Mat rtemp = cv::Mat::zeros(3,1,CV_64F);
	        //cv::Mat ttemp = cv::Mat::zeros(3,1,CV_64F);

	        //cv::undistortPoints(imgPoints[i], PointBuffer, cameraMatrix, distCoeffs, cv::noArray(),OptimalMatrix);

	        // Distorsión Inversa
	        cv::undistortPoints(points_buffer2,corrected_points,OptimalMatrix,-distCoeffs,cv::noArray(),cameraMatrix);
	        //cv::convertPointsToHomogeneous(points_buffer2,corrected_points_tmp);
	        //cv::projectPoints(corrected_points_tmp, rtemp, ttemp, cameraMatrix, distCoeffs, corrected_points);

	        //PrintSTDVector(corrected_points);
	       	
	       	cv::drawChessboardCorners(imgWarp_inv, patternSize, corrected_points, true);
	       	cv::drawChessboardCorners(imgWarp_inv, patternSize, imgPoints[i], true);
	       	//vector<Point2f> corrected_points;
	       	//cv::projectPoints( corrected_points,  )
	       	//PrintSTDVector(corrected_points);
	       	//PrintSTDVector(imgPoints[i]);

	       	imgPoints2.push_back( corrected_points );

	       

	        cv::imshow("h",imgWarp);
	        cv::imshow("inv",imgWarp_inv);
			cv::imshow(windowName,frame);

			int key = cv::waitKey(10);

			bool c = true;
			switch(key){
				case 27:{
					c = false;
					break;
				}
				case 'c': //Pasar al siguiente frame
					break;
				default:
					break;
			}
			if(c) continue;
			else break;

		}
		
		FOR(i,noImages)
			FOR(j,patternSize.width * patternSize.height){
				imgPoints[i][j].x = (imgPoints[i][j].x +  imgPoints2[i][j].x) / 2.0;
				imgPoints[i][j].y = (imgPoints[i][j].y +  imgPoints2[i][j].y) / 2.0;
			}

		cout << printAvgColinearity(v) << endl;

		//rms = cv::calibrateCamera(objPoints,imgPoints2, imgPixelSize,cameraMatrix,distCoeffs,rvecs,tvecs);
		//cout << "El error de reproyeccion obtenido fue de " << rms << endl;
	}

	//std::cout << std::min_element( std::begin(rms_set), std::end(rms_set) ) << std::endl;
	std::sort( rms_set.begin(), rms_set.end() );

	// cout << "El menor rms obtenido: "<< rms_set[0] << endl;

	//terminando el programa
    cv::destroyAllWindows();

	return 0;
}