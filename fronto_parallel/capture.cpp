/**
Capturar Imagenes del video
g++ -std=c++11 capture.cpp `pkg-config opencv --cflags --libs` -o capture && ./capture
**/
#include <iostream>
#include <string>
#include <cstring>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define windowName "video"

/*
#define video_path "../videos/ps3/rings.avi"
#define video_path "../videos/ps3/chess.avi"
#define video_path "../videos/ps3/acircles.avi"
#define video_path "../videos/lifecam/rings.avi"
#define video_path "../videos/lifecam/chess.avi"
#define video_path "../videos/lifecam/acircles.avi"
*/

int main(){

	std::string video_path[6] = { "../videos/ps3/rings.webm",
							 "../videos/ps3/chess.webm",
							 "../videos/ps3/acirc.webm", 
							 "../videos/lif/rings.webm", 
							 "../videos/lif/chess.webm",  
							 "../videos/lif/acirc.webm"};

	for( int i=0;i<6;i++)
	{
		cv::VideoCapture cap(video_path[i]);
		cv::namedWindow(windowName,0);
		cv::resizeWindow(windowName,1000,1000);

		cv::Mat frame;

		int count = 0;
		std::cout << "==================================\n";
		std::cout << "Opciones:\n" << "\tx : Capturar Imagen\n" << "\tc : Pasar al Siguiente\n" << "\tESC : Terminar\n";
		std::cout << "==================================\n";

		for( int j=0;j<10;){

			cap >> frame;
			if(frame.empty()) break;
			cv::imshow(windowName,frame);
			int key = cv::waitKey(10000);
			bool c = true; // Check key for continue

			switch(key){
				// Capture Frame
				case 'x':{					
					std::string str = "../img/" + video_path[i].substr(10,9) + "/" + std::to_string(j) + ".jpg";
					std::cout << "path : " << str << std::endl;
					bool captured = cv::imwrite(str, frame);
					if(captured){
						std::cout << "Imagen Capturada\n" << "escrito en: " << str << std::endl;	
						j++;
					} 
					else std::cout << "Problema al Capturar Imagen\n";
					break;
				}
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
		//terminando el programa
	    cap.release();
	}
		
	    cv::destroyAllWindows();
	
	
}