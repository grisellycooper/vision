#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
//#include <opencv2/videoio.hpp>

#include <iostream>
#include <fstream>
#include <math.h>

using namespace cv;
using namespace std;

//Useful MACROS
#define FOR(i,n) for(int i = 0; i < n; i++)

cv::Vec3f pointFromDepth(cv::Vec2f uv, float depth){

	float fx = 525.0;  // focal length x
	float fy = 525.0;  // focal length y
	float cx = 319.5;  // optical center x
	float cy = 239.5;  // optical center y

	float factor = 1000; // for the 16-bit PNG files
	//# OR: factor = 1 # for the 32-bit float images in the ROS bag files

	 float Z = depth / factor;
	 float X = (uv[1] - cx) * Z / fx;
	 float Y = (uv[0] - cy) * Z / fy;

	 return cv::Vec3f(X,Y,Z);
}

//Funcion que ayuda a visualizar los Depth frames
cv::Mat displayDepth(cv::Mat depthMap){
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

	return FalseColorMap;
}

//Devuelve un Depth Map en CV_32FC1
cv::Mat RectifyDepthMap(const cv::Mat & depthMap){
	cv:: Mat mat(depthMap.size(),CV_32FC1);
	uint16_t *pSource = (uint16_t*) depthMap.data;

	FOR(x,depthMap.rows)
		FOR(y,depthMap.cols){
			uint16_t value = (uint16_t) (*(pSource));

			if(value == 0)
				mat.at<float>(x,y) = 0.0f;
			else
				//mat.at<float>(x,y) = std::pow(value, 3.0);
				mat.at<float>(x,y) = value;


			pSource++;
		}

	cout << "Depth at center: " << mat.at<float>(240,320) << endl;
	//cv::medianBlur(mat,mat,3);
	cv::GaussianBlur(mat, mat, Size(3,3),0,0);
	//cv::GaussianBlur(mat, mat, Size(3,3),0,0);

	return mat;
}

//Funcion para estimar las normales
// Input DepthMap(CV_32FC1)
// Output NormalMap(CV_32FC3)
cv::Mat estimateNormals( cv::Mat depthMap ){
	cv::Mat normals(depthMap.size(),CV_32FC3);

	for(int x = 0; x < depthMap.rows ; x++){
		for(int y = 0; y < depthMap.cols ; y++)
		{

			cv::Vec3f P = pointFromDepth(cv::Vec2f(x,y),depthMap.at<float>(x,y));

			cv::Vec3f Px = pointFromDepth(cv::Vec2f(x-1,y),depthMap.at<float>(x-1,y));
			//cv::Vec3f Pxx = pointFromDepth(cv::Vec2f(x+1,y),depthMap.at<float>(x+1,y));
			cv::Vec3f Py = pointFromDepth(cv::Vec2f(x,y-1),depthMap.at<float>(x,y-1));
			//cv::Vec3f Pyy = pointFromDepth(cv::Vec2f(x,y+1),depthMap.at<float>(x,y+1));

			/**

			float gx = cv::norm(Pxx-Px);
			float gy = cv::norm(Pyy-Py);

			float ax = atan(gx), ay = atan(gy);

			float dx = cos(ax) - sin(ax);
			float dy = cos(ay) + sin(ay);

			cv::Vec3f xv = cv::Vec3f(dx,0,0);
			cv::Vec3f yv = cv::Vec3f(0,dy,0);

			cv::Vec3f n = xv.cross(yv);

			normals.at<cv::Vec3f>(x,y) = cv::normalize(n);
			**/

			/**

			float dzdx = (depthMap.at<float>(x+1,y) - depthMap.at<float>(x-1,y)) / 2.0;
			float dzdy = (depthMap.at<float>(x,y+1) - depthMap.at<float>(x,y-1)) / 2.0;

			//cout << "dx:"<< dzdx << "dy:" << dzdy << " ";

			cv::Vec3f d( - dzdx , - dzdy , 1.0f );
			cv::Vec3f n = cv::normalize(d);

			normals.at<cv::Vec3f>(x,y) = n;

			//cout << "(" << normals.at<cv::Vec3f>(x,y)<< ")";
			**/

			
			
			cv::Vec3f t = Py;
			cv::Vec3f l = Px;
			cv::Vec3f c = P;

			cv::Vec3f d = (Py - P).cross(Px - P);

			cv::Vec3f n = cv::normalize(d);

			normals.at<cv::Vec3f>(x,y) = n;

		}
		//cout << endl;
	}

	cv::Vec3f C = pointFromDepth(cv::Vec2f(240,320), depthMap.at<float>(240,320));
	cv::Vec3f D = pointFromDepth(cv::Vec2f(220,280), depthMap.at<float>(220,280));
	cout << "Coord Central: " << C[0] << " " << C[1] << " " << C[2] << endl;
	cout << "Coord Central D: " << D[0] << " " << D[1] << " " << D[2] << endl;
	cout << "Normal Central: " << normals.at<cv::Vec3f>(240,320)[0] << " " << normals.at<cv::Vec3f>(240,320)[1] << " " << normals.at<cv::Vec3f>(240,320)[2] << endl;

	int min_x = 200;
	int max_x = 300;
	int min_y = 200;
	int max_y = 300;

	std::ofstream myfile("test.txt");
	for(int x = min_x; x < max_x ; x++){	
		for(int y = min_y; y < max_y ; y++){
			if(depthMap.at<float>(x,y) != 0 && myfile.is_open() && normals.at<cv::Vec3f>(x,y)[0] != 0){
				cv::Vec3f P = pointFromDepth(cv::Vec2f(x,y),depthMap.at<float>(x,y));
				myfile << std::to_string(P[0]) << " " << std::to_string(P[1]) << " " << std::to_string(P[2]) << " ";
				myfile << normals.at<cv::Vec3f>(x,y)[0] << " " << normals.at<cv::Vec3f>(x,y)[1] << " " << normals.at<cv::Vec3f>(x,y)[2] << "\n";
			}
		}
		//cout << endl;
	}
	myfile.close();

	return normals;
}

cv::Mat displayNormal(cv::Mat depthMap){
	// Codigo para visualizar el Depth Image Correspondiente
	double min = -1.0,max = 1.0;
	cv::minMaxIdx(depthMap,&min,&max);
	cout << "max:" << max << "min:" << min << endl;
	cv::Mat adjMap;
	//src.convertTo(adjMap,CV_8UC1,(double)255/(256*100),-min); // Coloramiento Uniforme
	depthMap.convertTo(adjMap,CV_8UC1,255/(max-min),-min); // Coloramiento de acuerdo a valores maximos y minimos

	cv::Mat FalseColorMap;
	cv::applyColorMap(adjMap,FalseColorMap,cv::COLORMAP_RAINBOW);
	cv::cvtColor(FalseColorMap,FalseColorMap,CV_BGR2RGB);

	return FalseColorMap;
}

