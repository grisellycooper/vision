/**
g++ -std=c++11 main.cpp `pkg-config opencv --cflags --libs` -o main && ./main
**/

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <stdlib.h>

//Handle for our frames
cv::Mat frame; //Matriz fuente
cv::Mat gray;
cv::Mat detectedEdges;
cv::Mat Ellipsis;
cv::Mat dst; // Matriz Final

/**Variables for Global Threshold**/
bool playVideo =true;
//----------------------//
int edgeThresh = 1;
int lowThreshold = 371;
const int max_lowThreshold = 600;
int razon = 3;
//---------------------//
int thresh = 160;
int max_thresh = 255;

cv::RNG rng(12345);

using namespace std;
using namespace cv;

const char * WindowName = "Calibration";
const char * WindowRGB = "RGB";

void PreFilters(){
    /**Pasando a Grises**/
    cvtColor(frame,gray,CV_BGR2GRAY);
        //cout << gray.size().width << " "<< gray.size().height;

        /**
        //Acceso a los elementos BGR de un pixel
        for(int y = 0; y < 30;y++)
            for(int x = 0; x < 60; x++){
                // Los colores se enuentran en formato BGR
                frame.at<cv::Vec3b>(y,x)[0] = 255;
                frame.at<cv::Vec3b>(y,x)[1] = 255;
                frame.at<cv::Vec3b>(y,x)[2] = 255;
            }
        **/

    /**Equalizaciones**/
    //equalizeHist(gray,gray);
    GaussianBlur(gray,gray,Size(3,3),0);
    adaptiveThreshold(gray,gray,255,ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY,41,15);

    Mat kernel = getStructuringElement(MORPH_RECT,Size(3,3));
    //erode(gray,gray,kernel);
    //dilate(gray,gray,kernel);
    //dilate(gray,gray,kernel);
    //erode(gray,gray,kernel);
    //morphologyEx(gray,gray,MORPH_CLOSE,kernel);

    medianBlur(gray,gray,3);
    //GaussianBlur(gray,gray,Size(3,3),0);
}

void CannyThreshold(int,void *){
    //aplicando Canny
    detectedEdges = gray;
    //Canny(gray,detectedEdges,lowThreshold,lowThreshold*razon,3); //El ultimo es kernel size
    dst = Scalar::all(0);
    gray.copyTo(dst,detectedEdges);
}

bool cmpx(Point2f a,Point2f b){
    return a.x < b.x;
}

bool cmpy(Point2f a, Point2f b){
    return a.y < b.y;
}

float dist(Point2f a, Point2f b){
    return sqrt( pow(a.x-b.x,2)+pow(a.y-b.y,2) );
}

float NearestNeighbor(const vector<Point2f>& v, Point2f a){
    float min = 1000000.0;
    for(int i = 0; i < v.size(); i++){
        float d = dist(v[i],a);
        if(min > d)
            min = d;
    }
    return min;
}

vector<Point2f> getControlPoints(const vector<Point2f> & centers){
    std::vector<Point2f> v;
    float t = 5;
    for(int i = 0; i < centers.size();i++){
        std::vector<Point2f> tmp = centers;
        tmp.erase(tmp.begin()+i);
        if(NearestNeighbor(tmp,centers[i]) <= t)
            v.push_back(centers[i]);
    }
    return v;
}

void EraseDuplicates(vector<Point2f> & v){
    std::vector<Point2f> v2;
    for(int i = 0; i < v.size(); i++){
        std::vector<Point2f> tmp = v;
        tmp.erase(tmp.begin()+i);
        for(int j = 0; j < tmp.size();j++){
            float d = dist(v[i],tmp[j]);
            if(d < 0.5){
                bool flag = false;
                for(int k = 0; k < v2.size();k++){
                    if(dist(v[i],v2[k]) < 0.5) flag = true;
                }
                if(!flag)
                    v2.push_back(v[i]);
            }
        }
    }
    v = v2;
}

void EllipsisDetection(){
    vector<vector<Point>> contours;
    vector<Vec4i> hierachy;

    //Find Contours
    findContours(detectedEdges,contours,hierachy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,Point(0,0));

    //Find rotated rectangles and ellipsis
    vector<RotatedRect>minRect(contours.size());
    vector<RotatedRect>minEllipse(contours.size());


    for( int i = 0; i < contours.size(); i++ ){
        minRect[i] = minAreaRect( Mat(contours[i]) );
        if( contours[i].size() > 5 ){
            minEllipse[i] = fitEllipse( Mat(contours[i]) ); }
    }

    /// Draw contours + rotated rects + ellipses
    Ellipsis = detectedEdges;
    cvtColor(detectedEdges, Ellipsis,CV_GRAY2BGR);
    //Ellipsis = Mat::zeros( gray.size(), CV_8UC3 );
    //cout << "==================================================\n";

    vector<RotatedRect> selected;
    for( int i = 0; i< contours.size(); i++ ){
        float w = minEllipse[i].size.width;
        float h = minEllipse[i].size.height;
        float c_x = minEllipse[i].center.x;
        float c_y = minEllipse[i].center.y;
        float dif =w - h;
        float sum = w + h;
        float area = w*h;
        //if( abs(dif) <= 20 && sum > 4 && area > 20 && area < 1800){ //&& dist(minEllipse[i].center,Point2f(xm,ym)) < 100){
        if( abs(dif) <= 10){ //&& dist(minEllipse[i].center,Point2f(xm,ym)) < 100){
            if(hierachy[i][2] != -1)
                if(hierachy[ hierachy[i][2] ][2] == -1){
                    selected.push_back(minEllipse[i]);
                    //putText(Ellipsis,to_string(selected.size()),Point(c_x,c_y),FONT_HERSHEY_SIMPLEX,0.4,Scalar(255,0,0),1,CV_AA);
                    //cout << selected.size() << endl;
                    //cout <<"width: "<< w <<" height: "<< h << endl;
                    //cout <<"dif: "<< abs(dif) <<" Semiperimetro: "<< sum << " area: " << area<< endl; 
                }

            if(hierachy[i][3] != -1)
                if(hierachy[ hierachy[i][3] ][3] == -1){
                    selected.push_back(minEllipse[i]);
                    //putText(Ellipsis,to_string(selected.size()),Point(c_x,c_y),FONT_HERSHEY_SIMPLEX,0.4,Scalar(255,0,0),1,CV_AA);
                    //cout << selected.size() << endl;
                    //cout <<"width: "<< w <<" height: "<< h << endl;
                    //cout <<"dif: "<< abs(dif) <<" Semiperimetro: "<< sum << " area: " << area<< endl; 
                }

            ellipse( Ellipsis, minEllipse[i], Scalar(0,255,0), 1, 8 );
        }
    }

    
    vector<Point2f> centers;
    for( int i = 0; i < selected.size(); i++ ){        
        centers.push_back(selected[i].center);        
    }
/**
    // Calculating the mediana
    vector<Point2f> c1 = centers;
    vector<Point2f> c2 = centers;
    sort(c1.begin(),c1.end(),cmpx);
    sort(c2.begin(),c2.end(),cmpy);

    int n = c1.size()/2;
    float xm = c1[ n ].x;
    float ym = c2[ n ].y;  

    for( int i = 0; i< selected.size(); i++ ){
        if( dist(selected[i].center,Point2f(xm,ym)) < 200){
            Scalar color = Scalar( 0,255,0 );        
            //drawContours( Ellipsis, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
            ellipse( Ellipsis, selected[i], color, 1, 8 );
        }
    }
*/
    vector<Point2f> CPs = getControlPoints(centers);
    EraseDuplicates(CPs);
    cout << CPs.size()<<endl;

    for(int i = 0; i < CPs.size();i++){
        circle(Ellipsis,CPs[i],1,Scalar(0,0,255),1.5,8);
    }
    


    // Calculating the mediana
    vector<Point2f> c1 = CPs;
    vector<Point2f> c2 = CPs;
    sort(c1.begin(),c1.end(),cmpx);
    sort(c2.begin(),c2.end(),cmpy);

    int n = c1.size()/2;
    float xm = c1[ n ].x;
    float ym = c2[ n ].y; 

    int r;
    for(r = 1; r < 400; r++){
        int count = 0;
        for(int i = 0; i < CPs.size(); i++){
            if(dist(Point2f(xm,ym),CPs[i]) <= r)
                count++;
        }
        if(abs(count - 20)<2)
            break;
    }

    circle( Ellipsis, Point2f(xm,ym),r + 25,Scalar( 255,0,0 ), 1, 8 );

    for( int i = 0; i< CPs.size(); i++ ){
        if( dist(CPs[i],Point2f(xm,ym)) < r+25){
            Scalar color = Scalar( 255,0,0 );        
            //drawContours( Ellipsis, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
            circle( frame, CPs[i],2,color, 2, 8 );
        }
    }




}



int main(){

    //cv::VideoCapture cap("videos/calibration_mslifecam.avi");
    cv::VideoCapture cap("videos/calibration_ps3eyecam.avi");
    //cv::VideoCapture cap("videos/Kinect2_rgb.avi");
    //cv::VideoCapture cap("videos/realsense_Depth.avi");
    //cv::VideoCapture cap("videos/realsense_RGB.avi");

    if(!cap.isOpened()){
        cout << "Cannot open the video file" << endl;
        return -1;
    }

    double count = cap.get(CV_CAP_PROP_FRAME_COUNT); //get the frame count

    //Procesamos todos los frames

    namedWindow(WindowName,0);
    resizeWindow(WindowName,1000,1000);
    createTrackbar("Min Threshold:",WindowName,&lowThreshold,max_lowThreshold,CannyThreshold);
    createTrackbar("Min Threshold:",WindowName,&lowThreshold,max_lowThreshold,CannyThreshold);

    namedWindow(WindowRGB,0);
    resizeWindow(WindowRGB,800,600);


    int r = 0;
    bool finish = 1;

    while(finish)
    {
        cap.set(1,r);
        cap.read(frame);

        if(frame.empty()) break;
        dst.create( frame.size(), frame.type() );

        PreFilters();

        CannyThreshold(0,0);

        EllipsisDetection();

        imshow(WindowName,Ellipsis);
        imshow(WindowRGB,frame);

        //----Teclas para analizar los frames
        int key = waitKey(100000);//Espera 5 seg a que se presione un key
        switch(key){
        case 'l':
            r++;
            break;
        case 'j':
            r--;
            break;
        case 'd':
            r++;
            break;
        case 'a':
            r--;
            break;
        case 27:
            finish = 0;
            break;
        }

    }

    cap.release();
    destroyAllWindows();

}

