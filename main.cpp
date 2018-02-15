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

using namespace std;
using namespace cv;

//*****Control Points Vector********//
std::vector<Point2f> trackedPoints;
std::vector<Point2f> ControlPoints;
bool patternWasFound;
int numTrackedItems = 20;
bool isTracking = false;

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

const char * WindowName = "Calibration";
const char * WindowRGB = "RGB";

//Video Files
//#define video_path "videos/calibration_mslifecam.avi"
#define video_path "videos/calibration_ps3eyecam.avi"
//#define video_path "videos/Kinect2_rgb.avi"
//#define video_path "videos/realsense_Depth.avi"
//#define video_path "videos/realsense_RGB.avi"

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
    adaptiveThreshold(gray,gray,255,ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY,41,6);

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
    return sqrt( pow(a.x-b.x,2.0f)+pow(a.y-b.y,2.0f) );
}

// Funcion para Obtener los puntos de Control
// Entra un conjunto de puntos y devuelve los puntos casi iguales (20 max)
vector<Point2f> getControlPoints(const vector<Point2f> & centers){
    std::vector<Point2f> v;
    std::vector<int> alreadyRegistered;
    float t = 3.0f; // Valor de Desviacion Maxima
    for(int i = 0; i < centers.size();i++)
        for(int j= 0; j < centers.size(); j++){
            if(i != j && dist(centers[i],centers[j]) < t &&
            (std::find(alreadyRegistered.begin(), alreadyRegistered.end(),i) == alreadyRegistered.end() ||
            std::find(alreadyRegistered.begin(), alreadyRegistered.end(),j) == alreadyRegistered.end()) //&&
            //v.size() <= 20)
            )
            {
                // Aqui va el promedio de ambos
                float d_x = centers[i].x + centers[j].x;
                float d_y = centers[i].y + centers[j].y;
                v.push_back(Point2f(d_x/2.0f,d_y/2.0f));

                //Registramos los centros para no repetirlos
                alreadyRegistered.push_back(i);
                alreadyRegistered.push_back(j);
            }
        }

    return v;    
}
//=============================================================
//================ FUNCION SUPERIMPORTANTE ====================
//=============================================================
// Si la funcion tiene exito devulve true, mas la modificacion del vector de entrada
// Con el numero de filas y columnas especificadas
bool isColinear(Vec4f line, Point2f point){
    // Evaluamos para x e y
    float tx = (point.x - line[2]) / line[0];
    float ty = (point.y - line[3]) / line[1];

    if(abs(tx-ty) < 10.0f) return true;
    return false;
}

bool FindRingPattern(vector<Point2f> &probableCPs,int num_rows,int num_cols){
    int n = probableCPs.size();
    std::vector<Vec4f> lines;
    for(int i = 0; i < probableCPs.size();i++){
        for(int j = i+1; j < probableCPs.size();j++){
            std::vector<Point2f> probableLinePoints;
            Vec4f tmpLine;
            probableLinePoints.push_back(probableCPs[i]);probableLinePoints.push_back(probableCPs[j]);
            fitLine(probableLinePoints,tmpLine, CV_DIST_L2, 0, 0.01,0.01);
            lines.push_back(tmpLine);

            float vx = tmpLine[0],vy = tmpLine[1], x0 = tmpLine[2],y0 = tmpLine[3];
            float m = 80.0;
            //line(frame, Point2f(x0-m*vx, y0-m*vy), Point2f(x0+m*vx, y0+m*vy),Scalar(0,255,0));
        }
    }

    std::vector<Vec4f> selected;
    std::vector<int> already;
    for(int i = 0; i < lines.size();i++){
        for(int j = i+1; j < lines.size();j++){
            float a[2] = {lines[i][0],lines[i][1]};
            float b[2] = {lines[j][0],lines[j][1]};

            cv::Mat AA(1,2,CV_32FC1,a);
            cv::Mat BB(1,2,CV_32FC1,b);

            int count_1 = 0, count_2 = 0;

            for(int k = 0; k < probableCPs.size();k++){
                if(isColinear(lines[i],probableCPs[k])) count_1++;
                if(isColinear(lines[j],probableCPs[k])) count_2++;
            }


            if(AA.dot(BB) < 0.1 && count_1 == 4 && count_2 == 5 && find(already.begin(),already.end(),i) == already.end()){
                selected.push_back(lines[i]);
                selected.push_back(lines[j]);
                float vx = lines[i][0],vy = lines[i][1], x0 = lines[i][2],y0 = lines[i][3];
                float m = 80.0;
                line(frame, Point2f(x0-m*vx, y0-m*vy), Point2f(x0+m*vx, y0+m*vy),Scalar(0,255,250));
                vx = lines[j][0],vy = lines[j][1], x0 = lines[j][2],y0 = lines[j][3];
                line(frame, Point2f(x0-m*vx, y0-m*vy), Point2f(x0+m*vx, y0+m*vy),Scalar(0,255,250));
                already.push_back(i);

            }
        }
    }
    /*
    for(int i = 0; i < selected.size(); i++){
        cout << selected[i] << endl; 
    }
    */

    std::vector<Vec4f> selected_positive;
    std::vector<Vec4f> selected_negative;
    for(int i = 0; i < selected.size();i++){
        float m = selected[i][1] / selected[i][0];
        //cout << m << endl;
        if(m > 0) selected_positive.push_back(selected[i]);
        else selected_negative.push_back(selected[i]);
    }
    std::vector<float> x_negative;
    for(int i = 0; i < selected_negative.size();i++){
        float t = -selected_negative[i][3] / selected_negative[i][1];
        float x = selected_negative[i][2] + t * selected_negative[i][0];
        x_negative.push_back(x);
    }

    std::vector<float> x_positive;
    for(int i = 0; i < selected_positive.size();i++){
        float t = -selected_positive[i][3] / selected_positive[i][1];
        float x = selected_positive[i][2] + t * selected_positive[i][0];
        x_positive.push_back(x);
    }

    //Hallamos las cuatro lineas horizontales
    std::vector<int> alr;
    for(int y= 0 ; y < 4; y++){
        //Bucle -->
        //Creamos un vector con los indices ordenados
        int max_index; float max = -1000000.0;
        for(int i = 0; i < selected_positive.size();i++){
            //cout << x_positive[i] << " ";
            if(max < x_positive[i] && find(alr.begin(), alr.end(),i) == alr.end()){
                max = x_positive[i];
                max_index = i;
            }
        }
        alr.push_back(max_index);
        //cout << "MAXINDEX: " << max_index;

        //Get Colinear Points with this line
        std::vector<Point2f> row;
        for(int i = 0; i < probableCPs.size();i++){
            if(isColinear(selected_positive[max_index],probableCPs[i]))
                row.push_back(probableCPs[i]);
        }
        cout << row.size()<<endl;

        sort(row.begin(), row.end(),cmpx);
        for(int i = 0; i < row.size();i++){
            //circle(frame,row[i],5,Scalar(255,0,255),3,8);
            ControlPoints.push_back(row[i]);
        }

    }

    //Calculando el valor de x cuando y es igual a 0
    return true;
}// fin de funcion importante

//=============================================================
//================ FUNCION SUPERIMPORTANTE ====================
//=============================================================

void EllipsisDetection(){
    vector<vector<Point>> contours;
    vector<Vec4i> hierachy;

    //Find Contours
    findContours(detectedEdges,contours,hierachy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,Point(0,0));

    //Find rotated rectangles and ellipsis
    //vector<RotatedRect>minRect(contours.size());
    vector<RotatedRect>minEllipse(contours.size());


    for( int i = 0; i < contours.size(); i++ ){
        //minRect[i] = minAreaRect( Mat(contours[i]) );
        if( contours[i].size() > 4 ){
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
        float dif = w - h;

        //Imprimiendo las jerarquias
        // Posicion: Next | Previous | First_Child | Parent
        //cout << i << ": ";
        //for(int k = 0; k < 4; k++)
        //    cout << hierachy[i][k] << ",";
        //cout << endl;

        /**
        selected.push_back(minEllipse[i]);
                    //putText(Ellipsis,to_string(i),Point(c_x,c_y),FONT_HERSHEY_SIMPLEX,0.4,Scalar(255,0,0),1,CV_AA);
                    //cout << selected.size() << endl;
                    //cout <<"width: "<< w <<" height: "<< h << endl;
                    //cout <<"dif: "<< abs(dif) <<" Semiperimetro: "<< sum << " area: " << area<< endl; 
        ellipse( Ellipsis, minEllipse[i], Scalar(0,0,255), 1, 8 );
        **/
        
        if(abs(dif) < 20){
            if(hierachy[i][2] != -1){ // Si el Contour tiene Hijo que hijo sea unico
                int child_index = hierachy[i][2];
                if(hierachy[child_index][0] == -1 && hierachy[child_index][1] == -1 && hierachy[child_index][2] == -1){
                    selected.push_back(minEllipse[i]);
                    selected.push_back(minEllipse[child_index]);
                    //putText(Ellipsis,to_string(i),Point(c_x,c_y),FONT_HERSHEY_SIMPLEX,0.4,Scalar(255,0,0),1,CV_AA);
                    //cout << selected.size() << endl;
                    //cout <<"width: "<< w <<" height: "<< h << endl;
                    //cout <<"dif: "<< abs(dif) <<" Semiperimetro: "<< sum << " area: " << area<< endl; 
                    ellipse( Ellipsis, minEllipse[i], Scalar(0,0,255), 1, 8 );
                    ellipse( Ellipsis, minEllipse[child_index], Scalar(0,0,255), 1, 8 );          
                }
            }
        }
        
        
    }


    //Extraemos los centros de todas las elipses Seleccionadas
    cout << "Number Selected Ellipsises: " << selected.size() << endl;
    vector<Point2f> centers;
    for( int i = 0; i < selected.size(); i++ ){
        centers.push_back(selected[i].center);      
    }

    //Eliminamos Duplicados y nos quedamos con el promedio de Centros similares('permanezcan a un conjunto de elipses')
    vector<Point2f> CPs = getControlPoints(centers);
    cout << "Number of Control Points: "<< CPs.size() <<endl;

    for(int i = 0; i < CPs.size();i++){
        circle(Ellipsis,CPs[i],1,Scalar(0,0,255),3,8);
    }


    //Secuencia de Tracking
    if(isTracking){
        std::vector<float> distances;
        for(int k = 0; k < numTrackedItems;k++){
            Point2f tmp = trackedPoints[k];
            float min = 100000.0f;
            int index = 0;
            for(int i = 0; i < CPs.size(); i++){
                if( min > dist(trackedPoints[k],CPs[i]) ){
                    min = dist(trackedPoints[k],CPs[i]);
                    index = i;
                }
            }
            distances.push_back(dist(trackedPoints[k],CPs[index]));
            trackedPoints[k] = CPs[index]; // Actualizamos la posicion de los puntos
        }
        bool isCorrect = true;
        float dmean = 0.0;
        float dstddev = 0.0;

        // Mean standard algorithm
        for (int i = 0; i < numTrackedItems; ++i)
        {
            dmean += distances[i];
        }
        dmean /= (float)numTrackedItems;

        // Standard deviation standard algorithm
        std::vector<float> var(numTrackedItems);

        for (int i = 0; i < numTrackedItems; ++i){
            var[i] = (dmean - distances[i]) * (dmean - distances[i]);
        }

        for (int i = 0; i < numTrackedItems; ++i){
            dstddev += var[i];
        }
        dstddev = sqrt(dstddev / (float)numTrackedItems);
        std::cout << "Mean: " << dmean << "   StdDev: " << dstddev << std::endl;

        //Aumentar validaciones en esta zona
        if(dstddev >5.0f)
            isCorrect = false;

        //Si no es correcto el mandar señal para tratar de capturar el tracking
        if(!isCorrect){
            cout << "Couldnt keep tracking\n";
            isTracking = false;
        }

    }
    else{
        cout << "Start Tracking\n";
        // Buscamos encontrar el patron, devolvemos solo el numero correspondiente de nodos
        // Ademas Ordenamos los nodos, primero por fila, luego por columna
        patternWasFound = FindRingPattern(CPs,4,5);
        //  patternWasFound = false;



        //Esta parte del codigo debe enviar 20 puntos Ordenados y en grilla hacia TrackedPoints
        //En cualquier otro caso debe pasar al siguiente frame y tratar otra vez
        //El ordenamiento a pasar es el siguiente
        /**
        0 1 2 3 4
        5 6 7 8 9
        10 11 12 13 14
        15 16 17 18 19
        **/
        if(patternWasFound){
            trackedPoints.clear();
            for(int i = 0; i < numTrackedItems; i++){
                trackedPoints.push_back(ControlPoints[i]);
            }

            isTracking = true;
        }
    }

    //Finalmente Dibuja el vector preordenado y verificado
    if(isTracking){

        //for(int i = 0; i < numTrackedItems; i++)
        //    circle(frame, trackedPoints[i],1,Scalar(0,0,255),3,8);
        drawChessboardCorners(frame, Size(5,4), trackedPoints, patternWasFound);
    }


    /**

    // Calculating the mediana
    vector<Point2f> c1 = CPs;
    vector<Point2f> c2 = CPs;
    sort(c1.begin(),c1.end(),cmpx);
    sort(c2.begin(),c2.end(),cmpy);

    int n = c1.size()/2;
    float xm = c1[ n ].x;
    float ym = c2[ n ].y; 

    int r;
    bool patternWasFound = true;

    // Esta parte debe ayudar a validar nuestros PC
    // Como maximo deberiamos tener 20
    // Ademas debe ordenarlos de la sgte manera fila por fila, columna por columna
    // Debe retornar true si se cumplen los requisitos
    for(r = 1; r < 200; r++){
        int count = 0;
        for(int i = 0; i < CPs.size(); i++){
            if(dist(Point2f(xm,ym),CPs[i]) < r)
                count++;
        }
        //================================================
        // Hace una verificacion fuerte de 20 elementos!!!!
        //================================================
        if(count >= 20){
            for(int i = 0; i < CPs.size(); i++)
                if(dist(Point2f(xm,ym),CPs[i]) < r)
                    FinalControlPoints.push_back(CPs[i]);
            break;
        } 
            
    }

    circle( Ellipsis, Point2f(xm,ym),r + 15,Scalar( 255,0,0 ), 1, 8 );
    for( int i = 0; i< FinalControlPoints.size(); i++ ){
        circle( frame, FinalControlPoints[i] ,2,Scalar( 255,0,0 ), 2, 8 );
    }
    cout << "FinalControlPoints: "<< FinalControlPoints.size() <<endl;
    **/

}



int main(){

    cv::VideoCapture cap(video_path);
    //cv::VideoCapture cap(0); // --> For video Capture

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
        
        //cap >> frame; // --> For video Capture

        cout << "===========================\n";
        cout << "Frame No " << r <<endl;
        cout << "===========================\n";

        if(frame.empty()) break;
        dst.create( frame.size(), frame.type() );

        PreFilters();

        CannyThreshold(0,0);//---> Solo funciona como ByPass, no se usa realmente

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
            if(r < 0) r = 0;
            break;
        case 'd':
            r++;
            break;
        case 'a':
            r--;
            if(r < 0) r = 0;
            break;
        case 27:
            finish = 0;
            break;
        }
        

        //For Video Capture
        //int key = waitKey(10);
        //if(key == 27)
        //    break;

    }

    cap.release();
    destroyAllWindows();

}

