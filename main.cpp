/**
g++ -std=c++11 main.cpp addFunctions.cpp `pkg-config opencv --cflags --libs` -o main && ./main
**/

#include "includes.h"

//#define VIDEO

//Handles for Frames
cv::Mat frame; //Matriz fuente
cv::Mat gray;
cv::Mat Ellipsis;

//*****Control Points Vector********//
vector<Point2f> CPs;
std::vector<Point2f> trackedPoints;
std::vector<Point2f> ControlPoints;

int numTrackedItems = 20;
bool isTracking = false;
unsigned int num_Frames;
unsigned int num_TrackedFrames;

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

    //Mat kernel = getStructuringElement(MORPH_RECT,Size(3,3));
    //erode(gray,gray,kernel);
    //dilate(gray,gray,kernel);
    //dilate(gray,gray,kernel);
    //erode(gray,gray,kernel);
    //morphologyEx(gray,gray,MORPH_CLOSE,kernel);

    //medianBlur(gray,gray,3);
    //GaussianBlur(gray,gray,Size(3,3),0);
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

    if(abs(tx-ty) < 5.0f) return true;
    return false;
}

bool FindRingPattern(vector<Point2f> &probableCPs,int num_rows,int num_cols){
    int n = probableCPs.size();
    std::vector<Vec4f> lines;
    // Generamos todas las Combinaciones de "Lineas" en grupos de 5 posibles
    // Es 5 xq seleccionamos las lineas
    std::vector< std::vector<int> > combinations = GenerateCombinations(probableCPs.size(),num_cols);

    //Aprovechamos las lineas temporales y Selecionamos las que tengan Baja Desviacion
    std::vector<Vec4f> preSelectedLines;
    std::vector<std::vector<int> > combination_preSelectedLines;
    for(int i = 0; i < combinations.size();i++){
        std::vector<Point2f> tmpPoints(num_cols);
        Vec4f tmpLine;
        for(int j = 0; j < num_cols; j++){
            tmpPoints[j] = probableCPs[ combinations[i][j] ];
        }
        fitLine(tmpPoints,tmpLine,CV_DIST_L2,0,0.01,0.01);
        // Extraction of Features
        //Le damos forma a los valores vectoriales que nos devuelve fitline
        // r = a + r*b --> p0 punto de paso, v vector director normalizado
        float vx = tmpLine[0],vy = tmpLine[1], x0 = tmpLine[2],y0 = tmpLine[3];
        Point2f a = Point2f(x0,y0), b = Point2f(vx,vy);

        float m = 80.0;

        std::vector<float> distances;
        for(int k = 0; k < num_cols; k++){
            //Calculamos la distancia del punto a la recta y almacenamos para el calculo de la desviacion
            float t = ( tmpPoints[k].dot(b) - a.dot(b) ) / (cv::norm(b) * cv::norm(b));
            float dist = cv::norm(tmpPoints[k] - (a + t * b));
            distances.push_back(dist);
        }

        float stddev = StandarDesviation(distances);

        //Si el error de la linea no es mucho. Seleccionamos la linea
        if(stddev < 0.5f){
            preSelectedLines.push_back(tmpLine);
            line(frame, Point2f(x0-m*vx, y0-m*vy), Point2f(x0+m*vx, y0+m*vy),Scalar(0,255,0));
            //Guardamos la Combinacion
            combination_preSelectedLines.push_back(combinations[i]);
        }

    }

    // Apply some filters here to verify line selection
    // Then Order Points and Store in CPs(Hard verification of only 20 Ordered and Aligned Control Points)
    // Acordemonos que ya seleccionamos solo lineas con 5 puntos
    if(preSelectedLines.size() == 4){
        //Tenemos que ordenar las lineas. (Recordemos que son lineas paralelas)
        //Primero verificamos la pendiente

        //LINE ORDERING
            //Recordemos la grilla que presenta openCV 
            // -------> x+
            // |
            // |
            // y+

            Vec4f Line = preSelectedLines[0];
            float vx = Line[0],vy = Line[1], x0 = Line[2],y0 = Line[3];
            //Pendiente
            float slope = vy/vx;
            if(abs(slope) < 5.0f){ //Evaluamos las pendientes de casi 80 grados (Revisar esta funcion)
                std::vector<float> y_intersection(4);
                //Calcular el punto de interseccion por el eje y
                for(int i = 0; i < 4; i++){
                    Vec4f tmpLine = preSelectedLines[0];
                    float vx = tmpLine[0],vy = tmpLine[1], x0 = tmpLine[2],y0 = tmpLine[3];

                    float t = -x0 / vx;
                    float y = y0 + t*vy;

                    y_intersection[i] = y;
                }

                //Realizamos un bubble sort en base a las intersecciones con el eje y
                //ordenamiento por burbuja
                bool swapp = true;
                while(swapp)
                {
                    swapp = false;
                    for (int i = 0; i < preSelectedLines.size()-1; i++)
                    {
                        if (y_intersection[i] > y_intersection[i+1] ){
                            //Cambiamos en todos nuestros vectores
                            std::swap(y_intersection[i],y_intersection[i+1]);
                            std::swap(preSelectedLines[i],preSelectedLines[i+1]);
                            std::swap(combination_preSelectedLines[i],combination_preSelectedLines[i+1]);
                            swapp = true;
                        }
                    }
                }// Fin del ordenamiento

                // Para Cada Linea obtener los CP segun la combinacion y ordenarlos por el eje X
                // Obtenemos los puntos desde el CP


                std::vector<Point2f> tmpCPs;
                for(int i = 0; i < num_rows; i++){
                    std::vector<Point2f> tmpCenters(num_cols);
                    for(int j = 0; j < num_cols; j++){
                        tmpCenters[j] = probableCPs[ combination_preSelectedLines[i][j] ];
                    }
                    sort(tmpCenters.begin(), tmpCenters.end(),cmpx);
                    for(int j = 0; j < num_cols; j++){
                        tmpCPs.push_back(tmpCenters[j]);
                    }
                }

                probableCPs.clear();
                probableCPs = tmpCPs;



                return true;

            }

        
    }
    return false;


}// fin de funcion importante

//=============================================================
//================ ELIPSIS DETECTION ====================
//=============================================================

void EllipsisDetection(){
    vector<vector<Point>> contours;
    vector<Vec4i> hierachy;

    //Find Contours
    findContours(gray,contours,hierachy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,Point(0,0));

    //Find rotated rectangles and ellipsis
    //vector<RotatedRect>minRect(contours.size());
    vector<RotatedRect>minEllipse(contours.size());


    for( int i = 0; i < contours.size(); i++ ){
        //minRect[i] = minAreaRect( Mat(contours[i]) );
        if( contours[i].size() > 4 ){
            minEllipse[i] = fitEllipse( Mat(contours[i]) ); }
    }

    /// Draw contours + rotated rects + ellipses
    Ellipsis = gray;
    cvtColor(gray, Ellipsis,CV_GRAY2BGR);
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
        
        if(abs(dif) < 40){ //-->>> CAMBIAR ESTE PARAMETRO PARA FILTRAR LAS ELIPSES
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
    CPs = getControlPoints(centers);
    cout << "Number of Control Points(No duplicates): "<< CPs.size() <<endl;

    for(int i = 0; i < CPs.size();i++){
        circle(Ellipsis,CPs[i],1,Scalar(0,0,255),3,8);
        //circle(frame,CPs[i],5,Scalar(255,255,0),2,8);
    }

    //=============================
    // Calculating the mediana
    //=============================
    vector<Point2f> tmpCPs;
    vector<Point2f> c1 = CPs;
    vector<Point2f> c2 = CPs;
    sort(c1.begin(),c1.end(),cmpx);
    sort(c2.begin(),c2.end(),cmpy);

    int n = c1.size()/2;
    float xm = c1[ n ].x;
    float ym = c2[ n ].y; 

    // Esta parte debe ayudar a validar nuestros PC
    // Como maximo deberiamos tener 20
    // Ademas debe ordenarlos de la sgte manera fila por fila, columna por columna
    // Debe retornar true si se cumplen los requisitos
    int r;
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
            break;
        } 
            
    }

    //Displaying the range Circles
    int padding = 30;
    circle( Ellipsis, Point2f(xm,ym),r+padding,Scalar( 255,0,0 ), 1, 8 );
    for(int i = 0; i < CPs.size(); i++)
        if(dist(Point2f(xm,ym),CPs[i]) < r +padding)
            tmpCPs.push_back(CPs[i]);

    //Displaying filtered circles
    for( int i = 0; i< tmpCPs.size(); i++ ){
        circle( Ellipsis, tmpCPs[i] ,5,Scalar( 255,0,0 ), 2, 8 );
    }
    cout << "Filtered Control Points(Median): "<< tmpCPs.size() <<endl;

    CPs.clear();
    CPs = tmpCPs;
    /**
    for(int i = 0; i < CPs.size();i++){
        //circle(Ellipsis,CPs[i],1,Scalar(0,0,255),3,8);
        circle(frame,CPs[i],5,Scalar(255,255,0),2,8);
    }
    **/
    //=============================
    // Fin mediana
    //=============================

}// Fin EllipsisDetección

//Función para Trackear
void Track(){

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

        float dstddev = StandarDesviation(distances);

        //Aumentar validaciones en esta zona
        if(dstddev >1.5f){
            isCorrect = false;
        }
        num_TrackedFrames++;

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
        bool patternWasFound = FindRingPattern(CPs,4,5);
        //patternWasFound = false;

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
                trackedPoints.push_back(CPs[i]);
            }

            isTracking = true;
            num_TrackedFrames++;
        }
    }

    //Finalmente Dibuja el vector preordenado y verificado

    if(isTracking)
        drawChessboardCorners(frame,Size(5,4),trackedPoints,isTracking);
        //for(int i = 0; i < numTrackedItems; i++)
        //    circle(frame, trackedPoints[i],1,Scalar(0,0,255),3,8);
} // Fin de la funcion tracking

//=============================================================
//================ MAIN FUNCTION ====================
//=============================================================


int main(){

#ifndef VIDEO
    cv::VideoCapture cap(video_path);
#else
    cv::VideoCapture cap(0); // --> For video Capture
#endif

    if(!cap.isOpened()){
        cout << "Cannot open the video file" << endl;
        return -1;
    }

    double count = cap.get(CV_CAP_PROP_FRAME_COUNT); //get the frame count

    //Procesamos todos los frames

    namedWindow(WindowName,0);
    resizeWindow(WindowName,1000,1000);

    namedWindow(WindowRGB,0);
    resizeWindow(WindowRGB,800,600);



    int r = 0;
    bool finish = 1;

    
    double elapsed_seconds = 0.0;
    double time = 0.0;
    num_Frames = 1; // Para que empiece a trackear hasta el segundo conteo
    num_TrackedFrames = 1;

    while(finish)
    {
        auto start = std::chrono::system_clock::now();

        #ifndef VIDEO
        cap.set(1,r);
        cap.read(frame);
        #else
        cap >> frame; // --> For video Capture
        #endif

        cout << "===========================\n";
        cout << "Frame No " << r <<endl;
        cout << "===========================\n";

        if(frame.empty()) break;

        PreFilters();

        EllipsisDetection();

        Track();

        
        auto end = std::chrono::system_clock::now();
        elapsed_seconds += std::chrono::duration_cast<std::chrono::duration<double> >(end - start).count();

        if(num_Frames % 30 == 0){
            elapsed_seconds *= 1000.0;
            time = elapsed_seconds / 30.0;

            cout << "the time was: " << elapsed_seconds << endl;
            elapsed_seconds = 0.0;
        }
        cout << "Tracked Num Frames: " << num_TrackedFrames << " Total Frames: "<<num_Frames<<endl; 
        putText(frame,to_string((float) num_TrackedFrames / num_Frames * 100.0) + " %",Point(400,430),FONT_HERSHEY_SIMPLEX,1,Scalar(200,200,0),2,CV_AA);
        putText(frame,to_string(time) + " ms",Point(400,470),FONT_HERSHEY_SIMPLEX,1,Scalar(200,200,0),2,CV_AA);

        imshow(WindowName,Ellipsis);
        imshow(WindowRGB,frame);
        num_Frames++;
        
        #ifndef VIDEO
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
        #else    
        //For Video Capture
        int key = waitKey(10);
        if(key == 27)
            break;
        #endif
        
    }

    cap.release();
    destroyAllWindows();

}

