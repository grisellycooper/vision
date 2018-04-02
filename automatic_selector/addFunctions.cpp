/**FUNCIONES ADICIONALES PARA EL PROGRAMA**/

#include "includes.h"

bool findRingsGridPattern(cv::Mat Input, cv::Size size, std::vector<cv::Point2f>& points, bool& isTracking, std::vector<cv::Point2f>& oldPoints){
    // ===================
    // PRE - FILTERS
    // ===================
    cv::Mat gray;
    cv::cvtColor(Input,gray,CV_BGR2GRAY);
    GaussianBlur(gray,gray,Size(3,3),0);
    adaptiveThreshold(gray,gray,255,ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY,41,6);

    //cv::imshow("g",gray);

    
    // ===================
    // ELLIPSIS DETECTION
    // ===================
    vector<vector<Point>> contours;
    vector<Vec4i> hierachy;
    findContours(gray.clone(),contours,hierachy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,Point(0,0));
    //cv::imshow("g",gray);

    vector<RotatedRect>minEllipse(contours.size());

    // Fitear una elipse a los contornos detectados
    for( int i = 0; i < contours.size(); i++ ){
        //minRect[i] = minAreaRect( Mat(contours[i]) );
        if( contours[i].size() > 4 ){
            minEllipse[i] = fitEllipse( Mat(contours[i]) ); }
    }

    //Filtrar las ellipses
    cv::cvtColor(gray,gray, CV_GRAY2BGR);

    vector<RotatedRect> selected;
    for( int i = 0; i< contours.size(); i++ ){
        float w = minEllipse[i].size.width;
        float h = minEllipse[i].size.height;
        float c_x = minEllipse[i].center.x;
        float c_y = minEllipse[i].center.y;
        float dif = w - h;
        
        if(abs(dif) < 40){ //-->>> CAMBIAR ESTE PARAMETRO PARA FILTRAR LAS ELIPSES
            if(hierachy[i][2] != -1){ // Si el Contour tiene Hijo que hijo sea unico
                int child_index = hierachy[i][2];
                if(hierachy[child_index][0] == -1 && hierachy[child_index][1] == -1 && hierachy[child_index][2] == -1){
                    selected.push_back(minEllipse[i]);
                    selected.push_back(minEllipse[child_index]);
                    ellipse( gray, minEllipse[i], Scalar(0,0,255), 1, 8 );
                    ellipse( gray, minEllipse[child_index], Scalar(0,0,255), 1, 8 );          
                }
            }
        }
    }

    //Como minimo debemos capturar 40 elipses para continuar
    cv::imshow(windowGray,gray);
    if(selected.size() < 40) return false;

    //cv::imshow("g",gray);

    //Extraemos los centros de todas las elipses Seleccionadas
    //cout << "Number Selected Ellipsises: " << selected.size() << endl;
    vector<Point2f> centers;
    for( int i = 0; i < selected.size(); i++ ){
        centers.push_back(selected[i].center);      
    }

    //Eliminamos Duplicados y nos quedamos con el promedio de Centros similares('permanezcan a un conjunto de elipses')
    vector<Point2f> CPs;
    CPs = getControlPoints(centers);
    for(int i = 0; i < CPs.size();i++)
        circle(gray,CPs[i],1,Scalar(0,0,255),3,8);

    cv::imshow(windowGray,gray);
    if(CPs.size() < 20) return false;


    //cv::imshow(windowGray,gray);

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
    for(int i = 0; i < CPs.size(); i++)
        if(dist(Point2f(xm,ym),CPs[i]) < r +padding)
            tmpCPs.push_back(CPs[i]);

    //cout << "Filtered Control Points(Median): "<< tmpCPs.size() <<endl;

    CPs.clear();
    CPs = tmpCPs;


    for(int i = 0; i < CPs.size();i++){
        circle(gray,CPs[i],5,Scalar(255,0,0),3,8);
    }
    
    cv::imshow(windowGray,gray);
    if(CPs.size() < 20) return false;

    

    // ===========================================
    // ORDERING AND SETTINGS POINTS ( Tracking )
    // ===========================================
    std::vector<Point2f> trackedPoints;
    int numTrackedItems = 20;

    
    if(isTracking){
        trackedPoints.resize(numTrackedItems);
        std::vector<float> distances;
        for(int k = 0; k < numTrackedItems;k++){
            Point2f tmp = oldPoints[k]; // Aqui esta el error
            float min = 100000.0f;
            int index = 0;
            for(int i = 0; i < CPs.size(); i++){
                if( min > dist(oldPoints[k],CPs[i]) ){
                    min = dist(oldPoints[k],CPs[i]);
                    index = i;
                }
            }
            distances.push_back(dist(oldPoints[k],CPs[index]));
            trackedPoints[k] = CPs[index]; // Actualizamos la posicion de los puntos
        }
        bool isCorrect = true;

        float dstddev = StandarDesviation(distances);

        //Aumentar validaciones en esta zona
        if(dstddev > 3.0f)
            isCorrect = false;

        //Revisar que np haya duplicados
        for(int i = 0; i < trackedPoints.size()-1;i++)
            for(int j = i+1; j < trackedPoints.size();j++)
                if(trackedPoints[i] == trackedPoints[j])
                    isCorrect = false;

        //Si no es correcto el mandar señal para tratar de capturar el tracking
        if(!isCorrect){
            cout << "Couldnt keep tracking\n";
            isTracking = false;
        }

    }
    
    //isTracking = false;
    //if(!isTracking){
    else{
        //cout << "Start Tracking\n";
        // Buscamos encontrar el patron, devolvemos solo el numero correspondiente de nodos
        // Ademas Ordenamos los nodos, primero por fila, luego por columna
        bool patternWasFound = FindRingPattern(CPs,4,5);
        //patternWasFound = false;

        //Esta parte del codigo debe enviar 20 puntos Ordenados y en grilla hacia TrackedPoints
        //En cualquier otro caso debe pasar al siguiente frame y tratar otra vez
        //El ordenamiento a pasar es el siguiente

        if(patternWasFound){
            trackedPoints.clear();
            for(int i = 0; i < numTrackedItems; i++){
                trackedPoints.push_back(CPs[i]);
                circle(gray,CPs[i],5,Scalar(255,0,0),3,8);
            }

            isTracking = true;
        }
    }

    cv::imshow(windowGray,gray);

    // Copiamos el vector a points que seran nuestros CPs
    points = trackedPoints;

    return isTracking;
}

void calcBoardCornerPositions(cv::Size size, float squareSize, std::vector<cv::Point3f> &corners, int patternType){
    corners.clear();

    switch(patternType){
        case CHESSBOARD:
        case CIRCLES_GRID:
        case RINGS_GRID:
            for(int i = 0 ; i < size.height; i++)
                for(int j = 0; j < size.width; j++)
                    corners.push_back(cv::Point3f( float(j * squareSize),float(i*squareSize),0) );
            break;
        case ASYMMETRIC_CIRCLES_GRID:
            for( int i = 0; i < size.height; i++ )
                for( int j = 0; j < size.width; j++ )
                    corners.push_back(Point3f(float((2*j + i % 2)*squareSize), float(i*squareSize), 0));
            break;
    }
}

double computeReprojectionErrors(const std::vector< std::vector<cv::Point3f> >& objectPoints,
                                    const std::vector< std::vector<cv::Point2f> >& imagePoints,
                                    const std::vector<cv::Mat>& rvecs,const std::vector<cv::Mat>& tvecs, 
                                    const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
                                    std::vector<float> & perFrameErrors){

    std::vector<cv::Point2f> imagePoints2;
    size_t totalPoints = 0;
    double totalErr = 0, err;
    perFrameErrors.resize(objectPoints.size());

    for(size_t i = 0; i < objectPoints.size(); ++i ){

        cv::projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);

        err = norm(imagePoints[i], imagePoints2, NORM_L2);

        size_t n = objectPoints[i].size();
        perFrameErrors[i] = (float) std::sqrt(err*err/n);
        totalErr        += err*err;
        totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);

}

// RING PATTERN DETECTION ADDITIONAL FUNCTIONS
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


// OTHER FUNCTIONS

bool cmpx(Point2f a,Point2f b){
    return a.x < b.x;
}

bool cmpy(Point2f a, Point2f b){
    return a.y < b.y;
}

float dist(Point2f a, Point2f b){
    return sqrt( pow(a.x-b.x,2.0f)+pow(a.y-b.y,2.0f) );
}

float StandarDesviation(const std::vector<float> & values ){
	int n = values.size();
    float dmean = 0.0;
    float dstddev = 0.0;

    // Mean standard algorithm
    for (int i = 0; i < n; ++i)
    {
       dmean += values[i];
    }
    dmean /= (float)n;

    // Standard deviation standard algorithm
    std::vector<float> var(n);

    for (int i = 0; i < n; ++i){
        var[i] = (dmean - values[i]) * (dmean - values[i]);
    }

    for (int i = 0; i < n; ++i){
        dstddev += var[i];
    }
    dstddev = sqrt(dstddev / (float)n);
    //std::cout << "Mean: " << dmean << "   StdDev: " << dstddev << std::endl;

    return dstddev;
}

void printCombination(std::vector< std::vector<int> >& v, int arr[], int n, int r)
{
    std::vector<int> data(r);

    combinationUtil(v, arr, data, 0, n-1, 0, r);
}
 
void combinationUtil(std::vector< std::vector<int> >& v, int arr[], std::vector<int> &data, int start, int end,
                     int index, int r)
{
    if (index == r)
    {
        v.push_back(data);
        return;
    }
 
    for (int i=start; i<=end && end-i+1 >= r-index; i++)
    {
        data[index] = arr[i];
        combinationUtil(v, arr, data, i+1, end, index+1, r);
    }
}

std::vector< std::vector<int> > GenerateCombinations(int n, int r){
    std::vector< std::vector<int> > v;

    int arr[n];
    for(int i = 0; i < n; i++)
        arr[i] = i;

    printCombination(v, arr, n, r);

    return v;
}


//Calculating Colinearity
//Esta funcion se usa para calcular la colinealidad
float getAvgColinearityFromVector(const std::vector<cv::Point2f>& PointBuffer, cv::Size size){
    std::vector<float> v;
    //First we fit a line with the size.width first points
    for(int i = 0; i < size.height * size.width;i+= size.width){
        std::vector<Point2f> tmpPoints(size.width);
        Vec4f tmpLine;
        for(int j = i,r=0; j < i+size.width; j++,r++){
            tmpPoints[r] = PointBuffer[j];
        }
        //PrintSTDVector(tmpPoints);
        fitLine(tmpPoints,tmpLine,CV_DIST_L2,0,0.01,0.01);
        // Extraction of Features
        //Le damos forma a los valores vectoriales que nos devuelve fitline
        // r = a + r*b --> p0 punto de paso, v vector director normalizado
        float vx = tmpLine[0],vy = tmpLine[1], x0 = tmpLine[2],y0 = tmpLine[3];
        Point2f a = Point2f(x0,y0), b = Point2f(vx,vy);

        std::vector<float> distances;
        for(int k = 0; k < size.width; k++){
            //Calculamos la distancia del punto a la recta y almacenamos para el calculo de la desviacion
            float t = ( tmpPoints[k].dot(b) - a.dot(b) ) / (cv::norm(b) * cv::norm(b));
            float dist = cv::norm(tmpPoints[k] - (a + t * b));
            distances.push_back(dist);
        }
        //PrintSTDVector(distances);

        //For each line Calculate their Standart Deviation with respect of points
        float avg = SimpleAverage(distances);
        v.push_back(avg);
    }

    //Return their Avg Standart Deviation
    double overallAverage = SimpleAverage(v);

    return overallAverage;
}

float SimpleAverage(const std::vector<float> & v){
    double sum = 0.0;
    for(int i = 0; i < v.size();i++)
        sum += v[i];
    //cout << "El Promedio es " << sum / v.size() << endl;
    return sum / v.size();
}




float printAvgColinearity(const std::vector<float>& v){
    double sum = 0.0;
    for(int i = 0; i < v.size();i++)
        sum += v[i];
    cout << "El Promedio es " << sum / v.size() << endl;
    return sum / v.size();
}

std::vector<cv::Point2f> extractCorners(std::vector<cv::Point2f>& v, cv::Size size){
    std::vector<cv::Point2f> corners;

    // tenemos que separar las 4 esquinas del patron
    corners.push_back(v[0]);

    corners.push_back(v[size.width - 1]);

    corners.push_back(v[v.size() - size.width]);

    corners.push_back(v[v.size()-1]);

    return corners;
}

std::vector<cv::Point2f> getFrontoParallelCorners(cv::Size imgSize, cv::Size patternSize){
    float tx = 40.0f, ty = 25.0f;
    float dim = 45.0f;
    
    std::vector<cv::Point2f> corners;

    corners.push_back(cv::Point2f(tx,ty + patternSize.height * dim));

    corners.push_back(cv::Point2f(tx + patternSize.width * dim, ty + patternSize.height * dim));

    corners.push_back(cv::Point2f(tx,ty));

    corners.push_back(cv::Point2f(tx + patternSize.width * dim,ty));

    return corners;

}

vector<Point2f>  distortion(vector<Point2f> cp,const cv::Mat& intrinsics,const cv::Mat& dist_coeff )
{
    float cx = intrinsics.at<double>(0,2), cy = intrinsics.at<double>(1,2), fx = intrinsics.at<double>(0,0), fy= intrinsics.at<double>(1,1);
    float k1 = dist_coeff.at<double>(0,0), k2 = dist_coeff.at<double>(1,0), p1 = dist_coeff.at<double>(2,0), p2 = dist_coeff.at<double>(3,0), k3 = dist_coeff.at<double>(4,0);
    vector<Point2f> corrected_points;


    for(int i = 0; i < cp.size(); i++ )
    {
        float x = (cp[i].x - cx)/fx;
        float y = (cp[i].y - cy)/fy;


        //cout << cp[i].x << "," << cp[i].y << endl;

        float r_2 = x*x + y*y;

        float x_distort = x*(1+k1*r_2+k2*r_2*r_2 + k3*r_2*r_2*r_2);
        float y_distort = y*(1+k1*r_2+k2*r_2*r_2 + k3*r_2*r_2*r_2);

        x_distort += ( 2*p1*y*x + p2*( r_2 + 2*x*x) );
        y_distort += ( p1*(r_2+2*y*y ) + 2*p2*x*y );

        x_distort = x_distort*fx + cx; 
        y_distort = y_distort*fy + cy; 

        corrected_points.push_back( cv::Point2f(x_distort,y_distort) );
    }

    //cout << cx << " " << cy << " " << fx << " " << fy << endl;
    //cout << k1 << " " << k2 << " " << p1 << " " << p2 << " " << k3 << endl;


    return corrected_points;
}

// Automatic image Detector Functions
std::vector<bool> calc_affection(const std::vector<cv::Point2f> & PointBuffer,const cv::Size & imgPixelSize, const cv::Size & GridSize ){
    int linearSize = GridSize.width * GridSize.height;

    float x_division = imgPixelSize.width / GridSize.width;
    float y_division = imgPixelSize.height / GridSize.height;


    std::vector<bool> v(linearSize,false);

    // Actualizamos todos las cuadriculas que contienen al menos 1 punto
    FOR(i,PointBuffer.size()){
        int t_x = PointBuffer[i].x / x_division;
        int t_y = PointBuffer[i].y / y_division;

        v[t_x + t_y * (int)GridSize.width] = true;
    }

    //PrintSTDVector(v);

    return v;
}

std::vector<int> calc_BestFrameCombination(const std::vector< std::vector<bool> > &voAffections, int noImages){
    std::vector<int> indices(voAffections.size());
    std::iota(indices.begin(), indices.end(), 0); // Fill 0,1,...

    std::vector<int> best_sol; // best combination solution
    int noIterations = 10000;
    int max_diff = 100000;

    FOR(i,noIterations){
        std::random_shuffle(indices.begin(), indices.end());

        std::vector<int> tmp(indices.begin(),indices.begin() + noImages);

        std::vector<int> acc(voAffections[0].size(),0);

        FOR(j,tmp.size())
            FOR(k,voAffections[ tmp[j] ].size()){
                acc[k] += voAffections[ tmp[j] ][k];
            }

        // PrintSTDVector(acc);

        //Check that solution dont have zeros on it
        bool test1 = true;
        FOR(j,acc.size())
            if(acc[j] == 0)
                test1 = false;

        if(!test1){
            i--;
            continue; // Ignore this iteration and get another one
        } 

        auto min_it = std::min_element(acc.begin(), acc.end());
        auto max_it = std::max_element(acc.begin(), acc.end());

        if(*max_it - *min_it < max_diff){
            max_diff = *max_it - *min_it;
            best_sol = tmp;
            PrintSTDVector(acc);
            cout << "Max: " << *max_it << " Min: " << *min_it << " Homogeneidad: " << max_diff << endl;
        }
        //cout << "Current Iteration: " << i << endl;

        //PrintSTDVector(acc);

    }

    PrintSTDVector(best_sol);


    return best_sol;


}
