
/**FUNCIONES ADICIONALES PARA EL PROGRAMA**/

#include "includes.h"

bool findRingGridPattern(cv::Mat Input, cv::Size size, std::vector<cv::Point2f>& points){
    for(int i = 0 ; i < size.height; i++)
        for(int j = 0; j < size.width;j++)
            points.push_back(cv::Point2f(i,j));
    return true;
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