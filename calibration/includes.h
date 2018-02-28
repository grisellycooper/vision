#ifndef INCLUDES_H
#define INCLUDES_H value

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <chrono>

#include <iostream>
#include <vector>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

//RING GRID FUNCTION
bool findRingGridPattern(cv::Mat Input, cv::Size size, std::vector<cv::Point2f>& points);

// Funciones adicionales para el Ring Grid Function


// Otras Funciones
bool cmpx(Point2f a,Point2f b);
bool cmpy(Point2f a, Point2f b);
float dist(Point2f a, Point2f b);

float StandarDesviation(const std::vector<float> & values );


#endif // INCLUDES_H