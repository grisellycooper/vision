#ifndef INCLUDES_H
#define INCLUDES_H value

#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <vector>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;


bool cmpx(Point2f a,Point2f b);
bool cmpy(Point2f a, Point2f b);
float dist(Point2f a, Point2f b);

//Funciones para Generar los Numeros Combinatorios

void combinationUtil(std::vector< std::vector<int> >& v, int arr[], std::vector<int> &data, int start, int end, int index, int r);
 
// The main function that prints all combinations of size r
// in arr[] of size n. This function mainly uses combinationUtil()
void printCombination(std::vector< std::vector<int> >& v, int arr[], int n, int r);

std::vector< std::vector<int> > GenerateCombinations(int n, int r);

//Funcion para calcular la desviacion estandar de un vector

float StandarDesviation(const std::vector<float> & values );


#endif // INCLUDES_H