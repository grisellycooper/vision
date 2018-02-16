
/**FUNCIONES ADICIONALES PARA EL PROGRAMA**/

#include "includes.h"

bool cmpx(Point2f a,Point2f b){
    return a.x < b.x;
}

bool cmpy(Point2f a, Point2f b){
    return a.y < b.y;
}

float dist(Point2f a, Point2f b){
    return sqrt( pow(a.x-b.x,2.0f)+pow(a.y-b.y,2.0f) );
}

void combinationUtil(std::vector< std::vector<int> >& v, int arr[], std::vector<int> &data, int start, int end, 
                     int index, int r);
 
// The main function that prints all combinations of size r
// in arr[] of size n. This function mainly uses combinationUtil()
void printCombination(std::vector< std::vector<int> >& v, int arr[], int n, int r)
{
    // A temporary array to store all combination one by one
    std::vector<int> data(r);
 
    // Print all combination using temprary array 'data[]'
    combinationUtil(v, arr, data, 0, n-1, 0, r);
}
 
/* arr[]  ---> Input Array
   data[] ---> Temporary array to store current combination
   start & end ---> Staring and Ending indexes in arr[]
   index  ---> Current index in data[]
   r ---> Size of a combination to be printed */
void combinationUtil(std::vector< std::vector<int> >& v, int arr[], std::vector<int> &data, int start, int end,
                     int index, int r)
{
    // Current combination is ready to be printed, print it
    if (index == r)
    {
        v.push_back(data);
        return;
    }
 
    // replace index with all possible elements. The condition
    // "end-i+1 >= r-index" makes sure that including one element
    // at index will make a combination with remaining elements
    // at remaining positions
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