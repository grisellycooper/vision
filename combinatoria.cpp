// Program to print all combination of size r in an array of size n
#include <stdio.h>
#include <vector>
#include <iostream>

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
 
// Driver program to test above functions
int main()
{
    std::vector< std::vector<int> > v = GenerateCombinations(5,3);
    for(int i = 0; i < v.size();i++){
        for (int j=0; j<3; j++)
            printf("%d ", v[i][j]);
        printf("\n");
    }
}