#ifndef GRID_H
#define GRID_H

#include "includes.h"

struct Box{
	int PointCount;

	cv::Point2f min;
	cv::Point2f max;

	Box(cv::Point2f _min, cv::Point2f _max){
		min = _min; max = _max;
		PointCount = 0;
	}

	bool isIn(const cv::Point2f & point){
		if( point.x >= min.x && point.x < max.x &&
			point.y >= min.y && point.y < max.y )
			return true;
		return false;
	}
}


class Grid
{
	cv::Size imgSize;
	std::vector<Box> voBoxes;
public:
	Grid(cv::Size size){
		imgSize = cv::Size(640,480);
		float dx = imgSize.width / size.width;
		float dy = imgSize.height / size.height;

		for(int i = 0; i < size.width;i++)
			for(int j = 0; j < size.height; j++){
				Box b(cv::Point2f(i*dx,j*dy),cv::Point2f(i*dx+dx,j*dy+dy));
				voBoxes.push_back(b);
			}
	}

	//Aumenta el PointCount para cada Box segun un vector de puntos dados
	void FillBoxes(std::vector<cv::Point2f> v){
		FOR(i,v.size())
			FOR(j,voBoxes.size())
				if(voBoxes[j].isIn(v[i]))
					voBoxes[j].PointCount++;
	}



	~Grid();
	
};



#endif // GRID_H