#ifndef MYPOINT

#define MYPOINT

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "cv.h"
#include "highgui.h"
#include"Car.h"

class storePoint
{
private:
	
	void swap(int index1,int index2);
	double getAngle(CvPoint p1, CvPoint p2);
	void deleteOverLap();                  //ШЅжи
	void sort();
	void smooth();
	int distance(int index);
	int distance(CvPoint p);
	double distance(CvPoint p1,CvPoint p2);
	int startP;
	int endP;
public:
	struct store
	{
		CvPoint start;
		CvPoint end;
		CvPoint minP;
		double angle;
	}point[1000];
	int size;
	int getSize(){
		return size;
	}
	storePoint();
	void add(CvPoint s, CvPoint e,int index);
	void initial();
	void test();
	void reverse();
};



#endif