#include"MyPoint.h"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "cv.h"
#include "highgui.h"
#include"Car.h"
#include<iostream>
#include<math.h>

using namespace std;


const double pi =3.1415926535898;



storePoint::storePoint()
{
	size = 0;
}

void storePoint::add(CvPoint s, CvPoint e,int index)
{
	point[index].start = s;
	point[index].end = e;
	if (s.x*s.x + s.y*s.y < e.x*e.x + e.y*e.y)
	{
		point[index].minP = s;
	}
	else
	{
		point[index].minP = e;
	}
	size = index + 1;
	point[index].angle = getAngle(s, e);
}

int storePoint::distance(int index)
{
	return pow(point[index].minP.x, 2) + pow(point[index].minP.y, 2);
}

void storePoint::sort()
{
	for (int i = size-2; i >= 0; i--)
	{
		for (int j = i; j < size-1; j++)
		{
			if (distance(j)>distance(j + 1))
			{
				swap(j, j + 1);
			}
			else
			{
				break;
			}
		}
	}
}

void storePoint::swap(int index1, int index2)
{
	CvPoint p1, p2,p3;
	double tmpA;
	p1 = point[index1].start;
	p2 = point[index1].end;
	p3 = point[index1].minP;
	tmpA = point[index1].angle;

	point[index1].start = point[index2].start;
	point[index1].end = point[index2].end;
	point[index1].minP = point[index2].minP;
	point[index1].angle = point[index2].angle;

	point[index2].start = p1;
	point[index2].end = p2;
	point[index2].minP = p3;
	point[index2].angle = tmpA;

}

double storePoint::getAngle(CvPoint p1, CvPoint p2)
{
	double angle;
	int x1 = p1.x;
	int x2 = p2.x;
	int y1 = p1.y;
	int y2 = p2.y;
	if (x2 == x1)
	{
		angle = pi/2;
	}
	else
	{
		angle = atan(abs(y2 - y1) / abs(x2 - x1));
	}
	return angle;
}

void storePoint::deleteOverLap()
{
	double len1, len2;
	double threshAngle = 0.1;
	double threshLen = 10;
	int start = 0;
	
	int reflect[1000];       //重复点对应下标
	int lastInde = -1;

	int i = 0;
	int firstIndex = 0;
	while (i < size - 1)
	{
		if (sqrt((point[firstIndex].minP.x - point[i].minP.x)*(point[firstIndex].minP.x - point[i].minP.x) + (point[firstIndex].minP.y - point[i].minP.y)*(point[firstIndex].minP.y - point[i].minP.y) < threshLen))
		{
			len2 = ((point[i].start.x - point[i].end.x)*(point[i].start.x - point[i].end.x) + (point[i].start.y - point[i].end.y)*(point[i].start.y - point[i].end.y));
			len1 = ((point[firstIndex].start.x - point[firstIndex].end.x)*(point[firstIndex].start.x - point[firstIndex].end.x) + (point[firstIndex].start.y - point[firstIndex].end.y)*(point[firstIndex].start.y - point[firstIndex].end.y));
			if (abs(point[i].angle - point[firstIndex].angle) < threshAngle)
			{
				if (len1 < len2)
				{
					if (point[i].start.x == point[i].minP.x)
					{
						if (point[firstIndex].start.x == point[firstIndex].minP.x)
						{
							point[firstIndex].end.x = point[i].end.x;
							point[firstIndex].end.y = point[i].end.y;
						}
						else
						{
							point[firstIndex].start.x = point[i].end.x;
							point[firstIndex].start.y = point[i].end.y;
						}
					}
					else
					{
						if (point[firstIndex].start.x == point[firstIndex].minP.x)
						{
							point[firstIndex].end.x = point[i].start.x;
							point[firstIndex].end.y = point[i].start.y;
						}
						else
						{
							point[firstIndex].start.x = point[i].start.x;
							point[firstIndex].start.y = point[i].start.y;
						}
					}
				}
			}
		}
		else
		{
			reflect[++lastInde] = firstIndex;
			firstIndex = i;
		}
		i++;
	}

	size = lastInde + 1;
	for (int i = 0; i <= lastInde; i++)
	{
		swap(i, reflect[i]);
	}


}


void storePoint::initial()
{
	sort();
	deleteOverLap();
	smooth();

	for (int i = 0; i < size; i++)
	{
		point[i].angle = getAngle(point[i].start, point[i].end);
	}
}


void storePoint::test()
{
	for (int i = 0; i < size; i++)
	{
		cout << point[i].start.x<<" "<<point[i].start.y<< " " <<point[i].end.x<<" "<< point[i].end.y << endl;
	}
}

double storePoint::distance(CvPoint p1, CvPoint p2)
{
	return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
}

int storePoint::distance(CvPoint p)
{
	return p.x*p.x + p.y*p.y;
}

void storePoint::smooth()
{
	cout << "Before: ";
	test();
	cout << endl;


	store s[500];
	int lastIndex = -1;
	int startIndex = 0;
	bool isFind = true;
	bool isStart = false;
	                 
	for (int i = 0; i < size-1; i++)             //找起点
	{
		isFind = true;
		isStart = false;
		for (int j = 0; j < size; j++)
		{
			if (i == j)
			{
				continue;
			}
			else
			{
				if (distance(point[j].start, point[i].start) < 10)
				{
					isFind = false;
					break;
				}
				if (distance(point[j].end, point[i].start) < 10)
				{
					isFind = false;
					break;
				}
			}
		}
		if (isFind == true)
		{
			startIndex = i;
			isStart = true;
			break;
		}

		isFind = true;
		for (int j = 0; j < size; j++)
		{
			if (i == j)
			{
				continue;
			}
			else
			{
				if (distance(point[j].start, point[i].end) < 10)
				{
					isFind = false;
					break;
				}
				if (distance(point[j].end, point[i].end) < 10)
				{
					isFind = false;
					break;
				}
			}
		}
		if (isFind == true)
		{
			startIndex = i;
			isStart = false;
			break;
		}
	}


	//重新排序
	if (isStart == true)
	{
		s[0].start = point[startIndex].start;
		s[0].end = point[startIndex].end;
	}
	else
	{
		s[0].start = point[startIndex].end;
		s[0].end = point[startIndex].start;
	}
	
	int first = 0;

	int minL = 10000;

	int FIndex=0;
	bool isS;

	for (int i = 0; i <size-1; i++)
	{
		minL = 10000;
		for (int j = 0; j < size; j++)
		{
			if (point[j].start.x == s[first].end.x && point[j].start.y == s[first].end.y)
			{
				continue;
			}
			if (point[j].end.x == s[first].end.x && point[j].end.y == s[first].end.y)
			{
				continue;
			}

			if (distance(point[j].start, s[first].end) < minL)
			{
				minL = distance(point[j].start, s[first].end);
				FIndex = j;
				isS = true;
			}
			if (distance(point[j].end, s[first].end) < minL)
			{
				minL = distance(point[j].end, s[first].end);
				FIndex = j;
				isS = false;
			}

			
		}
		if (isS)
		{
			++first;
			s[first].start = s[first - 1].end;
			s[first].end = point[FIndex].start;
			++first;
			s[first].start = point[FIndex].start;
			s[first].end = point[FIndex].end;
		}
		else
		{
			CvPoint tmps, tmpe;
			tmps = point[FIndex].start;
			tmpe = point[FIndex].end;
			point[FIndex].start = tmpe;
			point[FIndex].end = tmps;

			++first;
			s[first].start = s[first - 1].end;
			s[first].end = point[FIndex].start;
			s[++first].start = point[FIndex].start;
			s[first].end = point[FIndex].end;

		}
	}


	for (int i = 0; i <= first; i++)
	{
		point[i].start = s[i].start;
		point[i].end = s[i].end;
		point[i].angle = getAngle(point[i].start, point[i].end);
		if (distance(point[i].start) < distance(point[i].end))
		{
			point[i].minP = point[i].start;
		}
		else
		{
			point[i].minP = point[i].end;
		}
	}
	size = first + 1;

	//cout<<"s: " << s[1].start.x << " " << s[1].end.x << endl;

	/*int tmps[1000];
	int firstIndex = 0;
	int i = 1;
	int tmpx,tmpy;
	for (; i < size; i++)
	{
		if (sqrt(pow((point[i].start.x - point[firstIndex].end.x), 2) + pow((point[i].start.y - point[firstIndex].end.y), 2)) < 10)
		{
			if (abs(point[firstIndex].angle - point[i].angle) < 0.1)
			{
				point[firstIndex].end = point[i].end;
			}
			else
			{
				tmpx = (point[firstIndex].end.x + point[i].start.x) / 2;
				tmpy = (point[firstIndex].end.y + point[i].start.y) / 2;
				point[firstIndex].end.x = tmpx;
				point[firstIndex].end.y = tmpy;
				point[i].start.x = tmpx;
				point[i].end.y = tmpy;
				tmps[++firstIndex] = i;
			}
		}
	}*/


	cout << "After: ";
	test();
	cout << endl;

}

void storePoint::reverse()
{
	store tmps[1000];
	for (int i = 0; i < size; i++)
	{
		tmps[i].start = point[i].start;
		tmps[i].end = point[i].end;
	}

	for (int i = 0; i < size; i++)
	{
		point[i].start = tmps[size - i-1].end;
		point[i].end = tmps[size - i - 1].start;
		point[i].angle = getAngle(point[i].start, point[i].end);
	}

}