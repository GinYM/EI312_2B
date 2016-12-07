#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "cv.h"
#include "highgui.h"
#include"Car.h"

#include <iostream>
#include <ctype.h>
#include<math.h>
#include <windows.h>
#include"MyPoint.h"
#include<opencv\highgui.h>
#include<opencv2\core\core.hpp>

const double pi = 3.1415926535898;

using namespace cv;
using namespace std;

int sx, sy;
int count0 = 0;
Point2f originpoints[4];




enum Turn{ Left, Right, Ahead };         //行进方向



Mat image,image2;
Mat warpMatrix;          //透视变形矩阵
Mat rotated;             //转换后的图像

RotatedRect trackBox1;//定义一个旋转的矩阵类对象
RotatedRect trackBox2;//定义一个旋转的矩阵类对象

bool backproj1Mode = false; //表示是否要进入反向投影模式，ture表示准备进入反向投影模式
bool selectObject = false;//代表是否在选要跟踪的初始目标，true表示正在用鼠标选择
int trackObject = 0; //代表跟踪目标数目
bool showhist1 = true;//是否显示直方图
Point origin;//用于保存鼠标选择第一次单击时点的位置
Rect selection1,selection2;//用于保存鼠标选择的矩形框
int vmin = 10, vmax = 256, smin = 30;
bool mark1 = false;    // mark1 = true 代表选择标志1，mark1 = false 代表没有执行完标志1的选取 
bool mark2 = false;    //mark2 标志2

int type = -1;         //-1 代表初始状态，0 代表进行透视变形 ， 1代表进行

int vis[10000000];

int startPoint = 0;

storePoint sp;


void initialPoint();

void mouse(int event, int x, int y, int flags, void *)
{
	if (type != 0)
	{
		return;
	}
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		if (count0 >= 4)
		{
			return;
		}
		originpoints[count0] = Point2f(x, y);
		count0++;
		sx = x;
		sy = y;
	}
	else
	{
		cvWaitKey(50);
	}

	return;
}

void onMouse(int event, int x, int y, int, void*)
{
	if (type != 1)
	{
		return;
	}
	if (mark1 == false)
	{
		if (selectObject)//只有当鼠标左键按下去时才有效，然后通过if里面代码就可以确定所选择的矩形区域selection11了
		{
			selection1.x = MIN(x, origin.x);//矩形左上角顶点坐标
			selection1.y = MIN(y, origin.y);
			selection1.width = std::abs(x - origin.x);//矩形宽
			selection1.height = std::abs(y - origin.y);//矩形高

			selection1 &= Rect(0, 0, image.cols, image.rows);//用于确保所选的矩形区域在图片范围内
		}

		switch (event)
		{
		case CV_EVENT_LBUTTONDOWN:
			origin = Point(x, y);
			selection1 = Rect(x, y, 0, 0);//鼠标刚按下去时初始化了一个矩形区域
			selectObject = true;
			break;
		case CV_EVENT_LBUTTONUP:
			selectObject = false;
			if (selection1.width > 0 && selection1.height > 0)
				trackObject = -1;
			break;
		}
	}
	else if ( mark1 == true && mark2 == false)
	{
		//cout << "he" << endl;
		if (selectObject)//只有当鼠标左键按下去时才有效，然后通过if里面代码就可以确定所选择的矩形区域selection2了
		{
			selection2.x = MIN(x, origin.x);//矩形左上角顶点坐标
			selection2.y = MIN(y, origin.y);
			selection2.width = std::abs(x - origin.x);//矩形宽
			selection2.height = std::abs(y - origin.y);//矩形高

			selection2 &= Rect(0, 0, image.cols, image.rows);//用于确保所选的矩形区域在图片范围内
		}

		switch (event)
		{
		case CV_EVENT_LBUTTONDOWN:

			//trackObject = 2;
			origin = Point(x, y);
			selection2 = Rect(x, y, 0, 0);//鼠标刚按下去时初始化了一个矩形区域
			selectObject = true;
			break;
		case CV_EVENT_LBUTTONUP:
			//cout << "he" << endl;
			if (selectObject == false)
			{
				break;
			}
			selectObject = false;
			if (selection2.width > 0 && selection2.height > 0)
				trackObject = 3;

			//cout << "he" << endl;
			break;
		}
	}
}

/*void help()
{
	cout << "\nThis is a demo that shows mean-shift based tracking\n"
		"You select a color objects such as your face and it tracks it.\n"
		"This reads from video camera (0 by default, or the camera number the user enters\n"
		"Usage: \n"
		"    ./camshiftdemo [camera number]\n";

	cout << "\n\nHot keys: \n"
		"\tESC - quit the program\n"
		"\tc - stop the tracking\n"
		"\tb - switch to/from backproj1ection view\n"
		"\th - show/hide object hist1ogram\n"
		"\tp - pause video\n"
		"To initialize tracking, select the object with mouse\n";
}*/


void change(Mat frame)
{
	setMouseCallback("CamShift Demo", mouse, 0);

	cvWaitKey(0);
	Point2f dst_points[4];
	dst_points[0] = Point(0, 0);
	dst_points[1] = Point(frame.cols, 0);
	dst_points[2] = Point(0, frame.rows);
	dst_points[3] = Point(frame.cols, frame.rows);
	//cout << dst_points[3].x << endl;
	warpMatrix = getPerspectiveTransform(originpoints, dst_points);
	//Mat rotated;
	warpPerspective(frame, rotated, warpMatrix, rotated.size(), INTER_LINEAR, BORDER_CONSTANT);

	//namedWindow("warp perspective", 0);
	//imshow("warp perspective", rotated);
	type = 1;
}

//二值化
IplImage *src, *gray, *binary, *img1;
CvSeq* lines;
CvMemStorage* storage;
IplImage *dst;
int minLength = 30, thresh = 40, terval = 40;

void call_back(int n)
{
	cvThreshold(gray, binary, n, 255, CV_THRESH_BINARY);
	cvXorS(binary, cvScalarAll(255), binary);
	cvShowImage("Binary Image", binary);
}

//霍夫变换
void hough_call_back0(int Thresh)
{
	thresh = Thresh;
	if (thresh < 0)
	{
		thresh = 0;
	}
	//delete storage;
	//delete lines;
	//delete img1;
	storage = cvCreateMemStorage();
	lines = cvHoughLines2(dst, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI / 180, thresh, minLength, terval);
	img1 = cvCreateImage(cvGetSize(dst), 8, 1);
	cvSetZero(img1);
	for (int i = 0; i < lines->total; i++)
	{
		CvPoint* line = (CvPoint*)cvGetSeqElem(lines, i);
		cvLine(img1, line[0], line[1], CvScalar(255));
	
	}
	cvShowImage("霍夫变换", img1);
}

void hough_call_back1(int MinLength)
{
	minLength = MinLength;
	//delete storage;
	//delete lines;
	//delete img1;
	storage = cvCreateMemStorage();
	lines = cvHoughLines2(dst, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI / 180, thresh, minLength, terval);
	img1 = cvCreateImage(cvGetSize(dst), 8, 1);
	cvSetZero(img1);
	for (int i = 0; i < lines->total; i++)
	{
		CvPoint* line = (CvPoint*)cvGetSeqElem(lines, i);
		cvLine(img1, line[0], line[1], CvScalar(255));
		sp.add(line[0], line[1],i);
	}
	sp.initial();
	cvShowImage("霍夫变换", img1);
}


void thinImage(IplImage* src, IplImage* dst, int maxIterations = -1)
{
	CvSize size = cvGetSize(src);
	cvCopy(src, dst);//将src中的内容拷贝到dst中
	int count = 0;	//记录迭代次数
	while (true)
	{

		count++;
		if (maxIterations != -1 && count > maxIterations) //限制次数并且迭代次数到达
		{
			break;
		}

		//std::cout << count << ' ';输出迭代次数
		vector<pair<int, int> > mFlag; //用于标记需要删除的点
		//对点标记
		for (int i = 0; i<size.height; ++i)
		{
			for (int j = 0; j<size.width; ++j)
			{
				//如果满足四个条件，进行标记
				//  p9 p2 p3
				//  p8 p1 p4
				//  p7 p6 p5
				int p1 = (CV_IMAGE_ELEM(dst, uchar, i, j)>0 ? 1 : 0);
				int p2 = (i == 0) ? 0 : (CV_IMAGE_ELEM(dst, uchar, i - 1, j)>0 ? 1 : 0);
				int p3 = (i == 0 || j == size.width - 1) ? 0 : (CV_IMAGE_ELEM(dst, uchar, i - 1, j + 1)>0 ? 1 : 0);
				int p4 = (j == size.width - 1) ? 0 : (CV_IMAGE_ELEM(dst, uchar, i, j + 1)>0 ? 1 : 0);
				int p5 = (i == size.height - 1 || j == size.width - 1) ? 0 : (CV_IMAGE_ELEM(dst, uchar, i + 1, j + 1)>0 ? 1 : 0);
				int p6 = (i == size.height - 1) ? 0 : (CV_IMAGE_ELEM(dst, uchar, i + 1, j)>0 ? 1 : 0);
				int p7 = (i == size.height - 1 || j == 0) ? 0 : (CV_IMAGE_ELEM(dst, uchar, i + 1, j - 1)>0 ? 1 : 0);
				int p8 = (j == 0) ? 0 : (CV_IMAGE_ELEM(dst, uchar, i, j - 1)>0 ? 1 : 0);
				int p9 = (i == 0 || j == 0) ? 0 : (CV_IMAGE_ELEM(dst, uchar, i - 1, j - 1)>0 ? 1 : 0);


				if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) >= 2 && (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) <= 6)
				{
					int ap = 0;
					if (p2 == 0 && p3 == 1) ++ap;
					if (p3 == 0 && p4 == 1) ++ap;
					if (p4 == 0 && p5 == 1) ++ap;
					if (p5 == 0 && p6 == 1) ++ap;
					if (p6 == 0 && p7 == 1) ++ap;
					if (p7 == 0 && p8 == 1) ++ap;
					if (p8 == 0 && p9 == 1) ++ap;
					if (p9 == 0 && p2 == 1) ++ap;

					if (ap == 1)
					{
						if (p2*p4*p6 == 0)
						{
							if (p4*p6*p8 == 0)
							{
								//标记
								mFlag.push_back(make_pair(i, j));
							}
						}
					}
				}
			}
		}

		//将标记的点删除
		for (vector<pair<int, int> >::iterator i = mFlag.begin(); i != mFlag.end(); ++i)
		{
			CV_IMAGE_ELEM(dst, uchar, i->first, i->second) = 0;
		}

		//直到没有点满足，算法结束
		if (mFlag.size() == 0)
		{
			break;
		}
		else
		{
			mFlag.clear();//将mFlag清空
		}

		//对点标记
		for (int i = 0; i<size.height; ++i)
		{
			for (int j = 0; j<size.width; ++j)
			{
				//如果满足四个条件，进行标记
				//  p9 p2 p3
				//  p8 p1 p4
				//  p7 p6 p5
				int p1 = CV_IMAGE_ELEM(dst, uchar, i, j)>0 ? 1 : 0;
				if (p1 != 1) continue;
				int p2 = (i == 0) ? 0 : (CV_IMAGE_ELEM(dst, uchar, i - 1, j)>0 ? 1 : 0);
				int p3 = (i == 0 || j == size.width - 1) ? 0 : (CV_IMAGE_ELEM(dst, uchar, i - 1, j + 1)>0 ? 1 : 0);
				int p4 = (j == size.width - 1) ? 0 : (CV_IMAGE_ELEM(dst, uchar, i, j + 1)>0 ? 1 : 0);
				int p5 = (i == size.height - 1 || j == size.width - 1) ? 0 : (CV_IMAGE_ELEM(dst, uchar, i + 1, j + 1)>0 ? 1 : 0);
				int p6 = (i == size.height - 1) ? 0 : (CV_IMAGE_ELEM(dst, uchar, i + 1, j)>0 ? 1 : 0);
				int p7 = (i == size.height - 1 || j == 0) ? 0 : (CV_IMAGE_ELEM(dst, uchar, i + 1, j - 1)>0 ? 1 : 0);
				int p8 = (j == 0) ? 0 : (CV_IMAGE_ELEM(dst, uchar, i, j - 1)>0 ? 1 : 0);
				int p9 = (i == 0 || j == 0) ? 0 : (CV_IMAGE_ELEM(dst, uchar, i - 1, j - 1)>0 ? 1 : 0);

				if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) >= 2 && (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) <= 6)
				{
					int ap = 0;
					if (p2 == 0 && p3 == 1) ++ap;
					if (p3 == 0 && p4 == 1) ++ap;
					if (p4 == 0 && p5 == 1) ++ap;
					if (p5 == 0 && p6 == 1) ++ap;
					if (p6 == 0 && p7 == 1) ++ap;
					if (p7 == 0 && p8 == 1) ++ap;
					if (p8 == 0 && p9 == 1) ++ap;
					if (p9 == 0 && p2 == 1) ++ap;

					if (ap == 1)
					{
						if (p2*p4*p8 == 0)
						{
							if (p2*p6*p8 == 0)
							{
								//标记
								mFlag.push_back(make_pair(i, j));
							}
						}
					}
				}
			}
		}
		//删除
		for (vector<pair<int, int> >::iterator i = mFlag.begin(); i != mFlag.end(); ++i)
		{
			CV_IMAGE_ELEM(dst, uchar, i->first, i->second) = 0;
		}

		//直到没有点满足，算法结束
		if (mFlag.size() == 0)
		{
			break;
		}
		else
		{
			mFlag.clear();//将mFlag清空
		}
	}
}


void hough_call_back2(int Terval)
{
	terval = Terval;
	//delete storage;
	//delete lines;
	//delete img1;
	storage = cvCreateMemStorage();
	lines = cvHoughLines2(dst, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI / 180, thresh, minLength, terval);
	img1 = cvCreateImage(cvGetSize(dst), 8, 1);
	cvSetZero(img1);
	for (int i = 0; i < lines->total; i++)
	{
		CvPoint* line = (CvPoint*)cvGetSeqElem(lines, i);
		cvLine(img1, line[0], line[1], CvScalar(255));
		sp.add(line[0], line[1], i);
	}
	sp.initial();
	cvShowImage("霍夫变换", img1);
}

/*void bfs(int tmpX,int tmpY,int&x, int &y, int r, int c,int& minLen)
{
	
	if (tmpX<0 || tmpY<0 || tmpX>dst->width || tmpY>dst->height)
	{
		return;
	}
	if (vis[tmpX + tmpY*dst->widthStep] == 1)
	{
		return;
	}
	vis[tmpX + tmpY*dst->widthStep] = 1;

	uchar* data = (uchar *)dst->imageData;
	if (data[tmpX + tmpY*dst->widthStep] != 0)
	{
		
		double tmp = sqrt((tmpX - c)*(tmpX - c) + (tmpY - r)*(tmpY - r));
		if (tmp > minLen)
		{
			return;
		}
		else
		{
			x = tmpX;
			y = tmpY;
			minLen = tmp;
		}
	}
	bfs(tmpX - 1, tmpY - 1, x, y, r, c, minLen);
	bfs(tmpX , tmpY -1, x, y, r, c, minLen);
	bfs(tmpX + 1, tmpY - 1, x, y, r, c, minLen);
	bfs(tmpX - 1, tmpY , x, y, r, c, minLen);
	bfs(tmpX + 1, tmpY , x, y, r, c, minLen);
	bfs(tmpX - 1, tmpY +1, x, y, r, c, minLen);
	bfs(tmpX , tmpY +1, x, y, r, c, minLen);
	bfs(tmpX +1, tmpY +1, x, y, r, c, minLen);
	


}*/


double getAngle(int x1,int y1,int x2,int y2)
{
	double angle;
	if (x2 == x1)
	{
		angle = pi / 2;
	}
	else
	{
		angle = atan(abs(y2 - y1) / abs(x2 - x1));
	}
	return angle;
}



Turn getTurn()
{
	memset(vis, 0, sizeof(int)* 10000000);          //r1表示车头
	int r1, c1, r2, c2;
	r1 = trackBox1.center.y;
	c1 = trackBox1.center.x;
	r2 = trackBox2.center.y;
	c2 = trackBox2.center.x;

	double angleCar = getAngle(c2, r2, c1, r1);

	double tmp;

	double minL = 100000;
	double minAng = 100;
	int minIndex;
	bool isS;

	for (int i = 0; i < sp.size; i++)
	{
		cout << "getTurn" << endl;
		if (sqrt(pow(sp.point[i].start.x - c1, 2) + pow(sp.point[i].start.y - r1, 2)) < minL && abs(sp.point[i].angle-angleCar))
		{
			minL = sqrt(pow(sp.point[i].start.x - c1, 2) + pow(sp.point[i].start.y - r1, 2));
			minIndex = i;
			isS = true;
		}
		if (sqrt(pow(sp.point[i].end.x - c1, 2) + pow(sp.point[i].end.y - r1, 2)) < minL  && abs(sp.point[i].angle - angleCar))
		{
			minL = sqrt(pow(sp.point[i].start.x - c1, 2) + pow(sp.point[i].start.y - r1, 2));
			minIndex = i;
			isS = false;
		}
	}


	int px1 = 0, py1 = 0, px2 = 0, py2 = 0;    //前进方向为px2 到px1
	px1 = sp.point[minIndex].end.x;
	py1 = sp.point[minIndex].end.y;
	px2 = sp.point[minIndex].start.x;
	py2 = sp.point[minIndex].start.y;
	
	/*int minLen1 = 1000000, minLen2 = 10000000;
	uchar* data = (uchar *)dst->imageData;
	for (int j = 0; j < dst->height; j++)
	{
		data = (uchar *)(dst->imageData + j*dst->widthStep);
		for (int i = 0; i < dst->width; i++)
		{
			if (data[i] != 0)
			{
				tmp = sqrt((i - c1)*(i - c1) + (j - r1)*(j - r1));
				if (tmp < minLen1)
				{
					minLen1 = tmp;
					px1 = i;
					py1 = j;
				}
			}
		}
	}

	for (int j = 0; j < dst->height; j++)
	{
		data = (uchar *)(dst->imageData + j*dst->widthStep);
		for (int i = 0; i < dst->width; i++)
		{
			if (data[i] != 0)
			{
				tmp = sqrt((i - c2)*(i - c2) + (j - r2)*(j - r2));
				if (tmp < minLen2)
				{
					minLen2 = tmp;
					px2 = i;
					py2 = j;
				}
			}
		}
	}*/

	//以上获得了离两个标志点最近直线上的两个点

	double thth1,thth2;
	thth1 = getAngle(px2, py2, px1, py1);
	if (abs(thth1 - angleCar) < 0.1)
	{
		return Ahead;
	}
	if (thth1 < 0 && px1<px2 && py1<py2)
	{
		thth1 += pi;
	}
	if (thth1 > 0 && thth1 < pi / 2 && px1<px2 && py1>py2)
	{
		thth1 += pi;
	}
	if (r1 < r2 && c1 < c2)
	{
		angleCar += pi;
	}
	if (r1<r2 && c1>c2)
	{
		angleCar += pi;
	}

	if (angleCar < thth1)
	{
		return Left;
	}
	else
	{
		return Right;
	}


	/*int threshold = 5;

	bool goAhead = false;

	if ((r2 - r1)*(py2 - py1) + (c2 - c1)*(px2 - px1) > 0)
	{
		goAhead = true;
	}
	else
	{
		goAhead = false;
	}

	bool isLeft = false;

	double thL0 = 0,thL = 0, thP = 0;  //分别是直线的角度跟点的角度

	int tmprL = px1 - px2;
	int tmpcL = py2 - py1;
	int thirdL = sqrt(tmprL*tmprL + tmpcL*tmpcL);

	thL0 = atan((double)tmpcL / tmprL);
	thL = thL0;

	int rootX = 0, rootY = 0;
	if (thL < 0)
	{
		thL += pi;
	}

	if (thL0 > 0)
	{
		if (-py1 > -py2)
		{
			rootX = (double)(-dst->height + py1) / thL0 + px1;
			rootY = -dst->height;
		}
		else
		{
			rootY = 0;
			rootX = (double)py1 / thL0 + px1;
			
		}

	}
	else if (thL0 < 0)
	{
		if (px1 < px2)
		{
			rootX = (double)(-dst->height + py1) / thL0 + px1;
			rootY = -dst->height;
		}
		else
		{
			rootY = 0;
			rootX =(double) py1 / thL0 + px1;
		}
	}

	if (-r1 - rootY != 0)
		thP = atan((double)(c1 - rootX) / (-r1 - rootY));


	if (thP < 0)
	{
		thP += pi;
	}

	if (thP < thL)
	{
		isLeft = true;
	}
	else
	{
		isLeft = false;
	}

	int pr = r1 - r2;
	int pc = c2 - c1;
	
	if (isLeft == true && goAhead == true)
	{
		return Left;
	}
	if (isLeft == false)
	{
		return Right;
	}
	return Straight;
	
	//bfs(r1, c1, px1, py1,r1,c1, minLen1);
	//bfs(r2, c2, px2, py2, r2, c2, minLen2);*/
	
}

Car mycar;

void InitBluetooth()
{   //  初始化蓝牙
	cout << "=================================" << endl;
	if (mycar.initialPort("COM8")) {
		cout << "!!!build connection completed.!!!" << endl;
		mycar.changeSpeed("125");
	}
	else cout << "!!!Build connection failed.!!!" << endl;
	cout << "=================================" << endl;
}


void carTest()
{
	for (int i = 0; i < 100;i++)
	{
		mycar.up();
	}
	
}



int main()
{
	InitBluetooth();
	//carTest();
	//system("pause");
	

	VideoCapture cap; //定义一个摄像头捕捉的类对象
	Rect trackwindow1;     //第一个标志
	Rect trackwindow2;     //第二个标志
	int hsize = 16;
	float hranges[] = { 0, 180 };//hranges在后面的计算直方图函数中要用到
	const float* phranges1 = hranges;
	const float* phranges2 = hranges;
	Mat frame, hsv, hue1, hue2, mask1, mask2, hist1, hist2, histimg1 = Mat::zeros(200, 320, CV_8UC3), histimg2 = Mat::zeros(200, 320, CV_8UC3), backproj1, backproj2;


	cap.open(0);//直接调用成员函数打开摄像头

	if (!cap.isOpened())
	{
		cout << "Error" << endl;
		return -1;
	}

	//namedWindow("hist1ogram", 0);
	namedWindow("CamShift Demo", 0);
	
	namedWindow("Rotated", 0);
	//透视变换

	while (1)
	{
		cap >> frame;
		if (frame.empty())
		{
			break;
		}
		if (type == 0)
		{
			change(frame);
		}
		if (type == 1)
		{
			break;
		}

		char c = (char)waitKey(10);
		if (c == 'p')              //按下p表示开始惊醒透视变换选点，选点方向左上右上左下右下
		{
			type = 0;
		}
		imshow("CamShift Demo", frame);
	}
	imshow("Rotated", rotated);


	//二值化
	src = &IplImage(rotated);
	gray = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);
	cvCvtColor(src, gray, CV_BGR2GRAY);
	binary = cvCreateImage(cvGetSize(gray), IPL_DEPTH_8U, 1);
	cvThreshold(gray, binary, 128, 1, CV_THRESH_BINARY);
	cvNamedWindow("Binary Image", 0);
	cvShowImage("Binary Image", binary);
	int n = 0;
	cvCreateTrackbar("二值化阈值", "Binary Image", &n, 254, call_back);
	call_back(1);
	cvWaitKey(0);

	//细化
	dst = cvCreateImage(cvGetSize(binary), binary->depth, binary->nChannels);
	thinImage(binary, dst);
	cvNamedWindow("细化", 0);
	cvShowImage("细化", dst);

	//霍夫变换
	cvNamedWindow("霍夫变换", 0);
	storage = cvCreateMemStorage();
	lines = cvHoughLines2(dst, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI / 180, 30, 30, 40);
	img1 = cvCreateImage(cvGetSize(dst), 8, 1);
	cvSetZero(img1);
	for (int i = 0; i < lines->total; i++)
	{
		CvPoint* line = (CvPoint*)cvGetSeqElem(lines, i);
		cvLine(img1, line[0], line[1], CvScalar(255));
		sp.add(line[0], line[1],i);
		sp.initial();
	}
	cvShowImage("霍夫变换", img1);
	int Thresh = 20;
	int MinLength = 40;
	int Terval = 40;
	cvCreateTrackbar("阈值", "霍夫变换", &Thresh, 100, hough_call_back0);
	cvCreateTrackbar("最短线段长度", "霍夫变换", &MinLength, 100, hough_call_back1);
	cvCreateTrackbar("线段间隔最大值", "霍夫变换", &Terval, 100, hough_call_back2);
	cvWaitKey(0);
	cvDestroyWindow("Binary Image");
	cvDestroyWindow("细化");
	cvReleaseImage(&gray);
	cvReleaseImageHeader(&binary);
	//cvReleaseImage(&dst);
		

	
	sp.test();
	cout << "Please select the start point: ";
	cin >> startPoint;
	if (startPoint == 1)
	{
		sp.reverse();
	}


	//追踪
	setMouseCallback("Rotated", onMouse, 0);//消息响应机制
	createTrackbar("Vmin", "Rotated", &vmin, 256, 0);//createTrackbar函数的功能是在对应的窗口创建滑动条，滑动条Vmin,vmin表示滑动条的值，最大为256
	createTrackbar("Vmax", "Rotated", &vmax, 256, 0);//最后一个参数为0代表没有调用滑动拖动的响应函数
	createTrackbar("Smin", "Rotated", &smin, 256, 0);//vmin,vmax,smin初始值分别为10,256,30

	
	bool paused = false;

	while (1)
	{
		if (type == 10 && mark2 == true)
		{
			Turn t = getTurn();
			if (t == Left)
			{
				mycar.left();
				cout << "Left" << endl;
			}
			else if (t == Right)
			{
				mycar.right();
				cout << "Right" << endl;
			}
			else if (t == Ahead)
			{
				mycar.up();
				cout << "Ahead" << endl;
			}
		}


		if (!paused)//没有暂停
		{
			cap >> frame;//从摄像头抓取一帧图像并输出到frame中
			if (frame.empty())
				break;
		}

		warpPerspective(frame, image, warpMatrix, rotated.size(), INTER_LINEAR, BORDER_CONSTANT);
		

		//frame.copyTo(image);
		frame.copyTo(image2);

		if (!paused)//没有按暂停键
		{
			cvtColor(image, hsv, CV_BGR2HSV);//将rgb摄像头帧转化成hsv空间的
			if (trackObject)//trackObject初始化为0,或者按完键盘的'c'键后也为0，当鼠标单击松开后为-1
			{
				int _vmin = vmin, _vmax = vmax;

				//inRange函数的功能是检查输入数组每个元素大小是否在2个给定数值之间，可以有多通道,mask1保存0通道的最小值，也就是h分量
				//这里利用了hsv的3个通道，比较h,0~180,s,smin~256,v,min(vmin,vmax),max(vmin,vmax)。如果3个通道都在对应的范围内，则
				//mask1对应的那个点的值全为1(0xff)，否则为0(0x00).
				inRange(hsv, Scalar(0, smin, MIN(_vmin, _vmax)),
					Scalar(180, 256, MAX(_vmin, _vmax)), mask1);
				int ch[] = { 0, 0 };
				hue1.create(hsv.size(), hsv.depth());
				hue2.create(hsv.size(), hsv.depth());
				mixChannels(&hsv, 1, &hue1, 1, ch, 1);
				mixChannels(&hsv, 1, &hue2, 1, ch, 1);

				if (trackObject < 0)//鼠标选择区域松开后，该函数内部又将其赋值1
				{
					//ROI指感兴趣的区域
					Mat roi(hue1, selection1), maskroi(mask1, selection1);//mask1保存的hsv的最小值

					calcHist(&roi, 1, 0, maskroi, hist1, 1, &hsize, &phranges1);//将roi的0通道计算直方图并通过mask1放入hist1中，hsize为每一维直方图的大小
					normalize(hist1, hist1, 0, 255, CV_MINMAX);//将hist1矩阵进行数组范围归一化，都归一化到0~255

					trackwindow1 = selection1;
					trackObject = 1;//只要鼠标选完区域松开后，且没有按键盘清0键'c'，则trackObject一直保持为1，因此该if函数只能执行一次，除非重新选择跟踪区域

					histimg1 = Scalar::all(0);//清0
					int binW = histimg1.cols / hsize;  
					Mat buf(1, hsize, CV_8UC3);
					for (int i = 0; i < hsize; i++)
						buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180. / hsize), 255, 255);
					cvtColor(buf, buf, CV_HSV2BGR);//将hsv转换成bgr

					for (int i = 0; i < hsize; i++)
					{
						int val = saturate_cast<int>(hist1.at<float>(i)*histimg1.rows / 255);
						rectangle(histimg1, Point(i*binW, histimg1.rows), Point((i + 1)*binW, histimg1.rows - val), Scalar(buf.at<Vec3b>(i)), -1, 8);
					}

				}
				calcBackProject(&hue1, 1, 0, hist1, backproj1, &phranges1);//计算直方图的反向投影，计算hue1图像0通道直方图hist1的反向投影，并赋值到backproj1中
				backproj1 &= mask1;

				trackBox1 = CamShift(backproj1, trackwindow1, TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1));

				if (trackwindow1.area() <= 1)
				{
					int cols = backproj1.cols, rows = backproj1.rows, r = (MIN(cols, rows) + 5) / 6;
					trackwindow1 = Rect(trackwindow1.x - r, trackwindow1.y - r, trackwindow1.x + r, trackwindow1.y + r) &Rect(0, 0, cols, rows);//Rect函数为矩阵的偏移和大小，即第一二个参数为矩阵的左上角点坐标，第三四个参数为矩阵的宽和高
				}

				if (backproj1Mode)
				{
					cvtColor(backproj1, image, CV_GRAY2BGR);//因此投影模式下显示的也是rgb图？
				}
				//cvtColor(backproj1, image, CV_GRAY2BGR);//因此投影模式下显示的也是rgb图？

				ellipse(image, trackBox1, Scalar(0, 0, 255), 3, CV_AA); //画出椭圆
			}

			if (trackObject >= 2)//trackObject初始化为0,或者按完键盘的'c'键后也为0，当鼠标单击松开后为-1
			{
				int _vmin = vmin, _vmax = vmax;

				//inRange函数的功能是检查输入数组每个元素大小是否在2个给定数值之间，可以有多通道,mask1保存0通道的最小值，也就是h分量
				//这里利用了hsv的3个通道，比较h,0~180,s,smin~256,v,min(vmin,vmax),max(vmin,vmax)。如果3个通道都在对应的范围内，则
				//mask1对应的那个点的值全为1(0xff)，否则为0(0x00).
				inRange(hsv, Scalar(0, smin, MIN(_vmin, _vmax)),
					Scalar(180, 256, MAX(_vmin, _vmax)), mask2);
				int ch[] = { 0, 0 };
				hue1.create(hsv.size(), hsv.depth());//hue1初始化为与hsv大小深度一样的矩阵，色调的度量是用角度表示的，红绿蓝之间相差120度，反色相差180度
				hue2.create(hsv.size(), hsv.depth());
				mixChannels(&hsv, 1, &hue1, 1, ch, 1);//将hsv第一个通道(也就是色调)的数复制到hue1中，0索引数组
				mixChannels(&hsv, 1, &hue2, 1, ch, 1);

				if (trackObject == 3)
				{
					//此处的构造函数roi用的是Mat hue1的矩阵头，且roi的数据指针指向hue1，即共用相同的数据，select为其感兴趣的区域

					Mat roi2(hue2, selection2), maskroi(mask2, selection2);//mask1保存的hsv的最小值
					//calchist1()函数第一个参数为输入矩阵序列，第2个参数表示输入的矩阵数目，第3个参数表示将被计算直方图维数通道的列表，第4个参数表示可选的掩码函数
					//第5个参数表示输出直方图，第6个参数表示直方图的维数，第7个参数为每一维直方图数组的大小，第8个参数为每一维直方图bin的边界
					calcHist(&roi2, 1, 0, maskroi, hist2, 1, &hsize, &phranges2);//将roi的0通道计算直方图并通过mask1放入hist1中，hsize为每一维直方图的大小
					normalize(hist2, hist2, 0, 255, CV_MINMAX);//将hist1矩阵进行数组范围归一化，都归一化到0~255

					trackwindow2 = selection2;
					trackObject = 4;//只要鼠标选完区域松开后，且没有按键盘清0键'c'，则trackObject一直保持为1，因此该if函数只能执行一次，除非重新选择跟踪区域

					histimg2 = Scalar::all(0);//与按下'c'键是一样的，这里的all(0)表示的是标量全部清0
					int binW = histimg2.cols / hsize;  //hist1ing是一个200*300的矩阵，hsize应该是每一个bin的宽度，也就是hist1ing矩阵能分出几个bin出来
					Mat buf(1, hsize, CV_8UC3);//定义一个缓冲单bin矩阵
					for (int i = 0; i < hsize; i++)//saturate_case函数为从一个初始类型准确变换到另一个初始类型
						buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180. / hsize), 255, 255);//Vec3b为3个char值的向量
					cvtColor(buf, buf, CV_HSV2BGR);//将hsv又转换成bgr

					for (int i = 0; i < hsize; i++)
					{
						//cout << "he" << endl;
						int val = saturate_cast<int>(hist2.at<float>(i)*histimg2.rows / 255);//at函数为返回一个指定数组元素的参考值
						rectangle(histimg2, Point(i*binW, histimg2.rows),    //在一幅输入图像上画一个简单抽的矩形，指定左上角和右下角，并定义颜色，大小，线型等
							Point((i + 1)*binW, histimg2.rows - val),
							Scalar(buf.at<Vec3b>(i)), -1, 8);
					}

				}
				calcBackProject(&hue2, 1, 0, hist2, backproj2, &phranges2);//计算直方图的反向投影，计算hue1图像0通道直方图hist1的反向投影，并赋值到backproj2中	
				backproj2 &= mask2;
				trackBox2 = CamShift(backproj2, trackwindow2, TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1));
				if (  trackwindow2.area() <= 1)
				{
					int cols = backproj2.cols, rows = backproj2.rows, r = (MIN(cols, rows) + 5) / 6;
					trackwindow2 = Rect(trackwindow2.x - r, trackwindow2.y - r, trackwindow2.x + r, trackwindow2.y + r) &Rect(0, 0, cols, rows);//Rect函数为矩阵的偏移和大小，即第一二个参数为矩阵的左上角点坐标，第三四个参数为矩阵的宽和高
				}

				if (backproj1Mode)
				{
					
					cvtColor(backproj2, image2, CV_GRAY2BGR);//因此投影模式下显示的也是rgb图？
					
				}

				ellipse(image, trackBox2, Scalar(0, 0, 255), 3, CV_AA); //画出椭圆
			}
		}
		else if (trackObject < 0)
			paused = false;
		//拖出矩形框
		if (mark1 == false &&  selectObject && selection1.width > 0 && selection1.height > 0)
		{
			Mat roi(image, selection1);
			bitwise_not(roi, roi);//bitwise_not为将每一个bit位取反

		}
		else if (mark1 == true && selectObject && selection2.width > 0 && selection2.height > 0)
		{
			//cout << "he" << endl;
			Mat roi2(image, selection2);
			bitwise_not(roi2, roi2);//bitwise_not为将每一个bit位取反
		}

		imshow("CamShift Demo", frame);
		imshow("Rotated", image);
		imshow("histogram", histimg1);
		imshow("histogram2", histimg2);

		char c = (char)waitKey(10);
		if (c == 27)              //退出键
			break;
		switch (c)
		{
		case 's':
			type = 10;
			break;
		case 'b':             //反向投影模型交替
			backproj1Mode = !backproj1Mode;
			break;
		case 'c':            //清零跟踪目标对象
			trackObject = 0;
			histimg1 = Scalar::all(0);
			mark1 = false;
			mark2 = false;
			break;
		case '1':          //标志标志1选取完成
			trackObject=1; 
			mark1 = true;
			break;
		case '2':
			trackObject++;
			mark2 = true;
			break;
		case 'h':          //显示直方图交替
			showhist1 = !showhist1;
			if (!showhist1)
			{
				destroyWindow("histogram");
				destroyWindow("histogram2");
			}
			else
			{
				namedWindow("histogram", 1);
				namedWindow("histogram2", 1);
			}
			break;
		case 'p':          //透视变形
			type = 0;     
			break;
		default:
			;
		}
	}

	
	return 0;
}