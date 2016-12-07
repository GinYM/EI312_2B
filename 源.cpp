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




enum Turn{ Left, Right, Ahead };         //�н�����



Mat image,image2;
Mat warpMatrix;          //͸�ӱ��ξ���
Mat rotated;             //ת�����ͼ��

RotatedRect trackBox1;//����һ����ת�ľ��������
RotatedRect trackBox2;//����һ����ת�ľ��������

bool backproj1Mode = false; //��ʾ�Ƿ�Ҫ���뷴��ͶӰģʽ��ture��ʾ׼�����뷴��ͶӰģʽ
bool selectObject = false;//�����Ƿ���ѡҪ���ٵĳ�ʼĿ�꣬true��ʾ���������ѡ��
int trackObject = 0; //�������Ŀ����Ŀ
bool showhist1 = true;//�Ƿ���ʾֱ��ͼ
Point origin;//���ڱ������ѡ���һ�ε���ʱ���λ��
Rect selection1,selection2;//���ڱ������ѡ��ľ��ο�
int vmin = 10, vmax = 256, smin = 30;
bool mark1 = false;    // mark1 = true ����ѡ���־1��mark1 = false ����û��ִ�����־1��ѡȡ 
bool mark2 = false;    //mark2 ��־2

int type = -1;         //-1 �����ʼ״̬��0 �������͸�ӱ��� �� 1�������

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
		if (selectObject)//ֻ�е�����������ȥʱ����Ч��Ȼ��ͨ��if�������Ϳ���ȷ����ѡ��ľ�������selection11��
		{
			selection1.x = MIN(x, origin.x);//�������ϽǶ�������
			selection1.y = MIN(y, origin.y);
			selection1.width = std::abs(x - origin.x);//���ο�
			selection1.height = std::abs(y - origin.y);//���θ�

			selection1 &= Rect(0, 0, image.cols, image.rows);//����ȷ����ѡ�ľ���������ͼƬ��Χ��
		}

		switch (event)
		{
		case CV_EVENT_LBUTTONDOWN:
			origin = Point(x, y);
			selection1 = Rect(x, y, 0, 0);//���հ���ȥʱ��ʼ����һ����������
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
		if (selectObject)//ֻ�е�����������ȥʱ����Ч��Ȼ��ͨ��if�������Ϳ���ȷ����ѡ��ľ�������selection2��
		{
			selection2.x = MIN(x, origin.x);//�������ϽǶ�������
			selection2.y = MIN(y, origin.y);
			selection2.width = std::abs(x - origin.x);//���ο�
			selection2.height = std::abs(y - origin.y);//���θ�

			selection2 &= Rect(0, 0, image.cols, image.rows);//����ȷ����ѡ�ľ���������ͼƬ��Χ��
		}

		switch (event)
		{
		case CV_EVENT_LBUTTONDOWN:

			//trackObject = 2;
			origin = Point(x, y);
			selection2 = Rect(x, y, 0, 0);//���հ���ȥʱ��ʼ����һ����������
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

//��ֵ��
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

//����任
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
	cvShowImage("����任", img1);
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
	cvShowImage("����任", img1);
}


void thinImage(IplImage* src, IplImage* dst, int maxIterations = -1)
{
	CvSize size = cvGetSize(src);
	cvCopy(src, dst);//��src�е����ݿ�����dst��
	int count = 0;	//��¼��������
	while (true)
	{

		count++;
		if (maxIterations != -1 && count > maxIterations) //���ƴ������ҵ�����������
		{
			break;
		}

		//std::cout << count << ' ';�����������
		vector<pair<int, int> > mFlag; //���ڱ����Ҫɾ���ĵ�
		//�Ե���
		for (int i = 0; i<size.height; ++i)
		{
			for (int j = 0; j<size.width; ++j)
			{
				//��������ĸ����������б��
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
								//���
								mFlag.push_back(make_pair(i, j));
							}
						}
					}
				}
			}
		}

		//����ǵĵ�ɾ��
		for (vector<pair<int, int> >::iterator i = mFlag.begin(); i != mFlag.end(); ++i)
		{
			CV_IMAGE_ELEM(dst, uchar, i->first, i->second) = 0;
		}

		//ֱ��û�е����㣬�㷨����
		if (mFlag.size() == 0)
		{
			break;
		}
		else
		{
			mFlag.clear();//��mFlag���
		}

		//�Ե���
		for (int i = 0; i<size.height; ++i)
		{
			for (int j = 0; j<size.width; ++j)
			{
				//��������ĸ����������б��
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
								//���
								mFlag.push_back(make_pair(i, j));
							}
						}
					}
				}
			}
		}
		//ɾ��
		for (vector<pair<int, int> >::iterator i = mFlag.begin(); i != mFlag.end(); ++i)
		{
			CV_IMAGE_ELEM(dst, uchar, i->first, i->second) = 0;
		}

		//ֱ��û�е����㣬�㷨����
		if (mFlag.size() == 0)
		{
			break;
		}
		else
		{
			mFlag.clear();//��mFlag���
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
	cvShowImage("����任", img1);
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
	memset(vis, 0, sizeof(int)* 10000000);          //r1��ʾ��ͷ
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


	int px1 = 0, py1 = 0, px2 = 0, py2 = 0;    //ǰ������Ϊpx2 ��px1
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

	//���ϻ������������־�����ֱ���ϵ�������

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

	double thL0 = 0,thL = 0, thP = 0;  //�ֱ���ֱ�ߵĽǶȸ���ĽǶ�

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
{   //  ��ʼ������
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
	

	VideoCapture cap; //����һ������ͷ��׽�������
	Rect trackwindow1;     //��һ����־
	Rect trackwindow2;     //�ڶ�����־
	int hsize = 16;
	float hranges[] = { 0, 180 };//hranges�ں���ļ���ֱ��ͼ������Ҫ�õ�
	const float* phranges1 = hranges;
	const float* phranges2 = hranges;
	Mat frame, hsv, hue1, hue2, mask1, mask2, hist1, hist2, histimg1 = Mat::zeros(200, 320, CV_8UC3), histimg2 = Mat::zeros(200, 320, CV_8UC3), backproj1, backproj2;


	cap.open(0);//ֱ�ӵ��ó�Ա����������ͷ

	if (!cap.isOpened())
	{
		cout << "Error" << endl;
		return -1;
	}

	//namedWindow("hist1ogram", 0);
	namedWindow("CamShift Demo", 0);
	
	namedWindow("Rotated", 0);
	//͸�ӱ任

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
		if (c == 'p')              //����p��ʾ��ʼ����͸�ӱ任ѡ�㣬ѡ�㷽������������������
		{
			type = 0;
		}
		imshow("CamShift Demo", frame);
	}
	imshow("Rotated", rotated);


	//��ֵ��
	src = &IplImage(rotated);
	gray = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);
	cvCvtColor(src, gray, CV_BGR2GRAY);
	binary = cvCreateImage(cvGetSize(gray), IPL_DEPTH_8U, 1);
	cvThreshold(gray, binary, 128, 1, CV_THRESH_BINARY);
	cvNamedWindow("Binary Image", 0);
	cvShowImage("Binary Image", binary);
	int n = 0;
	cvCreateTrackbar("��ֵ����ֵ", "Binary Image", &n, 254, call_back);
	call_back(1);
	cvWaitKey(0);

	//ϸ��
	dst = cvCreateImage(cvGetSize(binary), binary->depth, binary->nChannels);
	thinImage(binary, dst);
	cvNamedWindow("ϸ��", 0);
	cvShowImage("ϸ��", dst);

	//����任
	cvNamedWindow("����任", 0);
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
	cvShowImage("����任", img1);
	int Thresh = 20;
	int MinLength = 40;
	int Terval = 40;
	cvCreateTrackbar("��ֵ", "����任", &Thresh, 100, hough_call_back0);
	cvCreateTrackbar("����߶γ���", "����任", &MinLength, 100, hough_call_back1);
	cvCreateTrackbar("�߶μ�����ֵ", "����任", &Terval, 100, hough_call_back2);
	cvWaitKey(0);
	cvDestroyWindow("Binary Image");
	cvDestroyWindow("ϸ��");
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


	//׷��
	setMouseCallback("Rotated", onMouse, 0);//��Ϣ��Ӧ����
	createTrackbar("Vmin", "Rotated", &vmin, 256, 0);//createTrackbar�����Ĺ������ڶ�Ӧ�Ĵ��ڴ�����������������Vmin,vmin��ʾ��������ֵ�����Ϊ256
	createTrackbar("Vmax", "Rotated", &vmax, 256, 0);//���һ������Ϊ0����û�е��û����϶�����Ӧ����
	createTrackbar("Smin", "Rotated", &smin, 256, 0);//vmin,vmax,smin��ʼֵ�ֱ�Ϊ10,256,30

	
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


		if (!paused)//û����ͣ
		{
			cap >> frame;//������ͷץȡһ֡ͼ�������frame��
			if (frame.empty())
				break;
		}

		warpPerspective(frame, image, warpMatrix, rotated.size(), INTER_LINEAR, BORDER_CONSTANT);
		

		//frame.copyTo(image);
		frame.copyTo(image2);

		if (!paused)//û�а���ͣ��
		{
			cvtColor(image, hsv, CV_BGR2HSV);//��rgb����ͷ֡ת����hsv�ռ��
			if (trackObject)//trackObject��ʼ��Ϊ0,���߰�����̵�'c'����ҲΪ0������굥���ɿ���Ϊ-1
			{
				int _vmin = vmin, _vmax = vmax;

				//inRange�����Ĺ����Ǽ����������ÿ��Ԫ�ش�С�Ƿ���2��������ֵ֮�䣬�����ж�ͨ��,mask1����0ͨ������Сֵ��Ҳ����h����
				//����������hsv��3��ͨ�����Ƚ�h,0~180,s,smin~256,v,min(vmin,vmax),max(vmin,vmax)�����3��ͨ�����ڶ�Ӧ�ķ�Χ�ڣ���
				//mask1��Ӧ���Ǹ����ֵȫΪ1(0xff)������Ϊ0(0x00).
				inRange(hsv, Scalar(0, smin, MIN(_vmin, _vmax)),
					Scalar(180, 256, MAX(_vmin, _vmax)), mask1);
				int ch[] = { 0, 0 };
				hue1.create(hsv.size(), hsv.depth());
				hue2.create(hsv.size(), hsv.depth());
				mixChannels(&hsv, 1, &hue1, 1, ch, 1);
				mixChannels(&hsv, 1, &hue2, 1, ch, 1);

				if (trackObject < 0)//���ѡ�������ɿ��󣬸ú����ڲ��ֽ��丳ֵ1
				{
					//ROIָ����Ȥ������
					Mat roi(hue1, selection1), maskroi(mask1, selection1);//mask1�����hsv����Сֵ

					calcHist(&roi, 1, 0, maskroi, hist1, 1, &hsize, &phranges1);//��roi��0ͨ������ֱ��ͼ��ͨ��mask1����hist1�У�hsizeΪÿһάֱ��ͼ�Ĵ�С
					normalize(hist1, hist1, 0, 255, CV_MINMAX);//��hist1����������鷶Χ��һ��������һ����0~255

					trackwindow1 = selection1;
					trackObject = 1;//ֻҪ���ѡ�������ɿ�����û�а�������0��'c'����trackObjectһֱ����Ϊ1����˸�if����ֻ��ִ��һ�Σ���������ѡ���������

					histimg1 = Scalar::all(0);//��0
					int binW = histimg1.cols / hsize;  
					Mat buf(1, hsize, CV_8UC3);
					for (int i = 0; i < hsize; i++)
						buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180. / hsize), 255, 255);
					cvtColor(buf, buf, CV_HSV2BGR);//��hsvת����bgr

					for (int i = 0; i < hsize; i++)
					{
						int val = saturate_cast<int>(hist1.at<float>(i)*histimg1.rows / 255);
						rectangle(histimg1, Point(i*binW, histimg1.rows), Point((i + 1)*binW, histimg1.rows - val), Scalar(buf.at<Vec3b>(i)), -1, 8);
					}

				}
				calcBackProject(&hue1, 1, 0, hist1, backproj1, &phranges1);//����ֱ��ͼ�ķ���ͶӰ������hue1ͼ��0ͨ��ֱ��ͼhist1�ķ���ͶӰ������ֵ��backproj1��
				backproj1 &= mask1;

				trackBox1 = CamShift(backproj1, trackwindow1, TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1));

				if (trackwindow1.area() <= 1)
				{
					int cols = backproj1.cols, rows = backproj1.rows, r = (MIN(cols, rows) + 5) / 6;
					trackwindow1 = Rect(trackwindow1.x - r, trackwindow1.y - r, trackwindow1.x + r, trackwindow1.y + r) &Rect(0, 0, cols, rows);//Rect����Ϊ�����ƫ�ƺʹ�С������һ��������Ϊ��������Ͻǵ����꣬�����ĸ�����Ϊ����Ŀ�͸�
				}

				if (backproj1Mode)
				{
					cvtColor(backproj1, image, CV_GRAY2BGR);//���ͶӰģʽ����ʾ��Ҳ��rgbͼ��
				}
				//cvtColor(backproj1, image, CV_GRAY2BGR);//���ͶӰģʽ����ʾ��Ҳ��rgbͼ��

				ellipse(image, trackBox1, Scalar(0, 0, 255), 3, CV_AA); //������Բ
			}

			if (trackObject >= 2)//trackObject��ʼ��Ϊ0,���߰�����̵�'c'����ҲΪ0������굥���ɿ���Ϊ-1
			{
				int _vmin = vmin, _vmax = vmax;

				//inRange�����Ĺ����Ǽ����������ÿ��Ԫ�ش�С�Ƿ���2��������ֵ֮�䣬�����ж�ͨ��,mask1����0ͨ������Сֵ��Ҳ����h����
				//����������hsv��3��ͨ�����Ƚ�h,0~180,s,smin~256,v,min(vmin,vmax),max(vmin,vmax)�����3��ͨ�����ڶ�Ӧ�ķ�Χ�ڣ���
				//mask1��Ӧ���Ǹ����ֵȫΪ1(0xff)������Ϊ0(0x00).
				inRange(hsv, Scalar(0, smin, MIN(_vmin, _vmax)),
					Scalar(180, 256, MAX(_vmin, _vmax)), mask2);
				int ch[] = { 0, 0 };
				hue1.create(hsv.size(), hsv.depth());//hue1��ʼ��Ϊ��hsv��С���һ���ľ���ɫ���Ķ������ýǶȱ�ʾ�ģ�������֮�����120�ȣ���ɫ���180��
				hue2.create(hsv.size(), hsv.depth());
				mixChannels(&hsv, 1, &hue1, 1, ch, 1);//��hsv��һ��ͨ��(Ҳ����ɫ��)�������Ƶ�hue1�У�0��������
				mixChannels(&hsv, 1, &hue2, 1, ch, 1);

				if (trackObject == 3)
				{
					//�˴��Ĺ��캯��roi�õ���Mat hue1�ľ���ͷ����roi������ָ��ָ��hue1����������ͬ�����ݣ�selectΪ�����Ȥ������

					Mat roi2(hue2, selection2), maskroi(mask2, selection2);//mask1�����hsv����Сֵ
					//calchist1()������һ������Ϊ����������У���2��������ʾ����ľ�����Ŀ����3��������ʾ��������ֱ��ͼά��ͨ�����б���4��������ʾ��ѡ�����뺯��
					//��5��������ʾ���ֱ��ͼ����6��������ʾֱ��ͼ��ά������7������Ϊÿһάֱ��ͼ����Ĵ�С����8������Ϊÿһάֱ��ͼbin�ı߽�
					calcHist(&roi2, 1, 0, maskroi, hist2, 1, &hsize, &phranges2);//��roi��0ͨ������ֱ��ͼ��ͨ��mask1����hist1�У�hsizeΪÿһάֱ��ͼ�Ĵ�С
					normalize(hist2, hist2, 0, 255, CV_MINMAX);//��hist1����������鷶Χ��һ��������һ����0~255

					trackwindow2 = selection2;
					trackObject = 4;//ֻҪ���ѡ�������ɿ�����û�а�������0��'c'����trackObjectһֱ����Ϊ1����˸�if����ֻ��ִ��һ�Σ���������ѡ���������

					histimg2 = Scalar::all(0);//�밴��'c'����һ���ģ������all(0)��ʾ���Ǳ���ȫ����0
					int binW = histimg2.cols / hsize;  //hist1ing��һ��200*300�ľ���hsizeӦ����ÿһ��bin�Ŀ�ȣ�Ҳ����hist1ing�����ֳܷ�����bin����
					Mat buf(1, hsize, CV_8UC3);//����һ�����嵥bin����
					for (int i = 0; i < hsize; i++)//saturate_case����Ϊ��һ����ʼ����׼ȷ�任����һ����ʼ����
						buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180. / hsize), 255, 255);//Vec3bΪ3��charֵ������
					cvtColor(buf, buf, CV_HSV2BGR);//��hsv��ת����bgr

					for (int i = 0; i < hsize; i++)
					{
						//cout << "he" << endl;
						int val = saturate_cast<int>(hist2.at<float>(i)*histimg2.rows / 255);//at����Ϊ����һ��ָ������Ԫ�صĲο�ֵ
						rectangle(histimg2, Point(i*binW, histimg2.rows),    //��һ������ͼ���ϻ�һ���򵥳�ľ��Σ�ָ�����ϽǺ����½ǣ���������ɫ����С�����͵�
							Point((i + 1)*binW, histimg2.rows - val),
							Scalar(buf.at<Vec3b>(i)), -1, 8);
					}

				}
				calcBackProject(&hue2, 1, 0, hist2, backproj2, &phranges2);//����ֱ��ͼ�ķ���ͶӰ������hue1ͼ��0ͨ��ֱ��ͼhist1�ķ���ͶӰ������ֵ��backproj2��	
				backproj2 &= mask2;
				trackBox2 = CamShift(backproj2, trackwindow2, TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1));
				if (  trackwindow2.area() <= 1)
				{
					int cols = backproj2.cols, rows = backproj2.rows, r = (MIN(cols, rows) + 5) / 6;
					trackwindow2 = Rect(trackwindow2.x - r, trackwindow2.y - r, trackwindow2.x + r, trackwindow2.y + r) &Rect(0, 0, cols, rows);//Rect����Ϊ�����ƫ�ƺʹ�С������һ��������Ϊ��������Ͻǵ����꣬�����ĸ�����Ϊ����Ŀ�͸�
				}

				if (backproj1Mode)
				{
					
					cvtColor(backproj2, image2, CV_GRAY2BGR);//���ͶӰģʽ����ʾ��Ҳ��rgbͼ��
					
				}

				ellipse(image, trackBox2, Scalar(0, 0, 255), 3, CV_AA); //������Բ
			}
		}
		else if (trackObject < 0)
			paused = false;
		//�ϳ����ο�
		if (mark1 == false &&  selectObject && selection1.width > 0 && selection1.height > 0)
		{
			Mat roi(image, selection1);
			bitwise_not(roi, roi);//bitwise_notΪ��ÿһ��bitλȡ��

		}
		else if (mark1 == true && selectObject && selection2.width > 0 && selection2.height > 0)
		{
			//cout << "he" << endl;
			Mat roi2(image, selection2);
			bitwise_not(roi2, roi2);//bitwise_notΪ��ÿһ��bitλȡ��
		}

		imshow("CamShift Demo", frame);
		imshow("Rotated", image);
		imshow("histogram", histimg1);
		imshow("histogram2", histimg2);

		char c = (char)waitKey(10);
		if (c == 27)              //�˳���
			break;
		switch (c)
		{
		case 's':
			type = 10;
			break;
		case 'b':             //����ͶӰģ�ͽ���
			backproj1Mode = !backproj1Mode;
			break;
		case 'c':            //�������Ŀ�����
			trackObject = 0;
			histimg1 = Scalar::all(0);
			mark1 = false;
			mark2 = false;
			break;
		case '1':          //��־��־1ѡȡ���
			trackObject=1; 
			mark1 = true;
			break;
		case '2':
			trackObject++;
			mark2 = true;
			break;
		case 'h':          //��ʾֱ��ͼ����
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
		case 'p':          //͸�ӱ���
			type = 0;     
			break;
		default:
			;
		}
	}

	
	return 0;
}