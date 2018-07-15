//#include <highgui.h>  
//#include <imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <highgui.hpp>
#include <core.hpp>
//#include <cv.h>  
//#include <math.h>  
#include <iostream>
#include <fstream>
#include <string>
#include <time.h>
#include <unistd.h>

using namespace std;
using namespace cv;



#define RoadWidthPixel 30
#define	VIDEO_PORT 0
#define StraightValue 90  // the value of x, which will make the car drive straight
#define TOTAL_RUN_TIME 1  // the time this car runs before he stops to detect moving cars
#define DETECT_TIME 1200 // the total time for the car to detect moving cars
#define kMin 0.17
#define kMax 1.8
#define kForEmergency 1
#define TurnDevideValue 10 // the devider to calculate turnValue, devisor is l1, which is 11/12 time frame width 
#define MovingThreshold 500

void myImagePreProcess(IplImage* pCutFrImg, int otsuT, int roadwidth);
void probabilityOfLine(int x1, int y1, int x2, int y2, int imgWidth, double *lweight, double *rweight, int picHalfHeight);
int Otsu(IplImage* src);
void fixSingleLine(int type, double *k, double *x, double k1);
bool stateCheck(IplImage *src, IplImage *grayImage, double *kFinal, double *k1Final, double *distance);
void turnLeft();
void turnRight();
void goStraight();
void stopCar();
int check_moving_object();
void set_pin_mode(string pin_BCM_number, string mode); 
void write_pin(string pin_BCM_number, string val);


//ÀëÉ¢žßË¹·Ö²ŒÊý×é
double Gussian_left[20] = { 1.0000, 0.9802, 0.9231, 0.8353, 0.7261, 0.6065, 0.4868, 0.3753, 0.2780, 0.1979,0.1353,0.0889,0.0561,0.0340,0.0198,0.0111,0.0060,0.0031,0.0015,0.0007 };//10¶Î
double Gussian_right[20] = { 1.0000, 0.9802, 0.9231, 0.8353, 0.7261, 0.6065, 0.4868, 0.3753, 0.2780, 0.1979, 0.1353, 0.0889, 0.0561, 0.0340, 0.0198, 0.0111, 0.0060, 0.0031, 0.0015, 0.0007 };//  0.01, 0.03, 0.034, 0.044, 0.05, 0.058, 0.063, 0.07, 0.077, 0.1,
double Gussian_backup[20] = { 1.0000, 0.9802, 0.9231, 0.8353, 0.7261, 0.6065, 0.4868, 0.3753, 0.2780, 0.1979,0.1353,0.0889,0.0561,0.0340,0.0198,0.0111,0.0060,0.0031,0.0015,0.0007 };//10¶Î
int u1 = 4;//×ó³µµÀ    
int u2 = 15;//ÓÒ³µµÀ
double k1 = 100;//×ó³µµÀ³õÊŒÐ±ÂÊ
double k2 = -100;//ÓÒ³µµÀ³õÊŒÐ±ÂÊ


				 //1·ÅµÃÊÇyÖµÐ¡µÄÄÇžö 2ÊÇyÖµŽóµÄÄÇžö
CvPoint l_line1, l_line2;
CvPoint r_line1, r_line2;

//³µµÀÏßµÄÊµŒÊÏñËØ¿í¶È
int l1;

//  ÕâÁœžö±äÁ¿ÓŠžÃÊÇÓëÃ¿Ò»Ö¡°ó¶š£¬×ÔÊÊÓŠ±ä»¯µÄ

double turnValue;

//ground truth for line of road
CvPoint gr_l_point1, gr_l_point2;
CvPoint gr_r_point1, gr_r_point2;
CvSize frameSize;

// ÉèÖÃ³ßŽç²ÃŒôµôÌì¿Õ
int cutWidth = 1;
//int cutHeight = 197;
int cutHeight = 240;

//car's direction
enum directionState { lEft, rIght, sTraight, bAck, sTop };
enum directionState carS = sTraight;

bool turnAround = false;

int currentX = 90; // ×÷ÎªŽ®¿Ú¿ØÖÆÖžÁîX×ø±êµÄ³õÊŒÖµ
ofstream out("/dev/ttyUSB0");

//int hough()
int main(int argc, char** argv)
{

	VideoCapture capture(VIDEO_PORT);
	//VideoCapture capture("C:\\Users\\mayuankai\\Desktop\\cvTest\\roadTest.wmv"); // caputure from video 	
	//VideoCapture capture("C:\\Users\\mayuankai\\Desktop\\cvTest\\TurnLeft.wmv"); // caputure from video 	
	//VideoCapture capture("C:\\Users\\mayuankai\\Desktop\\cvTest\\TurnRight.wmv"); // caputure from video 	
																				 //VideoCapture capture("C:\\Users\\mayuankai\\Desktop\\cvTest\\roadTest.mp4"); // caputure from video 	
	//VideoCapture capture("C:\\Users\\mayuankai\\Desktop\\cvTest\\roadTest2.mp4"); // caputure from video 	
	//VideoCapture capture("C:\\Users\\mayuankai\\Desktop\\cvTest\\Video.wmv"); // caputure from video 	
	//VideoCapture capture("C:\\Users\\mayuankai\\Desktop\\cvTest\\Video2.mp4");	

	//VideoWriter writer;
	//string outputFile = "C:\\Users\\mayuankai\\Desktop\\cvTest\\outputVideos\\ouputVideo.avi";
	//获得帧的宽高
	//int w = static_cast<int>(capture.get(CV_CAP_PROP_FRAME_WIDTH));
	//int h = static_cast<int>(capture.get(CV_CAP_PROP_FRAME_HEIGHT));
	//Size S(w, h);
	//frequence get	
	//double fps = capture.get(CV_CAP_PROP_FPS);
	//open the video file and prepare to write in 
	//writer.open(outputFile, CV_FOURCC('D', 'I', 'V', 'X'), 30, S, true);
	//writer = VideoWriter(outputFile, CV_FOURCC('F', 'L', 'V', '1'), 30, Size(640, 480));
	//VideoCapture capture(VIDEO_PORT);
	if (!capture.isOpened()) {
		cout << "Capture opens fails" << endl;
		return -1;
	}
	//else if (!writer.isOpened()) {
	//	cout << "Writer opens fails" << endl;
		//	return -1;
	//}

	//capture.release();
	//writer.release();
	//return 0;


	Mat srcFrame;

	//for (int i = 0; i < 10; i++) {
	//	capture >> srcFrame;
	//}


//	bool is_open = true;

	int count = 0;

	IplImage temp;

	Mat imgTemp;
	//, imgToWrite;
	
	static clock_t startTime = clock();
	static clock_t currentTime = clock();
	double runTime = (currentTime - startTime) / CLOCKS_PER_SEC;
	while (runTime < TOTAL_RUN_TIME) {
		capture >> srcFrame;
		if (srcFrame.empty()) {
			cout << "Video finishes!" << endl;
			break;
		}
		//imshow("before", srcFrame);
		GaussianBlur(srcFrame, srcFrame, Size(5, 5), 2, 2, BORDER_DEFAULT);  //高斯滤波
		//imshow("after", srcFrame);
		//waitKey(0);
		temp = (IplImage)IplImage(srcFrame);
		IplImage *pBinary = &temp;
		IplImage *src = cvCloneImage(pBinary);

		cvSetImageROI(src, cvRect(cutWidth, cutHeight, src->width - cutWidth, src->height - cutHeight));
		frameSize = cvGetSize(src);
		cout << frameSize.width << "   " << frameSize.height << endl;

		IplImage *grayImage = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);
		cvCvtColor(src, grayImage, CV_BGR2GRAY);

		l1 = frameSize.width * 11 / 12;//~~~~~~~~~~~~~!!!!!!!!!!!!!!!!!!!!!!!!!!!~~~~~~~~~~~~~~~~~	
		turnValue = l1 / TurnDevideValue;

		double kFinal = 0, k1Final = 0, distance = 0;
		bool lineFound = false;

		lineFound = stateCheck(src, grayImage, &kFinal, &k1Final, &distance); // 当两侧道路线同时找到，返回真，否则返回假
		
																			  //	/******* If we show the images, we may get memory leak
		/*cvNamedWindow("Hough", 1);
		cvShowImage("Hough", src);
		cvWaitKey(0);
		*/

		cvReleaseImage(&grayImage);
		cout << "After stateCheck" << endl;
		if (!turnAround) {
			if (!lineFound) {  // Ã»ÕÒµœ£¬Î¬³ÖÖ±ÐÐ£¬²»¿É×ªÍä
				if (carS != sTraight) {
					goStraight();
					carS = sTraight;
					cout << "Line not found, Keep going straight, no other control" << endl;
				}
				else {
					cout << "Line detected uncompletely, keep going straight" << endl;
				}
			}
			else if (distance > src->width || distance < -src->width) {
				goStraight();
				cout << "No control" << endl;
				if (carS != sTraight) {
					carS = sTraight;
					goStraight();
					cout << "Distance too long, Keep going straight, no other control" << endl;
				}
			}
			else if (distance > turnValue) { // ³µÁŸÔËÐÐ·œÎ»Æ«ÀëÖÐÏß×ó²à³¬³öãÐÖµ
				cout << "state is turning left" << endl;
				if (k1Final >= kForEmergency) {// ÓÒ²àÖ±Ïß¹ý¶ž£¬ËµÃ÷³µ¹ýÓÚ¿¿œüÓÒ²à³µµÀ
					carS = lEft;
					turnLeft();
					cout << "\n\n!!!!!!!!!!!!!!!!!!!!!!!!!!! keep go left   !!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n";
				}
				else if (carS != lEft) {
					carS = lEft;
					cout << "\n\nTurn Left\n" << endl;
					turnLeft();
				}
			}
			else if (distance < -turnValue) {
				cout << "state is turning right" << endl;
				if (kFinal <= -kForEmergency) {
					carS = rIght;
					turnRight();
					cout << "\n\n!!!!!!!!!!!!!!!!!!!!!!!!!! keep go right  !!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n";
				}
				else if (carS != rIght) {
					carS = rIght;
					cout << "\nTurn Right\n" << endl;
					turnRight();
				}


			}
			else { // the distance is not big enough for turning
				goStraight();
				carS = sTraight;
				cout << "Keep going straight" << endl;
			}
		}
		else if (turnAround) {
			cout << "\n\nTurn around!\n" << endl;
		}

		usleep(100000);
		//imgTemp = cvarrToMat(src);
		//imgToWrite = imgTemp.clone();
		//writer << imgToWrite;  // write to the video
		//cout << "Writing finished!" << endl;

		cout << "End of control Analysing next frame" << endl << endl << endl;
	
	
		
		cvReleaseImage(&src);

		/*	for (int i = 0; i < 100; i++)
		{
		cout << "Waiting for next frame count " << i << endl;
		}*/
		
		currentTime = clock();
		runTime = (currentTime - startTime) / CLOCKS_PER_SEC;

	} // end of while
	//cvDestroyWindow("Hough");
	
	//cout << "Before stop" << endl;	
	capture.release();
	//writer.release();
	
	stopCar();

 	startTime = clock();
	currentTime = clock();
	set_pin_mode("4", "out");	

	while ( (((currentTime = clock()) - startTime) / CLOCKS_PER_SEC ) < DETECT_TIME ) {
		int currentMovingValue = check_moving_object();	
		if (currentMovingValue > MovingThreshold) {
			write_pin("4", "0");
			cout << "Moving value is " << currentMovingValue << ". Something is coming fast!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;	
		} else {
			write_pin("4", "1");
			cout << "Moving value is " << currentMovingValue << ". Safe and sound.." << endl;		
		}
		usleep(500000);	
	}
	set_pin_mode("4", "in");
	//system("pause");
	return 0;
}

int Otsu(IplImage* src)  // ·µ»ØµÄÊÇ×îŽóÀàŒä·œ²î¶ÔÓŠµÄ»Ò¶È
{
	int height = src->height;
	int width = src->width;

	//histogram
	float histogram[256] = { 0 };
	for (int i = 0; i < height; i++)
	{
		unsigned char* p = (unsigned char*)src->imageData + src->widthStep * i;
		for (int j = 0; j < width; j++)
		{
			histogram[*p++]++;
		}
	}
	//normalize histogram
	int frameSize = height * width;
	for (int i = 0; i < 256; i++)
	{
		histogram[i] = histogram[i] / frameSize;
	}

	//average pixel value
	float avgValue = 0;
	for (int i = 0; i < 256; i++)
	{
		avgValue += i * histogram[i];  //Õû·ùÍŒÏñµÄÆœŸù»Ò¶È
	}

	int threshold;
	float maxVariance = 0;
	float w = 0, u = 0;
	for (int i = 0; i < 256; i++)
	{
		w += histogram[i];  //ŒÙÉèµ±Ç°»Ò¶ÈiÎªãÐÖµ, 0~i »Ò¶ÈµÄÏñËØ(ŒÙÉèÏñËØÖµÔÚŽË·¶Î§µÄÏñËØœÐ×öÇ°Ÿ°ÏñËØ) ËùÕŒÕû·ùÍŒÏñµÄ±ÈÀý
		u += i * histogram[i];  // »Ò¶Èi Ö®Ç°µÄÏñËØ(0~i)µÄÆœŸù»Ò¶ÈÖµ£º Ç°Ÿ°ÏñËØµÄÆœŸù»Ò¶ÈÖµ

		float t = avgValue * w - u;
		float variance = t * t / (w * (1 - w));
		if (variance > maxVariance)
		{
			maxVariance = variance;   // ÕÒµœ×îŽó·œ²î
			threshold = i;              // ÕÒµœ×îŽó·œ²î¶ÔÓŠµÄµ±Ç°»Ò¶È
		}
	}

	return threshold;
}



void myImagePreProcess(IplImage* pCutFrImg, int otsuT, int roadwidth) {
	//uchar* data = (uchar *)pCutFrImg->imageData;
	//int step = pCutFrImg->widthStep / sizeof(uchar);
	int T = 0.5* otsuT;
	uchar* tmp;
	int l = roadwidth;//³µµÀÏñËØ¿í¶È
	int c = 0;
	int N = pCutFrImg->height;
	int M = pCutFrImg->width;
	double mean1 = 0, mean2 = 0;//mean1 for pi   mean2 for pj
	for (int y = 0; y<N; y++)
		for (int x = 0; x < M; x++)
		{
			mean1 = 0; mean2 = 0;
			tmp = &((uchar *)(pCutFrImg->imageData + y * pCutFrImg->widthStep))[x];
			//tmp = cvGet2D(pCutFrImg, y, x).val[0];
			c = y * 0.5*l / (M - 1) + 0.5*l;

			if (y<N - c && y>c) {
				for (int i = y - c; i < y; i++) {//ŒÆËãmean(V(x,pi))
					if (i<N && i >= 0)
						mean1 += ((uchar *)(pCutFrImg->imageData + i * pCutFrImg->widthStep))[x];//cvGet2D(pCutFrImg, i, x).val[0];
				}
				mean1 /= c;
				for (int i = y; i < y + c; i++) {//ŒÆËãmean(V(x,pj))
					if (i<N && i >= 0)
						mean2 += ((uchar *)(pCutFrImg->imageData + i * pCutFrImg->widthStep))[x];//cvGet2D(pCutFrImg, i, x).val[0];
				}
				mean2 /= c;
				if (*tmp>mean1 + T && *tmp>mean2 + T) {
					*tmp = 255;
				}
				else
				{
					*tmp = 0;
				}
			}
			else if (y >= N - c) {
				for (int i = y - c; i < y; i++) {//ŒÆËãmean(V(x,pi))
					if (i<N && i >= 0)
						mean1 += ((uchar *)(pCutFrImg->imageData + i * pCutFrImg->widthStep))[x];//cvGet2D(pCutFrImg, i, x).val[0];
				}
				mean1 /= c;
				if (*tmp>mean1 + T) {
					*tmp = 255;
				}
				else
				{
					*tmp = 0;
				}
			}
			else
			{
				for (int i = y; i < y + c; i++) {//ŒÆËãmean(V(x,pj))
					if (i<N && i >= 0)
						mean2 += ((uchar *)(pCutFrImg->imageData + i * pCutFrImg->widthStep))[x];//cvGet2D(pCutFrImg, i, x).val[0];
				}
				mean2 /= c;
				if (*tmp>mean2 + T) {
					*tmp = 255;
				}
				else
				{
					*tmp = 0;
				}
			}
		}
}

// ÊÊÓŠÐÔ²éÕÒÈë¿ÚËã·šÖÐÊ¹ÓÃŽËº¯Êý ŒÆËã ÏßÊÇµÀÂ·ÏßµÄžÅÂÊ
void probabilityOfLine(int x1, int y1, int x2, int y2, int imgWidth, double *lweight, double *rweight, int picHalfHeight) {

	double k = ((y1 - y2)*1.0 / (x1 - x2));
	double b = y1 - k * x1;
	double b1 = (picHalfHeight - b) / k;    //y==half heightÊ±£¬xµÄ×ø±ê£¬ÓÃÓÚÈšÖµÅÐ¶Ï
	double maxX = x1 > x2 ? x1 : x2;
	double minX = x1 < x2 ? x1 : x2;
	int margin;

	b1 = b1 > 0 ? b1 : -b1;
	int d = (int)(b1 / imgWidth * 20);


	if (k<-kMin && k>-kMax) {
		*rweight = 0;
		margin = d > u1 ? d - u1 : u1 - d;
		if (margin > 19) {
			margin = 19;
		}
		*lweight = Gussian_left[margin];
		Gussian_left[margin] = 0;
		return;
	}
	else {
		*lweight = 0;
	}


	if (k>kMin && k<kMax) {
		*lweight = 0;
		margin = d > u2 ? d - u2 : u2 - d;
		if (margin>19) {
			margin = 19;
		}
		*rweight = Gussian_right[margin];
		Gussian_right[margin] = 0;
	}
	else {
		*rweight = 0;
	}

}



// ÐÞÕýÂ·Ïß typeÎª0 ±íÊŸ×ó±ßµÄÏß£¬ typeÎª1 ±íÊŸÓÒ±ßµÄÏß
void fixSingleLine(int type, double *k, double *x, double k1) {//type ==0 , know left line;  type==1 , know right line
															   //ground truth of right line
	double krt = (gr_r_point1.y*1.0 - gr_r_point2.y) / (gr_r_point1.x*1.0 - gr_r_point2.x);  //ground truth's k
																							 //double brt = gr_r_point1.y - krt * gr_r_point1.x;
																							 //ground truth of left line
	double klt = (gr_l_point1.y*1.0 - gr_l_point2.y) / (gr_l_point1.x*1.0 - gr_l_point2.x);  //ground truth's k
																							 //double blt = gr_l_point1.y - klt * gr_l_point1.x;

	if (type == 1) {//detected right line

					//calculate the rotated angle
		double theate = -(atan(krt) - atan(k1));
		//translation to origin
		double tempx = gr_l_point2.x - gr_l_point1.x;
		double tempy = gr_l_point2.y - gr_l_point1.y;
		//translate
		double transx = tempx * cos(theate) - tempy * sin(theate);
		double transy = tempx * sin(theate) + tempy * cos(theate);
		//translation back to what it was
		double backx = transx + gr_l_point1.x;
		double backy = transy + gr_l_point1.y;
		//result
		*k = (backy - gr_l_point1.y) / (backx - gr_l_point1.x);
		double tempb = gr_l_point1.y - *k * gr_l_point1.x;
		*x = (frameSize.height / 2 - tempb) / *k;
	}
	else if (type == 0) {//detected left line

						 //calculate the rotated angle
		double theate = -(atan(klt) - atan(k1));
		//translation to origin
		double tempx = gr_r_point2.x - gr_r_point1.x;
		double tempy = gr_r_point2.y - gr_r_point1.y;
		//translate
		double transx = tempx * cos(theate) - tempy * sin(theate);
		double transy = tempx * sin(theate) + tempy * cos(theate);
		//translation back to what it was
		double backx = transx + gr_r_point1.x;
		double backy = transy + gr_r_point1.y;
		//result
		*k = (backy - gr_r_point1.y) / (backx - gr_r_point1.x);
		double tempb = gr_r_point1.y - *k * gr_r_point1.x;	// ÍŒÏñÔ€ŽŠÀí

		*x = (frameSize.height / 2 - tempb) / *k;
	}
}

bool stateCheck(IplImage *src, IplImage *grayImage, double *kFinal, double *k1Final, double *distance) {
	/*	// 灰度化
	IplImage *grayImage = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);
	cvCvtColor(src, grayImage, CV_BGR2GRAY);
	*/


	// 使用大津算法计算对称阈值切割的阈值
	int otsuThreshold = Otsu(grayImage);

	// 创建结果图像
	IplImage* dst = cvCreateImage(cvGetSize(grayImage), IPL_DEPTH_8U, 1);
	CvMemStorage *storage = cvCreateMemStorage(0);
	CvSeq *lines = 0;

	//myImagePreProcess(grayImage, otsuThreshold, RoadWidthPixel);
	//cvThreshold(grayImage, grayImage, 159, 255, CV_THRESH_BINARY_INV); // 大于阈值设为0(黑色)，否则为255(白色) 
	cvCanny(grayImage, dst, 30, 45, 3);  //首先运行边缘检测，结果以灰度图显示（只有边缘）   两个阈值，大于大的认为是边缘，低于小的则抛弃像素，介于两者中间则如果像素点和边缘连接则保留，否则删除

										   //cvNamedWindow("afterCanny", 2);
										   //cvShowImage("afterCanny", dst);
										   //cvWaitKey(0);
	/*
	cvNamedWindow("cannyResult", 1);
	cvShowImage("cannyResult", dst);
	cvNamedWindow("grayImage", 1);
	cvShowImage("grayImage", grayImage);
	cvWaitKey(0);
	*/
	
	lines = cvHoughLines2(dst, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI / 180, 50, 20, 5); // 距离精度， 角度精度， 阈值参数， 最小线段长度， 碎线段最大间隔值
																							// the last three parameter could try 50, 80, 5 or 50, 25, 5
																						//	cout << "Total lines amount is " << lines->total << endl;

	double bLeftMax = -1000, bRightMax = 1000;
	bool leftFound = false, rightFound = false;
	//循环直线序列  
	// 处理一帧图像中所有的直线，得到两条道路线的方程（k和b）
	for (int i = 0; i < lines->total; i++)  //lines存储的是直线  
	{
		CvPoint* line = (CvPoint*)cvGetSeqElem(lines, i);  //lines序列里面存储的是像素点坐标  

		double k = ((line[0].y - line[1].y)*1.0 / (line[0].x - line[1].x));
		double lweight = 0;
		double rweight = 0;
		double l_total_weight = 0;
		double r_total_weight = 0;
		double maxLP = 0;
		double maxRP = 0;
		int l_num = 0;
		int r_num = 0;

		// 计算这条线段是车道线的概率
		probabilityOfLine(line[0].x, line[0].y, line[1].x, line[1].y, frameSize.width, &lweight, &rweight, frameSize.height / 2);

		//画出直线
		l_line1 = cvPoint(0, 0);//1 is the smaller one, 2 is the bigger one
		l_line2 = cvPoint(0, 0);
		r_line1 = cvPoint(0, 0);
		r_line2 = cvPoint(0, 0);

		if (lweight>0) {
			//cvLine(pCutFrame, line[0], line[1], CV_RGB(255, 0, 0), 6, CV_AA);
			if (maxLP<lweight)
				maxLP = lweight;
			l_total_weight += lweight;
			l_num++;
			if (line[0].y < line[1].y) {
				l_line1.y += line[0].y *lweight;
				l_line1.x += line[0].x *lweight;
				l_line2.y += line[1].y *lweight;
				l_line2.x += line[1].x *lweight;
			}
			else
			{
				l_line1.y += line[1].y *lweight;
				l_line1.x += line[1].x *lweight;
				l_line2.y += line[0].y *lweight;
				l_line2.x += line[0].x *lweight;
			}
		}
		else if (rweight>0) {
			if (maxRP < rweight)
				maxRP = rweight;
			r_total_weight += rweight;
			r_num++;
			if (line[0].y < line[1].y) {
				r_line1.y += line[0].y *rweight;
				r_line1.x += line[0].x *rweight;
				r_line2.y += line[1].y *rweight;
				r_line2.x += line[1].x *rweight;
			}
			else {
				r_line1.y += line[1].y *rweight;
				r_line1.x += line[1].x *rweight;
				r_line1.y += line[0].y *rweight;
				r_line1.x += line[0].x *rweight;
			}
		}


		if (l_total_weight > 0) {//归一化
			l_line1.x /= (l_total_weight);
			l_line1.y /= (l_total_weight);
			l_line2.x /= (l_total_weight);
			l_line2.y /= (l_total_weight);
		}
		if (r_total_weight>0) {//归一化
			r_line1.x /= (r_total_weight);
			r_line1.y /= (r_total_weight);
			r_line2.x /= (r_total_weight);
			r_line2.y /= (r_total_weight);
		}

		/*/begin------------------------------------------ for  test
		if (l_total_weight > 0)
		cvLine(src, l_line1, l_line2, CV_RGB(255, 0, 0), 10, CV_AA);
		if (r_total_weight > 0)
		cvLine(src, r_line1, r_line2, CV_RGB(255, 0, 0), 10, CV_AA);
		//end------------------------------------------ for  test */

		//cvLine(src, line[0], line[1], CV_RGB(255, 0, 0));  //将找到的直线标记为绿色  
		//color_dst是三通道图像用来存直线图像  



		// Gussian_weight 高斯加权道路线补齐
		//re-initialize the gussian-array
		for (int e = 0; e<20; e++) {
			Gussian_right[e] = Gussian_backup[e];
			Gussian_left[e] = Gussian_backup[e];
		}
		//-----------------------------------------

		bool f1 = false, f2 = false; // 表示左右两个直线是否找到
		double k1 = 0; // 左直线斜率 k 右直线斜率 k1
		double b, b1;  // 两个直线方程的b值
		double x01 = 0, x02 = 0, centerx = 0;
		double distance = 0;


		//fittable parameter for control the car 
		int adjustValue = l1 * 3 / 2;
		int axisValue = l1 * 5 / 16;
		int abanValue = l1 / 4;
		double kValue = 0.2;

		//calculate line that detected
		if (l_total_weight>0) {
			k = (l_line1.y *(1.0) - l_line2.y) / (l_line1.x *(1.0) - l_line2.x);  // 求出左边直线的斜率
			b = l_line1.y *1.0 - k * l_line1.x; // b = y - kx 
			x01 = (frameSize.height / 2 - b) / k;  // x01 是图像水平方向的中线（从height的中点向右平行于width方向）与左直线交点的横坐标
			f1 = true; // 找到左直线，下面右直线操作同理 
					   //			cout << "line number " << i << "finds left line, k and b are " << k << ", " << b << endl;
		}
		if (r_total_weight>0) {
			k1 = (r_line1.y *(1.0) - r_line2.y) / (r_line1.x *(1.0) - r_line2.x);  // 右直线的斜率是k1
			b1 = r_line1.y *1.0 - k1 * r_line1.x;
			x02 = (frameSize.height / 2 - b1) / k1;
			f2 = true;
			//			cout << "line number " << i << "finds right line, k1 and b1 are " << k1 << ", " << b1 << endl;
		}

		//		cout << "Line number " << i << ", k and k1 are " << k << " " << k1 << endl;

		//to make two line
		if (!f1) {  // 如果未找到左直线
			if (f2) {  // 但是找到了右直线
					   //x01 = x02 - adjustValue; 用的是梯形中位线性质

					   //				printf("1\n");
				rightFound = true;
				fixSingleLine(1, &k, &x01, k1);
				b = frameSize.height / 2 - k * x01;
				//centerx = (b1 - b) / (k - k1);

				//distance = (frameSize.width / 2 - centerx);   // centerx 和 distance的含义不太理解
				//cout << "1 frameSize.width = " << frameSize.width << " centerx = " << centerx << endl;
			}
			else {
				//go straight
				//lineFound = false; // 两个直线都没有找到
				//				printf("2\n");
			}
		}
		else {  // 找到了左直线
			leftFound = true;
			if (!f2) {  // 但是没找到右直线
						//x02 = x01 + adjustValue;
						//k1=-k;
						//				printf("3\n");

				fixSingleLine(0, &k1, &x02, k);
				b1 = frameSize.height / 2 - k1 * x02;
				//centerx = (b1 - b) / (k - k1);

				//distance = (frameSize.width / 2 - centerx);
				//cout << "3, frameSize.width = " << frameSize.width << " centerx = " << centerx << endl;
			}
			else { // 左右两个直线都找到了
				   //				printf("4\n");
				rightFound = true;
				if (x02 - x01 - l1 > axisValue || x02 - x01 - l1 < -axisValue) {// use probability to adjust when two line detected
					if (maxLP > maxRP) {//l_total_weight/l_num > r_total_weight/r_num
						x02 = x01 + adjustValue;
						k1 = -k;
					}
					else {
						x01 = x02 - adjustValue;
						k = -k1;
					}
				}
				//centerx = (x01 + x02) / 2;
				//distance = (frameSize.width / 2 - centerx);
				//cout << "4 frameSize.width = " << frameSize.width << " centerx = " << centerx << endl;
			}
		} // end of else

		  //-----------------------------------------for test


		if (k == 0 || k1 == 0) {
			//				cout << "k or k1 is 0" << endl
			continue;
		}
		else {
			//				printf("------%f-------%d---------\n", centerx, frameSize.width / 2);
			//				cvLine(src, cvPoint(centerx, 100), cvPoint(frameSize.width / 2, 100), CV_RGB(255, 0, 0), 3, CV_AA);  // 画面中线

			b = frameSize.height / 2 - k * x01;
			b1 = frameSize.height / 2 - k1 * x02;
			//				cout << "IN" << endl;
			if (bLeftMax < b) {
				bLeftMax = b;
				*kFinal = k;
				//					cout << "LINE FOUND! b and k update to " << b << " " << k << endl << endl;
			}
			if (bRightMax > b1) {
				bRightMax = b1;
				*k1Final = k1;
				//					cout << "LINE FOUND! b1 and k1 update to " << b1 << " " << k1 << endl << endl;
			}

		}
	} // endl of for 

	cvReleaseImage(&dst);
	cvReleaseMemStorage(&storage);
	  	cout << "kFinal and k1Final are " << *kFinal << " " << *k1Final << endl;
	  	printf("(0, bLeftMax) (-bLeftMax/kFinal, 0) is (0, %f) (%f, 0)\n\n", bLeftMax, -bLeftMax / *kFinal);
	  	printf("(0, bRightMax) (-bRightMax/k1Final, 0) is (0, %f) (%f, 0)", bRightMax, -bRightMax / *k1Final);


	if (!leftFound || !rightFound) { // 只找到一条时，再去判断斜率，决定是否要控制
		cout << "Not found both at the same time\n\n";
		//return false;
	}

	double x1 = 0, x2 = 0, x3 = 0; // x3 is the intersection between the line and the top horizontal line
	
	/*
		如果只找到一条直线，那么就看这条直线和最上方的横线交点偏左还是偏右，结合直线的位置来判断车辆位置状态
	*/
	if (*kFinal == 0 && *k1Final != 0) {
		x1 = 0;
		x2 = (frameSize.height / 2 - bRightMax) / *k1Final;
		x3 = (frameSize.height - bRightMax) / *k1Final;
		cvLine(src, cvPoint(0, 0), cvPoint(0, src->height), CV_RGB(150, 10, 10), 5, CV_AA);  // 左直线
		cvLine(src, cvPoint(0, (int)bRightMax), cvPoint(src->width, *k1Final * src->width + bRightMax), CV_RGB(150, 10, 10), 5, CV_AA);  // 右直线
	//	cvLine(src, cvPoint(0, frameSize.height / 2), cvPoint(frameSize.width, frameSize.height / 2), CV_RGB(150, 10, 10), 5, CV_AA);  // 中心线
		if (x3 < frameSize.width / 2) { // 此时需要左转
			cout << "detect one line, according to the intersection point, now we turn left" << endl;
			*distance = turnValue + 1;
		}
		else {
			cout << "detect one line, according to the intersection point, now we stay straight" << endl;
			*distance = 0;
		}

	}
	else if (*kFinal != 0 && *k1Final == 0) {
		x1 = (frameSize.height / 2 - bLeftMax) / *kFinal;
		x2 = src->width;
		x3 = (frameSize.height - bLeftMax) / *kFinal;
		cvLine(src, cvPoint(0, (int)bLeftMax), cvPoint((int)-bLeftMax / *kFinal, 0), CV_RGB(150, 10, 10), 5, CV_AA);  // 左直线
		cvLine(src, cvPoint(x2, 0), cvPoint(x2, src->height), CV_RGB(150, 10, 10), 5, CV_AA);  // 右直线
	//	cvLine(src, cvPoint(0, frameSize.height / 2), cvPoint(frameSize.width, frameSize.height / 2), CV_RGB(150, 10, 10), 5, CV_AA);  // 中心线	
		
		if (x3 < frameSize.width / 2) { // 此时需要右转
			cout << "detect one line, according to the intersection point, now we turn right" << endl;
			*distance = -turnValue - 1;
		}
		else {
			cout << "detect one line, according to the intersection point, now we stay straight" << endl;
			*distance = 0;
		}
	
	}
	else {
		x1 = (frameSize.height / 2 - bLeftMax) / *kFinal;   // 中位线左交点
		x2 = (frameSize.height / 2 - bRightMax) / *k1Final;  // 右交点
		cvLine(src, cvPoint(0, (int)bLeftMax), cvPoint((int)-bLeftMax / *kFinal, 0), CV_RGB(150, 10, 10), 5, CV_AA);  // 左直线
		cvLine(src, cvPoint(0, (int)bRightMax), cvPoint(src->width, *k1Final * src->width + bRightMax), CV_RGB(150, 10, 10), 5, CV_AA);  // 右直线
	//	cvLine(src, cvPoint(0, frameSize.height / 2), cvPoint(frameSize.width, frameSize.height / 2), CV_RGB(150, 10, 10), 5, CV_AA);  // 中心线	
		double centerX = (x1 + x2) / 2;   // 中位线的中点 即道路中央
		*distance = (frameSize.width / 2 - centerX);  // distance = 车的视线中点减去道路线梯形中位线中点 为正说明航线偏右要左拐 为负则右拐
													  //cout << "x1, x2, centerX, frameSize.width / 2, distance: " << x1 << " " << x2 << " " << centerX << " " << frameSize.width / 2 << " " << *distance << endl;
		cout << "k and k1 are " << *kFinal << ", " << *k1Final << ", distance is " << *distance << ", turnValue is " << turnValue << endl;
	
	}

	

	
	
	return true;

}

void turnLeft() {
	if (currentX > StraightValue)
	currentX = StraightValue;
	if (currentX > 40) {
		currentX -= 30;
		cout << "Turning left!! current X is" << currentX << endl;
	}
	else {
		cout << "Keep turning left, X stays " << currentX << endl;
	}

	string output = "$AP0:", tempX = to_string(currentX);
	output = output + tempX + "X254Y127A127B!";
	out << output << endl;
}

void turnRight() {
	if (currentX < StraightValue)
		currentX = StraightValue;
	if (currentX < 140) {
		currentX += 30;
		cout << "Turning Right!! current X is" << currentX << endl;
	}
	else {
		cout << "Keep turning right, X stays " << currentX << endl;
	}
	string output = "$AP0:", tempX = to_string(currentX);
	output = output + tempX + "X254Y127A127B!";
	out << output << endl;
}

void goStraight() {
	currentX = StraightValue;
	string tempX = to_string(currentX);
	string output = "$AP0:" + tempX + "X254Y127A127B!";
	out << output << endl;
	cout << "Go straight, X is " << currentX << endl;
}

void stopCar() {
	string output = "$AP0:127X127Y127A127B!";
	for (int i = 0; i < 3; i++) {
		out << output << endl;
		usleep(10000);	
	}
	cout << "The car has stopped and detect moving things" << endl;
}


// Before using this function, we have to close the camera first
int check_moving_object() { 
	int activate_frames = 50; //相机启动延时
    int delay_frames = 5;      //两帧之间的延迟 ms


    int diffVal = 0; //最终白色像素个数
    VideoCapture capture(0);
    Mat frame1;
    Mat frame2;
    //recordTime("begin");
    if(!capture.isOpened())
    {
        cout<<"摄像头打开失败！"<<endl;
        return -1;
    }
    capture >> frame1;
    //sleep(3); //启动延时 200ms


    for (int k = 0; k < activate_frames; k++) {
        capture >> frame1;
    }

    for (int k = 0; k < delay_frames; k++) {
        capture >> frame2;
    }
    capture.release();//关闭摄像头



    string basepath = "/home/myk/workspace_szh/mid/";
    imwrite(basepath + "input1.jpg", frame1);//图片保存到本工程目录中
    imwrite(basepath + "input2.jpg", frame2);//图片保存到本工程目录中
    FILE *input;
    char buf[1000];
    if((input=popen("python /home/myk/workspace_szh/py_src/checkCar.py", "r"))!=NULL)
    {
         while(fgets(buf, 100, input))//getline 获取一行数据
         {
            diffVal = atoi(buf);
            //printf("%d", diffVal);
         }
         pclose(input);
    }

    return diffVal;
   // popen("python /home/myk/workspace_szh/py_src/checkCar.py", "r")
}


void write_pin(string pin_BCM_number, string val) {
    char buf[5];
    system(("gpio -g write " + pin_BCM_number+ " " + val).c_str());
}


void set_pin_mode(string pin_BCM_number, string mode) {
    char buf[5];
    system(("gpio -g mode " + pin_BCM_number + " " + mode).c_str());
}



