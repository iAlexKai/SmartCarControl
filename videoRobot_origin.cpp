//#include <highgui.h>  
//#include <imgproc.hpp>
#include <highgui.hpp>
#include <core.hpp>
//#include <cv.h>  
//#include <math.h>  
#include <iostream>
#include <fstream>
#include <string>
using namespace std;
using namespace cv;



#define RoadWidthPixel 30
#define	VIDEO_PORT 0

void myImagePreProcess(IplImage* pCutFrImg, int otsuT, int roadwidth);
void probabilityOfLine(int x1, int y1, int x2, int y2, int imgWidth, double *lweight, double *rweight, int picHalfHeight);
int Otsu(IplImage* src);
void fixSingleLine(int type, double *k, double *x, double k1);
bool stateCheck(IplImage *src, double *kFinal, double *k1Final, double *distance);
void turnLeft();
void turnRight();


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
double kForEmergency = 0.9;
double turnValue = l1 / 10;

//ground truth for line of road
CvPoint gr_l_point1, gr_l_point2;
CvPoint gr_r_point1, gr_r_point2;
CvSize frameSize;

// ÉèÖÃ³ßŽç²ÃŒôµôÌì¿Õ
int cutWidth = 1;
int cutHeight = 400;

//car's direction
enum directionState { lEft, rIght, sTraight, bAck, sTop };
enum directionState carS = sTraight;

bool turnAround = false;

int currentX = 90; // ×÷ÎªŽ®¿Ú¿ØÖÆÖžÁîX×ø±êµÄ³õÊŒÖµ
ofstream out("/dev/ttyUSB0");

//int hough()
int main(int argc, char** argv)
{
	//VideoCapture capture(VIDEO_PORT);
	VideoCapture cap("roadTest.avi"); // caputure from video 	
	if (!capture.isOpened()) {
		cout << "nani??" << endl;
		return -1;
	}
	if (out.is_open()) { // ÍšÏòÊ÷Ý®ÅÉµÄŽ®¿ÚÊä³öŽò¿ª
		Mat srcFrame;
	
		for (int i = 0; i < 10; i++) {
			capture >> srcFrame;						
		}		


		bool is_open = true;
		while (is_open) {
			capture >> srcFrame;
			IplImage temp = (IplImage)IplImage(srcFrame);			
			IplImage *pBinary = &temp;
			IplImage *src = cvCloneImage(pBinary);

			//src = cvLoadImage("road8.jpg");
			//src = cvLoadImage("fig1.png");
						

			cout << "src µÄÁœžöÊýŸÝÊÇ" << src->width << "  " << src->height << endl;
			

			cvSetImageROI(src, cvRect(cutWidth, cutHeight, src->width - cutWidth, src->height - cutHeight));
			frameSize = cvGetSize(src);
			cout << frameSize.width << "   " << frameSize.height << endl;


			//ŒÆËãl1 ³µµÀµÄÊµŒÊ¿í¶È
			l1 = frameSize.width * 11 / 12;//~~~~~~~~~~~~~!!!!!!!!!!!!!!!!!!!!!!!!!!!~~~~~~~~~~~~~~~~~	


			double kFinal = 0, k1Final = 0, distance = 0;
			bool lineFound = false;
			lineFound = stateCheck(src, &kFinal, &k1Final, &distance); // µ±Áœ²àµÀÂ·ÏßÍ¬Ê±ÕÒµœ£¬·µ»ØÕæ£¬·ñÔò·µ»ØŒÙ
																	   // œÓÏÂÀŽœøÐÐ¿ØÖÆ
			cout << "After stateCheck" << endl;
			if (!turnAround) {
				if (!lineFound) {  // Ã»ÕÒµœ£¬Î¬³ÖÖ±ÐÐ£¬²»¿É×ªÍä
					if (carS != sTraight) {
						//cGoStraight();
						//cSetSpeed(1);
						carS = sTraight;
						cout << "Keep going straight" << endl;
					}
				}
				else if (distance > turnValue) { // ³µÁŸÔËÐÐ·œÎ»Æ«ÀëÖÐÏß×ó²à³¬³öãÐÖµ
					cout << "state is turning left" << endl;											
					if (carS != lEft) {
						carS = lEft;
						cout << "Turn Left" << endl;
						turnLeft();
						
					}
					if (k1Final >= kForEmergency) {// ÓÒ²àÖ±Ïß¹ý¶ž£¬ËµÃ÷³µ¹ýÓÚ¿¿œüÓÒ²à³µµÀ
						cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!! keep go left   !!!!!!!!!!!!!!!!!!!!!!!!!!!";
					}
				}
				else if (distance < -turnValue) {
					cout << "state is turning right" << endl;						
					if (carS != rIght) {
						carS = rIght;
						cout << "Turn Right" << endl;
						turnRight();
					}
					
					if (kFinal <= -kForEmergency) {
						cout << "!!!!!!!!!!!!!!!!!!!!!!!!!! keep go right  !!!!!!!!!!!!!!!!!!!!!!!!!!!";
					}
				}
				else if (carS != sTraight) { // °ÚÕý³µÍ·
					carS = sTraight;
					cout << "Keep going straight" << endl;
				}

				// É²³µ 

			}
			else if (turnAround) {
				cout << "Turn around!" << endl;
			}
			
			cout << "end of while" << endl;
			//cvNamedWindow("Hough", 1);
			//cvShowImage("Hough", src);
			//cvWaitKey(0);
			
		/*	for (int i = 0; i < 100; i++)
			{
				cout << "Waiting for next frame count " << i << endl;
			}*/
		} // end of while
	}
	
	
	
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


	if (k<-0.17 && k>-2) {
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


	if (k>0.17 && k<2) {
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

bool stateCheck(IplImage *src, double *kFinal, double *k1Final, double *distance) {
	// »Ò¶È»¯
	IplImage *grayImage = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);
	cvCvtColor(src, grayImage, CV_BGR2GRAY);

	// Ê¹ÓÃŽóœòËã·šŒÆËã¶Ô³ÆãÐÖµÇÐžîµÄãÐÖµ
	int otsuThreshold = Otsu(grayImage);

	// ŽŽœšœá¹ûÍŒÏñ
	IplImage* dst = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);
	CvMemStorage *storage = cvCreateMemStorage(0);
	CvSeq *lines = 0;

	myImagePreProcess(grayImage, otsuThreshold, RoadWidthPixel);
	cvCanny(src, dst, 300, 500, 3);  //Ê×ÏÈÔËÐÐ±ßÔµŒì²â£¬œá¹ûÒÔ»Ò¶ÈÍŒÏÔÊŸ£šÖ»ÓÐ±ßÔµ£©   ÁœžöãÐÖµ£¬Ð¡µÄ¿ØÖÆ±ßÔµÁ¬œÓ£¬ŽóµÄ¿ØÖÆÇ¿±ßÔµµÄ³õÊŒ·Öžî

	lines = cvHoughLines2(dst, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI / 180, 50, 30, 5); // ŸàÀëŸ«¶È£¬ œÇ¶ÈŸ«¶È£¬ ãÐÖµ²ÎÊý£¬ ×îÐ¡Ïß¶Î³€¶È£¬ ËéÏß¶Î×îŽóŒäžôÖµ

	double bLeftMax = 0, bRightMax = 0;

	bool lineFound = true;
	//Ñ­»·Ö±ÏßÐòÁÐ  
	// ŽŠÀíÒ»Ö¡ÍŒÏñÖÐËùÓÐµÄÖ±Ïß£¬µÃµœÁœÌõµÀÂ·ÏßµÄ·œ³Ì£škºÍb£©
	for (int i = 0; i < lines->total; i++)  //linesŽæŽ¢µÄÊÇÖ±Ïß  
	{
		CvPoint* line = (CvPoint*)cvGetSeqElem(lines, i);  //linesÐòÁÐÀïÃæŽæŽ¢µÄÊÇÏñËØµã×ø±ê  

		double k = ((line[0].y - line[1].y)*1.0 / (line[0].x - line[1].x));
		double lweight = 0;
		double rweight = 0;
		double l_total_weight = 0;
		double r_total_weight = 0;
		double maxLP = 0;
		double maxRP = 0;
		int l_num = 0;
		int r_num = 0;

		// ŒÆËãÕâÌõÏß¶ÎÊÇ³µµÀÏßµÄžÅÂÊ
		probabilityOfLine(line[0].x, line[0].y, line[1].x, line[1].y, frameSize.width, &lweight, &rweight, frameSize.height / 2);

		//»­³öÖ±Ïß
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


		if (l_total_weight > 0) {//¹éÒ»»¯
			l_line1.x /= (l_total_weight);
			l_line1.y /= (l_total_weight);
			l_line2.x /= (l_total_weight);
			l_line2.y /= (l_total_weight);
		}
		if (r_total_weight>0) {//¹éÒ»»¯
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

		//cvLine(src, line[0], line[1], CV_RGB(255, 0, 0));  //œ«ÕÒµœµÄÖ±Ïß±êŒÇÎªÂÌÉ«  
		//color_dstÊÇÈýÍšµÀÍŒÏñÓÃÀŽŽæÖ±ÏßÍŒÏñ  



		// Gussian_weight žßË¹ŒÓÈšµÀÂ·Ïß²¹Æë
		//re-initialize the gussian-array
		for (int e = 0; e<20; e++) {
			Gussian_right[e] = Gussian_backup[e];
			Gussian_left[e] = Gussian_backup[e];
		}
		//-----------------------------------------

		bool f1 = false, f2 = false; // ±íÊŸ×óÓÒÁœžöÖ±ÏßÊÇ·ñÕÒµœ
		double k1 = 0; // ×óÖ±ÏßÐ±ÂÊ k ÓÒÖ±ÏßÐ±ÂÊ k1
		double b, b1;  // ÁœžöÖ±Ïß·œ³ÌµÄbÖµ
		double x01 = 0, x02 = 0, centerx = 0;
		double distance = 0;


		//fittable parameter for control the car 
		int turnValue = l1 / 10;
		int adjustValue = l1 * 3 / 2;
		int axisValue = l1 * 5 / 16;
		int abanValue = l1 / 4;
		double kValue = 0.2;
		double kForEmergency = 0.9;

		//calculate line that detected
		if (l_total_weight>0) {
			k = (l_line1.y *(1.0) - l_line2.y) / (l_line1.x *(1.0) - l_line2.x);  // Çó³ö×ó±ßÖ±ÏßµÄÐ±ÂÊ
			b = l_line1.y *1.0 - k * l_line1.x; // b = y - kx 
			x01 = (frameSize.height / 2 - b) / k;  // x01 ÊÇÍŒÏñË®Æœ·œÏòµÄÖÐÏß£šŽÓheightµÄÖÐµãÏòÓÒÆœÐÐÓÚwidth·œÏò£©Óë×óÖ±Ïßœ»µãµÄºá×ø±ê
			f1 = true; // ÕÒµœ×óÖ±Ïß£¬ÏÂÃæÓÒÖ±Ïß²Ù×÷Í¬Àí
		}
		if (r_total_weight>0) {
			k1 = (r_line1.y *(1.0) - r_line2.y) / (r_line1.x *(1.0) - r_line2.x);  // ÓÒÖ±ÏßµÄÐ±ÂÊÊÇk1
			b1 = r_line1.y *1.0 - k1 * r_line1.x;
			x02 = (frameSize.height / 2 - b1) / k1;
			f2 = true;
		}

		cout << "\n\n\nk and k1 are " << k << " " << k1 << endl;

		//to make two line
		if (!f1) {  // Èç¹ûÎŽÕÒµœ×óÖ±Ïß
			if (f2) {  // µ«ÊÇÕÒµœÁËÓÒÖ±Ïß
					   //x01 = x02 - adjustValue; ÓÃµÄÊÇÌÝÐÎÖÐÎ»ÏßÐÔÖÊ
					   //k = -k1;
					   // << "In 1, k and k1 are " << k << " " << k1 << endl;
				printf("1\n");
				fixSingleLine(1, &k, &x01, k1);
				b = frameSize.height / 2 - k * x01;
				//centerx = (b1 - b) / (k - k1);

				//distance = (frameSize.width / 2 - centerx);   // centerx ºÍ distanceµÄº¬Òå²»Ì«Àíœâ
				//cout << "1 frameSize.width = " << frameSize.width << " centerx = " << centerx << endl;
			}
			else {
				//go straight
				lineFound = false; // ÁœžöÖ±Ïß¶ŒÃ»ÓÐÕÒµœ
				printf("2\n");
			}
		}
		else {  // ÕÒµœÁË×óÖ±Ïß
			if (!f2) {  // µ«ÊÇÃ»ÕÒµœÓÒÖ±Ïß
						//x02 = x01 + adjustValue;
						//k1=-k;
				printf("3\n");
				fixSingleLine(0, &k1, &x02, k);
				b1 = frameSize.height / 2 - k1 * x02;
				//centerx = (b1 - b) / (k - k1);

				//distance = (frameSize.width / 2 - centerx);
				//cout << "3, frameSize.width = " << frameSize.width << " centerx = " << centerx << endl;
			}
			else { // ×óÓÒÁœžöÖ±Ïß¶ŒÕÒµœÁË
				printf("4\n");
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
		}

		//-----------------------------------------for test
		if (lineFound) { // Ö±ÏßÕÒµœ
			if (k == 0 || k1 == 0)
				return false;
			else {
				b = frameSize.height / 2 - k * x01;
				b1 = frameSize.height / 2 - k1 * x02;
				if (bLeftMax < b) {
					bLeftMax = b;
					*kFinal = k;
				}
				if (bRightMax > b1) {
					bRightMax = b1;
					*k1Final = k1;
				}
			} // end of else
			//printf("left:  %f x + %d \n", k, b);
			//printf("right:   %f x + %d  \n", k1, b1);
			//printf("\n\n");
		} // end of if
		//else {
			//return false;
		//}
		

	}

	cout << "kFinal and k1Final are " << *kFinal << " " << k1Final << endl;
	printf("(0, bLeftMax) (-bLeftMax/kFinal, 0) is (0, %f) (%f, 0)\n\n", bLeftMax, -bLeftMax / *kFinal);
	printf("(0, bRightMax) (-bRightMax/k1Final, 0) is (0, %f) (%f, 0)", bRightMax, -bRightMax / *k1Final);
	cvLine(src, cvPoint(0, (int)bLeftMax), cvPoint((int)-bLeftMax / *kFinal, 0), CV_RGB(150, 10, 10), 5, CV_AA);  // ×óÖ±Ïß
	cvLine(src, cvPoint(0, (int)bRightMax), cvPoint(src->width, *k1Final * src->width + bRightMax), CV_RGB(150, 10, 10), 5, CV_AA);  // ÓÒÖ±Ïß
	cvLine(src, cvPoint(0, frameSize.height / 2), cvPoint(frameSize.width, frameSize.height / 2), CV_RGB(150, 10, 10), 5, CV_AA);  // ÖÐÐÄÏß

	double x1 = (frameSize.height / 2 - bLeftMax) / *kFinal;   // ÖÐÎ»Ïß×óœ»µã
	double x2 = (frameSize.height / 2 - bRightMax) / *k1Final;  // ÓÒœ»µã
	double centerX = (x1 + x2) / 2;   // ÖÐÎ»ÏßµÄÖÐµã ŒŽµÀÂ·ÖÐÑë
	*distance = (frameSize.width / 2 - centerX);  // ³µÁŸºœÐÐÖÐÐÄÏßºÍµÀÂ·ÖÐÑëµÄÆ«ÒÆŸàÀë
	cout << "\n\ndistance to the middle is " << distance << endl;
	return true;

}

void turnLeft() {
	currentX -= 5;
	cout << "Turning left!! current X is" << currentX << endl;
	string output = "$AP0:", tempX = to_string(currentX);
	output = output + tempX + "X254Y127A127B!";
	out << output << endl;
}

void turnRight() {
	currentX += 5;
	cout << "Turning left!! current X is" << currentX << endl;
	string output = "$AP0:", tempX = to_string(currentX);
	output = output + tempX + "X254Y127A127B!";
	out << output << endl;
}
