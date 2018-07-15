//#include "opencv2\opencv.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string>
#include <time.h>
#define VIDEO_PORT 1
using namespace cv;
using namespace std;

void recordTime() {
    static int i = 0;
    static clock_t startTime, endTime;
    if (i==0) {
        startTime = clock();
        i = 1;
    }
    else {
        endTime = clock();
        i = 0;
        cout << "Totle Time : " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
    }
}

int main(int argc, char** argv)
{
    VideoCapture capture(VIDEO_PORT);
    Mat frame;
    if(!capture.isOpened())
    {
       cout<<"摄像头打开失败！"<<endl;
        return -1;
    }

    int i = 0;
    recordTime();
    while(i < 500)
    {
        capture >> frame;
        cout << "video" << VIDEO_PORT << " get frame " << i << endl;
        imwrite("imgs/v" + to_string(VIDEO_PORT) + "_frame_" + to_string(i) + ".jpg", frame);//图片保存到本工程目录中
        i++;
    }
    recordTime();

    return 0;
}
