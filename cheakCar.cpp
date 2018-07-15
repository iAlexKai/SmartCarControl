#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string>
#include <unistd.h>

using namespace cv;
using namespace std;
void recordTime(string info) {
    static int i = 0;
    static clock_t lastTime=clock(), thisTime=clock();

    thisTime = clock();
    i = 0;
    cout << "Time  " << info << ":" <<(double)(thisTime - lastTime) / CLOCKS_PER_SEC << "s" << endl;
    lastTime = thisTime;
}

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

    recordTime("video begin");
    for (int k = 0; k < activate_frames; k++) {
        capture >> frame1;
    }

    for (int k = 0; k < delay_frames; k++) {
        capture >> frame2;
    }
    capture.release();//关闭摄像头

    recordTime("handle pic");

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
    recordTime("finish");
    return diffVal;
   // popen("python /home/myk/workspace_szh/py_src/checkCar.py", "r")
}

int main(int argc, char** argv)
{
    int i = 0;
    printf("diff val is %d\n", check_moving_object());
    return 0;
}





