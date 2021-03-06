#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    VideoCapture capture(argv[1][0]-'0');
    Mat frame;
    if(!capture.isOpened())
    {
        cout<< "cap open failed!" << "try to open cap "  << argv[1][0]-'0' <<endl;
        return -1;
    }

    int i = 0;
	
    while(i < 100)
    {	
        capture >> frame;
        cout << "get frame " << i << endl;
        imwrite("imgs/v0_frame_" + to_string(i) + ".jpg", frame);//图片保存到本工程目录中
        i++;
    }
    return 0;
}
