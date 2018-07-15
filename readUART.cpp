#include <fstream>
#include <iostream>
using namespace std;
int main () {
     ifstream input("/dev/ttyUSB1");
     string line;
     while(getline(input,line))//getline 获取一行数据
     {
        cout << "get line :" << line << endl;
     }

     //if (input.is_open())
     //{
     //    input >> x;
     //    cout << "got input from port: " << x << endl;
     //    这种方式只能获取到单词 一行中间的空格后面的字符被忽略
     //    input.close();
     //}
     return 0;
 }