#include <fstream>
#include <iostream>
using namespace std;
int main () {  
     ofstream out("/dev/ttyUSB0");
     if (out.is_open())   
    {
         cout << "out is open" << endl;

         //要加上 endl 或者\n来实现 否则没有输出
         out << "$AP0:95X254Y127A127B!" << endl;
         //out << "This is a line. with \n";
         //out << "This is 2 line. with \n";
         //out << "command from PC with nothing";
         //out << "flus0";
         //out << "flus1";
         out.close();
     }  
     return 0;  
 }  
