#include <fstream>
#include <iostream>
using namespace std;
int main () {  
     ifstream input("outputtest.txt");
     string x;
     if (input.is_open())
    {
         input >> x;
         cout << "got input " << x << endl;
         input.close();
     }  
     return 0;  
 }  
