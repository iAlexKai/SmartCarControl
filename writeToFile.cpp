#include <fstream>
using namespace std;
int main () {  
     ofstream out("outputtest.txt");
     if (out.is_open())   
    {
         out << "This is a line.\n";  
         out << "This is another line.\n";  
         out.close();  
     }  
     return 0;  
 }  
