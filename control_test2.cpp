#include <highgui.hpp>
#include <core.hpp>
#include <iostream>
#include <fstream>
#include <string>
using namespace std;
using namespace cv;

ofstream out("/dev/ttyUSB0");

int main(int argc, char** argv)
{
	string output = "$AP0:90X254Y127A127B!";
	out << output << endl;
}
