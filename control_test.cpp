#include <highgui.hpp>
#include <core.hpp>
#include <iostream>
#include <fstream>
#include <string>
using namespace std;
using namespace cv;

ofstream out("/dev/ttyUSB1");

int main(int argc, char** argv)
{
	string output = "$AP0:127X127Y127A127B!";
	out << output << endl;
}
