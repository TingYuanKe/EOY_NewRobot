#ifndef COLORMEMORY_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define COLORMEMORY_H

#include <string>
#include <cmath>        // std::abs

using namespace std;

class ColorMemory
{
public:
	static int RGB[3];
	static int RRange[5];
	static float HSL[3];

	static float Min(float a, float b);
	static float Max(float a, float b);
	static void setRGB(int r, int g, int b); 
	static float* RGBToHSL();
	static string ColorClassification();
	static bool identifyPersonByHist(int* Hist1, int* Hist2, int threshold);
};

#endif