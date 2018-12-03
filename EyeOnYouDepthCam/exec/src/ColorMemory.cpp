#include "ColorMemory.h"

int ColorMemory::RGB[3] = {0, 0, 0};
float ColorMemory::HSL[3] = {0.0, 0.0, 0.0};

float ColorMemory::Min(float a, float b) {
	return a <= b ? a : b;
}

float ColorMemory::Max(float a, float b) {
	return a >= b ? a : b;
}

void ColorMemory::setRGB(int r, int g, int b) {
	RGB[0] = r;
	RGB[1] = g;
	RGB[2] = b;
}

float* ColorMemory::RGBToHSL() {
	HSL[0] = 0.0;
	HSL[1] = 0.0;
	HSL[2] = 0.0;

	float r = (RGB[0] / 255.0f);
	float g = (RGB[1] / 255.0f);
	float b = (RGB[2] / 255.0f);

	float min = ColorMemory::Min(ColorMemory::Min(r, g), b);
	float max = ColorMemory::Max(ColorMemory::Max(r, g), b);
	float delta = max - min;

	HSL[2] = (max + min) / 2;

	if (delta == 0)
	{
		HSL[0] = 0;
		HSL[1] = 0.0f;
	}
	else
	{
		HSL[1] = (HSL[3] <= 0.5) ? (delta / (max + min)) : (delta / (2 - max - min));

		float hue;

		if (r == max)
		{
			hue = ((g - b) / 6) / delta;
		}
		else if (g == max)
		{
			hue = (1.0f / 3) + ((b - r) / 6) / delta;
		}
		else
		{
			hue = (2.0f / 3) + ((r - g) / 6) / delta;
		}

		if (hue < 0)
			hue += 1;
		if (hue > 1)
			hue -= 1;

		HSL[0] = (int)(hue * 360);
	}

	return HSL;
}

string ColorMemory::ColorClassification()
{
	float hue = HSL[0];
	float sat = HSL[1];
	float lgt = HSL[2];

	if (lgt < 0.2)  return "Blacks";
	if (lgt > 0.8)  return "Whites";

	if (sat < 0.25) return "Grays";

	if (hue < 30)   return "Reds";
	if (hue < 90)   return "Yellows";
	if (hue < 150)  return "Greens";
	if (hue < 210)  return "Cyans";
	if (hue < 270)  return "Blues";
	if (hue < 330)  return "Magentas";
	return "Reds";
}

bool ColorMemory::identifyPersonByHist(int* Hist1, int* Hist2, int threshold)
{
	int difference = 0;

	for (int i = 0; i < 125; i++) {
		difference = difference + abs(Hist1[i] - Hist2[i]);
	}

	return difference <= threshold;
}