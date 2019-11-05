#include <iostream>
#include "bmpreader.h"


int main()
{
	BitmapFile file;
	try
	{
		file = loadBMPFile("pxs_aligned.bmp");
	}
	catch (std::runtime_error & e)
	{
		std::cout << e.what();
		return -1;
	}
	auto file_yuv = convertRGBtoYUV(file);
	auto file_yuv2 = file_yuv.clone();
	//logarithmicOperation(file_yuv2);
	auto file_11 = convertYUVtoRGB(file_yuv2);
	histogramEqualization(file_11);
	saveBMPFile("pxs_2.bmp", file_11);

	auto file2 = file.clone();
	auto file2_g = toGray(file2);
	saveBMPFile("pxs_gray.bmp", file2_g);
	auto file2_ghe = file2_g.clone();
	histogramEqualization8bit(file2_ghe);
	saveBMPFile("pxs_gray_histo.bmp", file2_ghe);


	return 0;
}