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
	auto file_yuv = file;
	file_yuv.data = convertRGBtoYUV(file_yuv.data);
	//changeLuminanceYUV(25.5, file_yuv.data);
	logarithmicOperationLab(file_yuv);
	file_yuv.data = convertYUVtoRGB(file_yuv.data);
	saveBMPFile("pxs_yuv.bmp",file_yuv);

	auto file_lab = file;
	file_lab.data = convertRGBtoLab(file_lab.data);
	//changeLuminanceLab(10, file_lab.data);
	logarithmicOperationLab(file_lab);
	file_lab.data = convertLabtoRGB(file_lab.data);
	saveBMPFile("pxs_lab.bmp", file_lab);


	return 0;
}