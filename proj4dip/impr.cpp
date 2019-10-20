#include <iostream>
#include "bmpreader.h"


int main()
{
	BitmapFile file;
	try
	{
		file = loadBMPFile("pxs.bmp");
	}
	catch (std::runtime_error & e)
	{
		std::cout << e.what();
		return -1;
	}
	auto file_yuv = convertRGBtoYUV(file);

	auto file2 = file.clone();
	auto file2_g = toGray(file2);
	saveBMPFile("pxs_gray.bmp", file2_g);

	auto file3 = file2_g.clone();
	binarize8BitFile(generateThreshold_Otsu(file3), file3);
	saveBMPFile("pxs_bi_otsu.bmp", file3);

	auto file3_1 = file2_g.clone();
	binarize8BitFile(generateThreshold(file3), file3);
	saveBMPFile("pxs_bi.bmp", file3);

	StructuringElement se{ 3,3,1,1,
	{1,1,1,
			1,1,1,
			1,1,1} };
	auto file4 = binaryImageErosion(file3, se);
	saveBMPFile("pxs_ero.bmp", file4);


	auto file5 = binaryImageDelation(file3, se);
	saveBMPFile("pxs_del.bmp", file5);

	auto file6 = binaryImageOpening(file3, se);
	saveBMPFile("pxs_opening.bmp", file6);

	auto file7 = binaryImageClosing(file3, se);
	saveBMPFile("pxs_closing.bmp", file7);


	return 0;
}