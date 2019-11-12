#pragma once
#include "mat.h"

struct BitmapFileHead
{
	uint16_t bfType = 0;
	uint32_t bfSize = 0;
	uint16_t bfReserved1 = 0;
	uint16_t bfReserved2 = 0;
	uint32_t bfOffBits = 0;
	friend std::istream& operator>>(std::istream& is, BitmapFileHead& bfh);
	friend std::ostream& operator<<(std::ostream& os, BitmapFileHead& bfh);
};

struct BitmapInfoHead
{
	uint32_t biSize = 0;
	uint32_t biWidth = 0;
	uint32_t biHeight = 0;
	uint16_t biPlanes = 0;
	uint16_t biBitCount = 0;
	uint32_t biCompression = 0;
	uint32_t biSizeImage = 0;
	uint32_t biXPelsPerMeter = 0;
	uint32_t biYPelsPerMeter = 0;
	uint32_t biClrUsed = 0;
	uint32_t biClrImportant = 0;
	friend std::istream& operator>>(std::istream& is, BitmapInfoHead& bih);
	friend std::ostream& operator<<(std::ostream& os, BitmapInfoHead& bih);
};


struct BitmapPalette
{
	Mat data;
	int index;
	BitmapPalette() : index(0) {}
	BitmapPalette(int index)
		:data(index, 4, 8), index(index) {}
	[[nodiscard]] BitmapPalette clone() const;
	friend std::ostream& operator<<(std::ostream& os, BitmapPalette& bp);

};


struct BitmapFile
{
	BitmapFileHead fileHead;
	BitmapInfoHead infoHead;
	BitmapPalette palette;
	Mat data;
	[[nodiscard]] BitmapFile clone() const;
};


BitmapFile loadBMPFile(std::string filename);
void saveBMPFile(std::string filename, BitmapFile& bmp);


BitmapFile toGrayRGB(BitmapFile& bmp);
BitmapFile toGrayYUV(BitmapFile& bmp);
BitmapFile toGrayLab(BitmapFile& bmp);


BitmapFile binaryImageErosion(BitmapFile binary, StructuringElement se);
BitmapFile binaryImageDilation(BitmapFile binary, StructuringElement se);
BitmapFile binaryImageOpening(BitmapFile binary, StructuringElement se);
BitmapFile binaryImageClosing(BitmapFile binary, StructuringElement se);


BitmapFile buildBMP(Mat mat);