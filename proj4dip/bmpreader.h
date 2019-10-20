#pragma once
#include <iostream>
#include <memory>
#include <vector>

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


class Mat
{
	unsigned row, col;
	unsigned bitPerElement;
	unsigned size;
	std::shared_ptr<char[]> data;
public:
	Mat() :row(0), col(0), bitPerElement(0), size(0), data(nullptr) {}
	Mat(unsigned row, unsigned col, unsigned bitPerElement)
		:row(row), col(col), bitPerElement(bitPerElement), size(row* col* bitPerElement / 8)
	{
		//C++17 needed
		data = std::shared_ptr<char[]>(new char[size]);
		//C++20 needed
		//data = std::make_shared<char[]>(new char[size]);
	}

	unsigned getRow() const { return row; }
	unsigned getCol() const { return col; }
	unsigned getBitCount() const { return bitPerElement; }
	unsigned getSize() const { return size; }
	char* getDataPtr() { return data.get(); }
	void resize(unsigned row, unsigned col, unsigned bitPerElement);
	Mat clone() const;
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

struct YUVData
{
	BitmapFileHead fileHead;
	BitmapInfoHead infoHead;
	BitmapPalette palette;
	Mat data;
	YUVData() = default;
	YUVData(BitmapFileHead fh, BitmapInfoHead ih, BitmapPalette p, Mat d) :
		fileHead(fh), infoHead(ih), palette(std::move(p)), data(std::move(d)) {}
	YUVData(BitmapFile& bmp) :fileHead(bmp.fileHead), infoHead(bmp.infoHead), palette(bmp.palette)
	{
		data = Mat(bmp.data.getRow(),bmp.data.getCol(), sizeof(double) * 8 * 3);
	}
	[[nodiscard]] YUVData clone() const;
};

BitmapFile loadBMPFile(std::string filename);
void saveBMPFile(std::string filename, BitmapFile& bmp);

YUVData convertRGBtoYUV(BitmapFile& bmp);
BitmapFile convertYUVtoRGB(YUVData& yuv);

BitmapFile toGray(BitmapFile& bmp);
BitmapFile toGray(YUVData& yuv);

void changeLuminanceValue(double deltaValue, YUVData& yuv);


struct StructuringElement
{
	unsigned row, col;
	unsigned OriginX, originY;
	std::vector<bool> element;
};

void binarize8BitFile(uint8_t threshold, BitmapFile gray);
uint8_t generateThreshold(BitmapFile gray);
uint8_t generateThreshold_Otsu(BitmapFile gray);
BitmapFile binaryImageErosion(BitmapFile binary, StructuringElement se);
BitmapFile binaryImageDilation(BitmapFile binary, StructuringElement se);
BitmapFile binaryImageOpening(BitmapFile binary, StructuringElement se);
BitmapFile binaryImageClosing(BitmapFile binary, StructuringElement se);