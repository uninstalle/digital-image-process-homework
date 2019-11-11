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

	char* getRawDataPtr() { return data.get(); }
	template <typename T> T getDataPtr() { return reinterpret_cast<T>(data.get()); }
	void resize(unsigned row, unsigned col, unsigned bitPerElement);
	Mat clone() const;
	virtual ~Mat() = default;
};

struct TransMat
{
	std::vector<double> data;
	TransMat operator*(const TransMat& m) const;
	TransMat invert() const;
	TransMat() { data.resize(9); }
	TransMat(std::vector<double> v):data(v) {}
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

Mat convertRGBtoYUV(Mat& rgb);
Mat convertYUVtoRGB(Mat& yuv);

Mat convertRGBtoXYZ(Mat& rgb);
Mat convertXYZtoRGB(Mat& xyz);
Mat convertRGBtoLab(Mat& rgb);

Mat convertXYZtoLab(Mat& xyz);
Mat convertLabtoXYZ(Mat& lab);
Mat convertLabtoRGB(Mat& lab);

BitmapFile toGrayRGB(BitmapFile& bmp);
BitmapFile toGrayYUV(BitmapFile& bmp);
BitmapFile toGrayLab(BitmapFile& bmp);

void changeLuminanceYUV(double deltaValue, Mat& yuv);
void changeLuminanceLab(double deltaValue, Mat& lab);


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

void logarithmicOperationYUV(BitmapFile& bmp);
void logarithmicOperationLab(BitmapFile& bmp);

void histogramEqualization8bit(BitmapFile gray);
void histogramEqualization(BitmapFile bmp);
void histogramEqualization_2(BitmapFile bmp);

TransMat translate(double x, double y);
TransMat rotate(double rad);
TransMat scale(double scaleX, double scaleY);
TransMat shear(double offsetX, double offsetY);
TransMat mirror(double normalX, double normalY);
Mat geometricTransform(Mat src, TransMat& transMat);
BitmapFile buildBMP(Mat mat);