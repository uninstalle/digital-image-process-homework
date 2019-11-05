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
	enum class TYPE;

	unsigned row, col;
	unsigned bitPerElement;
	unsigned size;
	std::shared_ptr<char[]> data;

	TYPE type;

public:

	enum class TYPE
	{
		DEFAULT,
		BINARY,
		GRAY_8BIT,
		RGB_16,
		RGB_24,
		RGBA_32,
		YUV,
		XYZ,
		Lab
	};

	Mat() :row(0), col(0), bitPerElement(0), size(0), data(nullptr), type(TYPE::DEFAULT) {}
	Mat(unsigned row, unsigned col, unsigned bitPerElement)
		:row(row), col(col), bitPerElement(bitPerElement), size(row* col* bitPerElement / 8), type(TYPE::DEFAULT)
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
	void setType(TYPE t) { type = t; }
	TYPE getType() const { return type; }

	char* getRawDataPtr() { return data.get(); }
	template <typename T> T getDataPtr() { return reinterpret_cast<T>(data.get()); }
	void resize(unsigned row, unsigned col, unsigned bitPerElement);
	Mat clone() const;
	virtual  ~Mat() = default;
};

class RGBData :public Mat
{
public:
	RGBData() = default;
	RGBData(unsigned row, unsigned col, unsigned bitPerElement)
		:Mat(row, col, bitPerElement)
	{
		switch (bitPerElement)
		{
		case 2:
			setType(TYPE::BINARY); break;
		case 8:
			setType(TYPE::GRAY_8BIT); break;
		case 16:
			setType(TYPE::RGB_16); break;
		case 24:
			setType(TYPE::RGB_24); break;
		case 32:
			setType(TYPE::RGBA_32); break;
		default:
			setType(TYPE::DEFAULT); break;
		}
	}
};

class YUVData :public Mat
{
public:
	YUVData() = default;
	YUVData(unsigned row, unsigned col)
		:Mat(row, col, sizeof(double) * 8 * 3)
	{
		setType(TYPE::YUV);
	}
	auto getDataPtr() ->double(*)[3]
	{
		return reinterpret_cast<double(*)[3]>(getRawDataPtr());
	}
	~YUVData() = default;
};

class XYZData :public Mat
{
public:
	XYZData() = default;
	XYZData(unsigned row, unsigned col)
		:Mat(row, col, sizeof(double) * 8 * 3)
	{
		setType(TYPE::XYZ);
	}
	auto getDataPtr() ->double(*)[3]
	{
		return  reinterpret_cast<double(*)[3]>(getRawDataPtr());
	}
	~XYZData() = default;
};

class LabData :public Mat
{
public:
	LabData() = default;
	LabData(unsigned row, unsigned col)
		:Mat(row, col, sizeof(double) * 8 * 3)
	{
		setType(TYPE::Lab);
	}
	auto getDataPtr() ->double(*)[3]
	{
		return reinterpret_cast<double(*)[3]>(getRawDataPtr());
	}
	~LabData() = default;
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

BitmapFile toGray(BitmapFile& bmp);

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