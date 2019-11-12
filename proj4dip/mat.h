#pragma once
#include <iostream>
#include <memory>
#include <vector>


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


struct StructuringElement
{
	unsigned row, col;
	unsigned OriginX, originY;
	std::vector<bool> element;
};

struct TransMat
{
	std::vector<double> data;
	TransMat operator*(const TransMat& m) const;
	TransMat invert() const;
	TransMat() { data.resize(9); }
	TransMat(std::vector<double> v):data(v) {}
};


Mat convertRGBtoYUV(Mat& rgb);
Mat convertYUVtoRGB(Mat& yuv);

Mat convertRGBtoXYZ(Mat& rgb);
Mat convertXYZtoRGB(Mat& xyz);
Mat convertRGBtoLab(Mat& rgb);

Mat convertXYZtoLab(Mat& xyz);
Mat convertLabtoXYZ(Mat& lab);
Mat convertLabtoRGB(Mat& lab);


void changeLuminanceYUV(double deltaValue, Mat& yuv);
void changeLuminanceLab(double deltaValue, Mat& lab);



void binarizeGray(uint8_t threshold, Mat& gray);
uint8_t generateThreshold(Mat& gray);
uint8_t generateThreshold_Otsu(Mat& gray);

Mat binaryImageErosionAndDilation(Mat& binary, StructuringElement se, bool retErosion);

void logarithmicOperationYUV(Mat& yuv);
void logarithmicOperationLab(Mat& lab);

void histogramEqualizationGray(Mat& gray);
void histogramEqualization(Mat& mat);
void histogramEqualization_2(Mat& mat);

TransMat translate(double x, double y);
TransMat rotate(double rad);
TransMat scale(double scaleX, double scaleY);
TransMat shear(double offsetX, double offsetY);
TransMat mirror(double normalX, double normalY);
Mat geometricTransform(Mat src, TransMat& transMat);