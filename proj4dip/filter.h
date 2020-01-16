#pragma once
#include "mat.h"

struct Kernel
{
	unsigned row, col;
	unsigned CenterX, CenterY;
	std::vector<double> element;
};

Mat applyConvolution(Mat& src, Kernel filter);
Mat applyConvolution_1Channel(Mat& src, Kernel filter);
// the returned mat stores data in double
Mat applyConvolutionDouble(Mat& src, Kernel filter);
Mat applyConvolutionDouble_1Channel(Mat& src, Kernel filter);


Mat meanFiltering(Mat& src);
Mat medianFiltering(Mat& src);
Mat gaussianFiltering(Mat& src,double sigma,int kernelRad);
Mat bilateralFiltering(Mat& src,double sigma_space,double sigma_range,int kernelRad);

Mat sharpen(Mat& src);
Mat sharpen_1Channel(Mat& src);