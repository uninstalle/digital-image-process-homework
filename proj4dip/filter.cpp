#include "filter.h"
#include <algorithm>
#include <array>

#define PI 3.1415926535

Kernel MeanKernel =
{ 3,3,1,1,
{1 / 9.0,1 / 9.0,1 / 9.0,
1 / 9.0,1 / 9.0,1 / 9.0,
1 / 9.0,1 / 9.0,1 / 9.0}
};

Kernel LaplacianMask =
{ 3,3,1,1,
{1,1,1,
1,-8,1,
1,1,1}
};

Mat applyConvolution(Mat& src, Kernel filter)
{
	Mat dst(src.getRow(), src.getCol(), sizeof(uint8_t) * 8 * 3);

	auto pixel = src.getDataPtr<uint8_t(*)[3]>();
	auto dst_pixel = dst.getDataPtr<uint8_t(*)[3]>();

	for (unsigned i = 0; i + filter.col - 1 < src.getCol(); ++i)
	{
		for (unsigned j = 0; j + filter.row - 1 < src.getRow(); ++j)
		{

			for (int iChannel = 0; iChannel < 3; ++iChannel) {
				double convolutionValue = 0;

				for (unsigned f_i = 0; f_i < filter.col; ++f_i)
				{
					for (unsigned f_j = 0; f_j < filter.row; ++f_j)
					{
						convolutionValue += pixel[(i + f_i) * src.getRow() + j + f_j][iChannel] * filter.element[f_i * filter.row + f_j];
					}
				}
				dst_pixel[(i + filter.CenterY) * src.getRow() + j + filter.CenterX][iChannel] = rangeFrom0to255(convolutionValue);
			}
		}
	}

	return dst;
}

Mat applyConvolution_1Channel(Mat& src, Kernel filter)
{
	Mat dst(src.getRow(), src.getCol(), sizeof(uint8_t) * 8);

	auto pixel = src.getDataPtr<uint8_t*>();
	auto dst_pixel = dst.getDataPtr<uint8_t*>();

	for (unsigned i = 0; i + filter.col - 1 < src.getCol(); ++i)
	{
		for (unsigned j = 0; j + filter.row - 1 < src.getRow(); ++j)
		{

			double convolutionValue = 0;

			for (unsigned f_i = 0; f_i < filter.col; ++f_i)
			{
				for (unsigned f_j = 0; f_j < filter.row; ++f_j)
				{
					convolutionValue += pixel[(i + f_i) * src.getRow() + j + f_j] * filter.element[f_i * filter.row + f_j];
				}
			}
			dst_pixel[(i + filter.CenterX) * src.getRow() + j + filter.CenterY] = rangeFrom0to255(convolutionValue);

		}
	}

	return dst;
}


class Mat applyConvolutionDouble(Mat& src, Kernel filter)
{
	Mat dst(src.getRow(), src.getCol(), sizeof(double) * 8 * 3);

	auto pixel = src.getDataPtr<uint8_t(*)[3]>();
	auto dst_pixel = dst.getDataPtr<double(*)[3]>();

	for (unsigned i = 0; i + filter.col - 1 < src.getCol(); ++i)
	{
		for (unsigned j = 0; j + filter.row - 1 < src.getRow(); ++j)
		{

			for (int iChannel = 0; iChannel < 3; ++iChannel) {
				double convolutionValue = 0;

				for (unsigned f_i = 0; f_i < filter.col; ++f_i)
				{
					for (unsigned f_j = 0; f_j < filter.row; ++f_j)
					{
						convolutionValue += pixel[(i + f_i) * src.getRow() + j + f_j][iChannel] * filter.element[f_i * filter.row + f_j];
					}
				}
				dst_pixel[(i + filter.CenterX) * src.getRow() + j + filter.CenterY][iChannel] = convolutionValue;
			}
		}
	}

	return dst;
}

class Mat applyConvolutionDouble_1Channel(class Mat& src, Kernel filter)
{
	Mat dst(src.getRow(), src.getCol(), sizeof(double) * 8);

	auto pixel = src.getDataPtr<uint8_t*>();
	auto dst_pixel = dst.getDataPtr<double*>();

	for (unsigned i = 0; i + filter.col - 1 < src.getCol(); ++i)
	{
		for (unsigned j = 0; j + filter.row - 1 < src.getRow(); ++j)
		{

			double convolutionValue = 0;

			for (unsigned f_i = 0; f_i < filter.col; ++f_i)
			{
				for (unsigned f_j = 0; f_j < filter.row; ++f_j)
				{
					convolutionValue += pixel[(i + f_i) * src.getRow() + j + f_j] * filter.element[f_i * filter.row + f_j];
				}
			}
			dst_pixel[(i + filter.CenterX) * src.getRow() + j + filter.CenterY] = convolutionValue;

		}
	}

	return dst;
}


Mat meanFilter(Mat& src)
{
	return applyConvolution(src, MeanKernel);
}

Mat medianFilter(Mat& src)
{
	Mat dst(src.getRow(), src.getCol(), sizeof(uint8_t) * 8 * 3);

	const unsigned filterRow = 3;
	const unsigned filterCol = 3;

	auto pixel = src.getDataPtr<uint8_t(*)[3]>();
	auto dst_pixel = dst.getDataPtr<uint8_t(*)[3]>();

	for (unsigned i = 0; i + filterCol - 1 < src.getCol(); ++i)
	{
		for (unsigned j = 0; j + filterRow - 1 < src.getRow(); ++j)
		{

			for (int iChannel = 0; iChannel < 3; ++iChannel) {
				double convolutionValue = 0;
				std::array<double, filterRow * filterCol> value;
				int count = 0;

				for (unsigned f_i = 0; f_i < filterCol; ++f_i)
				{
					for (unsigned f_j = 0; f_j < filterRow; ++f_j)
					{
						value[count++] = pixel[(i + f_i) * src.getRow() + j + f_j][iChannel];
					}
				}

				std::sort(value.begin(), value.end());

				dst_pixel[(i + filterRow / 2) * src.getRow() + j + filterCol / 2][iChannel] = rangeFrom0to255(value[filterRow * filterCol / 2]);
			}
		}
	}

	return dst;
}

double gaussianFunction(double sigma, int x, int y)
{
	return std::exp(-(x * x + y * y) / 2.0 / sigma / sigma);
}

Kernel generateGaussianKernel(double sigma, int kernelRad)
{
	Kernel k = { kernelRad * 2 + 1,kernelRad * 2 + 1,kernelRad,kernelRad,std::vector<double>() };
	double sum = 0;
	for (int i = -kernelRad; i < kernelRad + 1; ++i)
		for (int j = -kernelRad; j < kernelRad + 1; ++j)
		{
			auto gaussianFunctionResult = gaussianFunction(sigma, i, j);
			k.element.push_back(gaussianFunctionResult);
			sum += gaussianFunctionResult;
		}
	for (auto& i : k.element)
		i /= sum;
	return k;
}

Mat gaussianFilter(Mat& src, double sigma, int kernelRad)
{
	return applyConvolution(src, generateGaussianKernel(sigma, kernelRad));
}

double gaussianFunctionForRange(double sigma, int intensity)
{
	return std::exp(-intensity * intensity / 2.0 / sigma / sigma);
}

Kernel generateRangeDomainKernel(Mat& m, double sigma, int row, int col, int kernelRad)
{
	auto pixel = m.getDataPtr<uint8_t(*)[3]>();
	double sum = 0;
	Kernel k = { kernelRad * 2 + 1,kernelRad * 2 + 1,kernelRad,kernelRad,std::vector<double>() };
	for (int i = -kernelRad; i < kernelRad + 1; ++i)
		for (int j = -kernelRad; j < kernelRad + 1; ++j)
		{
			int intensity = 0;
			for (int iChannel = 0; iChannel < 3; ++iChannel)
				intensity += std::abs(pixel[col * m.getRow() + row][iChannel] - pixel[(col + i) * m.getRow() + row + j][iChannel]);
			auto gaussianFunctionResult = gaussianFunctionForRange(sigma, intensity);
			k.element.push_back(gaussianFunctionResult);
			sum += gaussianFunctionResult;
		}
	for (auto& i : k.element)
		i /= sum;
	return k;
}

Mat bilateralFilter(Mat& src, double sigma_space, double sigma_range,int kernelRad)
{
	auto gaussianKernel = generateGaussianKernel(sigma_space, kernelRad);
	Mat dst(src.getRow(), src.getCol(), sizeof(uint8_t) * 8 * 3);

	auto pixel = src.getDataPtr<uint8_t(*)[3]>();
	auto dst_pixel = dst.getDataPtr<uint8_t(*)[3]>();
	Kernel& filter = gaussianKernel;

	for (unsigned i = 0; i + filter.col - 1 < src.getCol(); ++i)
	{
		for (unsigned j = 0; j + filter.row - 1 < src.getRow(); ++j)
		{
			auto gaussianKernelForRange = generateRangeDomainKernel(src, sigma_range, j + kernelRad, i + kernelRad, kernelRad);
			
			for (int iChannel = 0; iChannel < 3; ++iChannel) {
				double convolutionValue = 0;
				double sum = 0;
				
				for (unsigned f_i = 0; f_i < filter.col; ++f_i)
				{
					for (unsigned f_j = 0; f_j < filter.row; ++f_j)
					{
						/*sum += gaussianKernelForRange.element[f_i * gaussianKernelForRange.row + f_j];
						convolutionValue += pixel[(i + f_i) * src.getRow() + j + f_j][iChannel] * gaussianKernelForRange.element[f_i * gaussianKernelForRange.row + f_j];*/
						sum += filter.element[f_i * filter.row + f_j] * gaussianKernelForRange.element[f_i * gaussianKernelForRange.row + f_j];
						convolutionValue += pixel[(i + f_i) * src.getRow() + j + f_j][iChannel] * filter.element[f_i * filter.row + f_j] * gaussianKernelForRange.element[f_i * gaussianKernelForRange.row + f_j];
					}
				}
				dst_pixel[(i + filter.CenterY) * src.getRow() + j + filter.CenterX][iChannel] = rangeFrom0to255(convolutionValue / sum);
			}
		}
	}

	return dst;
}




Mat sharpen(Mat& src)
{
	Mat dst(src.getRow(), src.getCol(), src.getBitCount());
	auto laplacianResult = applyConvolutionDouble(src, LaplacianMask);

	auto p_src = src.getDataPtr<uint8_t(*)[3]>();
	auto p_dst = dst.getDataPtr<uint8_t(*)[3]>();
	auto P_lresult = laplacianResult.getDataPtr<double(*)[3]>();

	for (int iChannel = 0; iChannel < 3; ++iChannel) {
		for (auto i = 0; i < laplacianResult.getRow() * laplacianResult.getCol(); ++i)
		{
			p_dst[i][iChannel] = rangeFrom0to255(p_src[i][iChannel] - P_lresult[i][iChannel]);
		}
	}

	return dst;
}

Mat getLaplacianResult_1Channel(Mat& lResult)
{
	Mat dst(lResult.getRow(), lResult.getCol(), sizeof(uint8_t) * 8);

	auto p_lResult = lResult.getDataPtr<double*>();
	auto p_dst = dst.getDataPtr<uint8_t*>();

	double max = p_lResult[0], min = p_lResult[0];

	for (auto i = 1; i < lResult.getRow() * lResult.getCol(); ++i)
	{
		if (p_lResult[i] > max) max = p_lResult[i];
		if (p_lResult[i] < min) min = p_lResult[i];
	}

	double rate = 255 / (max - min);

	for (auto i = 0; i < lResult.getRow() * lResult.getCol(); ++i)
		p_dst[i] = (p_lResult[i] - min) * rate;

	return dst;
}

Mat sharpen_1Channel(Mat& src)
{
	Mat dst(src.getRow(), src.getCol(), src.getBitCount());
	auto laplacianResult = applyConvolutionDouble_1Channel(src, LaplacianMask);

	auto p_src = src.getDataPtr<uint8_t*>();
	auto p_dst = dst.getDataPtr<uint8_t*>();
	auto p_lResult = laplacianResult.getDataPtr<double*>();

	for (auto i = 0; i < laplacianResult.getRow() * laplacianResult.getCol(); ++i)
	{
		p_dst[i] = rangeFrom0to255(p_src[i] - p_lResult[i]);
	}

	return dst;
}