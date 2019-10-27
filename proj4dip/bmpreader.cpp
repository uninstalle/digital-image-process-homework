#include <fstream>
#include <string>
#include "bmpreader.h"


std::istream& operator>>(std::istream& is, BitmapFileHead& bfh)
{
	is.read(reinterpret_cast<char*>(&bfh.bfType), sizeof(uint16_t));
	is.read(reinterpret_cast<char*>(&bfh.bfSize), sizeof(uint32_t));
	is.read(reinterpret_cast<char*>(&bfh.bfReserved1), sizeof(uint16_t));
	is.read(reinterpret_cast<char*>(&bfh.bfReserved2), sizeof(uint16_t));
	is.read(reinterpret_cast<char*>(&bfh.bfOffBits), sizeof(uint32_t));
	return is;
}

std::ostream& operator<<(std::ostream& os, BitmapFileHead& bfh)
{
	os.write(reinterpret_cast<char*>(&bfh.bfType), sizeof(uint16_t));
	os.write(reinterpret_cast<char*>(&bfh.bfSize), sizeof(uint32_t));
	os.write(reinterpret_cast<char*>(&bfh.bfReserved1), sizeof(uint16_t));
	os.write(reinterpret_cast<char*>(&bfh.bfReserved2), sizeof(uint16_t));
	os.write(reinterpret_cast<char*>(&bfh.bfOffBits), sizeof(uint32_t));
	return os;
}

std::istream& operator>>(std::istream& is, BitmapInfoHead& bih)
{
	is.read(reinterpret_cast<char*>(&bih.biSize), sizeof(uint32_t));
	is.read(reinterpret_cast<char*>(&bih.biWidth), sizeof(uint32_t));
	is.read(reinterpret_cast<char*>(&bih.biHeight), sizeof(uint32_t));
	is.read(reinterpret_cast<char*>(&bih.biPlanes), sizeof(uint16_t));
	is.read(reinterpret_cast<char*>(&bih.biBitCount), sizeof(uint16_t));
	is.read(reinterpret_cast<char*>(&bih.biCompression), sizeof(uint32_t));
	is.read(reinterpret_cast<char*>(&bih.biSizeImage), sizeof(uint32_t));
	is.read(reinterpret_cast<char*>(&bih.biXPelsPerMeter), sizeof(uint32_t));
	is.read(reinterpret_cast<char*>(&bih.biYPelsPerMeter), sizeof(uint32_t));
	is.read(reinterpret_cast<char*>(&bih.biClrUsed), sizeof(uint32_t));
	is.read(reinterpret_cast<char*>(&bih.biClrImportant), sizeof(uint32_t));
	return is;
}

std::ostream& operator<<(std::ostream& os, BitmapInfoHead& bih)
{
	os.write(reinterpret_cast<char*>(&bih.biSize), sizeof(uint32_t));
	os.write(reinterpret_cast<char*>(&bih.biWidth), sizeof(uint32_t));
	os.write(reinterpret_cast<char*>(&bih.biHeight), sizeof(uint32_t));
	os.write(reinterpret_cast<char*>(&bih.biPlanes), sizeof(uint16_t));
	os.write(reinterpret_cast<char*>(&bih.biBitCount), sizeof(uint16_t));
	os.write(reinterpret_cast<char*>(&bih.biCompression), sizeof(uint32_t));
	os.write(reinterpret_cast<char*>(&bih.biSizeImage), sizeof(uint32_t));
	os.write(reinterpret_cast<char*>(&bih.biXPelsPerMeter), sizeof(uint32_t));
	os.write(reinterpret_cast<char*>(&bih.biYPelsPerMeter), sizeof(uint32_t));
	os.write(reinterpret_cast<char*>(&bih.biClrUsed), sizeof(uint32_t));
	os.write(reinterpret_cast<char*>(&bih.biClrImportant), sizeof(uint32_t));
	return os;
}

std::ostream& operator<<(std::ostream& os, BitmapPalette& bp)
{
	unsigned dataSize = bp.data.getSize();
	os.write(bp.data.getDataPtr(), dataSize);
	return os;
}

void Mat::resize(unsigned row, unsigned col, unsigned bitPerElement)
{
	this->row = row;
	this->col = col;
	this->bitPerElement = bitPerElement;
	this->size = row * col * bitPerElement / 8;
	data = std::shared_ptr<char[]>(new char[size]);
}

Mat Mat::clone() const
{
	Mat clonedMat(row, col, bitPerElement);
	memcpy(clonedMat.getDataPtr(), data.get(), size);
	return clonedMat;
}

BitmapPalette BitmapPalette::clone() const
{
	BitmapPalette clonedBitmapPalette;
	clonedBitmapPalette.index = index;
	clonedBitmapPalette.data = data.clone();
	return clonedBitmapPalette;
}

BitmapFile BitmapFile::clone() const
{
	BitmapFile clonedBitmapFile{ fileHead, infoHead, palette.clone(), data.clone() };
	return clonedBitmapFile;
}

YUVData YUVData::clone() const
{
	YUVData clonedYUVData{ fileHead,infoHead,palette.clone(),data.clone() };
	return clonedYUVData;
}

BitmapFile loadBMPFile(std::string filename)
{
	std::fstream BMPfs(filename, std::ios::binary | std::ios::in);
	if (!BMPfs)
		throw std::runtime_error("Open BMP file failed.");

	BitmapFile bmp;
	BMPfs >> bmp.fileHead;
	BMPfs >> bmp.infoHead;
	if (bmp.infoHead.biClrUsed)
	{
		bmp.palette = BitmapPalette(bmp.infoHead.biClrUsed);
		BMPfs.read(reinterpret_cast<char*>(bmp.palette.data.getDataPtr()), bmp.palette.data.getSize());
	}
	// width is 4-byte aligned
	bmp.data = Mat((bmp.infoHead.biWidth * bmp.infoHead.biBitCount / 8 + 3) / 4 * 4 / bmp.infoHead.biBitCount * 8,
		bmp.infoHead.biHeight, bmp.infoHead.biBitCount);
	BMPfs.read(bmp.data.getDataPtr(), bmp.data.getSize());

	BMPfs.close();
	return bmp;
}

void saveBMPFile(std::string filename, BitmapFile& bmp)
{
	std::fstream BMPfs(filename, std::ios::binary | std::ios::out);
	if (!BMPfs)
		throw std::runtime_error("Open BMP file failed.");
	BMPfs << bmp.fileHead;
	BMPfs << bmp.infoHead;
	BMPfs << bmp.palette;
	BMPfs.write(bmp.data.getDataPtr(), bmp.data.getSize());

	BMPfs.close();
}

YUVData convertRGBtoYUV(BitmapFile& bmp)
{
	auto pixel = reinterpret_cast<uint8_t(*)[3]>(bmp.data.getDataPtr());
	YUVData yuv(bmp);
	auto yuv_pixel = reinterpret_cast<double(*)[3]>(yuv.data.getDataPtr());

	double k1 = 0.5 / 0.435, k2 = 0.5 / 0.615;
	for (unsigned i = 0; i < bmp.infoHead.biHeight; ++i)
	{
		for (unsigned j = 0; j < bmp.infoHead.biWidth; ++j)
		{
			unsigned pixelIndex = i * bmp.data.getRow() + j;
			unsigned char b = pixel[pixelIndex][0],
				g = pixel[pixelIndex][1],
				r = pixel[pixelIndex][2];
			double y = 0.299 * r + 0.587 * g + 0.114 * b,
				u = k1 * (-0.147 * r + -0.289 * g + 0.435 * b),
				v = k2 * (0.615 * r + -0.515 * g + -0.100 * b);
			yuv_pixel[pixelIndex][0] = v;
			yuv_pixel[pixelIndex][1] = u;
			yuv_pixel[pixelIndex][2] = y;
		}
	}

	return yuv;
}

BitmapFile convertYUVtoRGB(YUVData& yuv)
{
	auto yuv_pixel = reinterpret_cast<double(*)[3]>(yuv.data.getDataPtr());

	BitmapFile bmp{ yuv.fileHead,yuv.infoHead,yuv.palette };
	bmp.data = Mat(yuv.data.getRow(), yuv.data.getCol(), bmp.infoHead.biBitCount);

	auto pixel = reinterpret_cast<uint8_t(*)[3]>(bmp.data.getDataPtr());
	auto rangeFrom0to255 = [](double a, double b, double c) -> double {
		if ((a + b + c) > 255)
			return 255;
		else if ((a + b + c) < 0)
			return 0;
		else
			return a + b + c;
	};
	double k1 = 0.435 / 0.5, k2 = 0.615 / 0.5;

	for (unsigned i = 0; i < bmp.infoHead.biHeight; ++i)
	{
		for (unsigned j = 0; j < bmp.infoHead.biWidth; ++j)
		{
			unsigned pixelIndex = i * bmp.data.getRow() + j;
			double v = yuv_pixel[pixelIndex][0],
				u = yuv_pixel[pixelIndex][1],
				y = yuv_pixel[pixelIndex][2];


			uint8_t r = rangeFrom0to255(y, k1 * -0.00004 * u, k2 * 1.139828 * v),
				g = rangeFrom0to255(0.999605 * y, k1 * -0.395414 * u, k2 * -0.5805 * v),
				b = rangeFrom0to255(1.002036 * y, k1 * 2.036137 * u, k2 * -0.000482 * v);

			pixel[pixelIndex][0] = b;
			pixel[pixelIndex][1] = g;
			pixel[pixelIndex][2] = r;
		}
	}

	return bmp;
}

static BitmapPalette GrayPalette;
static BitmapPalette BinaryPalette;

struct BitmapPalette getGrayPalette()
{
	if (!GrayPalette.index) {
		BitmapPalette palette(256);
		auto pixel = reinterpret_cast<uint8_t(*)[4]>(palette.data.getDataPtr());
		for (int i = 0; i < 256; ++i)
		{
			pixel[i][0] = pixel[i][1] = pixel[i][2] = i;
			pixel[i][3] = 0;
		}
		GrayPalette = palette;
	}
	return GrayPalette;
}

struct BitmapPalette getBinaryPalette()
{
	if (!BinaryPalette.index)
	{
		BitmapPalette palette(2);
		auto pixel = reinterpret_cast<uint8_t(*)[4]>(palette.data.getDataPtr());
		pixel[0][0] = pixel[0][1] = pixel[0][2] = 0;
		pixel[1][0] = pixel[1][1] = pixel[1][2] = 255;
		BinaryPalette = palette;
	}
	return BinaryPalette;
}

BitmapFile build8bitBMPHead(BitmapFileHead fh, BitmapInfoHead ih)
{
	BitmapFile bmp;
	bmp.fileHead = fh;
	bmp.infoHead = ih;
	bmp.fileHead.bfOffBits = 14 + 40 + 256 * 4;
	bmp.fileHead.bfSize = bmp.fileHead.bfOffBits + bmp.data.getRow() * bmp.data.getCol();
	bmp.infoHead.biBitCount = 8;
	bmp.infoHead.biClrUsed = 256;
	bmp.infoHead.biSizeImage = 0;
	bmp.palette = getGrayPalette();
	return bmp;
}

BitmapFile buildbinaryBMPHead(BitmapFileHead fh, BitmapInfoHead ih)
{
	BitmapFile bmp;
	bmp.fileHead = fh;
	bmp.infoHead = ih;
	bmp.fileHead.bfOffBits = 14 + 40 + 2 * 4;
	bmp.fileHead.bfSize = bmp.fileHead.bfOffBits + bmp.data.getRow() * bmp.data.getCol() / 8;
	bmp.infoHead.biBitCount = 1;
	bmp.infoHead.biClrUsed = 2;
	bmp.infoHead.biSizeImage = 0;
	bmp.palette = getBinaryPalette();
	return bmp;
}

BitmapFile toGray(YUVData& yuv)
{
	auto yuv_pixel = reinterpret_cast<double(*)[3]>(yuv.data.getDataPtr());
	BitmapFile bmp_8bit = build8bitBMPHead(yuv.fileHead, yuv.infoHead);
	bmp_8bit.data = Mat(yuv.data.getRow(), yuv.data.getCol(), 8);
	auto pixel_8bit = reinterpret_cast<uint8_t*>(bmp_8bit.data.getDataPtr());

	for (unsigned i = 0; i < yuv.infoHead.biHeight; ++i)
	{
		for (unsigned j = 0; j < yuv.infoHead.biWidth; ++j)
		{
			unsigned pixelIndex = i * yuv.data.getRow() + j;
			pixel_8bit[pixelIndex] = yuv_pixel[pixelIndex][2];

		}
	}

	return bmp_8bit;
}

BitmapFile toGray(BitmapFile& bmp)
{
	auto pixel = reinterpret_cast<uint8_t(*)[3]>(bmp.data.getDataPtr());
	BitmapFile bmp_8bit = build8bitBMPHead(bmp.fileHead, bmp.infoHead);
	bmp_8bit.data = Mat(bmp.data.getRow(), bmp.data.getCol(), 8);
	auto pixel_8bit = reinterpret_cast<uint8_t*>(bmp_8bit.data.getDataPtr());

	for (unsigned i = 0; i < bmp.infoHead.biHeight; ++i)
	{
		for (unsigned j = 0; j < bmp.infoHead.biWidth; ++j)
		{
			unsigned pixelIndex = i * bmp.data.getRow() + j;
			unsigned char gray = unsigned char(
				pixel[pixelIndex][2] * 0.299 + pixel[pixelIndex][1] * 0.587 + pixel[pixelIndex][0] * 0.114);
			pixel_8bit[pixelIndex] = gray;

		}
	}


	return bmp_8bit;
}

void changeLuminanceValue(double deltaValue, YUVData& yuv)
{
	auto yuv_pixel = reinterpret_cast<double(*)[3]>(yuv.data.getDataPtr());

	for (unsigned i = 0; i < yuv.infoHead.biHeight; ++i)
	{
		for (unsigned j = 0; j < yuv.infoHead.biWidth; ++j)
		{
			unsigned pixelIndex = i * yuv.data.getRow() + j;
			yuv_pixel[pixelIndex][2] += deltaValue;
		}
	}

}

void binarize8BitFile(uint8_t threshold, BitmapFile gray)
{
	unsigned numOfPixels = gray.infoHead.biWidth * gray.infoHead.biHeight;
	auto pixel_8bit = reinterpret_cast<uint8_t*>(gray.data.getDataPtr());

	for (unsigned i = 0; i < gray.infoHead.biHeight; ++i)
	{
		for (unsigned j = 0; j < gray.infoHead.biWidth; ++j)
		{
			unsigned pixelIndex = i * gray.data.getRow() + j;

			if (pixel_8bit[pixelIndex] > threshold)
				pixel_8bit[pixelIndex] = 255;
			else
				pixel_8bit[pixelIndex] = 0;
		}
	}
}

uint8_t generateThreshold(BitmapFile gray)
{
	auto pixel_8bit = reinterpret_cast<uint8_t*>(gray.data.getDataPtr());


	int threshold = pixel_8bit[0];
	int count0 = 0, count255 = 0;
	int sum0 = 0, sum255 = 0;

	do {

		if (count0 && count255)
			threshold = (sum0 / count0 + sum255 / count255) / 2;
		else if (count0)
			threshold--;
		else
			threshold++;

		sum0 = count0 = sum255 = count255 = 0;

		for (unsigned i = 0; i < gray.infoHead.biHeight; ++i)
		{
			for (unsigned j = 0; j < gray.infoHead.biWidth; ++j)
			{
				unsigned pixelIndex = i * gray.data.getRow() + j;

				if (pixel_8bit[pixelIndex] > threshold)
				{
					count255++;
					sum255 += pixel_8bit[pixelIndex];
				}
				else
				{
					count0++;
					sum0 += pixel_8bit[pixelIndex];
				}
			}
		}
	} while (count0 == 0 || count255 == 0 || threshold != (sum0 / count0 + sum255 / count255) / 2);

	return threshold;
}

uint8_t generateThreshold_Otsu(BitmapFile gray)
{
	auto pixel_8bit = reinterpret_cast<uint8_t*>(gray.data.getDataPtr());

	int grayCountTable[256] = {};
	double grayPercentTable[256] = {};

	for (unsigned i = 0; i < gray.infoHead.biHeight; ++i)
	{
		for (unsigned j = 0; j < gray.infoHead.biWidth; ++j)
		{
			unsigned pixelIndex = i * gray.data.getRow() + j;
			grayCountTable[pixel_8bit[pixelIndex]]++;
		}
	}

	unsigned numOfPixels = gray.infoHead.biWidth * gray.infoHead.biHeight;
	for (unsigned i = 0; i < 256; ++i)
	{
		grayPercentTable[i] = double(grayCountTable[i]) / numOfPixels;
	}

	unsigned threshold = 0;
	double maxT = 0;

	for (unsigned i = 0; i < 256; ++i)
	{
		double count0 = 0, count255 = 0;
		double sum0 = 0, sum255 = 0;
		for (unsigned j = 0; j < i; ++j)
		{
			count0 += grayPercentTable[j];
			sum0 += grayPercentTable[j] * j;
		}
		for (unsigned k = i; k < 256; ++k)
		{
			count255 += grayPercentTable[k];
			sum255 += grayPercentTable[k] * k;
		}
		double t = count0 * count255
			* (sum0 / count0 - sum255 / count255)
			* (sum0 / count0 - sum255 / count255);
		if (t > maxT)
		{
			maxT = t;
			threshold = i;
		}
	}

	return threshold;
}

BitmapFile binaryImageErosionAndDelation(BitmapFile binary, StructuringElement se, bool retErosion)
{
	BitmapFile dst = binary.clone();

	auto pixel = reinterpret_cast<uint8_t*>(binary.data.getDataPtr());
	auto dst_pixel = reinterpret_cast<uint8_t*>(dst.data.getDataPtr());

	for (unsigned i = 0; i + se.col - 1 < binary.infoHead.biHeight; ++i)
	{
		for (unsigned j = 0; j + se.row - 1 < binary.infoHead.biWidth; ++j)
		{
			bool erosion = false;
			bool dilation = false;

			for (unsigned se_i = 0; se_i < se.col; ++se_i)
			{
				for (unsigned se_j = 0; se_j < se.row; ++se_j)
				{
					int value = pixel[(i + se_i) * binary.infoHead.biWidth + j + se_j] / 255;
					if (se.element[se_i * se.row + se_j] && !value)
						erosion = true;
					if (se.element[se_i * se.row + se_j] && value)
						dilation = true;
				}
			}
			if (retErosion)
				dst_pixel[(i + se.originY) * binary.infoHead.biWidth + j + se.OriginX] = erosion ? 0 : 255;
			else
				dst_pixel[(i + se.originY) * binary.infoHead.biWidth + j + se.OriginX] = dilation ? 255 : 0;


		}
	}

	return dst;
}

BitmapFile binaryImageErosion(BitmapFile binary, StructuringElement se)
{
	return  binaryImageErosionAndDelation(binary, se, true);
}

BitmapFile binaryImageDilation(BitmapFile binary, StructuringElement se)
{
	return binaryImageErosionAndDelation(binary, se, false);
}

BitmapFile binaryImageOpening(BitmapFile binary, StructuringElement se)
{
	auto bmp = binaryImageErosion(binary, se);
	bmp = binaryImageDilation(bmp, se);
	return bmp;
}

BitmapFile binaryImageClosing(BitmapFile binary, StructuringElement se)
{
	auto bmp = binaryImageDilation(binary, se);
	bmp = binaryImageErosion(bmp, se);
	return bmp;
}

void logarithmicOperation(YUVData yuv)
{
	auto yuv_pixel = reinterpret_cast<double(*)[3]>(yuv.data.getDataPtr());


	double Lmax = 0;

	for (unsigned i = 0; i < yuv.infoHead.biHeight; ++i)
		for (unsigned j = 0; j < yuv.infoHead.biWidth; ++j)
			if (yuv_pixel[i * yuv.data.getRow() + j][2] > Lmax)
				Lmax = yuv_pixel[i * yuv.data.getRow() + j][2];

	Lmax /= 255;

	for (unsigned i = 0; i < yuv.infoHead.biHeight; ++i)
		for (unsigned j = 0; j < yuv.infoHead.biWidth; ++j)
		{
			double Lw = yuv_pixel[i * yuv.data.getRow() + j][2] / 255;
			yuv_pixel[i * yuv.data.getRow() + j][2] = 255 * std::log(Lw + 1) / std::log(Lmax + 1);
		}
}

void histogramEqualization8bit(BitmapFile gray)
{
	unsigned numOfPixels = gray.infoHead.biWidth * gray.infoHead.biHeight;
	int numOfGrayLevels[256] = { 0 };
	int grayLevelsMapping[256];
	auto pixel = gray.data.getDataPtr<uint8_t*>();

	for (unsigned i = 0; i < gray.infoHead.biHeight; ++i)
		for (unsigned j = 0; j < gray.infoHead.biWidth; ++j)
			numOfGrayLevels[pixel[i * gray.data.getRow() + j]]++;

	double grayLevel = 0;

	for (int i = 0; i < 256; ++i)
	{
		grayLevel += double(numOfGrayLevels[i]) / numOfPixels * 255;
		grayLevelsMapping[i] = std::round(grayLevel);
	}

	for (unsigned i = 0; i < gray.infoHead.biHeight; ++i)
		for (unsigned j = 0; j < gray.infoHead.biWidth; ++j)
			pixel[i * gray.data.getRow() + j] = grayLevelsMapping[pixel[i * gray.data.getRow() + j]];
}

void histogramEqualization(BitmapFile bmp)
{
	unsigned numOfPixels = bmp.infoHead.biWidth * bmp.infoHead.biHeight * 3;
	int numOfLevels[256] = { 0 };
	int levelsMapping[256];
	auto pixel = bmp.data.getDataPtr<uint8_t(*)[3]>();

	for (unsigned i = 0; i < bmp.infoHead.biHeight; ++i)
		for (unsigned j = 0; j < bmp.infoHead.biWidth; ++j)
		{
			numOfLevels[pixel[i * bmp.data.getRow() + j][0]]++;
			numOfLevels[pixel[i * bmp.data.getRow() + j][1]]++;
			numOfLevels[pixel[i * bmp.data.getRow() + j][2]]++;
		}
	
	double level = 0;

	for (int i = 0; i < 256; ++i)
	{
		level += double(numOfLevels[i]) / numOfPixels * 255;
		levelsMapping[i] = std::round(level);
	}


	for (unsigned i = 0; i < bmp.infoHead.biHeight; ++i)	
		for (unsigned j = 0; j < bmp.infoHead.biWidth; ++j)
		{
			pixel[i * bmp.data.getRow() + j][0] = levelsMapping[pixel[i * bmp.data.getRow() + j][0]];
			pixel[i * bmp.data.getRow() + j][1] = levelsMapping[pixel[i * bmp.data.getRow() + j][1]];
			pixel[i * bmp.data.getRow() + j][2] = levelsMapping[pixel[i * bmp.data.getRow() + j][2]];
		}

}


void histogramEqualization_2(BitmapFile bmp)
{
	unsigned numOfPixels = bmp.infoHead.biWidth * bmp.infoHead.biHeight;
	int numOfRedLevels[256] = { 0 };
	int numOfGreenLevels[256] = { 0 };
	int numOfBlueLevels[256] = { 0 };
	int redLevelsMapping[256], greenLevelsMapping[256], blueLevelsMapping[256];
	auto pixel = bmp.data.getDataPtr<uint8_t(*)[3]>();

	for (unsigned i = 0; i < bmp.infoHead.biHeight; ++i)
		for (unsigned j = 0; j < bmp.infoHead.biWidth; ++j)
		{
			numOfBlueLevels[pixel[i * bmp.data.getRow() + j][0]]++;
			numOfGreenLevels[pixel[i * bmp.data.getRow() + j][1]]++;
			numOfRedLevels[pixel[i * bmp.data.getRow() + j][2]]++;
		}

	double redLevel = 0, greenLevel = 0, blueLevel = 0;

	for (int i = 0; i < 256; ++i)
	{
		redLevel += double(numOfRedLevels[i]) / numOfPixels * 255;
		redLevelsMapping[i] = std::round(redLevel);

		greenLevel += double(numOfGreenLevels[i]) / numOfPixels * 255;
		greenLevelsMapping[i] = std::round(greenLevel);

		blueLevel += double(numOfBlueLevels[i]) / numOfPixels * 255;
		blueLevelsMapping[i] = std::round(blueLevel);
	}


	for (unsigned i = 0; i < bmp.infoHead.biHeight; ++i)
		for (unsigned j = 0; j < bmp.infoHead.biWidth; ++j)
		{
			pixel[i * bmp.data.getRow() + j][0] = blueLevelsMapping[pixel[i * bmp.data.getRow() + j][0]];
			pixel[i * bmp.data.getRow() + j][1] = greenLevelsMapping[pixel[i * bmp.data.getRow() + j][1]];
			pixel[i * bmp.data.getRow() + j][2] = redLevelsMapping[pixel[i * bmp.data.getRow() + j][2]];
		}

}