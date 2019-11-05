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
	os.write(bp.data.getRawDataPtr(), dataSize);
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
	memcpy(clonedMat.getRawDataPtr(), data.get(), size);
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
		BMPfs.read(bmp.palette.data.getRawDataPtr(), bmp.palette.data.getSize());
	}

	bmp.data = Mat(bmp.infoHead.biWidth, bmp.infoHead.biHeight, bmp.infoHead.biBitCount);

	unsigned alignedByteWidth = (bmp.infoHead.biWidth * bmp.infoHead.biBitCount / 8 + 3) / 4 * 4;
	unsigned byteWidth = bmp.infoHead.biWidth * bmp.infoHead.biBitCount / 8;
	//not aligned, using the easy way to write
	if (alignedByteWidth == byteWidth)
		BMPfs.read(bmp.data.getRawDataPtr(), bmp.data.getSize());
	else
	{
		char* zeroBuffer = new char[4];

		char* dp = bmp.data.getRawDataPtr();
		for (unsigned i = 0; i < bmp.infoHead.biHeight; ++i)
		{
			BMPfs.read(dp, byteWidth);
			BMPfs.read(zeroBuffer, alignedByteWidth - byteWidth);
			dp += byteWidth / sizeof(char);
		}
		delete[]zeroBuffer;
	}

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

	unsigned alignedByteWidth = (bmp.infoHead.biWidth * bmp.infoHead.biBitCount / 8 + 3) / 4 * 4;
	unsigned byteWidth = bmp.infoHead.biWidth * bmp.infoHead.biBitCount / 8;
	//not aligned, using the easy way to write
	if (alignedByteWidth == byteWidth)
		BMPfs.write(bmp.data.getRawDataPtr(), bmp.data.getSize());
	else
	{
		char zero[4] = { 0,0,0,0 };

		char* dp = bmp.data.getRawDataPtr();
		for (unsigned i = 0; i < bmp.infoHead.biHeight; ++i)
		{
			BMPfs.write(dp, byteWidth);
			BMPfs.write(zero, alignedByteWidth - byteWidth);
			dp += byteWidth / sizeof(char);
		}
	}
	BMPfs.close();
}

Mat convertRGBtoYUV(Mat& rgb)
{
	auto pixel = rgb.getDataPtr<uint8_t(*)[3]>();

	Mat yuv = YUVData(rgb.getRow(), rgb.getCol());
	auto yuv_pixel = yuv.getDataPtr<double(*)[3]>();

	double k1 = 0.5 / 0.435, k2 = 0.5 / 0.615;

	unsigned numOfPixels = rgb.getRow() * rgb.getCol();
	for (unsigned i = 0; i < numOfPixels; ++i)
	{
		unsigned char b = pixel[i][0],
			g = pixel[i][1],
			r = pixel[i][2];
		double y = 0.299 * r + 0.587 * g + 0.114 * b,
			u = k1 * (-0.147 * r + -0.289 * g + 0.435 * b),
			v = k2 * (0.615 * r + -0.515 * g + -0.100 * b);
		yuv_pixel[i][0] = v;
		yuv_pixel[i][1] = u;
		yuv_pixel[i][2] = y;
	}

	return yuv;
}

Mat convertYUVtoRGB(Mat& yuv)
{
	auto yuv_pixel = yuv.getDataPtr<double(*)[3]>();

	Mat rgb = RGBData(yuv.getRow(), yuv.getCol(), sizeof(uint8_t) * 8 * 3);
	auto pixel = rgb.getDataPtr<uint8_t(*)[3]>();

	auto rangeFrom0to255 = [](double a, double b, double c) -> double {
		if ((a + b + c) > 255)
			return 255;
		else if ((a + b + c) < 0)
			return 0;
		else
			return a + b + c;
	};
	double k1 = 0.435 / 0.5, k2 = 0.615 / 0.5;

	unsigned numOfPixels = yuv.getRow() * yuv.getCol();
	for (unsigned i = 0; i < numOfPixels; ++i)
	{
		double v = k2 * yuv_pixel[i][0],
			u = k1 * yuv_pixel[i][1],
			y = yuv_pixel[i][2];


		uint8_t r = rangeFrom0to255(y, -0.00004 * u, 1.139828 * v),
			g = rangeFrom0to255(0.999605 * y, -0.395414 * u, -0.5805 * v),
			b = rangeFrom0to255(1.002036 * y, 2.036137 * u, -0.000482 * v);

		pixel[i][0] = b;
		pixel[i][1] = g;
		pixel[i][2] = r;
	}

	return rgb;
}

Mat convertRGBtoXYZ(Mat& rgb)
{
	auto pixel = rgb.getDataPtr<uint8_t(*)[3]>();

	Mat xyz = XYZData(rgb.getRow(), rgb.getCol());
	auto xyz_pixel = xyz.getDataPtr<double(*)[3]>();

	double k1 = 1 / 0.950456, k2 = 1 / 1.088754;

	unsigned numOfPixels = rgb.getRow() * rgb.getCol();
	for (unsigned i = 0; i < numOfPixels; ++i)
	{
		unsigned char b = pixel[i][0],
			g = pixel[i][1],
			r = pixel[i][2];
		double x = k1 * (0.412453 * r + 0.357580 * g + 0.180423 * b),
			y = 0.212671 * r + 0.715160 * g + 0.072169 * b,
			z = k2 * (0.019334 * r + 0.119193 * g + 0.950227 * b);
		xyz_pixel[i][0] = z;
		xyz_pixel[i][1] = y;
		xyz_pixel[i][2] = x;
	}

	return xyz;
}

double labFunction(double t)
{
	if (t > std::pow(6.0 / 29.0, 3))
		return std::pow(t, 1.0 / 3.0);
	else
		return 1.0 / 3.0 * (29.0 / 6.0) * (29.0 / 6.0) * t + 4.0 / 29.0;
}

double iLabFunction(double t)
{
	if (t > (6.0 / 29.0))
		return t * t * t;
	else
		return 3 * (6.0 / 29.0) * (6.0 / 29.0) * (t - 4.0 / 29.0);
}

Mat convertXYZtoLab(Mat& xyz)
{
	auto xyz_pixel = xyz.getDataPtr<double(*)[3]>();

	Mat lab = LabData(xyz.getRow(), xyz.getCol());
	auto lab_pixel = lab.getDataPtr<double(*)[3]>();

	unsigned numOfPixels = xyz.getRow() * xyz.getCol();
	for (unsigned i = 0; i < numOfPixels; ++i)
	{
		double z = xyz_pixel[i][0] / 255,
			y = xyz_pixel[i][1] / 255,
			x = xyz_pixel[i][2] / 255;
		double l = 116 * labFunction(y) - 16,
			a = 500 * (labFunction(x) - labFunction(y)),
			b = 200 * (labFunction(y) - labFunction(z));
		lab_pixel[i][0] = b;
		lab_pixel[i][1] = a;
		lab_pixel[i][2] = l;
	}

	return lab;
}

Mat convertXYZtoRGB(Mat& xyz)
{
	auto xyz_pixel = xyz.getDataPtr<double(*)[3]>();

	Mat rgb = Mat(xyz.getRow(), xyz.getCol(), sizeof(uint8_t) * 8 * 3);
	auto pixel = rgb.getDataPtr<uint8_t(*)[3]>();

	auto rangeFrom0to255 = [](double a, double b, double c) -> double {
		if ((a + b + c) > 255)
			return 255;
		else if ((a + b + c) < 0)
			return 0;
		else
			return a + b + c;
	};
	double k1 = 0.950456, k2 = 1.088754;

	unsigned numOfPixels = xyz.getRow() * xyz.getCol();
	for (unsigned i = 0; i < numOfPixels; ++i)
	{
		double z = k2 * xyz_pixel[i][0],
			y = xyz_pixel[i][1],
			x = k1 * xyz_pixel[i][2];


		uint8_t r = rangeFrom0to255(3.240479 * x, -1.537150 * y, -0.498535 * z),
			g = rangeFrom0to255(-0.969256 * x, 1.875992 * y, 0.041556 * z),
			b = rangeFrom0to255(0.055648 * x, -0.204043 * y, 1.057311 * z);

		pixel[i][0] = b;
		pixel[i][1] = g;
		pixel[i][2] = r;
	}

	return rgb;
}

Mat convertLabtoXYZ(Mat& lab)
{
	auto lab_pixel = lab.getDataPtr<double(*)[3]>();

	Mat xyz = XYZData(lab.getRow(), lab.getCol());
	auto xyz_pixel = xyz.getDataPtr<double(*)[3]>();

	unsigned numOfPixels = lab.getRow() * lab.getCol();
	for (unsigned i = 0; i < numOfPixels; ++i)
	{
		double b = lab_pixel[i][0],
			a = lab_pixel[i][1],
			l = lab_pixel[i][2];
		double x = 255 * iLabFunction(1.0 / 116.0 * (l + 16) + 1.0 / 500.0 * a),
			y = 255 * iLabFunction(1.0 / 116.0 * (l + 16)),
			z = 255 * iLabFunction(1.0 / 116.0 * (l + 16) - 1.0 / 200.0 * b);
		xyz_pixel[i][0] = z;
		xyz_pixel[i][1] = y;
		xyz_pixel[i][2] = x;
	}

	return xyz;
}

Mat convertRGBtoLab(Mat& rgb)
{
	auto m = convertRGBtoXYZ(rgb);
	return convertXYZtoLab(m);
}

Mat convertLabtoRGB(Mat& lab)
{
	auto m = convertLabtoXYZ(lab);
	return convertXYZtoRGB(m);
}

static BitmapPalette GrayPalette;
static BitmapPalette BinaryPalette;

struct BitmapPalette getGrayPalette()
{
	if (!GrayPalette.index) {
		BitmapPalette palette(256);
		auto pixel = palette.data.getDataPtr<uint8_t(*)[4]>();
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
		auto pixel = palette.data.getDataPtr<uint8_t(*)[4]>();
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

BitmapFile toGray(BitmapFile& bmp)
{
	if (bmp.data.getType() == Mat::TYPE::YUV) {

		auto yuv_pixel = bmp.data.getDataPtr<double(*)[3]>();
		BitmapFile bmp_8bit = bmp;
		bmp_8bit.data = Mat(bmp.data.getRow(), bmp.data.getCol(), 8);
		auto pixel_8bit = bmp_8bit.data.getDataPtr<uint8_t*>();

		unsigned numOfPixels = bmp.data.getRow() * bmp.data.getCol();
		for (unsigned i = 0; i < numOfPixels; ++i)
			pixel_8bit[i] = yuv_pixel[i][2];

		return bmp_8bit;
	}
	else
	{
		auto pixel = bmp.data.getDataPtr<uint8_t(*)[3]>();
		BitmapFile bmp_8bit = build8bitBMPHead(bmp.fileHead, bmp.infoHead);
		bmp_8bit.data = Mat(bmp.data.getRow(), bmp.data.getCol(), 8);
		auto pixel_8bit = bmp_8bit.data.getDataPtr<uint8_t*>();

		unsigned numOfPixels = bmp.data.getRow() * bmp.data.getCol();
		for (unsigned i = 0; i < numOfPixels; ++i) {
			unsigned char gray = unsigned char(
				pixel[i][2] * 0.299 + pixel[i][1] * 0.587 + pixel[i][0] * 0.114);
			pixel_8bit[i] = gray;

		}


		return bmp_8bit;
	}
}


void changeLuminanceYUV(double deltaValue, Mat& yuv)
{
	auto yuv_pixel = yuv.getDataPtr<double(*)[3]>();

	unsigned numOfPixels = yuv.getRow() * yuv.getCol();
	for (unsigned i = 0; i < numOfPixels; ++i) {
		yuv_pixel[i][2] += deltaValue;
		if (yuv_pixel[i][2] > 255) yuv_pixel[i][2] = 255;
		if (yuv_pixel[i][2] < 0) yuv_pixel[i][2] = 0;
	}
}
void changeLuminanceLab(double deltaValue, Mat& lab)
{
	auto lab_pixel = lab.getDataPtr<double(*)[3]>();

	unsigned numOfPixels = lab.getRow() * lab.getCol();
	for (unsigned i = 0; i < numOfPixels; ++i) {
		lab_pixel[i][2] += deltaValue;
		if (lab_pixel[i][2] > 100) lab_pixel[i][2] = 100;
		if (lab_pixel[i][2] < 0) lab_pixel[i][2] = 0;
	}
}

void binarize8BitFile(uint8_t threshold, BitmapFile gray)
{
	auto pixel_8bit = gray.data.getDataPtr<uint8_t*>();

	unsigned numOfPixels = gray.data.getRow() * gray.data.getCol();
	for (unsigned i = 0; i < numOfPixels; ++i)
	{

		if (pixel_8bit[i] > threshold)
			pixel_8bit[i] = 255;
		else
			pixel_8bit[i] = 0;

	}
}

uint8_t generateThreshold(BitmapFile gray)
{
	auto pixel_8bit = gray.data.getDataPtr<uint8_t*>();
	unsigned numOfPixels = gray.data.getRow() * gray.data.getCol();


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

		for (unsigned i = 0; i < numOfPixels; ++i) {
			if (pixel_8bit[i] > threshold)
			{
				count255++;
				sum255 += pixel_8bit[i];
			}
			else
			{
				count0++;
				sum0 += pixel_8bit[i];
			}
		}
	} while (count0 == 0 || count255 == 0 || threshold != (sum0 / count0 + sum255 / count255) / 2);

	return threshold;
}

uint8_t generateThreshold_Otsu(BitmapFile gray)
{
	auto pixel_8bit = gray.data.getDataPtr<uint8_t*>();
	unsigned numOfPixels = gray.data.getRow() * gray.data.getCol();

	int grayCountTable[256] = {};
	double grayPercentTable[256] = {};

	for (unsigned i = 0; i < numOfPixels; ++i)
		grayCountTable[pixel_8bit[i]]++;

	for (unsigned i = 0; i < 256; ++i)
		grayPercentTable[i] = double(grayCountTable[i]) / numOfPixels;

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

BitmapFile binaryImageErosionAndDilation(BitmapFile& binary, StructuringElement se, bool retErosion)
{
	BitmapFile dst = binary.clone();

	auto pixel = binary.data.getDataPtr<uint8_t*>();
	auto dst_pixel = dst.data.getDataPtr<uint8_t*>();

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
	return  binaryImageErosionAndDilation(binary, se, true);
}

BitmapFile binaryImageDilation(BitmapFile binary, StructuringElement se)
{
	return binaryImageErosionAndDilation(binary, se, false);
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

void logarithmicOperationYUV(BitmapFile& bmp)
{
	auto yuv_pixel = bmp.data.getDataPtr<double(*)[3]>();
	unsigned numOfPixels = bmp.data.getRow() * bmp.data.getCol();

	double Lmax = 0;

	for (unsigned i = 0; i < numOfPixels; ++i)
		if (yuv_pixel[i][2] > Lmax)
			Lmax = yuv_pixel[i][2];

	Lmax /= 255;

	for (unsigned i = 0; i < numOfPixels; ++i)
	{
		double Lw = yuv_pixel[i][2] / 255;
		yuv_pixel[i][2] = 255 * std::log(Lw + 1) / std::log(Lmax + 1);
	}
}

void logarithmicOperationLab(BitmapFile& bmp)
{
	auto lab_pixel = bmp.data.getDataPtr<double(*)[3]>();
	unsigned numOfPixels = bmp.data.getRow() * bmp.data.getCol();

	double Lmax = 0;

	for (unsigned i = 0; i < numOfPixels; ++i)
		if (lab_pixel[i][2] > Lmax)
			Lmax = lab_pixel[i][2];

	Lmax /= 100;

	for (unsigned i = 0; i < numOfPixels; ++i)
	{
		double Lw = lab_pixel[i][2] / 100;
		lab_pixel[i][2] = 100 * std::log(Lw + 1) / std::log(Lmax + 1);
	}
}

void histogramEqualization8bit(BitmapFile gray)
{
	unsigned numOfPixels = gray.data.getRow() * gray.data.getCol();
	int numOfGrayLevels[256] = { 0 };
	int grayLevelsMapping[256];
	auto pixel = gray.data.getDataPtr<uint8_t*>();

	for (unsigned i = 0; i < numOfPixels; ++i)
		numOfGrayLevels[pixel[i]]++;

	double grayLevel = 0;

	for (int i = 0; i < 256; ++i)
	{
		grayLevel += double(numOfGrayLevels[i]) / numOfPixels * 255;
		grayLevelsMapping[i] = std::round(grayLevel);
	}

	for (unsigned i = 0; i < numOfPixels; ++i)
		pixel[i] = grayLevelsMapping[pixel[i]];
}

void histogramEqualization(BitmapFile bmp)
{
	unsigned numOfPixels = bmp.infoHead.biWidth * bmp.infoHead.biHeight;
	int numOfLevels[256] = { 0 };
	int levelsMapping[256];
	auto pixel = bmp.data.getDataPtr<uint8_t(*)[3]>();

	for (unsigned i = 0; i < numOfPixels; ++i) {
		numOfLevels[pixel[i][0]]++;
		numOfLevels[pixel[i][1]]++;
		numOfLevels[pixel[i][2]]++;
	}

	double level = 0;

	for (int i = 0; i < 256; ++i)
	{
		level += double(numOfLevels[i]) / numOfPixels / 3 * 255;
		levelsMapping[i] = std::round(level);
	}


	for (unsigned i = 0; i < numOfPixels; ++i) {
		pixel[i][0] = levelsMapping[pixel[i][0]];
		pixel[i][1] = levelsMapping[pixel[i][1]];
		pixel[i][2] = levelsMapping[pixel[i][2]];
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


	for (unsigned i = 0; i < numOfPixels; ++i) {
		numOfBlueLevels[pixel[i][0]]++;
		numOfGreenLevels[pixel[i][1]]++;
		numOfRedLevels[pixel[i][2]]++;
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


	for (unsigned i = 0; i < numOfPixels; ++i) {
		pixel[i][0] = blueLevelsMapping[pixel[i][0]];
		pixel[i][1] = greenLevelsMapping[pixel[i][1]];
		pixel[i][2] = redLevelsMapping[pixel[i][2]];
	}

}