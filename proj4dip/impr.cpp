#include <iostream>
#include <fstream>
#include <string>
#include <cmath>

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
	uint8_t(*palette)[4];
	int index;
	BitmapPalette() : palette(nullptr), index(0) {}
	BitmapPalette(int index);
	BitmapPalette(BitmapPalette&& bp) noexcept;
	BitmapPalette& operator=(BitmapPalette&& bp) noexcept;
	~BitmapPalette();
};

std::istream& operator>>(std::istream& is, BitmapFileHead& bfh)
{
	is.read(reinterpret_cast<char*>(&bfh.bfType), sizeof(uint16_t));
	is.read(reinterpret_cast<char*>(&bfh.bfSize), sizeof(uint32_t));
	is.read(reinterpret_cast<char*>(&bfh.bfReserved1), sizeof(uint16_t));
	is.read(reinterpret_cast<char*>(&bfh.bfReserved2), sizeof(uint16_t));
	is.read(reinterpret_cast<char*>(&bfh.bfOffBits), sizeof(uint32_t));
	return is;
}

struct BitmapFile
{
	BitmapFileHead fileHead;
	BitmapInfoHead infoHead;
	BitmapPalette palette;
	void* data = nullptr;
	BitmapFile clone();
	//todo: destructor
};

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

BitmapPalette::BitmapPalette(int index) : index(index)
{
	palette = new uint8_t[index][4];
}

BitmapPalette::BitmapPalette(BitmapPalette&& bp) noexcept
{
	palette = bp.palette;
	index = bp.index;
	bp.palette = nullptr;
	bp.index = 0;
}

BitmapPalette& BitmapPalette::operator=(BitmapPalette&& bp) noexcept
{
	palette = bp.palette;
	index = bp.index;
	bp.palette = nullptr;
	bp.index = 0;
	return *this;
}

BitmapPalette::~BitmapPalette()
{
	delete[] palette;
}

BitmapFile BitmapFile::clone()
{
	BitmapFile clonedBitmapFile;
	clonedBitmapFile.fileHead = fileHead;
	clonedBitmapFile.infoHead = infoHead;
	//todo
	//clonedBitmapFile.palette = palette;
	unsigned dataSize = infoHead.biWidth * infoHead.biHeight * infoHead.biBitCount / 8;
	clonedBitmapFile.data = new uint8_t[dataSize];
	memcpy(clonedBitmapFile.data, data, dataSize);
	return clonedBitmapFile;
}

BitmapFile loadBMPFile(std::string filename)
{
	std::fstream bmpFile(filename, std::ios::binary | std::ios::in);
	if (!bmpFile)
		throw std::runtime_error("Open BMP file failed.");
	BitmapFile file;
	bmpFile >> file.fileHead;
	bmpFile >> file.infoHead;
	if (file.infoHead.biBitCount < 16)
	{
		file.palette = BitmapPalette(std::pow(2, file.infoHead.biBitCount));
		bmpFile.read(reinterpret_cast<char*>(file.palette.palette), file.palette.index * 4 * sizeof(uint8_t));
	}
	unsigned dataSize = file.infoHead.biWidth * file.infoHead.biHeight * file.infoHead.biBitCount / 8;
	file.data = new uint8_t[dataSize];
	bmpFile.read(reinterpret_cast<char*>(file.data), dataSize);
	bmpFile.close();
	return file;
}

void saveBMPFile(std::string filename, BitmapFile& bmp)
{
	std::fstream bmpFile(filename, std::ios::binary | std::ios::out);
	if (!bmpFile)
		throw std::runtime_error("Open BMP file failed");
	bmpFile << bmp.fileHead;
	bmpFile << bmp.infoHead;
	//todo
	//bmpFile >> bmp.palette;
	unsigned dataSize = bmp.infoHead.biWidth * bmp.infoHead.biHeight * bmp.infoHead.biBitCount / 8;
	bmpFile.write(reinterpret_cast<char*>(bmp.data), dataSize);
	bmpFile.close();
}

void convertRGBtoYUV(BitmapFile& bmp)
{
	uint8_t(*pixel)[3] = reinterpret_cast<uint8_t(*)[3]>(bmp.data);
	unsigned numOfPixels = bmp.infoHead.biWidth * bmp.infoHead.biHeight;
	double k1 = 0.5 / 0.435, k2 = 0.5 / 0.615;
	for (int i = 0; i < numOfPixels; ++i)
	{
		/*
		 * Warning: known problem
		 * see description in convertYUVtoRGB
		 */

		unsigned char b = pixel[i][0], g = pixel[i][1], r = pixel[i][2];
		unsigned char y = 0.299 * r + 0.587 * g + 0.114 * b,
			u = k1 * (-0.147 * r + -0.289 * g + 0.435 * b),
			v = k2 * (0.615 * r + -0.515 * g + -0.100 * b);
		pixel[i][0] = v, pixel[i][1] = u, pixel[i][2] = y;
	}
}

void toGray(BitmapFile& bmp)
{
	uint8_t(*pixel)[3] = reinterpret_cast<uint8_t(*)[3]>(bmp.data);
	unsigned numOfPixels = bmp.infoHead.biWidth * bmp.infoHead.biHeight;
	for (int i = 0; i < numOfPixels; ++i)
	{
		unsigned char gray = pixel[i][2] % 255;
		pixel[i][0] = pixel[i][1] = pixel[i][2] = gray;
	}
}

void changeLuminanceValue(int deltaValue, BitmapFile& bmp)
{
	uint8_t(*pixel)[3] = reinterpret_cast<uint8_t(*)[3]>(bmp.data);
	unsigned numOfPixels = bmp.infoHead.biWidth * bmp.infoHead.biHeight;
	for (int i = 0; i < numOfPixels; ++i)
	{
		int luminance = pixel[i][2] + static_cast<int>(deltaValue);
		if (luminance > 255)
			luminance = 255;
		if (luminance < 0)
			luminance = 0;
		pixel[i][2] = luminance;
	}
}

void convertYUVtoRGB(BitmapFile& bmp)
{
	uint8_t(*pixel)[3] = reinterpret_cast<uint8_t(*)[3]>(bmp.data);
	unsigned numOfPixels = bmp.infoHead.biWidth * bmp.infoHead.biHeight;
	for (int i = 0; i < numOfPixels; ++i)
	{
		/*
		 * Warning:
		 * This is a temporary way to fix YUV to RGB transformation.
		 * Since UV's value could be less than 0, unsigned char cannot store it properly.
		 * UV's value is saved in an unsigned char variable, but is in fact a char variable.
		 * Thus here char is used to perform YUV to RGB transformation.
		 * Known problem is, UV's value could be bigger than 128, which cannot be stored in char,
		 * making the transformation lossy.
		 */

		char v = pixel[i][0], u = pixel[i][1];
		unsigned char y = pixel[i][2];
		double k1 = 0.435 / 0.5, k2 = 0.615 / 0.5;
		auto field = [](double a, double b, double c) -> double {
			if ((a + b + c) > 255)
				return 255;
			else if ((a + b + c) < 0)
				return 0;
			else
				return a + b + c;
		};
		unsigned char r = field(y, k1 * -0.00004 * u, k2 * 1.139828 * v),
			g = field(0.999605 * y, k1 * -0.395414 * u, k2 * -0.5805 * v),
			b = field(1.002036 * y, k1 * 2.036137 * u, k2 * -0.000482 * v);
		pixel[i][0] = b, pixel[i][1] = g, pixel[i][2] = r;
	}
}

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
	convertRGBtoYUV(file);

	auto file2 = file.clone();
	toGray(file2);
	saveBMPFile("pxs_gray.bmp", file2);

	auto file3 = file.clone();
	changeLuminanceValue(20, file3);
	convertYUVtoRGB(file3);
	saveBMPFile("pxs+20.bmp", file3);

	auto file4 = file.clone();
	changeLuminanceValue(50, file4);
	convertYUVtoRGB(file4);
	saveBMPFile("pxs+50.bmp", file4);

	auto file5 = file.clone();
	changeLuminanceValue(100, file5);
	convertYUVtoRGB(file5);
	saveBMPFile("pxs+100.bmp", file5);

	auto file6 = file.clone();
	changeLuminanceValue(-100, file6);
	convertYUVtoRGB(file6);
	saveBMPFile("pxs-100.bmp", file6);

	return 0;
}