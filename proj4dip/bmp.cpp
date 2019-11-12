#include "bmp.h"
#include <fstream>

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


BitmapFile toGrayRGB(BitmapFile& bmp)
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

BitmapFile toGrayYUV(BitmapFile& bmp)
{
	auto yuv_pixel = bmp.data.getDataPtr<double(*)[3]>();
	BitmapFile bmp_8bit = build8bitBMPHead(bmp.fileHead, bmp.infoHead);
	bmp_8bit.data = Mat(bmp.data.getRow(), bmp.data.getCol(), 8);
	auto pixel_8bit = bmp_8bit.data.getDataPtr<uint8_t*>();

	unsigned numOfPixels = bmp.data.getRow() * bmp.data.getCol();
	for (unsigned i = 0; i < numOfPixels; ++i)
		pixel_8bit[i] = yuv_pixel[i][2];
	return bmp_8bit;
}

BitmapFile toGrayLab(BitmapFile& bmp)
{
	auto lab_pixel = bmp.data.getDataPtr<double(*)[3]>();
	BitmapFile bmp_8bit = build8bitBMPHead(bmp.fileHead, bmp.infoHead);
	bmp_8bit.data = Mat(bmp.data.getRow(), bmp.data.getCol(), 8);
	auto pixel_8bit = bmp_8bit.data.getDataPtr<uint8_t*>();

	unsigned numOfPixels = bmp.data.getRow() * bmp.data.getCol();
	for (unsigned i = 0; i < numOfPixels; ++i)
		pixel_8bit[i] = lab_pixel[i][2] * 2.55;
	return bmp_8bit;
}

BitmapFile binaryImageErosion(BitmapFile binary, StructuringElement se)
{
	auto newFile = binary;
	newFile.data = binaryImageErosionAndDilation(newFile.data, se, true);
	return  newFile;
}

BitmapFile binaryImageDilation(BitmapFile binary, StructuringElement se)
{
	auto newFile = binary;
	newFile.data = binaryImageErosionAndDilation(newFile.data, se, false);
	return  newFile;
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

BitmapFile buildBMP(Mat mat)
{
	BitmapFile bmp;

	bmp.data = mat;
	bmp.fileHead = { 19778,0
	,0,0,54 };
	bmp.fileHead.bfSize = bmp.fileHead.bfOffBits + bmp.data.getRow() * bmp.data.getCol() * 3;
	bmp.infoHead = { 40,bmp.data.getRow(),bmp.data.getCol(),
	1,24,0,0,5669,5669,0,0 };
	return bmp;
}
