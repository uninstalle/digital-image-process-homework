#include <iostream>
#include "mat.h"
#include "bmp.h"

//#define TEST_RGB_CONVERSION
//#define TEST_GRAY
//#define TEST_BINARIZATION
//#define TEST_BINARY_OPERATION
//#define TEST_HISTOGRAM_EQUALIZATION
//#define TEST_LUMINANCE_CHANGING
//#define TEST_LOGARITHMIC_OPERATION
#define TEST_GEO_TRANSFORM

int main()
{
	BitmapFile file;
	try
	{
		file = loadBMPFile("phub.bmp");
	}
	catch (std::runtime_error & e)
	{
		std::cout << e.what();
		return -1;
	}

#ifdef TEST_RGB_CONVERSION
	// test RGB to YUV and YUV to RGB convert
	auto file_yuv = file;
	file_yuv.data = convertRGBtoYUV(file_yuv.data);
	auto file_yuv_backup = file_yuv.clone();
	file_yuv.data = convertYUVtoRGB(file_yuv.data);
	saveBMPFile("yuv.bmp",file_yuv);

	// test RGB to Lab and Lab to RGB convert
	auto file_lab = file;
	file_lab.data = convertRGBtoLab(file_lab.data);
	auto file_lab_backup = file_lab.clone();
	file_lab.data = convertLabtoRGB(file_lab.data);
	saveBMPFile("lab.bmp", file_lab);



#ifdef TEST_GRAY
	// test gray bmp generating
	auto file_yuv_gray = toGrayYUV(file_yuv_backup);
	saveBMPFile("yuv_gray.bmp", file_yuv_gray);
	auto file_lab_gray = toGrayLab(file_lab_backup);
	saveBMPFile("lab_gray.bmp", file_lab_gray);

#ifdef TEST_BINARIZATION
	// test binarization
	auto file_binary = file_yuv_gray.clone();
	auto file_binary_otsu = file_yuv_gray.clone();
	binarizeGray(generateThreshold(file_binary.data), file_binary.data);
	binarizeGray(generateThreshold_Otsu(file_binary_otsu.data), file_binary_otsu.data);
	saveBMPFile("binary.bmp", file_binary);
	saveBMPFile("binary_otsu.bmp", file_binary_otsu);

#ifdef TEST_BINARY_OPERATION
	// test erosion, dilation, opening and closing
	StructuringElement se{ 3,3,1,1,
	{1,1,1,
			1,1,1,
			1,1,1} };
	auto file_erosion = binaryImageErosion(file_binary_otsu, se);
	saveBMPFile("erosion.bmp", file_erosion);

	auto file_dilation = binaryImageDilation(file_binary_otsu, se);
	saveBMPFile("dilation.bmp", file_dilation);

	auto file_opening = binaryImageOpening(file_binary_otsu, se);
	saveBMPFile("opening.bmp", file_opening);

	auto file_closing = binaryImageClosing(file_binary_otsu, se);
	saveBMPFile("closing.bmp", file_closing);
#endif

#ifdef TEST_HISTOGRAM_EQUALIZATION
	// test histogram equalization
	auto file_gray_histoeq = file_yuv_gray.clone();
	histogramEqualizationGray(file_gray_histoeq.data);
	saveBMPFile("gray_histoeq.bmp", file_gray_histoeq);

	auto file_histoeq_1 = file.clone();
	auto file_histoeq_2 = file.clone();
	histogramEqualization(file_histoeq_1.data);
	histogramEqualization_2(file_histoeq_2.data);
	saveBMPFile("histoeq_1.bmp", file_histoeq_1);
	saveBMPFile("histoeq_2.bmp", file_histoeq_2);
#endif
	
#endif

#endif
	

#ifdef TEST_LUMINANCE_CHANGING
	// test luminance changing
	auto file_yuv_lu = file_yuv_backup.clone();
	changeLuminanceYUV(20 * 2.55, file_yuv_lu.data);
	file_yuv_lu.data = convertYUVtoRGB(file_yuv_lu.data);
	saveBMPFile("yuv_luminance.bmp", file_yuv_lu);

	auto file_lab_lu = file_lab_backup.clone();
	changeLuminanceLab(20, file_lab_lu.data);
	file_lab_lu.data = convertLabtoRGB(file_lab_lu.data);
	saveBMPFile("lab_luminance.bmp", file_lab_lu);
#endif



#ifdef TEST_LOGARITHMIC_OPERATION
	// test logarithmic operation
	auto file_yuv_logop = file_yuv_backup.clone();
	logarithmicOperationYUV(file_yuv_logop.data);
	file_yuv_logop.data = convertYUVtoRGB(file_yuv_logop.data);
	saveBMPFile("yuv_logop.bmp", file_yuv_logop);
	
	auto file_lab_logop = file_lab_backup.clone();
	logarithmicOperationLab(file_lab_logop.data);
	file_lab_logop.data = convertLabtoRGB(file_lab_logop.data);
	saveBMPFile("lab_logop.bmp", file_lab_logop);
#endif


#endif


#ifdef TEST_GEO_TRANSFORM
	auto transmat = translate(100, 50);
	auto file_geo_trans = buildBMP(geometricTransform(file.data, transmat));
	/*saveBMPFile("geo_translate.bmp",file_geo_trans);

	transmat = rotate(1);
	file_geo_trans = buildBMP(geometricTransform(file.data, transmat));
	saveBMPFile("geo_rotate.bmp", file_geo_trans);

	transmat = scale(2, 1.5);
	file_geo_trans = buildBMP(geometricTransform(file.data, transmat));
	saveBMPFile("geo_scale.bmp", file_geo_trans);

	transmat = shear(0, 1);
	file_geo_trans = buildBMP(geometricTransform(file.data, transmat));
	saveBMPFile("geo_shear.bmp", file_geo_trans);

	transmat = mirror(0, 1);
	file_geo_trans = buildBMP(geometricTransform(file.data, transmat));
	saveBMPFile("geo_mirror.bmp", file_geo_trans);

	transmat = scale(0.5, 1.5) * rotate(3.14159 / 4) * translate(100, 0);
	file_geo_trans = buildBMP(geometricTransform(file.data, transmat));
	saveBMPFile("geo_multi.bmp", file_geo_trans);*/

	transmat = scale(0.5, 0.5);
	auto file_1 = buildBMP(geometricTransform(file.data, transmat, true));

	transmat = scale(2, 2);
	file_geo_trans = buildBMP(geometricTransform(file_1.data, transmat,true));

	saveBMPFile("phub_pro.bmp", file_geo_trans);
	
	
#endif


	return 0;
}