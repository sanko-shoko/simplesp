#include "simplesp.h"

using namespace sp;

int main(){

    Mem2<Col3> src;
    Mem2<Byte> gry;
    {
        SP_ASSERT(loadBMP(src, SP_DATA_DIR  "/image/Lenna.bmp"));
        saveBMP(src, "color.bmp");

        cnvImg(gry, src);
        saveBMP(gry, "gray.bmp");
    }

    // gaussian
    {
        Mem2<Byte> dst;
        gaussianFilter(dst, gry, 2.0);
        saveBMP(dst, "gaussian_g.bmp");
    }
    {
        Mem2<Col3> dst;
        gaussianFilter<Col3, Byte>(dst, src, 2.0);
        saveBMP(dst, "gaussian_c.bmp");
    }

    // bilateral
    {
        Mem2<Byte> dst;
        bilateralFilter(dst, gry, 2.0, 0.2 * SP_BYTEMAX);
        saveBMP(dst, "bilateral_g.bmp");
    }
    {
        Mem2<Col3> dst;
        bilateralFilter<Col3, Byte>(dst, src, 2.0, 0.2 * SP_BYTEMAX);
        saveBMP(dst, "bilateral_c.bmp");
    }

    // guided filter
    {
        Mem2<Byte> dst;
        guidedFilter(dst, gry, 11, square(0.2 * SP_BYTEMAX));
        saveBMP(dst, "guidedfilter_g.bmp");
    }
    {
        Mem2<Col3> dst;
        guidedFilter(dst, src, 11, square(0.2 * SP_BYTEMAX));
        saveBMP(dst, "guidedfilter_c.bmp");
    }

    // sobel
    {
        Mem2<float> dstX, dstY;
        sobelFilterX(dstX, gry);
        sobelFilterY(dstY, gry);
        
        Mem2<Byte> tmpX, tmpY;
        cnvMem(tmpX, dstX, 0.5, -255);
        cnvMem(tmpY, dstY, 0.5, -255);
        saveBMP(tmpX, "sobelX.bmp");
        saveBMP(tmpY, "sobelY.bmp");
    }

    // canny
    {
        Mem2<Col3> tmp;
        gaussianFilter3x3<Col3, Byte>(tmp, src);

        Mem2<Byte> dst;
        canny(dst, tmp, 5, 10);

        saveBMP(dst, "canny.bmp");
    }

    return 0;
}