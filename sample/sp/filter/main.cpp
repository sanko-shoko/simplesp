#include "simplesp.h"

using namespace sp;

int main(){

    Mem2<Col3> src;
    Mem2<Byte> gry;
    {
        SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/Lenna.bmp", src));
        saveBMP("color.bmp", src);

        cnvImg(gry, src);
        saveBMP("gray.bmp", gry);
    }

    // gaussian
    {
        Mem2<Byte> dst;
        gaussianFilter(dst, gry, 2.0);
        saveBMP("gaussian_g.bmp", dst);
    }
    {
        Mem2<Col3> dst;
        gaussianFilter<Col3, Byte>(dst, src, 2.0);
        saveBMP("gaussian_c.bmp", dst);
    }

    // bilateral
    {
        Mem2<Byte> dst;
        bilateralFilter(dst, gry, 2.0, 0.2 * SP_BYTEMAX);
        saveBMP("bilateral_g.bmp", dst);
    }
    {
        Mem2<Col3> dst;
        bilateralFilter<Col3, Byte>(dst, src, 2.0, 0.2 * SP_BYTEMAX);
        saveBMP("bilateral_c.bmp", dst);
    }

    // guided filter
    {
        Mem2<Byte> dst;
        guidedFilter(dst, gry, 11, square(0.2 * SP_BYTEMAX));
        saveBMP("guidedfilter_g.bmp", dst);
    }
    {
        Mem2<Col3> dst;
        guidedFilter(dst, src, 11, square(0.2 * SP_BYTEMAX));
        saveBMP("guidedfilter_c.bmp", dst);
    }

    // sobel
    {
        Mem2<float> dstX, dstY;
        sobelFilterX3x3(dstX, gry);
        sobelFilterY3x3(dstY, gry);
        
        Mem2<Byte> tmpX, tmpY;
        cnvMem(tmpX, dstX, 0.5, -255 );
        cnvMem(tmpY, dstY, 0.5, -255);
        saveBMP("sobelX.bmp", tmpX);
        saveBMP("sobelY.bmp", tmpY);
    }

    // canny
    {
        Mem2<Col3> tmp;
        gaussianFilter3x3<Col3, Byte>(tmp, src);

        Mem2<Byte> dst;
        canny(dst, tmp, 5, 10);

        saveBMP("canny.bmp", dst);
    }

    return 0;
}