#include "simplesp.h"

using namespace sp;

int main() {

    Mem2<Col3> lenna, neko;
    {
        SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/Lenna.bmp", lenna));
        SP_ASSERT(loadBMP(SP_DATA_DIR  "/marker/cap_neko.bmp", neko));
     
        saveBMP("lenna.bmp", lenna);
        saveBMP("neko.bmp", neko);
    }


    //--------------------------------------------------------------------------------
    // binarization
    //--------------------------------------------------------------------------------
    {
        Mem2<Byte> bin;

        Mem2<Col3> col;
        Mem2<Byte> gry;
        {// input
            col = neko;
            col = col.part(150, 120, 320, 240);
            cnvImg(gry, col);
        }

        Mem<int> hist;
        histogram(hist, gry, 255);
        saveText("neko_hist.csv", hist);

        // binalize thresh = 100
        binalize(bin, gry, 100);
        saveBMP("neko_bin100.bmp", bin);

        // binalize thresh = 140
        binalize(bin, gry, 140);
        saveBMP("neko_bin140.bmp", bin);

        // binalize thresh = adaptation
        binalizeAdapt(bin, gry);
        saveBMP("neko_binA.bmp", bin);
    }

    //--------------------------------------------------------------------------------
    // filter
    //--------------------------------------------------------------------------------
    {
        Mem2<Col3> col;
        Mem2<Byte> gry;
        {// input
            col = lenna;
            cnvImg(gry, col);
        }

        // gaussian
        {
            Mem2<Byte> dst;
            gaussianFilter(dst, gry, 2.0);
            saveBMP("gaussian_g.bmp", dst);
        }
        {
            Mem2<Col3> dst;
            gaussianFilter<Col3, Byte>(dst, col, 2.0);
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
            bilateralFilter<Col3, Byte>(dst, col, 2.0, 0.2 * SP_BYTEMAX);
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
            guidedFilter(dst, col, 11, square(0.2 * SP_BYTEMAX));
            saveBMP("guidedfilter_c.bmp", dst);
        }

        // sobel
        {
            Mem2<float> dstX, dstY;
            sobelFilter3x3(dstX, dstY, gry);

            Mem2<Byte> tmpX, tmpY;
            cnvMem(tmpX, dstX, 0.5, -255);
            cnvMem(tmpY, dstY, 0.5, -255);
            saveBMP("sobelX.bmp", tmpX);
            saveBMP("sobelY.bmp", tmpY);
        }

        // canny
        {
            Mem2<Col3> tmp;
            gaussianFilter<Col3, Byte>(tmp, col);

            Mem2<Byte> dst;
            canny(dst, tmp, 5, 10);

            saveBMP("canny.bmp", dst);
        }
    }

    //--------------------------------------------------------------------------------
    // integral image
    //--------------------------------------------------------------------------------
    {
        Mem2<Col3> col;
        Mem2<Byte> gry;
        {// input
            col = lenna;
            cnvImg(gry, col);
        }

        Mem2<Byte> dst;

        // box filter
        {
            boxFilter(dst, gry, 21);
        }
        saveBMP("boxFilter.bmp", dst);

        // box filter integral
        {
            Mem2<int> sum;
            makeIntegral(sum, gry);
            boxFilterIntegral(dst, sum, 21);
        }
        saveBMP("boxFilter_i.bmp", dst);
    }

    //--------------------------------------------------------------------------------
    // slic (simple linear iterative clustering)
    //--------------------------------------------------------------------------------
    {
        Mem2<Col3> col;
        Mem2<Byte> gry;
        {// input
            col = lenna;
            cnvImg(gry, col);
        }

        Mem2<int> map;

        // slic
        {
            slic(map, col, 20);
        }

        // visualize
        {
            Mem2<Col3> dst;
            cnvLabelToImg(dst, map);
            saveBMP("slic1.bmp", dst);

            Mem1<Mem1<Vec2> > contours = getLabelContour(map);
            for (int i = 0; i < contours.size(); i++) {
                renderPoint(col, contours[i], getCol3(0, 0, 0));
            }
            saveBMP("slic2.bmp", col);
        }
    }

    //--------------------------------------------------------------------------------
    // fourier
    //--------------------------------------------------------------------------------
    if (0) {
        Mem2<Col3> col;
        Mem2<Byte> gry;
        {// input
            col = lenna;
            cnvImg(gry, col);
        }

        Mem2<SP_REAL> re, im;
        {
            printf("dft ");
            dft(re, im, gry);
        }

        // cut
        {
            const Rect rect = getRect2(gry.dsize) - 200;
            for (int v = 0; v < gry.dsize[1]; v++) {
                for (int u = 0; u < gry.dsize[0]; u++) {
                    if (inRect2(rect, u, v) == false) {
                        re(u, v) = 0.0;
                        im(u, v) = 0.0;
                    }
                }
            }

            // visualize
            Mat mat(gry.dsize);
            for (int i = 0; i < mat.size(); i++) {
                mat[i] = ::log(pythag(re[i], im[i]) + 1.0);
            }

            const double maxv = maxVal(mat);
            const double minv = minVal(mat);

            Mem2<Byte> vis(mat.dsize);
            for (int i = 0; i < mat.size(); i++) {
                vis[i] = static_cast<Byte>(255.0 * (mat[i] - minv) / (maxv - minv));
            }
            saveBMP("dft.bmp", vis);
        }

        {
            printf("idft ");
            idft(gry, re, im);
        }

        saveBMP("idft.bmp", gry);
    }
    return 0;
}