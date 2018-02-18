#include "simplesp.h"

using namespace sp;

int main(){

    Mem2<Byte> img;
    SP_ASSERT(loadBMP(img, SP_DATA_DIR  "/image/Lenna.bmp"));

    saveBMP(img, "input.bmp");
    
    Mem2<double> re, im;
    {
        printf("dft\n");
        dft(re, im, img);
    }

    // cut
    {
        const int margin = 200;
        const Rect rect = adjustRect(getRect2(img.dsize), -margin);

        for (int v = 0; v < img.dsize[1]; v++) {
            for (int u = 0; u < img.dsize[0]; u++) {
                if (isInRect2(rect, u, v) == false) {
                    re(u, v) = 0.0;
                    im(u, v) = 0.0;
                }
            }
        }
    }
    {
        printf("idft\n");
        idft(img, re, im);
    }

    saveBMP(img, "output.bmp");

    // visualize
    {
        Mat tmp(img.dsize);
        for (int i = 0; i < tmp.size(); i++) {
            tmp[i] = ::log(pythag(re[i], im[i]) + 1.0);
        }

        const double maxv = maxVal(tmp);
        const double minv = minVal(tmp);

        Mem2<Byte> vis;
        cnvMem(vis, tmp, 255.0 / (maxv - minv), minv);
        saveBMP(vis, "vis.bmp");
    }

    return 0;
}