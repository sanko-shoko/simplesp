#include "simplesp.h"

using namespace sp;

int main(){

    //--------------------------------------------------------------------------------
    // belief propagation
    //--------------------------------------------------------------------------------

    Mem2<Byte> img;
    SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/Lenna.bmp", img));
    saveBMP("input.bmp", img);

    const int labelMax = 5;
    const int step = SP_BYTEMAX / labelMax;


    {
        Mem2<Byte> dst(img.dsize);
        for (int i = 0; i < dst.size(); i++) {
            dst[i] = ((img[i] + step / 2) / step) * step;
        }
        saveBMP("test.bmp", dst);
    }

    {
        const int kappa = 100;

        BeliefPropagation bp(labelMax, img.size(), img.size() * 2);

        Mem3<int> costMap(labelMax, img.dsize[0], img.dsize[1]);

        for (int v = 0; v < img.dsize[1]; v++) {
            for (int u = 0; u < img.dsize[0]; u++) {
                for (int i = 0; i < labelMax; i++) {
                    costMap(i, u, v) = ::abs(img(u, v) - i * step);
                }
                bp.setNode(acsid2(img.dsize, u, v), &costMap(0, u, v));

                const int link[][2] = { { -1, 0 }, { 0, -1 } };
                for (int d = 0; d < 4; d++) {
                    const int x = u + link[d][0];
                    const int y = v + link[d][1];

                    if (inRect2(img.dsize, x, y) == true) {
                        bp.setLink(acsid2(img.dsize, u, v), acsid2(img.dsize, x, y), kappa);
                    }
                }
            }
        }

        bp.execute(50);

        Mem2<Byte> dst(img.dsize);
        for (int i = 0; i < img.size(); i++) {
            dst[i] = bp.getLabel(i) * step;
        }
        saveBMP("bp.bmp", dst);
    }

    return 0;
}

