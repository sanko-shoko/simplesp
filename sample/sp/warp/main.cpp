#include "simplesp.h"

using namespace sp;

int main(){

    //--------------------------------------------------------------------------------
    // sift (scale invariant feature transform)
    //--------------------------------------------------------------------------------

    // init input image
    Mem2<Col3> imgs[2];
    SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/shiba00.bmp", imgs[0]));
    SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/shiba03.bmp", imgs[1]));

    const int w = imgs[0].dsize[0];
    const int h = imgs[0].dsize[1];

    Mem1<Vec2> pixs;
    pixs.push(getVec(0.0, 0.0));
    pixs.push(getVec(w, 0.0));
    pixs.push(getVec(w, h));
    pixs.push(getVec(0.0, h));

    // homograpy
    {
        Mem2<Col3> dst;

        double mat[3 * 3] = {
            +0.8000, -0.2000, +130.00,
            +0.2000, +0.8000, +50.000,
            +0.0002, +0.0002, +1.0000
        };
        const Mat hom(3, 3, mat);

        Rect rect = getRect2(getVec(0.0, 0.0));
        for (int i = 0; i < pixs.size(); i++) {
            rect = orRect(rect, getRect2(hom * pixs[i]));
        }
        print(rect);
        dst.resize(rect.dsize);
        warp<Col3, Byte>(dst, imgs[0], hom);
 
        saveBMP("hom0.bmp", dst);
    }

    {
        Mem1<Feature> fts[2];

        // get features
        fts[0] = SIFT::getFeatures(imgs[0]);
        fts[1] = SIFT::getFeatures(imgs[1]);

        // matching
        const Mem1<int> matches = findMatch(fts[0], fts[1]);

        const Mem1<Vec2> pixs0 = getMatchPixs(fts[0], matches, true);
        const Mem1<Vec2> pixs1 = getMatchPixs(fts[1], matches, false);

        Mat hom;
        if (calcHMatRANSAC(hom, pixs1, pixs0) == true) {
        }
    }
    return 0;
}

