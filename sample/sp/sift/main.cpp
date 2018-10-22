#include "simplesp.h"

using namespace sp;

int main(){

    //--------------------------------------------------------------------------------
    // sift (scale invariant feature transform)
    //--------------------------------------------------------------------------------

    // init input image
    Mem2<Col3> imgs[2];
    Mem2<Col3> imgM;
    
    {
        if (0) {
            SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/Lenna.bmp", imgs[0]));

            imgs[1].resize(imgs[0].dsize);
            setElm(imgs[1], getCol(127, 127, 127));

            double mat[3 * 3] = {
                +0.8000, -0.2000, +130.00,
                +0.2000, +0.8000, +50.000,
                +0.0002, +0.0002, +1.0000
            };
            const Mat hom(3, 3, mat);

            warp<Col3, Byte>(imgs[1], imgs[0], hom);
        }
        else {
            SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/shiba02.bmp", imgs[0]));
            SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/shiba03.bmp", imgs[1]));
        }

        merge(imgM, imgs[0], imgs[1]);
        saveBMP("input.bmp", imgM);
    }


    Mem1<Feature> fts[2];
 
    // get features
    {
        for (int i = 0; i < 2; i++) {
            fts[i] = SIFT::getFeatures(imgs[i]);

            for (int j = 0; j < fts[i].size(); j++) {
                renderCircle(imgs[i], fts[i][j].pix, fts[i][j].scl, getCol(100, 255, 100), 1);
            }
        }

        merge(imgM, imgs[0], imgs[1]);
        saveBMP("sift.bmp", imgM);
    }

    // matching
    {
        const Mem1<int> matches = findMatch(fts[0], fts[1]);

        const Mem1<Vec2> pixs0 = getMatchPixs(fts[0], matches, true);
        const Mem1<Vec2> pixs1 = getMatchPixs(fts[1], matches, false);

        const int w = imgs[0].dsize[0];
        const int h = imgs[0].dsize[1];

        for (int i = 0; i < pixs0.size(); i++) {
            renderLine(imgM, pixs0[i], pixs1[i] + getVec(w, 0), getCol(i), 1);
        }

        Mat hom;
        if (calcHMatRANSAC(hom, pixs1, pixs0) == true){
            const Vec2 pix[4] = { getVec(0.0, 0.0), getVec(w, 0.0), getVec(w, h), getVec(0.0, h) };
            for (int i = 0; i < 4; i++){
                const Vec2 p0 = pix[i] - getVec(0.5, 0.5);
                const Vec2 p1 = pix[(i + 1) % 4] - getVec(0.5, 0.5);
                renderLine(imgM, hom * p0 + getVec(w, 0), hom * p1 + getVec(w, 0), getCol(100, 200, 100), 2);
            }
            print(hom);
        }

        saveBMP("match.bmp", imgM);
    }
    return 0;
}

