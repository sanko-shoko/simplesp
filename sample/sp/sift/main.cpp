#define SP_USE_DEBUG 1

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
            SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/shiba00.bmp", imgs[0]));
            SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/shiba03.bmp", imgs[1]));
            //SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/shiba02.bmp", imgs[0]));
            //SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/shiba03.bmp", imgs[1]));
        }

        merge(imgM, imgs[0], imgs[1]);
        saveBMP("input.bmp", imgM);
    }


    Mem1<Ftr> ftrs[2];

    {
        SP_LOGGER_SET("features");

        // get features
        ftrs[0] = SIFT::getFtrs(imgs[0]);
        ftrs[1] = SIFT::getFtrs(imgs[1]);
    }

    Mem1<int> matches;
    {
        SP_LOGGER_SET("matching");
    
        // matching
        matches = findMatch(ftrs[0], ftrs[1]);
    }

    const Mem1<Vec2> pixs0 = getMatchPixs(ftrs[0], matches, true);
    const Mem1<Vec2> pixs1 = getMatchPixs(ftrs[1], matches, false);
    
    // print info
    printf("ftrs[0]: %d\n", ftrs[0].size());
    printf("ftrs[1]: %d\n", ftrs[1].size());

    printf("match [0->1]: cnt %d, eval %.2lf\n", getMatchCnt(matches), getMatchEval(matches));

    // render
    {
        if (0) {
            for (int i = 0; i < 2; i++) {
                for (int f = 0; f < ftrs[i].size(); f++) {
                    renderCircle(imgs[i], ftrs[i][f].pix, ftrs[i][f].scl, getCol(100, 255, 100), 1);
                }
            }
        }
        else {
            for (int f = 0, c = 0; f < ftrs[0].size(); f++) {
                const int g = matches[f];
                if (g < 0) continue;
                renderCircle(imgs[0], ftrs[0][f].pix, ftrs[0][f].scl, getCol(c), 1);
                renderCircle(imgs[1], ftrs[1][g].pix, ftrs[1][g].scl, getCol(c), 1);
                c++;
            }
        }

        merge(imgM, imgs[0], imgs[1]);
        saveBMP("sift.bmp", imgM);
   
        const int w = imgs[0].dsize[0];
        const int h = imgs[0].dsize[1];

        for (int i = 0; i < pixs0.size(); i++) {
            renderLine(imgM, pixs0[i], pixs1[i] + getVec2(w, 0), getCol(i), 1);
        }

        Mat hom;
        if (calcHMatRANSAC(hom, pixs1, pixs0) == true){
            const Vec2 pix[4] = { getVec2(0.0, 0.0), getVec2(w, 0.0), getVec2(w, h), getVec2(0.0, h) };
            for (int i = 0; i < 4; i++){
                const Vec2 p0 = pix[i] - getVec2(0.5, 0.5);
                const Vec2 p1 = pix[(i + 1) % 4] - getVec2(0.5, 0.5);
                renderLine(imgM, hom * p0 + getVec2(w, 0), hom * p1 + getVec2(w, 0), getCol(100, 200, 100), 2);
            }
            //print(hom);
        }

        saveBMP("match.bmp", imgM);
    }

    SP_LOGGER_PRINT(NULL);
    return 0;
}

