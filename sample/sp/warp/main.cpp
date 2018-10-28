#include "simplesp.h"

using namespace sp;

int main(){

    // init input image
    Mem2<Col3> imgs[2];
    SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/shiba00.bmp", imgs[0]));
    SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/shiba03.bmp", imgs[1]));

    CamParam cam;
    SP_ASSERT(loadText(SP_DATA_DIR  "/image/shiba.txt", cam));

    Mem1<Vec2> vtxs;
    {
        const int w = imgs[0].dsize[0];
        const int h = imgs[0].dsize[1];

        vtxs.push(getVec(0.0, 0.0));
        vtxs.push(getVec(w, 0.0));
        vtxs.push(getVec(w, h));
        vtxs.push(getVec(0.0, h));
    }

    // homograpy
    {

        double mat[3 * 3] = {
            +0.8000, -0.2000, +130.00,
            +0.2000, +0.8000, +50.000,
            +0.0002, +0.0002, +1.0000
        };
        const Mat hom(3, 3, mat);

        const Rect rect = getRect2(hom * vtxs);
        
        Mat offset = eyeMat(3, 3);
        offset(0, 2) = -rect.dbase[0];
        offset(1, 2) = -rect.dbase[1];

        Mem2<Col3> dst(rect.dsize);
        warp<Col3, Byte>(dst, imgs[0], offset * hom);
 
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
            const Rect rect = getRect2(hom * vtxs);

            Mat offset = eyeMat(3, 3);
            //offset(0, 2) = -rect.dbase[0];
            //offset(1, 2) = -rect.dbase[1];

            Mem2<Col3> dst(imgs[0].dsize);
            warp<Col3, Byte>(dst, imgs[0], offset * hom);

            blend(dst, dst, imgs[1]);
            saveBMP("hom1.bmp", dst);
        }

        Pose pose;
        if (calcPoseRANSAC(pose, cam, pixs1, cam, pixs0) == true) {

            const Mat pmat = getMat(pose);


        }



    }
    return 0;
}

