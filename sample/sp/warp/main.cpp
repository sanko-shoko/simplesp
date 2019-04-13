#include "simplesp.h"

using namespace sp;

int main(){

    // init input image
    Mem2<Col3> imgs[2];
    {
        SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/shiba00.bmp", imgs[0]));
        SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/shiba03.bmp", imgs[1]));

        saveBMP("src0.bmp", imgs[0]);
        saveBMP("src1.bmp", imgs[1]);
    }

    CamParam cam;
    SP_ASSERT(loadText(SP_DATA_DIR  "/image/shiba.txt", cam));

    Mem1<Vec2> vtxs;
    {
        const int w = imgs[0].dsize[0];
        const int h = imgs[0].dsize[1];

        vtxs.push(getVec2(0.0, 0.0));
        vtxs.push(getVec2(w, 0.0));
        vtxs.push(getVec2(w, h));
        vtxs.push(getVec2(0.0, h));
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
        Mem1<Ftr> ftrs[2];

        // get features
        ftrs[0] = SIFT::getFtrs(imgs[0]);
        ftrs[1] = SIFT::getFtrs(imgs[1]);

        // matching
        const Mem1<int> matches = findMatch(ftrs[0], ftrs[1]);

        const Mem1<Vec2> pixs0 = getMatchPixs(ftrs[0], matches, true);
        const Mem1<Vec2> pixs1 = getMatchPixs(ftrs[1], matches, false);
        {
            Mat hom;
            if (calcHMatRANSAC(hom, pixs1, pixs0) == true) {
                const Rect rect = getRect2(hom * vtxs);
                Mem2<Col3> dst;

                if(0){
                    const Rect rect = getRect2(hom * vtxs);

                    Mat offset = eyeMat(3, 3);
                    offset(0, 2) = -rect.dbase[0];
                    offset(1, 2) = -rect.dbase[1];

                    dst.resize(rect.dsize);
                    warp<Col3, Byte>(dst, imgs[0], offset * hom);
                }
                else {
                    dst.resize(imgs[0].dsize);
                    warp<Col3, Byte>(dst, imgs[0], hom);

                    blend(dst, dst, imgs[1]);
                }
                saveBMP("hom1.bmp", dst);
            }
        }

        {
            Pose pose;
            Mat hom;
            if (calcPoseRANSAC(pose, cam, pixs0, cam, pixs1) == true) {
                Mem1<Vec3> pnts;
                Mem1<bool> mask;
                calcPnt3d(pnts, mask, zeroPose(), cam, pixs0, pose, cam, pixs1);
                pnts = filter(pnts, mask);

                Mem1<double> zlist;
                for (int i = 0; i < pnts.size(); i++) {
                    zlist.push(pnts[i].z);
                }

                //const double z = medianVal(zlist);
                const double z = meanVal(zlist);

                {
                    const Mat cmat = extMat(4, 4, getMat(cam));
                    const Mat imat = invMat(cmat);

                    const Mat pmat = cmat * getMat(pose, 4, 4) * imat;

                    hom.resize(3, 3);
                    for (int r = 0; r < 3; r++) {
                        for (int c = 0; c < 3; c++) {
                            hom(r, c) = (c < 2) ? pmat(r, c) * z : pmat(r, c) * z + pmat(r, c + 1);
                        }
                    }
                    hom = hom / hom(2, 2);
                }

                Mem2<Col3> dst;

                if (0) {
                    const Rect rect = getRect2(hom * vtxs);

                    Mat offset = eyeMat(3, 3);
                    offset(0, 2) = -rect.dbase[0];
                    offset(1, 2) = -rect.dbase[1];

                    dst.resize(rect.dsize);
                    warp<Col3, Byte>(dst, imgs[0], offset * hom);
                }
                else {
                    dst.resize(imgs[0].dsize);
                    warp<Col3, Byte>(dst, imgs[0], hom);

                    blend(dst, dst, imgs[1]);
                }
                saveBMP("hom2.bmp", dst);
            }

        }

    }
    return 0;
}

