//#define SP_USE_DEBUG 1

#include "simplesp.h"

using namespace sp;

int main() {

    Mem2<Col3> imgs[2];
    {
        SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/shiba02.bmp", imgs[0]));
        SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/shiba04.bmp", imgs[1]));
    }

    // get camera parameter
    CamParam cam[2];
    {
        SP_ASSERT(loadText(SP_DATA_DIR  "/image/shiba.txt", cam[0]));
        SP_ASSERT(loadText(SP_DATA_DIR  "/image/shiba.txt", cam[1]));
    }

    // estimate camera pose
    Pose stereo;
    {
        // get feature
        const Mem1<Ftr> ftrs0 = SIFT::getFtrs(imgs[0]);
        const Mem1<Ftr> ftrs1 = SIFT::getFtrs(imgs[1]);
        const Mem1<int> matches = findMatch(ftrs0, ftrs1);

        const Mem1<Vec2> pixs0 = getMatchPixs(ftrs0, matches, true);
        const Mem1<Vec2> pixs1 = getMatchPixs(ftrs1, matches, false);

        calcPoseRANSAC(stereo, cam[0], pixs0, cam[1], pixs1);

        print(stereo);
    }


    RectParam rects[2];
    Mem2<Vec2> tables[2];
    
    // make remap table
    {
        rectify(rects[1], rects[0], cam[1], stereo, cam[0], zeroPose());

        for (int i = 0; i < 2; i++) {
            makeRemapTable(tables[i], rects[i]);
        }
    }

    // pre filter
    if(0){
        for (int i = 0; i < 2; i++) {
            normalizeFilter<Col3, Byte>(imgs[i], imgs[i], 7);
        }
    }

    Mem2<Col3> rimgs[2];

    // remap
    {
        for (int i = 0; i < 2; i++) {
            remap<Col3, Byte>(rimgs[i], imgs[i], tables[i]);

            saveBMP(strFormat("rect%d.bmp", i).c_str(), rimgs[i]);
        }
    }

    const int maxDisp = 140;
    const int minDisp = 90;

    StereoBase estimator;
    estimator.setRange(maxDisp, minDisp);
    estimator.setCam(rects[0].cam, rects[1].cam);

    // matching
    {
        estimator.execute(rimgs[0], rimgs[1]);
    }

    // output
    {
        Mem2<Col3> imgs[2];
        cnvDispToImg(imgs[0], estimator.getDispMap(StereoBase::StereoL), maxDisp, minDisp);
        cnvDispToImg(imgs[1], estimator.getDispMap(StereoBase::StereoR), maxDisp, minDisp);
        
        saveBMP("dispL.bmp", imgs[0]);
        saveBMP("dispR.bmp", imgs[1]);

        const Mem2<Vec3> &depthMap = estimator.getDepthMap(StereoBase::StereoL);

        Mem1<Vec3> pnts;
        for (int i = 0; i < depthMap.size(); i++) {
            if (depthMap[i].z > 0.0) {
                pnts.push(depthMap[i]);
            }
        }
        savePLY("test.ply", pnts);
    }

    return 0;
}