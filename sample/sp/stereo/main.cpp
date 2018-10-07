//#define SP_USE_DEBUG 1

#include "simplesp.h"

using namespace sp;

int main() {

    Mem2<Col3> imgs[2];
    {
        SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/shiba02.bmp", imgs[0]));
        SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/shiba04.bmp", imgs[1]));

        saveBMP("src0.bmp", imgs[0]);
        saveBMP("src1.bmp", imgs[1]);
    }

    // get camera parameter
    CamParam cam0, cam1;
    {
        SP_ASSERT(loadText(SP_DATA_DIR  "/image/shiba.txt", cam0));
        SP_ASSERT(loadText(SP_DATA_DIR  "/image/shiba.txt", cam1));
    }

    // estimate camera pose
    Pose stereo;
    {
        // sift execute
        SIFT sift0, sift1;
        sift0.execute(imgs[0]);
        sift1.execute(imgs[1]);

        // get feature
        const Mem1<Feature> &fts0 = *sift0.getFeatrue();
        const Mem1<Feature> &fts1 = *sift1.getFeatrue();
        const Mem1<int> matches = findMatch(fts0, fts1);

        Mem1<Vec2> mpixs0, mpixs1;
        for (int i = 0; i < matches.size(); i++) {
            const int j = matches[i];
            if (j < 0) continue;
            mpixs0.push(fts0[i].pix);
            mpixs1.push(fts1[j].pix);
        }

        calcPose(stereo, cam0, mpixs0, cam1, mpixs1);
    }


    RectParam rects[2];
    Mem2<Vec2> tables[2];
    
    // make remap table
    {
        rectify(rects[0], rects[1], cam0, cam1, stereo);

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