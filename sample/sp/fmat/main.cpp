#include "simplesp.h"

using namespace sp;

int main(){

    Mem2<Col3> img0, img1;
    SP_ASSERT(loadBMP(img0, SP_DATA_DIR  "/image/shiba02.bmp"));
    SP_ASSERT(loadBMP(img1, SP_DATA_DIR  "/image/shiba04.bmp"));

    // sift execute
    SIFT sift0, sift1;
    sift0.execute(img0);
    sift1.execute(img1);

    // get feature
    const Mem1<Feature> &fts0 = sift0.getFeatrue();
    const Mem1<Feature> &fts1 = sift1.getFeatrue();

    // get match pix
    Mem1<Vec2> pixs0, pixs1;
    {
        const Mem1<int> matches = findMatch(fts0, fts1);

        for (int i = 0; i < matches.size(); i++) {
            const int j = matches[i];
            if (j < 0) continue;

            pixs0.push(fts0[i].pix);
            pixs1.push(fts1[j].pix);
        }
    }

    // calc fundamental matrix
    Mat F;
    {
        SP_ASSERT(calcFMatRANSAC(F, pixs0, pixs1));

        printf("fundamental matrix\n");
        print(F);
    }

    // render image
    {
        const Mem1<double> errs = errFMat(F, pixs0, pixs1);

        // get points on epipolar line
        const Mem1<Vec2> dpixs0 = denoise(pixs0, errs);
        const Mem1<Vec2> dpixs1 = denoise(pixs1, errs);

        renderEpipolar(img0, trnMat(F), dpixs1, getCol(0, 200, 100), 2);
        renderPoint(img0, dpixs0, getCol(255, 255, 255), 4);

        renderEpipolar(img1, F, dpixs0, getCol(0, 200, 100), 2);
        renderPoint(img1, dpixs1, getCol(255, 255, 255), 4);

        saveBMP(img0, "imgL.bmp");
        saveBMP(img1, "imgR.bmp");
    }

    // get camera parameter (not fine)
    const CamParam cam0 = getCamParam(img0.dsize);
    const CamParam cam1 = getCamParam(img1.dsize);

    // calc pose
    Pose pose;
    {
        SP_ASSERT(dcmpFMat(pose, F, cam0, pixs0, cam1, pixs1));
    
        printf("stereo pose\n");
        print(pose);
    }

    // triangulation
    {

        Mem1<Vec3> pnts;
        Mem1<Col3> cols;

        for (int i = 0; i < pixs0.size(); i++) {
            Vec3 pnt;
            if (calcPnt3d(pnt, zeroPose(), cam0, pixs0[i], pose, cam1, pixs1[i]) == false) continue;
            
            const double err = errPose(zeroPose(), cam0, pixs0[i], pnt);
            if (evalErr(err) == 0.0) continue;

            pnts.push(pnt);
            cols.push(acsc(img0, pixs0[i].x, pixs0[i].y));
        }

        savePLY(pnts, cols, "pnts.ply");
    }

    //{
    //    pixs0.clear();
    //    pixs1.clear();

    //    const Mem1<int> matches = findMatchFMat(fts0, fts1, F);
    //    for (int i = 0; i < matches.size(); i++) {
    //        const int j = matches[i];
    //        if (j < 0) continue;

    //        pixs0.push(fts0[i].pix);
    //        pixs1.push(fts1[j].pix);
    //    }

    //    SP_ASSERT(calcFMatRANSAC(F, pixs0, pixs1));

    //    printf("refine matrix\n");
    //    print(F);


    //    SP_ASSERT(dcmpFMat(pose, F, cam0, pixs0, cam1, pixs1));

    //    printf("stereo pose\n");
    //    print(pose);
    //}

    return 0;
}

