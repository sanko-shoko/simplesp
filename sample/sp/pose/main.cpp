#include "simplesp.h"

using namespace sp;

int main(){
    
    // object coordinate
    Mem1<Vec3> objs;

    // camera coordinate
    CamParam cam;
    Mem1<Vec3> pnts;
    Mem1<Vec2> pixs;

    // generate test parameter
    {
        printf("--------------------------------------------------------------------------------\n");
        printf("ground truth\n");
        printf("--------------------------------------------------------------------------------\n");
        
        srand(0);

        cam = getCamParam(640, 480);

        // pose (camera <- object)
        const Pose pose = getPose(randRotGauss(10.0 * SP_PI / 180), getVec(0.0, 0.0, 400));

        // object coordinate
        for (int i = 0; i < 100; i++) {
            objs.push(randVecGauss(20.0, 20.0, 20.0));
        }

        // camera coordinate
        for (int i = 0; i < objs.size(); i++) {
            const Vec3 pos = pose * objs[i];
            const Vec2 pix = mulCam(cam, prjVec(pos));
            pnts.push(pos);
            pixs.push(pix);
        }

        // test image
        {
            Mem2<Byte> img(cam.dsize);
            img.zero();

            for (int i = 0; i < pixs.size(); i++) {
                renderPoint(img, pixs[i], (Byte)255, 2);
            }
            saveBMP("test.bmp", img);
        }

        print(pose);
    }

    {
        printf("\n\n");
        printf("--------------------------------------------------------------------------------\n");
        printf("3D-3D\n");
        printf("--------------------------------------------------------------------------------\n");

        Pose pose = getPose(getVec(0.0, 0.0, 400));
        calcPose(pose, pnts, objs);

        print(pose);
    }

    {
        printf("\n\n");
        printf("--------------------------------------------------------------------------------\n");
        printf("3D-3D RANSAC\n");
        printf("--------------------------------------------------------------------------------\n");

        Mem1<Vec3> pnts_n = pnts;
        const double rate = 0.5;
        for (int i = 0; i < pixs.size(); i++) {
            if (::fabs(randValUnif()) < rate) {
                pnts_n[i] += randVecUnif(100, 100, 100);
            }
        }

        Pose pose;
        calcPoseRANSAC(pose, pnts_n, objs);

        print(pose);
    }

    {
        printf("\n\n");
        printf("--------------------------------------------------------------------------------\n");
        printf("2D-3D P3P\n");
        printf("--------------------------------------------------------------------------------\n");  

        Mem1<Pose> poses;
        calcPoseP3P(poses, cam, pixs.slice(0, 0, 3), objs.slice(0, 0, 3));

        for (int i = 0; i < poses.size(); i++) {
            print(poses[i]);
        }
    }

    {
        printf("\n\n");
        printf("--------------------------------------------------------------------------------\n");
        printf("2D-3D P4P\n");
        printf("--------------------------------------------------------------------------------\n");

        Pose pose;
        calcPoseP4P(pose, cam, pixs.slice(0, 0, 4), objs.slice(0, 0, 4));

        print(pose);
    }

    {
        printf("\n\n");
        printf("--------------------------------------------------------------------------------\n");
        printf("2D-3D RANSAC\n");
        printf("--------------------------------------------------------------------------------\n");

        Mem1<Vec2> pixs_n = pixs;
        const double rate = 0.5;
        for (int i = 0; i < pixs.size(); i++) {
            if (::fabs(randValUnif()) < rate) {
                pixs_n[i] += randVecUnif(100, 100);
            }
        }

        Pose pose;
        calcPoseRANSAC(pose, cam, pixs_n, objs);

        print(pose);
    }

    return 0;
}

