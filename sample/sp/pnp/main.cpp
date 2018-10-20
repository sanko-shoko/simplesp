#include "simplesp.h"

using namespace sp;

int main(){
    
    CamParam cam;
    
    Mem1<Vec3> objs;
    Mem1<Vec2> pixs;

    // generate test parameter
    {
        cam = getCamParam(640, 480);

        const Pose pose = getPose(getVec(0.0, 0.0, 400)) * getRotAngleX(+30 * SP_PI / 180.0);

        for (int i = 0; i < 100; i++) {
            objs.push(randVecGauss(20.0, 20.0, 20.0));
        }

        for (int i = 0; i < objs.size(); i++) {
            const Vec3 pos = pose * objs[i];
            const Vec2 pix = mulCam(cam, prjVec(pos));
            pixs.push(pix);
        }

        Mem2<Byte> img(cam.dsize);
        img.zero();

        for (int i = 0; i < pixs.size(); i++) {
            renderPoint(img, pixs[i], (Byte)255, 2);
        }

        saveBMP("test.bmp", img);
        print(pose);

    }

    {
        printf("\n\n");
        printf("--------------------------------------------------------------------------------\n");
        printf("P3P\n");
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
        printf("P4P\n");
        printf("--------------------------------------------------------------------------------\n");

        Pose pose;
        calcPoseP4P(pose, cam, pixs.slice(0, 0, 4), objs.slice(0, 0, 4));

        print(pose);
    }

    {
        printf("\n\n");
        printf("--------------------------------------------------------------------------------\n");
        printf("RANSAC\n");
        printf("--------------------------------------------------------------------------------\n");

        const double noise = 0.5;
        for (int i = 0; i < pixs.size(); i++) {
            if ((randValUnif() + 1.0) / 2.0 < noise) {
                pixs[i] += randVecUnif(100, 100);
            }
        }

        Pose pose;
        calcPoseRANSAC(pose, cam, pixs, objs);

        print(pose);
    }

    return 0;
}

