#include "simplesp.h"

using namespace sp;

int main(){
    
    CamParam cam;
    Pose pose;
    
    Mem1<Vec3> objs;
    Mem1<Vec2> pixs;

    // generate test parameter
    {
        cam = getCamParam(640, 480);

        pose = getPose(getVec(0.0, 0.0, 400)) * getRotAngleX(+30 * SP_PI / 180.0);

        objs.push(getVec(-50.0, -50.0, 0.0));
        objs.push(getVec(+50.0, -50.0, 0.0));
        objs.push(getVec(-50.0, +50.0, 0.0));
        objs.push(getVec(+50.0, +50.0, 0.0));

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
        printf("P4P");

        Pose pose;
        calcPoseP4P(pose, cam, pixs, objs);

        print(pose);
    }


    return 0;
}

