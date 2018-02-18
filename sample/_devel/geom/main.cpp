#include "simplesp.h"

using namespace sp;

int main() {

    // test camera parameter
    CamParam cam = getCamParam(640, 480);

    // test marker pose
    Mem1<Pose> poses;

    const Pose stereo = getPose(getVec(-1.0, 0.0, 0.0));

    {
        cam.cx = 300;
        cam.cy = 250;
        cam.k1 = 0.1;
        cam.k2 = 0.1;
        cam.k3 = 0.1;
        cam.p1 = 0.1;
        cam.p2 = 0.1;
    }

    const Vec3 pos = getVec(0.0, 0.0, 400);
    Vec3 test = getVec(0.0, 0.0, 410);

    Vec2 pix0 = mulCamD(cam, prjVec(pos));
    Vec2 pix1 = mulCamD(cam, prjVec(stereo * pos));

    Mem1<Vec2> pixs;
    pixs.push(pix0);
    pixs.push(pix1);

    Mem1<CamParam> cams;
    cams.push(cam);
    cams.push(cam);

    poses.push(zeroPose());
    poses.push(stereo);

    refinePnt3d(test, poses, cams, pixs, 5);

    print(test);
    return 0;
}