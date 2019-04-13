#include "simplesp.h"
#include "spex/spcv.h"

using namespace sp;

int main() {
    // test camera parameter
    CamParam cam = getCamParam(640, 480);

    // test marker pose
    Pose pose0, pose1;

    // marker position
    const DotMarkerParam mrk(5, 5, 40);
    const Mem2<Vec2> mrkMap = mrk.map * mrk.distance;

    {
        cam.k1 = -0.5;

        // generate test pose
        pose0 = getPose(getVec3(+0.0, 0.0, 400));
        //pose0 = getPose(getVec3(+1.0, 0.0, 400)) * getRotAngleY(+30 * SP_PI / 180.0);
        pose1 = getPose(getVec3(-1.0, 0.0, 400)) * getRotAngleY(-30 * SP_PI / 180.0);
    }

    {

        Mem1<Vec2> pixs0;
        Mem1<Vec2> pixs1;

        // generate test points
        for (int n = 0; n < mrkMap.size(); n++) {
            const Vec2 pix0 = mulCamD(cam, prjVec(pose0 * getVec3(mrkMap[n].x, mrkMap[n].y, 0.0)));
            const Vec2 pix1 = mulCamD(cam, prjVec(pose1 * getVec3(mrkMap[n].x, mrkMap[n].y, 0.0)));

            pixs0.push(pix0);
            pixs1.push(pix1);
        }

        Mem2<Byte> img0;
        Mem2<Byte> img1;
        renderMarker(img0, cam, pose0, mrkMap);
        renderMarker(img1, cam, pose1, mrkMap);

        saveBMP("img0.bmp", img0);
        saveBMP("img1.bmp", img1);
    }
    return 0;

}