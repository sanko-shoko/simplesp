#include "simplesp.h"

using namespace sp;

int main() {

    // test camera parameter
    CamParam cam = getCamParam(640, 480);

    const Pose stereo = getPose(getVec3(-1.0, 0.0, 0.0));

    {
        cam.cx = 300;
        cam.cy = 250;
        cam.k1 = 0.1;
        cam.k2 = 0.1;
        cam.k3 = 0.1;
        cam.p1 = 0.1;
        cam.p2 = 0.1;

        printf("grand truth\n");
        print(cam);
        print(stereo);
    }

    // test marker pose
    Mem1<Pose> poses;

    // marker position
    const DotMarkerParam mrk(5, 5, 30);
    const Mem2<Vec2> mrkMap = mrk.map * mrk.distance;

    // generate test pose
    {
        poses.push(getPose(getVec3(+00.0, 0.0, 400)));
        poses.push(getPose(getVec3(0.0, +10.0, 400)) * getRotAngleX(+30 * SP_PI / 180.0));
        poses.push(getPose(getVec3(0.0, -10.0, 400)) * getRotAngleX(-30 * SP_PI / 180.0));
        poses.push(getPose(getVec3(+10.0, 0.0, 400)) * getRotAngleY(+30 * SP_PI / 180.0));
        poses.push(getPose(getVec3(-10.0, 0.0, 400)) * getRotAngleY(-30 * SP_PI / 180.0));
    }

    // test (generate test points)
    {
        printf("\n\n");
        printf("--------------------------------------------------------------------------------\n");
        printf("cam test (generate test points)\n");
        printf("--------------------------------------------------------------------------------\n");

        Mem1<Mem1<Vec2> > pixs, objs;

        // generate test points
        for (int i = 0; i < poses.size(); i++) {
            Mem1<Vec2> tpixs, tobjs;
            for (int n = 0; n < mrkMap.size(); n++) {
                const Vec3 pos = poses[i] * getVec3(mrkMap[n].x, mrkMap[n].y, 0.0);
                const Vec2 pix = mulCamD(cam, prjVec(pos)) + randgVec2(0.1, 0.1);

                tpixs.push(pix);
                tobjs.push(mrkMap[n]);
            }
            pixs.push(tpixs);
            objs.push(tobjs);
        }

        {
            printf("\n\n");

            // calibration
            CamParam dst;
            const double rms = calibCam(dst, cam.dsize[0], cam.dsize[1], pixs, objs);
            printf("rms error: %g\n", rms);

            print(dst);
            saveText("cam0.txt", dst);
        }
    }

    // cam test (generate test images)
    {
        printf("\n\n");
        printf("--------------------------------------------------------------------------------\n");
        printf("cam test (generate test images)\n");
        printf("--------------------------------------------------------------------------------\n");

        Mem1<Mem2<Byte> > imgs(poses.size());

        // generate test images
        for (int i = 0; i < poses.size(); i++) {
            renderMarker(imgs[i], cam, poses[i], mrkMap);
            char str[256];
            sprintf(str, "test%02d.bmp", i);
            saveBMP(str, imgs[i]);
        }

        // simplesp calibration 
        {
            printf("\n\n");

            DotMarker dotMarker;
            dotMarker.setMrk(mrk);

            Mem1<Mem1<Vec2> > pixs, objs;

            // detect points
            for (int i = 0; i < imgs.size(); i++) {
                if (dotMarker.execute(imgs[i])) {
                    pixs.push(*dotMarker.getCrspPixs());
                    objs.push(*dotMarker.getCrspObjs());
                }
            }

            // calibration
            CamParam dst;
            const double rms = calibCam(dst, cam.dsize[0], cam.dsize[1], pixs, objs);
            printf("rms error: %g\n", rms);

            print(dst);
            saveText("cam1.txt", dst);
        }
    }

    // stereo test (generate test points)
    {
        printf("\n\n");
        printf("--------------------------------------------------------------------------------\n");
        printf("stereo test (generate test points)\n");
        printf("--------------------------------------------------------------------------------\n");

        Mem1<Mem1<Vec2> > pixs0, pixs1, objs0, objs1;

        // generate test points
        for (int i = 0; i < poses.size(); i++) {
            Mem1<Vec2> tpixs0, tpixs1, tobjs;
            for (int n = 0; n < mrkMap.size(); n++) {
                const Vec3 pos = poses[i] * getVec3(mrkMap[n].x, mrkMap[n].y, 0.0);
                const Vec2 pix0 = mulCamD(cam, prjVec(pos)) + randgVec2(0.1, 0.1);
                const Vec2 pix1 = mulCamD(cam, prjVec(stereo * pos)) + randgVec2(0.1, 0.1);

                tpixs0.push(pix0);
                tpixs1.push(pix1);
                tobjs.push(mrkMap[n]);
            }
            pixs0.push(tpixs0);
            pixs1.push(tpixs1);
            objs0.push(tobjs);
            objs1.push(tobjs);
        }

        // simplesp calibration 
        {
            printf("\n\n");

            // calibration
            Pose dst;
            const double rms = calibStereo(dst, cam, pixs0, objs0, cam, pixs1, objs1);
            printf("rms error: %g\n", rms);

            print(dst);
            saveText("pose.txt", dst);
        }
    }

    return 0;
}

