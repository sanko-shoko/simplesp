#include "simplesp.h"

using namespace sp;

int main(){

    //--------------------------------------------------------------------------------
    // stereo rectification
    //--------------------------------------------------------------------------------

    // test parameter
    CamParam cams[2];
    Pose poses[2];
    {
        for (int i = 0; i < 2; i++){
            cams[i] = getCamParam(640, 480);
            cams[i].cx += randValGauss() * 10.0;
            cams[i].cy += randValGauss() * 10.0;

            cams[i].k1 = randValGauss() * 0.01;
            cams[i].k2 = randValGauss() * 0.01;
            cams[i].k3 = randValGauss() * 0.01;
            cams[i].p1 = randValGauss() * 0.01;
            cams[i].p2 = randValGauss() * 0.01;
        }

        poses[0] = zeroPose();
        poses[1] = getPose(randRotGauss(1.0 * SP_PI / 180.0), getVec(-1.0, 0.0, 0.0));
    }

    // test points
    Mem1<Vec3> list;
    {
        list.push(getVec(-50.0, -50.0, 500.0));
        list.push(getVec(+50.0, -50.0, 500.0));
        list.push(getVec(-50.0, +50.0, 500.0));
        list.push(getVec(+50.0, +50.0, 500.0));
    }

    // test images
    Mem2<Byte> imgs[2];
    for (int i = 0; i < 2; i++){
        imgs[i].resize(cams[i].dsize);
        setElm<Byte>(imgs[i], 100);
    
        for (int v = 0; v < imgs[i].dsize[1]; v++){
            for (int u = 0; u < imgs[i].dsize[0]; u++){
                if (v % 100 == 0 || u % 100 == 0){
                    imgs[i](u, v) = 50;
                }
            }
        }

        for (int p = 0; p < list.size(); p++){
            const Vec2 npx = prjVec(poses[i] * list[p]);
            const Vec2 pix = mulCamD(cams[i], npx);

            renderPoint<Byte>(imgs[i], pix, 255, 4);
        }

        char str[SP_STRMAX];
        sprintf(str, "img%d.bmp", i);
        saveBMP(str, imgs[i]);
    }

    // rectification
    RectParam rects[2];
    {
        const Pose stereo = poses[1] * invPose(poses[0]);
        rectify(rects[0], rects[1], cams[0], cams[1], stereo);
    }

    Mem2<Vec2> tables[2];
    Mem2<Byte> rimgs[2];

    for (int i = 0; i < 2; i++){
        makeRemapTable(tables[i], rects[i]);
        remap(rimgs[i], imgs[i], tables[i]);

        char str[SP_STRMAX];
        sprintf(str, "rect%d.bmp", i);
        saveBMP(str, rimgs[i]);
    }

    return 0;
}