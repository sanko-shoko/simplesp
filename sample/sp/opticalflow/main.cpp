#define SP_USE_DEBUG 1
#include "simplesp.h"

using namespace sp;

int main() {

    //--------------------------------------------------------------------------------
    // opticalflow
    //--------------------------------------------------------------------------------

    Mem2<Col3> img0, img1;

    // load image
    {
        if (1) {
            SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/shiba02.bmp", img0));
            SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/shiba03.bmp", img1));
        }
        else {
            Mem2<Col3> img;
            SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/Lenna.bmp", img));
            const Rect rect = adjustRect(getRect2(img.dsize), -64);

            Rect rect0 = rect;
            Rect rect1 = rect;

            rect1.dbase[0] += 30;
            rect1.dbase[1] += 30;

            crop<Col3, Byte>(img0, img, rect0);
            crop<Col3, Byte>(img1, img, rect1);
        }
        saveBMP("input0.bmp", img0);
        saveBMP("input1.bmp", img1);
    }


    Mem1<Vec2> pixs;

    // detect corner
    {
        harris(pixs, img1);

        Mem2<Col3> img = img1;
        renderCircle(img, pixs, 4, getCol(0, 255, 0), 1);

        saveBMP("corner.bmp", img);
    }

    // optical flow (Lucas Kanade method)
    {
        Mem1<Vec2> flows;
        Mem1<bool> masks;
        opticalFlowLK(flows, masks, img0, img1, pixs);
        Mem2<Col3> img = img0;

        for (int i = 0; i < flows.size(); i++) {
            if (masks[i] == false) continue;

            const Vec2 pix = pixs[i];
            const Vec2 flow = flows[i];

            const double angle = (flow.x != 0.0 || flow.y != 0.0) ? ::atan2(flow.x, flow.y) : 0.0;
            const double norm = normVec(flow) / 50.0;

            Col3 col;
            cnvHSVToCol(col, getVec(angle + SP_PI, minVal(1.0, norm), 1.0));

            renderLine(img, pix, pix + flow, col, 2);
        }
        saveBMP("opticalflowLK.bmp", img);
    }

    // optical flow (Patch Match)
    {
        Mem2<Vec2> flows;
        Mem2<bool> masks;
        opticalFlowPM(flows, masks, img0, img1, 11);

        Mem2<Col3> img(img0.dsize);
        img.zero();

        for (int v = 0; v < img0.dsize[1]; v++) {
            for (int u = 0; u < img0.dsize[0]; u++) {
                const Vec2 pix = getVec(u, v);
                const Vec2 flow = flows(u, v);

                const double angle = (flow.x != 0.0 || flow.y != 0.0) ? ::atan2(flow.x, flow.y) : 0.0;
                const double norm = normVec(flow) / 50.0;

                Col3 col;
                cnvHSVToCol(col, getVec(angle + SP_PI, minVal(1.0, norm), 1.0));

                img(u, v) = col;
            }
        }
        saveBMP("opticalflowPM.bmp", img);
    }
    return 0;
}