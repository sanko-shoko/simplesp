#include "simplesp.h"

using namespace sp;

int main() {
    {
        Mem2<Col3> src(640, 480);
        {
            setElm(src, getCol(255, 255, 255));

            renderPoint(src, getVec(320, 240), getCol(0, 180, 0), 100);
            renderPoint(src, getVec(400, 180), getCol(0, 180, 0), 60);
        }

        const Rect rect = getRect2(120, 90, 400, 300);
        Mem1<Vec2> vtxs;
        {
            vtxs.push(getVec(rect.dbase[0], rect.dbase[1]));
            vtxs.push(getVec(rect.dbase[0], rect.dbase[1] + rect.dsize[1]));
            vtxs.push(getVec(rect.dbase[0] + rect.dsize[0], rect.dbase[1] + rect.dsize[1]));
            vtxs.push(getVec(rect.dbase[0] + rect.dsize[0], rect.dbase[1]));
        }

        Mem1<Vec2> contour;
        snake(contour, src, vtxs);

        for (int i = 0; i < contour.size(); i++) {
            const Vec2 &A = contour[(i + 0) % contour.size()];
            const Vec2 &B = contour[(i + 1) % contour.size()];
            renderPoint(src, A, getCol(0, 0, 0), 3);
        }

        saveBMP("test0.bmp", src);
    }

    {
        Mem2<Col3> src;
        {
            SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/shiba02.bmp", src));
        }

        const Rect rect = getRect2(260, 60, 180, 260);
        Mem1<Vec2> vtxs;
        {
            vtxs.push(getVec(rect.dbase[0], rect.dbase[1]));
            vtxs.push(getVec(rect.dbase[0], rect.dbase[1] + rect.dsize[1]));
            vtxs.push(getVec(rect.dbase[0] + rect.dsize[0], rect.dbase[1] + rect.dsize[1]));
            vtxs.push(getVec(rect.dbase[0] + rect.dsize[0], rect.dbase[1]));
        }

        Mem1<Vec2> contour;
        snake(contour, src, vtxs);

        for (int i = 0; i < contour.size(); i++) {
            const Vec2 &A = contour[(i + 0) % contour.size()];
            const Vec2 &B = contour[(i + 1) % contour.size()];
            //renderLine(src, A, B, (Byte)255, 2);
            renderPoint(src, A, getCol(0, 0, 0), 3);
        }

        saveBMP("test1.bmp", src);
    }

    return 0;
}