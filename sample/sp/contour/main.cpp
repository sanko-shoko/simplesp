#include "simplesp.h"

using namespace sp;

int main() {
    {
        Mem2<Col3> src(640, 480);

        setElm(src, getCol(255, 255, 255));

        renderPoint(src, getVec(320, 240), getCol(0, 180, 0), 100);
        renderPoint(src, getVec(400, 180), getCol(0, 180, 0), 60);

        const Rect rect = getRect2(120, 90, 400, 300);

        Mem1<Vec2> contour;
        snake(contour, src, vertex2(rect));

        for (int i = 0; i < contour.size(); i++) {
            const Vec2 &A = contour[(i + 0) % contour.size()];
            const Vec2 &B = contour[(i + 1) % contour.size()];
            renderPoint(src, A, getCol(0, 0, 0), 2);
        }

        saveBMP(src, "test0.bmp");
    }

    {
        Mem2<Col3> src;
        {
            SP_ASSERT(loadBMP(src, SP_DATA_DIR  "/image/shiba02.bmp"));
        }

        const Rect rect = getRect2(260, 60, 180, 260);

        Mem1<Vec2> contour;
        snake(contour, src, vertex2(rect));

        for (int i = 0; i < contour.size(); i++) {
            const Vec2 &A = contour[(i + 0) % contour.size()];
            const Vec2 &B = contour[(i + 1) % contour.size()];
            //renderLine(src, A, B, (Byte)255, 2);
            renderPoint(src, A, getCol(0, 0, 0), 2);
        }

        saveBMP(src, "test1.bmp");
    }

    return 0;
}