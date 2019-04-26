#include "simplesp.h"

using namespace sp;

int main(){

    Mem2<Col3> src;
    SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/Lenna.bmp", src));

    Mem1<Vec2> pixs;
    harris(pixs, src);

    renderCircle(src, pixs, 4, getCol3(0, 255, 0), 1);

    saveBMP("harris.bmp", src);

    return 0;
}