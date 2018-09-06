#define SP_USE_DEBUG 1
#include "simplesp.h"

using namespace sp;

int main(){

    //--------------------------------------------------------------------------------
    // slic (simple linear iterative clustering)
    //--------------------------------------------------------------------------------

    Mem2<Col3> img;
    SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/Lenna.bmp", img));

    Mem2<int> map;

    // slic
    {
        slic(map, img, 20);
    }

    // visualize
    {
        Mem2<Col3> col;
        cnvLabelToImg(col, map);
        saveBMP("color.bmp", col);

        Mem1<Mem1<Vec2> > contours = getLabelContour(map);
        for (int i = 0; i < contours.size(); i++){
            renderPoint(img, contours[i], getCol(0, 0, 0));
        }
        saveBMP("contour.bmp", img);
    }

    return 0;
}