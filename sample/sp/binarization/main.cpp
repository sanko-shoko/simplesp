#include "simplesp.h"

using namespace sp;

int main(){

    //--------------------------------------------------------------------------------
    // binarization
    //--------------------------------------------------------------------------------

    Mem2<Byte> img;
    Mem2<Byte> bin;

    SP_ASSERT(loadBMP(SP_DATA_DIR  "/marker/cap_neko.bmp", img));

    crop(img, img, getRect2(150, 120, 320, 240));
    saveBMP("input.bmp", img);

    Mem<int> hist;
    histogram(hist, img, 255);
    saveText("hist.csv", hist);


    // binalize thresh = 100
    binalize(bin, img, 100);
    saveBMP("bin100.bmp", bin);


    // binalize thresh = 140
    binalize(bin, img, 140);
    saveBMP("bin140.bmp", bin);


    // binalize thresh = adaptation
    binalizeAdapt(bin, img);
    saveBMP("binA.bmp", bin);

    return 0;
}