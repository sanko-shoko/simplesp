#include "simplesp.h"

using namespace sp;

int main(){

    //--------------------------------------------------------------------------------
    // binalize
    //--------------------------------------------------------------------------------

    Mem2<Byte> img;
    Mem2<Byte> bin;

    SP_ASSERT(loadBMP(img, SP_DATA_DIR  "/marker/cap_neko.bmp"));

    crop(img, img, getRect2(150, 120, 320, 240));
    saveBMP(img, "input.bmp");

    Mem<int> hist;
    histogram(hist, img, 255);
    saveText(hist, "hist.csv");


    // binalize thresh = 100
    binalize(bin, img, 100);
    saveBMP(bin, "bin100.bmp");


    // binalize thresh = 140
    binalize(bin, img, 140);
    saveBMP(bin, "bin140.bmp");


    // binalize thresh = adaptation
    binalizeAdapt(bin, img);
    saveBMP(bin, "binA.bmp");

    return 0;
}