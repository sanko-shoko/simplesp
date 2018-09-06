#define SP_USE_DEBUG 1
#include "simplesp.h"

using namespace sp;

int main(){
    SP_LOGGER_INSTANCE;

    Mem2<Byte> src;
    {
        SP_ASSERT(loadBMP(SP_DATA_DIR  "/image/Lenna.bmp", src));
    }

    Mem2<Byte> dst0;
    Mem2<Byte> dst1;

    // box filter
    {
        SP_LOGGER_SET("boxfilter");

        boxFilter(dst0, src, 21);
    }

    // box filter integral
    {
        SP_LOGGER_SET("boxfilter integral");
    
        Mem2<int> sum;
        makeIntegral(sum, src);
        boxFilterIntegral(dst1, sum, 21);
    }

    saveBMP("test0.bmp", dst0);
    saveBMP("test1.bmp", dst1);


    return 0;
}