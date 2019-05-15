#include "simplesp.h"
using namespace sp;

int main() {

    // split
    {
        char *src = "aaa, bbb ccc\nddd";
        char dst[256];

        for (int i = 0; ; i++) {
            if(split(dst, src, i) == false) break;
            printf("[%s]\n", dst);
        }

        {
            split(dst, src, -4);
            printf("[%s]\n", dst);
        }
    }

    // character encode
    {

        char *src = "あいうえお";
        char dst[256];

        strcode(dst, src, SP_SJIS_TO_UTF8);
        printf("[%s]\n", dst);

    }


    return 0;
}
