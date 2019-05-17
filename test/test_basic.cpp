#include "simplesp.h"
using namespace sp;

int main() {

    // split
    {
        const char *src = "aaa, bbb ccc\nddd";
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

        const char *src = "あいうえお";
        char dst[256];

        //strcode(dst, src, SP_SJIS_TO_UTF8);
    }

    {
        printf("current dir [%s]\n", getCrntDir());

        printf("module dir [%s]\n", getModuleDir());
        //printf("module path [%s]\n", getModulePath());
        //printf("module name [%s]\n", getModuleName());

        //printf("path 0 [%s]\n", searchPath(getModuleDir(), 0).c_str());
        //printf("path 1 [%s]\n", searchPath(getModuleDir(), 1).c_str());

    }
    return 0;
}
