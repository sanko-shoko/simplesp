#include "simplesp.h"
using namespace sp;
#include <locale.h>
int main() {

    // strget
    {
        const char *src = "aaa, bbb ccc\nddd";
        char dst[256];

        for (int i = 0; ; i++) {
            if(strget(dst, src, i) == false) break;
            printf("[%s]\n", dst);
        }

        {
            strget(dst, src, -4);
            printf("[%s]\n", dst);
        }
    }

    // character encode
    {

        char utf8[256];
        char sjis[256];
        sprintf(sjis, "あいうえお");

        strcode(utf8, sjis, SP_SJIS_TO_UTF8);
        strcode(sjis, utf8, SP_UTF8_TO_SJIS);

        printf("%s\n", sjis);
    }

    {
        printf("current dir [%s]\n", getCrntDir());

        printf("module dir [%s]\n", getModuleDir());
        printf("module path [%s]\n", getModulePath());
        printf("module name [%s]\n", getModuleName());

        char path[256];
        printf("path 0 [%s]\n", searchPath(path, getModuleDir(), 0));
        printf("path 1 [%s]\n", searchPath(path, getModuleDir(), 1));

        printf("extcmp %d\n", extcmp("test.txt", "txt"));
        printf("extcmp %d\n", extcmp("test.txt", "dat"));
        printf("extcmp %d\n", extcmp("test", "dat"));

        ::strcpy(path, "test.txt");
        printf("extset %s\n", extset(path, "txt"));
        ::strcpy(path, "test");
        printf("extset %s\n", extset(path, "txt"));

    }
    return 0;
}
