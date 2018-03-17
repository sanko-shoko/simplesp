#define SP_USE_DEBUG 1
#define SP_USE_GLEW 1
#define SP_USE_IMGUI 1

#include "simplesp.h"
#include "spex/spcv.h"
#include "spex/spgl.h"

using namespace sp;

int main(){ 
    SP_PRINTF("compile test\n");

    printf("%s", getCrntDir().c_str());
    return 0;
    XML xml;

    xml.begin("test");
    xml.add("val0", "aaa");
    xml.add("val1", "bbb");
    {
        xml.begin("0");
        xml.add("val00", "0");
        xml.add("val01", "1");
        xml.end();
    }
    xml.end();

    xml.save("test.xml");
    return 0;
}