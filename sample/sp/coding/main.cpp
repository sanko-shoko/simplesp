#define SP_USE_DEBUG 1

#include "simplesp.h"

using namespace sp;

int main() {

    Mem1<int> data;
    {
        data.push(0);
        data.push(0);
        data.push(1);
        data.push(2);
        data.push(0);
        data.push(0);
        data.push(2);
        data.push(1);
    }

    const Mem1<Mem1<Byte>> table = hmMakeTable(getCodeCnts(data, 3));

    {
        const Mem1<Byte> e = hmEncode(table, data);


        const Mem1<int> d = hmDecode(table, e);
        print(d);

    }
    return 0;
}