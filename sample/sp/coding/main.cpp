#define SP_USE_DEBUG 1

#include "simplesp.h"

using namespace sp;

int main() {

    Mem1<int> data;
    {
        int src[] = {0, 0, 0, 1, 2, 2, 2, 2, 2, 2, 4, 3, 3, 0, 2, 1};
        data.push(src, sizeof(src) / sizeof(int));
    }
    {
        const int maxv = maxVal(data) + 1;
        Mem1<int> e = lzssEncode(data, maxv, 63, 3, 15);
        Mem1<int> cnts(5 + 1);
        cnts.zero();
        for (int i = 0; i < e.size(); i++) {
            const int p = e[i];
            cnts[p]++;
            if (cnts[p] == 5) {
                i += 2;
            }
        }
        Mem1<Byte> e2 = zlEncode(hmMakeTableFromCnts(cnts), e, maxv, 6, 4);

        Mem1<int> d2 = zlDecode(hmMakeTableFromCnts(cnts), e2, maxv, 6, 4);
        Mem1<int> d = lzssDecode(d2, 5);

        print(data);
        print(e);
        print(d2);
        print(d);
        return 0;
    }

    const Mem1<Mem1<Byte>> table = hmMakeTableFromCnts(getCodeCnts(data, 5));
    {
        const Mem1<Byte> e = hmEncode(table, data);
        print(e);


        const Mem1<int> d = hmDecode(table, e);
        print(d);

    }
    return 0;
}