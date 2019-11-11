#define SP_USE_DEBUG 1

#include "simplesp.h"

using namespace sp;

int main() {

    Mem1<int> data;
    {
        data.push(0);
        data.push(0);
        data.push(0);
        data.push(1);
        data.push(2);
        data.push(2);
        data.push(2);
        data.push(2);
        data.push(2);
        data.push(2);
        data.push(4);
        data.push(3);
        data.push(3);
        data.push(0);
        data.push(2);
        data.push(1);
    }
    {
        Mem1<int> e = lzssEncode(data, 5, 63, 3, 15);
        Mem1<int> cnts(5 + 1);
        cnts.zero();
        for (int i = 0; i < e.size(); i++) {
            const int p = e[i];
            cnts[p]++;
            if (cnts[p] == 5) {
                i += 2;
            }
        }
        print(e);
        Mem1<Byte> e2 = zlEncode(hmMakeTableFromCnts(cnts), e, 5, 6, 4);
        Mem1<int> d2 = zlDecode(hmMakeTableFromCnts(cnts), e2, 5, 6, 4);
        Mem1<int> d = lzssDecode(d2, 5);

        print(data);
        print(e);
        print(d2);
        print(d);
        return 0;
    }

    const Mem1<Mem1<Byte>> table = hmMakeTableFromCnts(getCodeCnts(data, 5));
    for (int i = 0; i < table.size(); i++) {
        print(table[i]);
    }
    {
        const Mem1<Byte> e = hmEncode(table, data);
        print(e);


        const Mem1<int> d = hmDecode(table, e);
        print(d);

    }
    return 0;
}