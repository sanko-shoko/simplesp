#define SP_USE_DEBUG 1

#include "simplesp.h"

using namespace sp;

int main() {

    Mem1<Mem1<Byte>> table;
    {
        Mem1<int> cnts;

        cnts.push(1);
        cnts.push(2);
        cnts.push(4);

        hmMakeTable(table, cnts);

        printf("table\n");
        for (int i = 0; i < table.size(); i++) {
            print(table[i]);
        }
    }
    {
        Mem1<int> data;
        data.push(0);
        data.push(0);
        data.push(1);
        data.push(2);
        data.push(0);
        data.push(0);
        data.push(2);
        data.push(1);

        printf("encode\n");
        Mem1<Byte> e;
        hmEncode(e, table, data);
        print(e);

        printf("decode\n");
        Mem1<int> d;
        hmDecode(d, table, e);
        print(d);
    }
    return 0;
}