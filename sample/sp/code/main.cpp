﻿#define SP_USE_DEBUG 1

#include "simplesp.h"

using namespace sp;

int main() {

    //--------------------------------------------------------------------------------
    // zip like encode & decode
    //--------------------------------------------------------------------------------
    {
        Mem1<int> src;
        {
            int _src[] = { 0, 0, 0, 1, 2, 2, 2, 2, 2, 2, 4, 3, 3, 0, 2, 1 };
            src.push(_src, sizeof(_src) / sizeof(int));

            print("src ", src);
        }
        const int maxv = max(src) + 1;
        {
            const Mem1<Mem1<Byte>> table = hmMakeTableFromCnts(getCodeCnts(src, maxv));
            const Mem1<Byte> enc = hmEncode(table, src);
            print("enc ", enc);

            const Mem1<int> dec = hmDecode(table, enc);
            print("dec", dec);

        }
        {
            const Mem1<int> encA = lzssEncode(src, maxv, 63, 3, 15);

            const Mem1<int> cnts = lzssCnts(encA, maxv);

            const Mem1<Byte> encB = zlEncode(hmMakeTableFromCnts(cnts), encA, maxv, 6, 4);

            const Mem1<int> decB = zlDecode(hmMakeTableFromCnts(cnts), encB, maxv, 6, 4);
            const Mem1<int> decA = lzssDecode(decB, 5);

            print("encA", encA);
            print("encB", encB);
            print("decB", decB);
            print("decA", decA);
        }
    }


    //--------------------------------------------------------------------------------
    // base 64 encode & decode
    //--------------------------------------------------------------------------------
    {
        Mem1<Byte> src;
        {
            Byte _src[] = { 0, 0, 0, 1, 2, 2, 2, 2, 2, 2, 4, 3, 3, 0, 2, 1 };
            src.push(_src, sizeof(_src) / sizeof(Byte));

            print("src ", src);
        }
        {
            const Mem1<char> enc = base64Encode(src.ptr, src.size());

            const Mem1<Byte> dec = base64Decode(enc.ptr);

            printf("enc %s\n", enc.ptr);
            print("dec", dec);
        }
    }
    return 0;
}