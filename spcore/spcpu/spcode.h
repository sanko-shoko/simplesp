//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_CODE_H__
#define __SP_CODE_H__

#include "spcore/spgen/spbase.h"
#include "spcore/spcpu/spmem.h"

namespace sp {

    //! @param src 8bit array
    SP_CPUFUNC Mem1<Byte> get1BitArray(const Mem1<Byte> &src, const int bits) {
        Mem1<Byte> dst(bits);
        dst.zero();

        for (int i = 0; i < dst.size(); i++) {
            const int a = i / 8;
            const int b = i % 8;
            dst[i] = getBit(&src[a], b);
        }
        return dst;
    }

    //! @param src 1bit array
    SP_CPUFUNC Mem1<Byte> get8BitArray(const Mem1<Byte> &src) {
        Mem1<Byte> dst((src.size() + 7) / 8);
        dst.zero();

        for (int i = 0; i < src.size(); i++) {
            const int a = i / 8;
            const int b = i % 8;
            setBit(&dst[a], b, src[i]);
        }
        return dst;
    }

    template<typename TYPE>
    SP_CPUFUNC Mem1<int> getCodeCnts(const Mem1<TYPE> &data, const int num) {
        Mem1<int> cnts(num);
        cnts.zero();

        for (int i = 0; i < data.size(); i++) {
            const int p = data[i];
            if (p >= 0 && p < num) {
                cnts[p]++;
            }
        }
        return cnts;
    }

    //--------------------------------------------------------------------------------
    // LZSS (Lempel–Ziv–Storer–Szymanski)
    //--------------------------------------------------------------------------------

    //! @param code
    //! @param search 
    //! @param minlen 
    //! @param maxlen 
    template<typename TYPE>
    SP_CPUFUNC Mem1<int> lzssEncode(const Mem1<TYPE> &data, const int code, const int maxSearch, const int minLength, const int maxLength) {
        Mem1<int> dst;
        if (data.size() == 0) return dst;

        dst.reserve(data.size());
        dst.push(data[0]);

        for (int i = 1; i < data.size(); ) {

            int search = 0;
            int length = minLength - 1;

            for (int j = maxVal(0, i - maxSearch); j < i; j++) {
                if (data[i] != data[j]) continue;

                int k = 1;
                const int maxk = minVal(maxLength, data.size() - 1 - i);
                for (; k <= maxk; k++) {
                    if (data[i + k] != data[j + k]) break;
                }
                if (k > length) {
                    search = i - j;
                    length = minVal(k, maxLength);
                }
            }
            if (search == 0) {
                dst.push(data[i]);
                i += 1;
            }
            else {
                dst.push(code);
                dst.push(search);
                dst.push(length);
                i += length;
            }
        }

        return dst;
    }

    //! @param code
    SP_CPUFUNC Mem1<int> lzssDecode(const Mem1<int> &data, const int code) {
        Mem1<int> dst;
        if (data.size() == 0) return dst;

        for (int i = 0; i < data.size(); i++) {
            if (data[i] != code) {
                dst.push(data[i]);
            }
            else {
                const int search = data[i + 1];
                const int length = data[i + 2];
                const int base = dst.size();
                for (int j = 0; j < length; j++) {
                    const int v = dst[base - search + j];
                    dst.push(v);
                }
                i += 2;
            }
        }

        return dst;
    }

    //--------------------------------------------------------------------------------
    // huffman coding
    //--------------------------------------------------------------------------------


    SP_CPUFUNC Mem1<Mem1<Byte>> hmMakeTableFromLngs(const Mem1<int> &lngs) {
        Mem1<Mem1<Byte>> table(lngs.size());

        int maxv = 0;
        int minv = SP_INTMAX;
        for (int i = 0; i < lngs.size(); i++) {
            const int n = lngs[i];
            if (n == 0) continue;
            maxv = maxVal(n, maxv);
            minv = minVal(n, minv);
        }
        if (maxv == 0) {
            return table;
        }

        Mem1<Byte> bits;
        for (int j = 0; j < minv; j++) {
            bits.push(0);
        }

        int prev = 0;
        for (int s = minv; s <= maxv; s++) {
            for (int i = 0; i < lngs.size(); i++) {
                if (lngs[i] != s) continue;

                if (prev > 0) {
                    for (int j = bits.size() - 1; j >= 0; j--) {
                        if (bits[j] == 0) {
                            bits[j] = 1;
                            break;
                        }
                        else {
                            bits[j] = 0;
                        }
                    }
                    for (int j = 0; j < s - prev; j++) {
                        bits.push(0);
                    }
                }

                prev = s;
                table[i] = bits;
            }
        }
        return table;
    }

    SP_CPUFUNC Mem1<Mem1<Byte>> hmMakeTableFromCnts(const Mem1<int> &cnts) {
        {
            int n = 0;
            int id = -1;
            for (int i = 0; i < cnts.size(); i++) {
                if (cnts[i] > 0) {
                    n++;
                    id = i;
                }
            }
            if (n == 0) {
                return Mem1<Mem1<Byte>>();
            }
            if (n == 1) {
                Mem1<Mem1<Byte>> table(cnts.size());
                table[id].push(0);
                return table;
            }
        }

        struct Node {
            int cnt;
            Node *parent;
        };

        Mem1<Node> nodes(cnts.size());
        for (int i = 0; i < cnts.size(); i++) {
            nodes[i].cnt = cnts[i];
            nodes[i].parent = NULL;
        }

        Mem1<Node*> heads;
        for (int i = 0; i < nodes.size(); i++) {
            if (nodes[i].cnt > 0) {
                heads.push(&nodes[i]);
            }
        }

        Mem1<Node> tmps;
        tmps.reserve(cnts.size() - 1);

        while (heads.size() >= 2) {
            Node &node = *tmps.extend();
            node.cnt = 0;
            node.parent = NULL;

            for (int j = 0; j < 2; j++) {
                int id = 0;
                int minv = SP_INTMAX;
                for (int k = 0; k < heads.size(); k++) {
                    if (heads[k]->cnt < minv) {
                        minv = heads[k]->cnt;
                        id = k;
                    }
                }
                node.cnt += heads[id]->cnt;
                heads[id]->parent = &node;

                heads.del(id);
            }

            heads.push(&node);
        }

        Mem1<int> lngs(cnts.size());
        for (int i = 0; i < lngs.size(); i++) {
            lngs[i] = 0;

            Node *node = &nodes[i];
            while (node->parent != NULL) {
                lngs[i]++;
                node = node->parent;
            }
        }
        return hmMakeTableFromLngs(lngs);
    }


    template<typename TYPE>
    SP_CPUFUNC Mem1<Byte> hmEncode(const Mem1<Mem1<Byte>> &table, const Mem1<TYPE> &src) {
        Mem1<Byte> dst;

        for (int i = 0; i < src.size(); i++) {
            const int s = src[i];
            const Mem1<Byte> &bits = table[s];
            dst.push(bits);
        }
        return dst;
    }

    struct hmNode {
        int val;
        int child[2];
    };

    SP_CPUFUNC Mem1<hmNode> hmMakeNode(const Mem1<Mem1<Byte>> &table) {
        Mem1<hmNode> nodes;
        nodes.reserve(2 * table.size() - 1);

        nodes.extend();
        nodes[0].val = -1;
        nodes[0].child[0] = -1;
        nodes[0].child[1] = -1;

        for (int i = 0; i < table.size(); i++) {
            const Mem1<Byte> &bits = table[i];
            if (bits.size() == 0) continue;

            hmNode *node = &nodes[0];
            for (int j = 0; j < bits.size(); j++) {
                const Byte bit = bits[j];
                if (node->child[bit] == -1) {
                    node->child[bit] = nodes.size();
                    
                    node = nodes.extend();
                    node->val = -1;
                    node->child[0] = -1;
                    node->child[1] = -1;
                }
                else {
                    node = &nodes[node->child[bit]];
                }
            }
            node->val = i;
        }
        return nodes;
    }

    SP_CPUFUNC Mem1<int> hmDecode(const Mem1<Mem1<Byte>> &table, const Mem1<Byte> &src) {
        Mem1<int> dst;

        const Mem1<hmNode> nodes = hmMakeNode(table);

        const hmNode *node = &nodes[0];
        for (int i = 0; i < src.size(); i++) {
            const Byte bit = src[i];

            node = &nodes[node->child[bit]];
            const int val = node->val;

            if (val >= 0) {
                dst.push(val);
                node = &nodes[0];
            }
        }

        return dst;
    }


    //--------------------------------------------------------------------------------
    // zip like code
    //--------------------------------------------------------------------------------

    template<typename TYPE>
    SP_CPUFUNC Mem1<Byte> zlEncode(const Mem1<Mem1<Byte>> &table, const Mem1<TYPE> &src, const int code, const int searchBit, const int lengthBit) {
        Mem1<Byte> dst;

        for (int i = 0; i < src.size(); i++) {
            const int s = src[i];
            const Mem1<Byte> &bits = table[s];

            dst.push(bits);
            if (s == code) {
                int v;
                v = (src[i + 1]);
                for (int j = 0; j < searchBit; j++) {
                    dst.push((v >> j) & 0x01);
                }
                v = (src[i + 2]);
                for (int j = 0; j < lengthBit; j++) {
                    dst.push((v >> j) & 0x01);
                }
                i += 2;
            }
        }
        return dst;
    }

    SP_CPUFUNC Mem1<int> zlDecode(const Mem1<Mem1<Byte>> &table, const Mem1<Byte> &src, const int code, const int searchBit, const int lengthBit) {
        Mem1<int> dst;

        const Mem1<hmNode> nodes = hmMakeNode(table);

        const hmNode *node = &nodes[0];
        for (int i = 0; i < src.size(); i++) {
            const Byte bit = src[i];

            node = &nodes[node->child[bit]];
            const int val = node->val;

            if (val >= 0) {
                dst.push(val);
                node = &nodes[0];
            }
            if (val == code) {
                {
                    int v = 0;
                    for (int j = 0; j < searchBit; j++, i++) {
                        v = v + (src[i + 1] << j);
                    }
                    dst.push(v);
                }
                {
                    int v = 0;
                    for (int j = 0; j < lengthBit; j++, i++) {
                        v = v + (src[i + 1] << j);
                    }
                    dst.push(v);
                }
            }
        }

        return dst;
    }

}

#endif