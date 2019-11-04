//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_CODE_H__
#define __SP_CODE_H__

#include "spcore/spgen/spbase.h"
#include "spcore/spcpu/spmem.h"


namespace sp{

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
                    length = k;
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

    //--------------------------------------------------------------------------------
    // huffman coding
    //--------------------------------------------------------------------------------

    SP_CPUFUNC Mem1<Mem1<Byte>> hmMakeTable(const Mem1<int> &cnts) {
        Mem1<Mem1<Byte>> table(cnts.size());

        struct Node {
            int cnt;
            Node *child[2];
            Mem1<Byte> *code;
        };

        Mem1<Node> nodes;
        nodes.reserve(2 * cnts.size() - 1);

        for (int i = 0; i < cnts.size(); i++) {
            if (cnts[i] == 0) continue;

            Node node;
            node.cnt = cnts[i];
            node.child[0] = NULL;
            node.child[1] = NULL;
            node.code = &table[i];
            nodes.push(node);
        }

        if (nodes.size() == 0) {
            return table;
        }
        if (nodes.size() == 1) {
            nodes[0].code->push(0);
            return table;
        }

        Node *root = NULL;
        {
            Mem1<Node*> heads;
            for (int i = 0; i < nodes.size(); i++) {
                heads.push(&nodes[i]);
            }
            while (heads.size() != 1) {
                Node node;
                node.cnt = 0;
                node.code = NULL;

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
                    node.child[1 - j] = heads[id];

                    heads.del(id);
                }

                nodes.push(node);
                heads.push(nodes.last());
            }
            root = heads[0];
        }
        {
            Mem1<Node*> stack;
            stack.push(root);

            Mem1<Byte> bits;

            int move = 0;
            while (!(move == -1 && stack.size() == 1)) {
                Node *node = *stack.last();

                if (node->code == NULL) {
                    if (move >= 0) {
                        stack.push(node->child[move]);
                        bits.push(move);
                        move = +0;
                    }
                    else {
                        move = (*bits.last() == 0) ? +1 : -1;
                        stack.pop();
                        bits.pop();
                    }
                }
                else {
                    *node->code = bits;
                    //print(bits);

                    move = (*bits.last() == 0) ? +1 : -1;

                    stack.pop();
                    bits.pop();
                }
            }
        }

        {
            int maxv = 0;
            int minv = SP_INTMAX;
            for (int i = 0; i < table.size(); i++) {
                const int n = table[i].size();
                if (n == 0) continue;
                maxv = maxVal(n, maxv);
                minv = minVal(n, minv);
            }

            Mem1<Byte> bits;

            int prev = minv;
            for (int s = minv; s <= maxv; s++) {
                for (int i = 0; i < table.size(); i++) {
                    if (table[i].size() == s) {

                        if (bits.size() == 0) {
                            for (int j = 0; j < minv; j++) {
                                bits.push(0);
                            }
                        }
                        else {
                            for (int j = bits.size() - 1; j >= 0; j--) {
                                if (bits[j] == 0) {
                                    bits[j] = 1;
                                    break;
                                }
                                else {
                                    bits[j] = 0;
                                }
                            }
                        }
                        if (table[i].size() > prev) {
                            for (int j = 0; j < table[i].size() - prev; j++) {
                                bits.push(0);
                            }
                            prev = table[i].size();
                        }

                        table[i] = bits;
                    }
                }
            }

        }
        return table;
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

    SP_CPUFUNC Mem1<int> hmDecode(const Mem1<Mem1<Byte>> &table, const Mem1<Byte> &src) {
        Mem1<int> dst;

        struct Node {
            int val;
            Node *child[2];
        };

        Mem1<Node> nodes;
        nodes.reserve(2 * table.size() - 1);
        Node *root = nodes.extend();
        root->val = -1;
        root->child[0] = NULL;
        root->child[1] = NULL;

        for (int i = 0; i < table.size(); i++) {
            const Mem1<Byte> &bits = table[i];
            if (bits.size() == 0) continue;
            Node *node = root;
            for (int j = 0; j < bits.size(); j++) {
                const Byte bit = bits[j];
                if (node->child[bit] == NULL) {
                    node->child[bit] = nodes.extend();
                    node = node->child[bit];
                    node->val = -1;
                    node->child[0] = NULL;
                    node->child[1] = NULL;
                }
                else {
                    node = node->child[bit];
                }
            }
            node->val = i;
        }
        {
            Node *node = NULL;
            for (int i = 0; i < src.size(); i++) {
                if (node == NULL) {
                    node = root;
                }
                const Byte bit = src[i];

                node = node->child[bit];
                if (node->val >= 0) {
                    dst.push(node->val);
                    node = NULL;
                }
            }
        }

        return dst;
    }
}

#endif