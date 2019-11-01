//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_CODE_H__
#define __SP_CODE_H__

#include "spcore/spgen/spbase.h"
#include "spcore/spcpu/spmem.h"


namespace sp{

    static int cnvBitToByte(Mem1<Byte> &dst, const Mem1<Byte> &src) {
        dst.resize((src.size() + 7) / 8);
        dst.clear();

        for (int i = 0; i < src.size(); i++) {
            const int a = i / 8;
            const int b = i % 8;
            //setBit(dst[a], 
        }
        return 0;
    }


    //--------------------------------------------------------------------------------
    // huffman coding
    //--------------------------------------------------------------------------------

    bool hmMakeTable(Mem1<Mem1<Byte>> &table, const Mem1<int> &cnts) {
        table.clear();
        table.resize(cnts.size());

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
            return false;
        }
        if (nodes.size() == 1) {
            nodes[0].code->push(0);
            return true;
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
        return true;
    }


    bool hmEncode(Mem1<Byte> &dst, const Mem1<Mem1<Byte>> &table, const Mem1<int> &src) {
        dst.clear();
        
        for (int i = 0; i < src.size(); i++) {
            const int s = src[i];
            const Mem1<Byte> &bits = table[s];
            dst.push(bits);
        }
        return true;
    }

    bool hmDecode(Mem1<int> &dst, const Mem1<Mem1<Byte>> &table, const Mem1<Byte> &src) {
        dst.clear();

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

        return true;
    }
}

#endif