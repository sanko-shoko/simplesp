//--------------------------------------------------------------------------------
// Copyright (c) 2017-2020, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------


#ifndef __SP_BPROP_H__
#define __SP_BPROP_H__

#include "spcore/spcore.h"


namespace sp {

    //--------------------------------------------------------------------------------
    // belief propagation
    //--------------------------------------------------------------------------------

    class BeliefPropagation {

    private:
        struct Node;
        struct Link;

        struct Node {
            // label 
            int label;

            const int *cost;

            // first link pointer
            Link *link;
        };

        struct Link {
            // id
            int id;

            // cost
            int cost;

            // connected node (root -> targ)
            Node *root;

            // connected node (root -> targ)
            Node *targ;

            // next link
            Link *next;

            // pair link (<=>)
            Link *pair;
        };

        int m_labelNum;

        Mem1<Node> m_nodes;
        Mem1<Link> m_links;

        Mem3<int> m_buff;
    public:

        BeliefPropagation() {
            m_labelNum = 0;
        }

        BeliefPropagation(const int labelNum, const int nodeMax, const int linkMax) {
            init(labelNum, nodeMax, linkMax);
        }

        void init(const int labelNum, const int nodeMax, const int linkMax) {
            m_labelNum = labelNum;

            m_nodes.clear();
            m_nodes.resize(nodeMax);
            m_nodes.zero();

            m_links.clear();
            m_links.reserve(2 * linkMax);

            m_buff.resize(2 * linkMax, labelNum, 2);
            m_buff.zero();
        }

        void setNode(const int i, const int *cost) {
            Node *node = &m_nodes[i];

            node->label = 0;
            node->cost = cost;
        }

        void setLink(int i, int j, int cost) {

            const int id = m_links.size();

            Link *linkAB = m_links.extend();
            Link *linkBA = m_links.extend();

            Node* A = &m_nodes[i];
            Node* B = &m_nodes[j];

            linkAB->id = id;
            linkAB->next = A->link;
            linkAB->pair = linkBA;
            linkAB->root = A;
            linkAB->targ = B;
            linkAB->cost = cost;
            A->link = linkAB;

            linkBA->id = id + 1;
            linkBA->next = B->link;
            linkBA->pair = linkAB;
            linkBA->root = B;
            linkBA->targ = A;
            linkBA->cost = cost;
            B->link = linkBA;
        }

        int getLabel(int i) {
            return m_nodes[i].label;
        }

        //--------------------------------------------------------------------------------
        // execute 
        //--------------------------------------------------------------------------------

        void execute(const int itmax = 10) {

            for (int it = 0; it < itmax; it++) {
                const int s = it % 2;
                const int d = 1 - s;

#if SP_USE_OMP
#pragma omp parallel for
#endif
                for (int i = 0; i < m_links.size(); i++) {
                    Link &link = m_links[i];
                    Node *node = link.root;

                    for(int a = 0; a < m_labelNum; a++){
                        int minv = SP_INTMAX;

                        for (int b = 0; b < m_labelNum; b++) {
                            int c = node->cost[b];
                            
                            if(a != b) c += link.cost;

                            Link *n = node->link;
                            while (n != NULL) {
                                c += m_buff(n->pair->id, b, s);

                                n = n->next;
                            }
                            if (c < minv) {
                                minv = c;
                            }
                        }
                        m_buff(i, a, d) = minv;
                    }
                }
            }

            {
                const int s = itmax % 2;

                for (int i = 0; i < m_nodes.size(); i++) {
                    Node &node = m_nodes[i];

                    int minv = SP_INTMAX;

                    for (int a = 0; a < m_labelNum; a++) {
                        int c = node.cost[a];

                        Link *n = node.link;
                        while (n != NULL) {
                            c += m_buff(n->pair->id, a, s);
                            n = n->next;
                        }
                        if (c < minv) {
                            minv = c;
                            node.label = a;
                        }
                    }
                }
            }
        }

    public:

    };

}

#endif