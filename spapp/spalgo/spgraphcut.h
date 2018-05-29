//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

// [reference]
// Y.Boykov and V.Kolmogorov, 
// "An Experimental Comparison of Min-Cut/Max-Flow Algorithms for Energy Minimization in Vision", 
// IEEE Transactions on Pattern Analysis and Machine Intelligence(PAMI), 2004


#ifndef __SP_GRAPHCUT_H__
#define __SP_GRAPHCUT_H__

#include "spcore/spcore.h"


namespace sp{

    //--------------------------------------------------------------------------------
    // graph cut
    //--------------------------------------------------------------------------------

    class GraphCut{

    private:
        struct Node;
        struct Link;

        struct Node{
            // label (0: source / 1: sink)
            int label;

            // cap > 0 : residual source capacity
            // cap < 0 : residual sink capacity
            int cap;

            // first link pointer
            Link *link;

            // parent link pointer
            Link *parent;

            // next active node
            Node *next;

            // timestamp
            int time;

            // distance to the terminal
            int dist;
        };

        struct Link{
            // residual capacity
            int cap;

            // connected node (root -> targ)
            Node *root;

            // connected node (root -> targ)
            Node *targ;

            // next link
            Link *next;

            // pair link (<=>)
            Link *pair;
        };

        struct Orphan {
            Node *node;
            Orphan *next;
        };

        template<typename TYPE> struct List{
            TYPE *first, *last;
        };

        Mem1<Node> m_nodes;
        Mem1<Link> m_links;

        MemP<Orphan> m_orphanPool;
        
        // active m_nodes
        List<Node> m_actives;

        // orphans
        List<Orphan> m_orphans;

        Link *const TERMINAL = (Link *const)1;
        Link *const ORPHAN = (Link *const)2;

    public:

        GraphCut(){
        }

        GraphCut(const int nodeMax, const int linkMax){
            init(nodeMax, linkMax);
        }

        void init(const int nodeMax, const int linkMax){
            m_actives.first = m_actives.last = NULL;
            m_orphans.first = m_orphans.last = NULL;

            m_nodes.clear();
            m_nodes.resize(nodeMax);
            m_nodes.zero();

            m_links.clear();
            m_links.reserve(2 * linkMax);
        }

        void setNode(const int i, const int source, const int sink){
            Node *node = &m_nodes[i];

            node->cap = source - sink;

            if (node->cap != 0){
                node->label = (node->cap > 0) ? 0 : 1;
                node->parent = TERMINAL;
                node->dist = 1;

                addActive(node);
            }
        }

        void setLink(const int i, const int j, const int cap){

            Link *linkAB = m_links.extend();
            Link *linkBA = m_links.extend();

            Node* A = &m_nodes[i];
            Node* B = &m_nodes[j];

            linkAB->pair = linkBA;
            linkAB->next = A->link;
            linkAB->root = A;
            linkAB->targ = B;
            linkAB->cap = cap;
            A->link = linkAB;

            linkBA->pair = linkAB;
            linkBA->next = B->link;
            linkBA->root = B;
            linkBA->targ = A;
            linkBA->cap = cap;
            B->link = linkBA;
        }

        int getLabel(const int i) const {
            return m_nodes[i].label;
        }

        //--------------------------------------------------------------------------------
        // execute min-cut / max-flow algorithm
        //--------------------------------------------------------------------------------

        int execute(){

            SP_LOGGER_INSTANCE;
            SP_LOGGER_SET("-graphcut");

            Node *node = NULL;

            int maxflow = 0;

            for (int time = 0;; time++){

                if (node == NULL || node->parent == NULL){
                    node = getActive();
                    if (node == NULL) break;
                }

                // grow
                Link *orphan = grow(node);

                if (orphan == NULL){
                    node = NULL;
                    continue;
                }

                // set active
                node->next = node;

                // augmentation
                maxflow += augment(orphan);

                // adoption
                Orphan *que = m_orphans.first;
                while (que != NULL){

                    Orphan *next = que->next;
                    que->next = NULL;

                    while (que != NULL){
                        m_orphans.first = que->next;
                        if (m_orphans.first == NULL) m_orphans.last = NULL;

                        adopt(que->node, time + 1);
                        m_orphanPool.free(que);

                        que = m_orphans.first;
                    }

                    que = next;
                }

                // remove active
                node->next = NULL;

            }

            return maxflow;
        }

    public:

        void addActive(Node *node){
            if (node->next != NULL) return;

            if (m_actives.last != NULL){
                m_actives.last->next = node;
            }
            else{
                m_actives.first = node;
            }
            m_actives.last = node;

            // end of list
            node->next = node;
        }

        Node* getActive(){

            while (m_actives.first != NULL){
                Node *node = m_actives.first;

                m_actives.first = node->next;
                node->next = NULL;

                // check end of list
                if (m_actives.first == node){
                    m_actives.first = NULL;
                    m_actives.last = NULL;
                }

                if (node->parent != NULL){
                    return node;
                }
            }

            return NULL;
        }

        Link* grow(Node *node){
            Link *orphan = NULL;

            for (Link *link = node->link; link != NULL; link = link->next){
                Link *base = (node->label == 0) ? link : link->pair;
                Node *ref = link->targ;

                if (base->cap == 0) continue;

                if (ref->parent == NULL){

                    ref->label = node->label;
                    ref->parent = link->pair;
                    ref->time = node->time;
                    ref->dist = node->dist + 1;

                    addActive(ref);
                }
                else if (ref->label != node->label){
                    orphan = base;
                    break;
                }
                else if (ref->time <= node->time && ref->dist > node->dist){

                    // update
                    ref->parent = link->pair;
                    ref->time = node->time;
                    ref->dist = node->dist + 1;
                }
            }

            return orphan;
        }

        void addOrphanFirst(Node *node){
            Orphan *np = m_orphanPool.malloc();

            node->parent = ORPHAN;
            np->node = node;

            np->next = m_orphans.first;
            m_orphans.first = np;
        }

        void addOrphanLast(Node *node){
            Orphan *np = m_orphanPool.malloc();

            node->parent = ORPHAN;
            np->node = node;

            if (m_orphans.last){
                m_orphans.last->next = np;
            }
            else{
                m_orphans.first = np;
            }
            m_orphans.last = np;
            np->next = NULL;
        }

        int augment(Link *middle){

            int bottleneck = middle->cap;

            // find bottleneck (0: to source / 1: to sink)
            for (int i = 0; i < 2; i++){
                Node *node = (i == 0) ? middle->root : node = middle->targ;

                while (1){
                    Link *link = node->parent;
                    if (link == TERMINAL) break;

                    const int cap = (i == 0) ? link->pair->cap : link->cap;
                    if (cap < bottleneck) bottleneck = cap;

                    node = link->targ;
                }

                const int cap = (i == 0) ? node->cap : -node->cap;
                if (cap < bottleneck) bottleneck = cap;
            }

            middle->pair->cap += bottleneck;
            middle->cap -= bottleneck;

            // augment (0: to source / 1: to sink)
            for (int i = 0; i < 2; i++){
                Node *node = (i == 0) ? middle->root : node = middle->targ;
                int sign = (i == 0) ? +1 : -1;

                while (1){
                    Link *link = node->parent;
                    if (link == TERMINAL) break;

                    link->cap += sign * bottleneck;
                    link->pair->cap -= sign * bottleneck;

                    const int cap = (i == 0) ? link->pair->cap : link->cap;
                    if (cap == 0){

                        // add orphane (start of list)
                        addOrphanFirst(node);
                    }

                    node = link->targ;
                }

                node->cap -= sign * bottleneck;
                if (node->cap == 0){

                    // add orphane (start of list)
                    addOrphanFirst(node);
                }
            }
            return bottleneck;
        }

        void adopt(Node *node, const int time){

            int minv = SP_INTMAX;
            Link *parent = NULL;

            // find parent
            for (Link *link = node->link; link != NULL; link = link->next){

                Link *base = ((node->label != 0) ? link : link->pair);
                if (base->cap == 0) continue;

                int dist = 0;

                // search path
                for (Node *ref = link->targ;; ref = ref->parent->targ){

                    if (node->label != ref->label || ref->parent == NULL || ref->parent == ORPHAN){
                        dist = SP_INTMAX;
                        break;
                    }

                    if (ref->time == time){

                        dist += ref->dist;
                        break;
                    }
                    dist++;

                    if (ref->parent == TERMINAL){
                        ref->time = time;
                        ref->dist = 1;
                        break;
                    }
                }
                if (dist == SP_INTMAX) continue;

                if (dist < minv){
                    parent = link;
                    minv = dist;
                }

                // set mark
                for (Node *ref = link->targ;; ref = ref->parent->targ){
                    if (ref->time == time){
                        break;
                    }
                    ref->time = time;
                    ref->dist = dist--;
                }
            }

            node->parent = parent;

            if (parent){
                node->time = time;
                node->dist = minv + 1;
            }
            else{

                for (Link *link = node->link; link != NULL; link = link->next){
                    Link *base = (node->label != 0) ? link : link->pair;
                    Node *ref = link->targ;

                    if (node->label != ref->label || ref->parent == NULL) continue;

                    if (base->cap){
                        addActive(ref);
                    }

                    if (ref->parent != TERMINAL && ref->parent != ORPHAN && ref->parent->targ == node){

                        // add orphane (end of list)
                        addOrphanLast(ref);
                    }
                }
            }
        }
    };

}

#endif