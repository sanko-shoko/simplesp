//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_KDTREE_H__
#define __SP_KDTREE_H__

#include "spcore/spcore.h"


namespace sp{

    //--------------------------------------------------------------------------------
    // kd tree
    //--------------------------------------------------------------------------------

    template<typename TYPE> class KdTree{

    private:

        // data dimension
        int m_dim;

        struct Node {
            // point data
            TYPE *data;

            // point index
            int index;

            // div dimension
            int div;

            // 0 : left node, 1 : right node
            Node *sub[2];
        };

        // point num;
        int m_size;

        // root node
        Node *m_root;

        // space rect 
        Mem2<TYPE> m_rect;

        // data ptr;
        Mem1<const TYPE*> m_stack;

        // memory pool
        MemP<Node> nodePool;
        MemP<TYPE> dataPool;

    public:

        KdTree(const int dim = 0){
            init(dim);
        }

        ~KdTree(){
        }

        //--------------------------------------------------------------------------------
        // initialize
        //--------------------------------------------------------------------------------

        void init(const int dim){
            dataPool.init(dim);

            m_dim = dim;
            m_size = 0;
            m_root = NULL;
            m_rect.resize(dim, 2);

            m_stack.clear();
        }

        //--------------------------------------------------------------------------------
        // util
        //--------------------------------------------------------------------------------

        int size() const {
            return m_size;
        }

        int dim() const {
            return m_dim;
        }

        const TYPE* getData(const int i) const {
            return &dataPool[i];
        }


        //--------------------------------------------------------------------------------
        // add data
        //--------------------------------------------------------------------------------

        void addData(const void *ptr){
            m_stack.push((const TYPE*)ptr);
        }

        void makeTree() {
            if (m_stack.size() == 0) return;

            shuffle(m_stack);
            for (int i = 0; i < m_stack.size(); i++) {
                const TYPE *data = m_stack[i];
                addNode(&m_root, data, 0, m_size);

                for (int i = 0; i < m_dim; i++) {
                    if (m_size == 0) {
                        m_rect(i, 0) = data[i];
                        m_rect(i, 1) = data[i];
                    }
                    else {
                        m_rect(i, 0) = (TYPE)minVal(data[i], m_rect(i, 0));
                        m_rect(i, 1) = (TYPE)maxVal(data[i], m_rect(i, 1));
                    }
                }

                m_size++;
            }

            m_stack.clear();
        }

        //--------------------------------------------------------------------------------
        // search data
        //--------------------------------------------------------------------------------

        int search(const void *ptr) const {
            SP_ASSERT(m_stack.size() == 0);
            if (m_root == NULL) return -1;

            const TYPE *data = (TYPE*)ptr;

            Mem2<TYPE> rect = m_rect;

            Node *result = m_root;
            double dist = normData(result->data, data);

            searchOne(dist, result, m_root, data, rect);

            return result->index;
        }

        Mem1<int> search(const void *ptr, double range) const {
            SP_ASSERT(m_stack.size() == 0);
            Mem1<int> index;

            const TYPE *data = (TYPE*)ptr;

            searchRange(index, m_root, data, range);
            
            return index;
        }

    private:

        double normData(const TYPE *data, const TYPE *base) const {
            double norm = 0.0;
            for (int i = 0; i < m_dim; i++) {
                norm += square(data[i] - base[i]);
            }
            return sqrt(norm);
        }

        double normRect(Mem2<TYPE> &rect, const TYPE *base) const {
            double norm = 0.0;

            for (int i = 0; i < m_dim; i++) {
                if (base[i] >= rect(i, 0) && base[i] <= rect(i, 1)) continue;

                const int t = (base[i] < rect(i, 0)) ? 0 : 1;
                norm += square(rect(i, t) - base[i]);
            }

            return sqrt(norm);
        }

        double cmpData(const TYPE *data0, const TYPE *data1, const int div) const {
            return data0[div] - data1[div];
        }

        void addNode(Node **parent, const TYPE *data, int div, const int index){

            if (!*parent) {
                Node *node = nodePool.malloc();
                node->data = dataPool.malloc();

                memcpy(node->data, data, m_dim * sizeof(TYPE));

                node->div = div % m_dim;
                node->sub[0] = NULL;
                node->sub[1] = NULL;
                node->index = index;

                *parent = node;
            }
            else{
                Node *node = *parent;
                const double d = cmpData(data, node->data, node->div);
                const int t = (d < 0.0) ? 0 : 1;

                addNode(&node->sub[t], data, node->div + 1, index);
            }
        }

        void searchOne(double &mindist, Node *&result, Node *node, const TYPE *data, Mem2<TYPE> &rect) const {

            const double d = cmpData(data, node->data, node->div);
            const int t = (d < 0.0) ? 0 : 1;

            // check near node
            if (node->sub[t] != NULL) {
                TYPE &side = rect(node->div, 1 - t);

                // cut rect
                const TYPE tmp = side;
                side = node->data[node->div];

                searchOne(mindist, result, node->sub[t], data, rect);
                
                // reset
                side = tmp;
            }

            const double dist = normData(node->data, data);
            if (dist < mindist) {
                result = node;
                mindist = dist;
            }

            // check far node
            if (node->sub[1 - t] != NULL) {
                TYPE *side = &rect(node->div, t);

                // cut rect
                const TYPE tmp = *side;
                *side = node->data[node->div];

                if (normRect(rect, data) < mindist) {
                    searchOne(mindist, result, node->sub[1 - t], data, rect);
                }

                // reset
                *side = tmp;
            }
        }

        void searchRange(Mem1<int> &index, Node *node, const TYPE *data, TYPE rect) const {
            if (node == NULL) return;

            const double dist = normData(node->data, data);
            if (dist <= rect) {
                index.push(node->index);
            }

            const double d = cmpData(data, node->data, node->div);
            const int t = (d < 0.0) ? 0 : 1;

            searchRange(index, node->sub[t], data, rect);
            if (fabs(d) < rect) {
                searchRange(index, node->sub[1 - t], data, rect);
            }
        }

    };
}

#endif