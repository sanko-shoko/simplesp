//--------------------------------------------------------------------------------
// Copyright (c) 2017-2020, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_RFOREST_H__
#define __SP_RFOREST_H__

#include "spcore/spcore.h"


namespace sp{

    //--------------------------------------------------------------------------------
    // random forest base class
    //--------------------------------------------------------------------------------

    template<typename TYPE>
    class RandomForest {

    public:
         struct Node{

            // node value
            TYPE val;

            // node deviation
            SP_REAL dev;


            // div parameter
            int param;

            // div thresh
            SP_REAL thresh;
            
            // next node ptr (X[i] < thresh) ? next[0] : next[1]
            Node *next[2];

            Node() {
                memset(this, 0, sizeof(Node));
            }
        };

    protected:

        MemP<MemP<Node> > m_trees;

    public:

        RandomForest() {
            reset();
        }
        void reset() {
            m_trees.clear();
        }

        void train(const Mem1<Mem<SP_REAL> >& Xs, Mem1<TYPE> &Ys, const int sampleNum = 100) {
            SP_ASSERT(sampleNum > 10);

            const int seed = m_trees.size();
            const Mem1<int> index = shuffle(Xs.size(), seed).part(0, sampleNum);

            divTree(*m_trees.malloc(), Xs, Ys, index, 0);
        }

        Mem1<TYPE> execute(const Mem<SP_REAL> &X){
            SP_ASSERT(m_trees.size() > 0);

            Mem1<TYPE> results;
            for (int i = 0; i < m_trees.size(); i++){
                const Node *node = traceNode(m_trees[i], X);
                results.push(node->val);
            }

            return results;
        }

        Mem1<const Node*> execute2(const Mem<SP_REAL> &X) {
            SP_ASSERT(m_trees.size() > 0);

            Mem1<const Node*> results;
            for (int i = 0; i < m_trees.size(); i++) {
                const Node *node = traceNode(m_trees[i], X);
                results.push(node);
            }

            return results;
        }

    private:

        virtual Node* getNode(MemP<Node> &tree, const Mem1<Mem<SP_REAL> > &Xs, const Mem1<TYPE> &Ys, const Mem1<int> &index) = 0;

        virtual SP_REAL calcGain(const Mem1<Mem<SP_REAL> > &Xs, const Mem1<TYPE> &Ys, const Mem1<int> &index, const int param, const double thresh) = 0;

        Node* divTree(MemP<Node> &tree, const Mem1<Mem<SP_REAL> >& Xs, const Mem1<TYPE> &Ys, const Mem1<int> &index, const int depth) {

            Node *node = getNode(tree, Xs, Ys, index);

            // check status
            {
                if (index.size() < 2) {
                    return node;
                }

                const int maxd = maxVal(floor(log(Xs.size()) / log(2.0) - 2), 2);
                if (depth >= maxd) {
                    return node;
                }

                const SP_REAL minv = 0.01;
                if (tree.size() > 1 && (tree[0].dev == 0.0 || node->dev / tree[0].dev < minv)) {
                    return node;
                }
            }
            
            SP_REAL maxg = -SP_INFINITY;

            // calc gain & thresh
            {
                const int dim = Xs[0].size();

                Mem1<SP_REAL> maxvs(dim);
                Mem1<SP_REAL> minvs(dim);
                for (int s = 0; s < dim; s++) {
                    SP_REAL maxv = -SP_INFINITY;
                    SP_REAL minv = +SP_INFINITY;
                    for (int n = 0; n < index.size(); n++) {
                        const SP_REAL v = Xs[index[n]][s];
                        maxv = maxVal(maxv, v);
                        minv = minVal(minv, v);
                    }
                    maxvs[s] = maxv;
                    minvs[s] = minv;
                }
                
                for (int i = 0; i < dim * 10; i++) {

                    const int param = rand() % Xs[0].size();

                    const SP_REAL thresh = randu() * (maxvs[param] - minvs[param]) + minvs[param];

                    const SP_REAL gain = calcGain(Xs, Ys, index, param, thresh);

                    if (gain > maxg) {
                        maxg = gain;
                        node->param = param;
                        node->thresh = thresh;
                    }
                }
            }

            // node division
            if (maxg > -SP_INFINITY) {

                Mem1<int> div[2];
                for (int n = 0; n < index.size(); n++) {
                    const int s = Xs[index[n]][node->param] < node->thresh ? 0 : 1;
                    div[s].push(index[n]);
                }

                node->next[0] = divTree(tree, Xs, Ys, div[0], depth + 1);
                node->next[1] = divTree(tree, Xs, Ys, div[1], depth + 1);
            }

            return node;
        }

        const Node* traceNode(const MemP<Node> &tree, const Mem<SP_REAL> &X) {

            const Node *node = &tree[0];
            while (node != NULL && node->next[0] != NULL && node->next[1] != NULL) {

                node = node->next[X[node->param] < node->thresh ? 0 : 1];
            }

            return node;
        }

    };



    //--------------------------------------------------------------------------------
    // random forest regression
    //--------------------------------------------------------------------------------

    class RandomForestReg : public RandomForest<SP_REAL> {
    private:
        typedef SP_REAL TYPE;


    public:

        RandomForestReg() : RandomForest<SP_REAL>() {
        }

    private:

        virtual Node* getNode(MemP<Node> &tree, const Mem1<Mem<SP_REAL> > &Xs, const Mem1<TYPE> &Ys, const Mem1<int> &index) {
            SP_ASSERT(index.size() > 0);

            Node *node = tree.malloc();

            Mem1<TYPE> tmp(index.size());
            for (int i = 0; i < index.size(); i++) {
                tmp[i] = Ys[index[i]];
            }

            const SP_REAL mean = meanVal(tmp);
            const SP_REAL sigma = sqrt(meanSq(tmp - mean));

            node->val = mean;
            node->dev = sigma;

            return node;
        }


        virtual SP_REAL calcGain(const Mem1<Mem<SP_REAL> >& Xs, const Mem1<TYPE> &Ys, const Mem1<int> &index, const int param, const double thresh) {

            SP_REAL sum[2] = { 0 };
            int cnt[2] = { 0 };

            for (int n = 0; n < index.size(); n++) {
                const int s = Xs[index[n]][param] < thresh ? 0 : 1;
                sum[s] += Ys[index[n]];
                cnt[s]++;
            }

            if (cnt[0] * cnt[1] == 0) return -SP_INFINITY;

            SP_REAL mean[2];
            mean[0] = sum[0] / cnt[0];
            mean[1] = sum[1] / cnt[1];

            SP_REAL gain = 0.0;
            for (int n = 0; n < index.size(); n++) {
                const int s = Xs[index[n]][param] < thresh ? 0 : 1;
                gain -= sq(Ys[index[n]] - mean[s]);
            }
            return gain;
        }

    };


    //--------------------------------------------------------------------------------
    // random forest classification
    //--------------------------------------------------------------------------------

    class RandomForestCls : public RandomForest<int> {
    private:
        typedef int TYPE;
        int m_classNum;

    public:

        RandomForestCls(const int classNum) : RandomForest<int>() {
            m_classNum = classNum;
        }

    private:

        virtual Node* getNode(MemP<Node> &tree, const Mem1<Mem<SP_REAL> > &Xs, const Mem1<TYPE> &Ys, const Mem1<int> &index) {
            Node *node = tree.malloc();

            Mem1<int> hist(m_classNum);
            hist.zero();

            for (int n = 0; n < index.size(); n++) {
                hist[Ys[index[n]]]++;
            }

            node->val = maxArg(hist);
            node->dev = 1.0 - static_cast<SP_REAL>(maxVal(hist)) / index.size();

            return node;
        }

        virtual SP_REAL calcGain(const Mem1<Mem<SP_REAL> >& Xs, const Mem1<TYPE> &Ys, const Mem1<int> &index, const int param, const double thresh) {
            
            int cnt[2] = { 0 };
            Mem2<int> hist(2, m_classNum);
            hist.zero();
            
            for (int n = 0; n < index.size(); n++) {
                const int s = Xs[index[n]][param] < thresh ? 0 : 1;
                hist(s, Ys[index[n]])++;
                cnt[s]++;
            }

            if (cnt[0] * cnt[1] == 0) return -SP_INFINITY;
    
            double gain = 0.0;
            for (int s = 0; s < 2; s++) {
                double sum = 0.0;
                for (int c = 0; c < m_classNum; c++) {
                    const double p = hist(s, c) / cnt[s];
                    sum += (p != 0.0) ? p * log(p) : 0.0;
                }
                gain -= sum * cnt[s] / index.size();
            }
            return SP_CAST_REAL(gain);
        }


    };

}

#endif