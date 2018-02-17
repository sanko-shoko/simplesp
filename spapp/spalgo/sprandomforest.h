//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_RANDOMFOREST_H__
#define __SP_RANDOMFOREST_H__

#include "spcore/spcore.h"


namespace sp{

	//--------------------------------------------------------------------------------
	// random forest base class
	//--------------------------------------------------------------------------------

	template<typename TYPE>
	class RandomForest {

	protected:
		struct Node {
			// div parameter
			int param;

			// div thresh
			double thresh;
			
			// next node ptr (X[i] < thresh) ? next[0] : next[1]
			Node *next[2];

			// node value
			TYPE val;

			Node() {
				memset(this, 0, sizeof(Node));
			}
		};

		MemP<MemP<Node> > m_trees;

		int m_maxDepth;
		int m_sampleNum;

	public:

		RandomForest() {
			reset();
		}
		void reset() {
			m_trees.clear();
		}

		void train(const Mem1<Mem<double> >& Xs, Mem1<TYPE> &Ys, const int sampleNum = 100) {

			const int num = sampleNum > 0 ? minVal(sampleNum, Xs.size()) : Xs.size();
			Mem1<int> index(num);

			for (int n = 0; n < num; n++) {
				index[n] = sampleNum > 0 ? rand() % Xs.size() : n;
			}

			m_maxDepth = floor(log(Xs.size()) / log(2.0));

			divTree(m_trees.malloc(), Xs, Ys, index, 0);
		}

		Mem1<TYPE> estimate(const Mem<double> &X){

			SP_ASSERT(m_trees.size());

			Mem1<TYPE> results;
			for (int i = 0; i < m_trees.size(); i++){
				results.push(traceNode(&m_trees[i], X));
			}

			return results;
		}

	private:

		Node* divTree(MemP<Node> *tree, const Mem1<Mem<double> >& Xs, Mem1<TYPE> &Ys, const Mem1<int> &index, const int depth) {
			Node *node = tree->malloc();

			if (index.size() < 2 || depth >= m_maxDepth) {
				setNode(node, Ys, index);
				return node;
			}

			double maxg = -SP_INFINITY;

			for (int i = 0; i < Xs[0].size() * 10; i++) {

				const int param = rand() % Xs[0].size();

				double maxv = -SP_INFINITY;
				double minv = +SP_INFINITY;
				for (int n = 0; n < index.size(); n++) {
					const double v = Xs[index[n]][param];
					maxv = maxVal(maxv, v);
					minv = minVal(minv, v);
				}

				const double thresh = randValUnif() * (maxv - minv) + minv;

				const double gain = calcGain(Xs, Ys, index, param, thresh);

				if (gain > maxg) {
					maxg = gain;
					node->param = param;
					node->thresh = thresh;
				}
			}

			if (maxg == -SP_INFINITY) {
				setNode(node, Ys, index);
				return node;
			}

			Mem1<int> div[2];
			for (int n = 0; n < index.size(); n++) {
				const int s = Xs[index[n]][node->param] < node->thresh ? 0 : 1;
				div[s].push(index[n]);
			}

			node->next[0] = divTree(tree, Xs, Ys, div[0], depth + 1);
			node->next[1] = divTree(tree, Xs, Ys, div[1], depth + 1);

			return node;
		}

		TYPE traceNode(const MemP<Node> *tree, const Mem<double> &X) {

			const Node *node = &(*tree)[0];
			while (node != NULL && node->next[0] != NULL && node->next[1] != NULL) {

				node = node->next[X[node->param] < node->thresh ? 0 : 1];
			}

			return node->val;
		}

		virtual void setNode(Node *node, const Mem1<TYPE>& Y, const Mem1<int> &index) = 0;

		virtual double calcGain(const Mem1<Mem<double> >& Xs, Mem1<TYPE> &Ys, const Mem1<int> &index, const int param, const double thresh) = 0;

	};



	//--------------------------------------------------------------------------------
	// random forest regression
	//--------------------------------------------------------------------------------

	class RandomForestReg : public RandomForest<double> {
	private:
		typedef double TYPE;

	public:

		RandomForestReg() : RandomForest<double>() {
		}


	private:

		virtual double calcGain(const Mem1<Mem<double> >& Xs, Mem1<TYPE> &Ys, const Mem1<int> &index, const int param, const double thresh) {
			double sum[2] = { 0 };
			int cnt[2] = { 0 };

			for (int n = 0; n < index.size(); n++) {
				const int s = Xs[index[n]][param] < thresh ? 0 : 1;
				sum[s] += Ys[index[n]];
				cnt[s]++;
			}

			if (cnt[0] * cnt[1] == 0) return -SP_INFINITY;

			double mean[2];
			mean[0] = sum[0] / cnt[0];
			mean[1] = sum[1] / cnt[1];

			double gain = 0;
			for (int n = 0; n < index.size(); n++) {
				const int s = Xs[index[n]][param] < thresh ? 0 : 1;
				gain -= square(Ys[index[n]] - mean[s]);
			}
			return gain;
		}

		virtual void setNode(Node *node, const Mem1<TYPE>& Y, const Mem1<int> &index) {

			double sum = 0.0;
			for (int p = 0; p < index.size(); p++) {
				sum += Y[index[p]];
			}

			node->val = sum / index.size();
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

		virtual double calcGain(const Mem1<Mem<double> >& Xs, Mem1<TYPE> &Ys, const Mem1<int> &index, const int param, const double thresh) {
			
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
			return gain;
		}

		virtual void setNode(Node *node, const Mem1<TYPE>& Y, const Mem1<int> &index) {

			Mem1<int> hist(m_classNum);
			hist.zero();

			for (int n = 0; n < index.size(); n++) {
				hist[Y[index[n]]]++;
			}
			
			node->val = maxArg(hist);
		}

	};

}

#endif