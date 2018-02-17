//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_NEUMODEL_H__
#define __SP_NEUMODEL_H__

#include "spcore/spcore.h"
#include "spapp/splearn/spneubase.h"
#include "spapp/splearn/spneulayer.h"

namespace sp{

	//--------------------------------------------------------------------------------
	// network model
	//--------------------------------------------------------------------------------

	class NetworkModel{

	private:
		Mem1<BaseLayer*> m_order;

	public:

		NetworkModel(){
		}

		~NetworkModel(){
			reset();
		}

		void reset(){
			for (int i = 0; i < m_order.size(); i++){
				delete m_order[i];
			}
			m_order.clear();
		}

		void addLayer(BaseLayer *layer){
			m_order.push(layer);
		}

		const Mem1<Mem<double> >& getResult(){
			return m_order[m_order.size() - 1]->getResult(true);
		}


		//--------------------------------------------------------------------------------
		// train
		//--------------------------------------------------------------------------------

		void train(const Mem1<Mem<double> > &src, const Mem1<Mem<double> > &truth){

			forward(src, true);

			backward(truth);
		}

		void train(const Mem1<Mem<double> > &src, const Mem1<int> &truth){
			forward(src, true);
			const int labelNum = getResult()[0].size();

			Mem1<Mem<double> > oh(truth.size());
			for (int i = 0; i < oh.size(); i++){
				oh[i] = oneHot<double>(truth[i], labelNum);
			}

			backward(oh);
		}


		//--------------------------------------------------------------------------------
		// forward
		//--------------------------------------------------------------------------------

		void forward(const Mem1<Mem<double> > &mem, bool train = false){
			const Mem1<Mem<double> > *prop = &mem;
			for (int i = 0; i < m_order.size(); i++){
				m_order[i]->m_train = train;
				prop = m_order[i]->execute(prop, true);
			}
		}

		void forward(const Mem<double>  &mem, bool train = false){
			forward(Mem1<Mem<double> >(1, &mem), train);
		}

	private:

		//--------------------------------------------------------------------------------
		// backward
		//--------------------------------------------------------------------------------

		void backward(const Mem1<Mem<double> > &mem){

			const Mem1<Mem<double> > *prop = &mem;
			for (int i = m_order.size() - 1; i >= 0; i--){
				prop = m_order[i]->execute(prop, false);
			}
		}

		void backward(const Mem<double>  &mem, bool train = false){
			backward(Mem1<Mem<double> >(1, &mem));
		}


	public:

		//--------------------------------------------------------------------------------
		// save / load
		//--------------------------------------------------------------------------------

		void save(const char *path){

			File file(path, "w");
			for (int i = 0; i < m_order.size(); i++){
				file.textf(m_order[i]->getName());
				file.textf(",\n");
				file.textex(m_order[i], 1);
			}
		}

#define _SP_IF_LAYER(LAYER, NAME, OBJECT) if (strstr(NAME, #OBJECT)){ LAYER = new OBJECT(); }

		void load(const char *path){

			reset();

			File file(path, "r");
			char name[SP_STRMAX];
			while (file.gets(name)){

				BaseLayer *layer = NULL;
				_SP_IF_LAYER(layer, name, AffineLayer);
				_SP_IF_LAYER(layer, name, SoftMaxLayer);
				_SP_IF_LAYER(layer, name, ActivationLayer);

				_SP_IF_LAYER(layer, name, ConvolutionLayer);
				_SP_IF_LAYER(layer, name, BatchNormLayer);
				_SP_IF_LAYER(layer, name, MaxPoolingLayer);
				_SP_IF_LAYER(layer, name, DropOutLayer);

				if (layer == NULL) break;
				file.textf(layer->getName());
				file.textf(",\n");
				file.textex(layer, 1);
				addLayer(layer);
			}
		}

	};

}
#endif
