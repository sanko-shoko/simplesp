//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_NEUBASE_H__
#define __SP_NEUBASE_H__

#include "spcore/spcore.h"

namespace sp{

    //--------------------------------------------------------------------------------
    // one hot
    //--------------------------------------------------------------------------------

    template <typename TYPE>
    SP_CPUFUNC Mem1<TYPE> oneHot(const int label, const int num){
        Mem1<TYPE> dst(num);
        dst.zero();
        dst[label] = static_cast<TYPE>(1);

        return dst;
    }


    //--------------------------------------------------------------------------------
    // cross entropy
    //--------------------------------------------------------------------------------

    SP_CPUFUNC double crossEntropy(const Mem<double>  &result, const int truth){

        double loss = -log(result[truth]);
        return loss;
    }

    SP_CPUFUNC double crossEntropy(const Mem1<Mem<double> > &result, const Mem1<int> &truth){

        Mem1<double> loss(result.size());
        for (int i = 0; i < result.size(); i++){
            loss[i] = crossEntropy(result[i], truth[i]);
        }
        return meanVal(loss);
    }


    //--------------------------------------------------------------------------------
    // accuracy
    //--------------------------------------------------------------------------------

    SP_CPUFUNC double testAccuracy(const int result, const int label) {
        return (result == label) ? 1.0 : 0.0;
    }

    SP_CPUFUNC double testAccuracy(const Mem<double>  &result, const int label) {
        return (maxArg(result) == label) ? 1.0 : 0.0;
    }

    SP_CPUFUNC double testAccuracy(const Mem<double>  &result, const Mem<double>  &label){
        return (maxArg(result) == maxArg(label)) ? 1.0 : 0.0;
    }

    template<typename TYPE0, typename TYPE1>
    SP_CPUFUNC double testAccuracy(const Mem1<TYPE0> &result, const Mem1<TYPE1> &truth) {

        Mem1<double> accuracy(result.size());
        for (int i = 0; i < result.size(); i++) {
            accuracy[i] = testAccuracy(result[i], truth[i]);
        }
        return  meanVal(accuracy);
    }


    //--------------------------------------------------------------------------------
    // node parameter
    //--------------------------------------------------------------------------------

    class NodeParam{
    public:

        // weight
        Mat w;
        
        // bias
        Mat b;

    public:

        void resize(const int nodeNum, const int weightNum){
            w.resize(nodeNum, weightNum);
            b.resize(nodeNum, 1);
        }

        void rand(const double scale = 0.05){
            for (int i = 0; i < w.size(); i++){
                w[i] = randValGauss() * scale;
            }
            for (int i = 0; i < b.size(); i++){
                b[i] = randValGauss() * scale;
            }
        }

        void zero(){
            w.zero();
            b.zero();
        }

    };

}
#endif
