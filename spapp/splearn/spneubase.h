//--------------------------------------------------------------------------------
// Copyright (c) 2017-2020, sanko-shoko. All rights reserved.
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

    SP_CPUFUNC SP_REAL crossEntropy(const Mem<SP_REAL>  &result, const int truth){

        SP_REAL loss = -log(result[truth]);
        return loss;
    }

    SP_CPUFUNC SP_REAL crossEntropy(const Mem1<Mem<SP_REAL> > &result, const Mem1<int> &truth){

        Mem1<SP_REAL> loss(result.size());
        for (int i = 0; i < result.size(); i++){
            loss[i] = crossEntropy(result[i], truth[i]);
        }
        return mean(loss);
    }


    //--------------------------------------------------------------------------------
    // accuracy
    //--------------------------------------------------------------------------------

    SP_CPUFUNC SP_REAL testAccuracy(const int result, const int label) {
        return (result == label) ? 1.0 : 0.0;
    }

    SP_CPUFUNC SP_REAL testAccuracy(const Mem<SP_REAL>  &result, const int label) {
        return (maxarg(result) == label) ? 1.0 : 0.0;
    }

    SP_CPUFUNC SP_REAL testAccuracy(const Mem<SP_REAL>  &result, const Mem<SP_REAL>  &label){
        return (maxarg(result) == maxarg(label)) ? 1.0 : 0.0;
    }

    template<typename TYPE0, typename TYPE1>
    SP_CPUFUNC SP_REAL testAccuracy(const Mem1<TYPE0> &result, const Mem1<TYPE1> &truth) {

        Mem1<SP_REAL> accuracy(result.size());
        for (int i = 0; i < result.size(); i++) {
            accuracy[i] = testAccuracy(result[i], truth[i]);
        }
        return  mean(accuracy);
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

        void rand(const SP_REAL scale = 0.05){
            for (int i = 0; i < w.size(); i++){
                w[i] = randg() * scale;
            }
            for (int i = 0; i < b.size(); i++){
                b[i] = randg() * scale;
            }
        }

        void zero(){
            w.zero();
            b.zero();
        }

    };

}
#endif
