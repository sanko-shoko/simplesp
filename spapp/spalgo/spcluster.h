//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_CLUSTER_H__
#define __SP_CLUSTER_H__

#include "spcore/spcore.h"

namespace sp{

    //--------------------------------------------------------------------------------
    // k-means clustering
    //--------------------------------------------------------------------------------

    SP_CPUFUNC Mem1<int> kmeans(const int dim, const void *ptr, const int size, const int K, const int maxit = 5){
        SP_ASSERT(dim > 0);

        const Mat data(size, dim, ptr);

        Mem1<int> index(size);
        setElm(index, -1);

        Mat cent(K, dim);
        {
            Mem1<int> tmp = shuffle(size);
            for (int k = 0; k < K; k++){
                setMem(&cent(k, 0), dim, &data(tmp[k], 0));
            }
        }

        Mem1<int> cnts(K);

        // iteration
        for (int it = 0; it < maxit; it++){

            // assign label
            for (int i = 0; i < size; i++){
                SP_REAL dist = SP_INFINITY;

                int &label = index[i];
                for (int k = 0; k < K; k++){
                    const SP_REAL d = normMat(&cent(k, 0), 1, dim, &data(i, 0));
                    if (d < dist){
                        dist = d;
                        label = k;
                    }
                }
            }

            if (it == maxit - 1) break;

            // mean
            {
                cent.zero();
                cnts.zero();

                for (int i = 0; i < size; i++){
                    const int label = index[i];

                    addMem(&cent(label, 0), dim, &cent(label, 0), &data(i, 0));
                    cnts[label]++;
                }

                for (int k = 0; k < K; k++){
                    divElm(&cent(k, 0), dim, &cent(k, 0), cnts[k]);
                }

            }
        }

        return index;
    };


}
#endif