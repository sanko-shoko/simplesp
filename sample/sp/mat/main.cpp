#include "simplesp.h"

using namespace sp;

int main(){
    // matrix test

    Mat M;
    {
        printf("\n\n");
        printf("set identity matrix\n");

        M = eyeMat(3, 3);
        print(M);
    }

    {
        printf("\n\n");
        printf("set random value\n");

        M.resize(5, 3);
        ::srand(0);
        for (int r = 0; r < M.rows(); r++){
            for (int c = 0; c < M.cols(); c++){
                M(r, c) = ::rand() % 21 - 10;
            }
        }
        print(M);
    }

    {
        printf("\n\n");
        printf("calc eigen\n");

        Mat eigVec, eigVal;
        eigMat(eigVec, eigVal, covMat(M));

        print(eigVec);
        print(eigVal);
    }

    {
        printf("\n\n");
        printf("calc svd\n");

        Mat U, V, S;
        svdMat(U, S, V, M);

        print(U);
        print(S);
        print(V);

        print(U * S * trnMat(V));
    }
    return 0;
}