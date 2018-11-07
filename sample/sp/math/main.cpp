#include "simplesp.h"

using namespace sp;

int main(){
    {
        printf("\n\n");
        printf("--------------------------------------------------------------------------------\n");
        printf("matrix test \n");
        printf("--------------------------------------------------------------------------------\n");

        Mat M;
        {
            printf("\n\n");
            printf("set random value\n");

            M.resize(5, 3);
            ::srand(0);
            for (int r = 0; r < M.rows(); r++) {
                for (int c = 0; c < M.cols(); c++) {
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
    }

    {
        printf("\n\n");
        printf("--------------------------------------------------------------------------------\n");
        printf("equation test \n");
        printf("--------------------------------------------------------------------------------\n");
        
        Mem1<double> c;

        c.push(1.0);
        c.push(2.0);
        c.push(3.0);
        c.push(4.0);
        c.push(5.0);

        Cmp xs[100];
        {
            printf("\n\n");
            printf("calc eq4\n");

            const int num = eq4(xs, c[0], c[1], c[2], c[3], c[4]);

            for (int i = 0; i < num; i++) {
                print(xs[i]);
            }
        }

        {
            printf("\n\n");
            printf("calc eqn\n");

            const int num = eqn(xs, c.size(), c.ptr);

            for (int i = 0; i < num; i++) {
                print(xs[i]);
            }
        }
    }
    return 0;
}