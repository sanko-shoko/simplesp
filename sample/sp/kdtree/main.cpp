#define SP_USE_DEBUG 1

#include "simplesp.h"

using namespace sp;

int main(){

    //--------------------------------------------------------------------------------
    // kd tree
    //--------------------------------------------------------------------------------

    const int dim = 2;
    typedef Vec2 VEC;

    const int dataNum = 1000;

    // search target data
    Mem1<VEC> targ;
    for (int i = 0; i < dataNum; i++) {
        targ.push(randVecUnif(100.0, 100.0));
    }

    // test data
    Mem1<VEC> test;
    for (int i = 0; i < 1000; i++) {
        test.push(randVecUnif(100.0, 100.0));
    }


    // kd tree
    {
        Mem1<VEC> result;
        result.reserve(test.size());
        
        {
            SP_LOGGER_INSTANCE;
            SP_LOGGER_SET("kd tree");
        
            // create instance
            KdTree<double> kdtree(dim);

            // set data
            for (int i = 0; i < targ.size(); i++){
                kdtree.addData(&targ[i]);
            }

            for (int i = 0; i < test.size(); i++){
                const int crsp = kdtree.search(&test[i]);
                result.push(targ[crsp]);
            }
        }

        printf("kd tree\n");
        {
            int step = (test.size() / 10);
            for (int i = 0; i < test.size(); i += step){
                print(result[i]);
            }
        }
        printf("\n");
    }

    // brute force search
    {
        Mem1<VEC> result;
        result.reserve(dim * test.size());

        {
            SP_LOGGER_INSTANCE;
            SP_LOGGER_SET("brute force search");

            for (int i = 0; i < test.size(); i++){
                int crsp = -1;

                double minv = SP_INFINITY;
                for (int j = 0; j < targ.size(); j++){
                    const double sq = sqVec(test[i] - targ[j]);
                    if (sq < minv){
                        minv = sq;
                        crsp = j;
                    }
                }

                result.push(targ[crsp]);
            }
        }

        printf("brute force search\n");
        {
            int step = (test.size() / 10);
            for (int i = 0; i < test.size(); i += step){
                print(result[i]);
            }
        }
        printf("\n");
    }


    return 0;
}