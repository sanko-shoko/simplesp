#define SP_USE_DEBUG 1

#include "simplesp.h"

using namespace sp;

int main(){

    //--------------------------------------------------------------------------------
    // kd tree
    //--------------------------------------------------------------------------------

    const int dataNum = 10000;

    // search target data
    Mem1<Vec2> targ;
    for (int i = 0; i < dataNum; i++) {
        targ.push(randVecUnif(100.0, 100.0));
    }

    // test data
    Mem1<Vec2> test;
    for (int i = 0; i < 1000; i++) {
        test.push(randVecUnif(100.0, 100.0));
    }


    // kd tree
    {
        Mem1<Vec2> result;
        result.reserve(test.size());
        
        {
            SP_LOGGER_SET("kd tree");

            // create instance
            KdTree<double> kdtree(2);

            // set data
            for (int i = 0; i < targ.size(); i++){
                kdtree.addData(&targ[i]);
            }

            for (int i = 0; i < test.size(); i++){
                const int crsp = kdtree.search(&test[i]);
                result.push(targ[crsp]);
            }
        }

        SP_LOGGER_PRINT("kd tree");
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
        Mem1<Vec2> result;
        result.reserve(test.size());

        {
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

        SP_LOGGER_PRINT("brute force search");
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