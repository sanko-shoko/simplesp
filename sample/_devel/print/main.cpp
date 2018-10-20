#include "simplesp.h"

using namespace sp;

int main(){

    print(getVec(1.0, 2.0));
    
    print(getVec(1.0, 2.0, 3.0));

    print(getRect2(0, 0, 640, 480));

    print(getVecPN(getVec(1.0, 2.0), getVec(3.0, 4.0)));

    print(getMesh(getVec(1.0, 2.0, 3.0), getVec(4.0, 5.0, 6.0), getVec(7.0, 8.0, 9.0)));

    Mem1<Vec2> vecs;
    for (int i = 0; i < 10; i++) {
        vecs.push(randVecUnif(1.0, 1.0));
    }
    print(vecs);

    return 0;
}