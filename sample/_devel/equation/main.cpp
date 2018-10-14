#include "simplesp.h"
using namespace sp;

int main(){

    Mem1<double> c;

    c.push(1.0);
    c.push(2.0);
    c.push(3.0);
    c.push(4.0);
    c.push(5.0);

    Cmp xs[100];
    {
        const int num = eq4(xs, c[0], c[1], c[2], c[3], c[4]);

        for (int i = 0; i < num; i++) {
            print(xs[i]);
        }
    }
    printf("\n");

    {
        const int num = eqn(xs, c.size(), c.ptr);

        for (int i = 0; i < num; i++) {
            print(xs[i]);
        }
    }


    return 0;
}
