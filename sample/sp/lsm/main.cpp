#include "simplesp.h"

using namespace sp;

int main(){

    //--------------------------------------------------------------------------------
    // least squares method
    //--------------------------------------------------------------------------------

    printf("--------------------------------------------------------------------------------\n");
    printf("no outliers\n");
    printf("--------------------------------------------------------------------------------\n");

    // define parameter (y = a * x + b)
    const double a = 1.2;
    const double b = 3.4;

    // generate test data
    Mem1<Vec2> data;

    for (int i = 0; i < 100; i++){
        const double noise = randValGauss() * 0.1;

        const double x = randValUnif() * 10;
        const double y = a * x + b + noise;

        data.push(getVec(x, y));
    }

    // lsm
    {
        printf("basic lsm\n");

        Mat A(data.size(), 2);
        Mat B(data.size(), 1);

        for (int i = 0; i < data.size(); i++) {
            A(i, 0) = data[i].x;
            A(i, 1) = 1.0;
            B(i, 0) = data[i].y;
        }

        // At * A * X = At * B
        const Mat At = trnMat(A);
        const Mat X = invMat(At * A) * At * B;
        print(X);
    }


    //--------------------------------------------------------------------------------
    // robust estimation
    //--------------------------------------------------------------------------------

    printf("\n\n");
    printf("--------------------------------------------------------------------------------\n");
    printf("add outliers\n");
    printf("--------------------------------------------------------------------------------\n");

    // add outlier
    for (int i = 0; i < 100; i++) {
        const double x = randValUnif() * 10;
        const double y = randValUnif() * 10;

        data.push(getVec(x, y));
    }

    // lsm
    {
        printf("basic lsm\n");
        Mat A(data.size(), 2);
        Mat B(data.size(), 1);

        for (int i = 0; i < data.size(); i++) {
            A(i, 0) = data[i].x;
            A(i, 1) = 1.0;
            B(i, 0) = data[i].y;
        }

        // At * A * X = At * B
        const Mat At = trnMat(A);
        const Mat X = invMat(At * A) * At * B;
        print(X);
    }


    Mat ransacX;

    // ransac
    {
        printf("ransac\n");

        int ransacMax = 100;
        const double thresh = 1.0;

        int maxv = 0;
        for (int r = 0; r < ransacMax; r++) {
            Mem1<Vec2> sample;
            for (int i = 0; i < 2; i++) {
                const int v = sp::rand() % data.size();
                sample.push(data[v]);
            }

            Mat A(sample.size(), 2);
            Mat B(sample.size(), 1);
            for (int i = 0; i < sample.size(); i++) {
                A(i, 0) = sample[i].x;
                A(i, 1) = 1.0;
                B(i, 0) = sample[i].y;
            }

            // At * A * X = At * B
            const Mat At = trnMat(A);
            const Mat X = invMat(At * A) * At * B;
            if (X.size() == 0) continue;


            // eval
            int eval = 0;
            for (int i = 0; i < data.size(); i++) {
                // err = |y - (a * x + b)|
                const double err = ::fabs(data[i].y - (X[0] * data[i].x + X[1]));
                if (err < thresh) {
                    eval++;
                }
            }
            if (eval > maxv) {
                maxv = eval;
                ransacX = X;
            }
        }
        print(ransacX);
    }
    

    // m-estimation (using ransacX)
    {
        printf("m-estimation\n");

        Mat A(data.size(), 2);
        Mat B(data.size(), 1);

        for (int i = 0; i < data.size(); i++) {
            A(i, 0) = data[i].x;
            A(i, 1) = 1.0;
            B(i, 0) = data[i].y;
        }

        Mat W = zeroMat(data.size(), data.size());
        {
            Mem1<double> errs;
            for (int i = 0; i < data.size(); i++) {
                // err = |y - (a * x + b)|
                const double err = ::fabs(data[i].y - (ransacX[0] * data[i].x + ransacX[1]));
                errs.push(err);
            }

            const double median = medianVal(errs);
            for (int i = 0; i < data.size(); i++) {
                const double w = funcTukey(errs[i], 3.0 * median);
                W(i, i) = w;
            }
        }

        // AtW * A * X = AtW * B
        const Mat AtW = trnMat(W * A);
        const Mat X = invMat(AtW * A) * AtW * B;
        print(X);
    }
    return 0;
}