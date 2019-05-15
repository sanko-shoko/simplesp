#include "simplesp.h"

using namespace sp;

void regression();
void classification(const char *path);

int main() {

    regression();

    classification(SP_DATA_DIR "/mnist");

    return 0;
}

void regression(){

    printf("\n\n");
    printf("--------------------------------------------------------------------------------\n");
    printf("regression\n");
    printf("--------------------------------------------------------------------------------\n");

    // define parameter (y = a * x0 + b * x1 + c)
    const double a = 1.2;
    const double b = 3.4;
    const double c = 5.6;

    // generate test data
    Mem1<Mem<SP_REAL> > Xs;
    Mem1<SP_REAL> Ys;

    for (int i = 0; i < 1500; i++) {
        const double noise = randg() * 0.1;

        Mem1<SP_REAL> X(2);
        X[0] = randu() * 10;
        X[1] = randu() * 10;

        const double Y = a * X[0] + b * X[1] + c + noise;

        Xs.push(X);
        Ys.push(Y);
    }

    RandomForestReg rf;

    const int treeNum = 20;
    for (int i = 0; i < treeNum; i++) {
        rf.train(Xs, Ys);
    }
    
    const int testNum = 5;
    for (int i = 0; i < testNum; i++) {
        Mem1<SP_REAL> X(2);
        X[0] = randu() * 10;
        X[1] = randu() * 10;

        const double Y = a * X[0] + b * X[1] + c;
        const double R = meanVal(rf.execute(X));

        printf("ground truth %.4lf\n", Y);
        printf("regression   %.4lf\n", R);
        printf("\n");
    }
}

void classification(const char *path) {

    printf("\n\n");
    printf("--------------------------------------------------------------------------------\n");
    printf("classification\n");
    printf("--------------------------------------------------------------------------------\n");

    // Mem1 image num  : train 60k, test 10k 
    // Mem<double>  image size : 28x28
    Mem1<Mem<SP_REAL> > trainImages, testImages;
    Mem1<int> trainLabels, testLabels;

    SP_ASSERT(loadMNIST(trainImages, trainLabels, testImages, testLabels, path));

    RandomForestCls rf(10);

    const int treeNum = 80;
    for (int i = 0; i < treeNum; i++) {
        rf.train(trainImages, trainLabels);
        printf("\rtrain [%s] ", progress(i, treeNum));
    }
    printf("\n");

    Mem1<int> results;
    for (int i = 0; i < testImages.size(); i++) {
        Mem<SP_REAL> &X = testImages[i];

        Mem1<int> hist;
        histogram(hist, rf.execute(X), 10);
        const int C = maxArg(hist);

        results.push(C);
    }

    const int testNum = 5;
    for (int i = 0; i < testNum; i++) {
        const int id = sp::rand() % testImages.size();

        const int Y = testLabels[id];
        const int C = results[id];

        printf("ground truth   %d\n", Y);
        printf("classification %d\n", C);
        printf("\n");
    }

    printf("test accuracy %.3lf\n", testAccuracy(results, testLabels));
}



