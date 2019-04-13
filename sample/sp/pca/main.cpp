#include "simplesp.h"

using namespace sp;


int main() {

    {
        //--------------------------------------------------------------------------------
        // principal component analysis (2D)
        //--------------------------------------------------------------------------------

        Mem1<Vec2> data;

        // generate data
        {
            const double angle = 30.0 * SP_PI / 180.0;

            Mat mat(2, 2);
            mat(0, 0) = +::cos(angle);
            mat(0, 1) = -::sin(angle);

            mat(1, 0) = +::sin(angle);
            mat(1, 1) = +::cos(angle);

            for (int i = 0; i < 200; i++) {
                const Vec2 vec = mat * randVecGauss(20.0, 5.0) + getVec2(50.0, 50.0);
                data.push(vec);
            }
        }
    
        {
            const Vec2 mean = meanVec(data);

            Mat mat(data.size(), 2);
            for (int i = 0; i < data.size(); i++) {
                mat(i, 0) = data[i].x - mean.x;
                mat(i, 1) = data[i].y - mean.y;
            }

            Mat eigVec, eigVal;
            eigMat(eigVec, eigVal, covMat(mat) / data.size(), false);


            printf("eigVec\n");
            print(eigVec);

            printf("eigVal\n");
            print(eigVal);

            printf("first principal component\n");
            printf("%+.3lf, %+.3lf\n", eigVec(0, 0), eigVec(1, 0));
            printf("second principal component\n");
            printf("%+.3lf, %+.3lf\n", eigVec(0, 1), eigVec(1, 1));

            printf("\n\n");
        }
    }

    {
        //--------------------------------------------------------------------------------
        // principal component analysis (Image)
        //--------------------------------------------------------------------------------

        Mem1<Mem<double> > imgs;
        {
            Mem1<Mem<double> > trainImages, testImages;
            Mem1<int> trainLabels, testLabels;

            SP_ASSERT(loadMNIST(trainImages, trainLabels, testImages, testLabels, SP_DATA_DIR "/mnist"));

            for (int i = 0; i < testImages.size(); i++) {
                if (testLabels[i] == 1) {
                    imgs.push(testImages[i]);
                }
            }
        }

        Mem<double> mean = imgs[0];
        for (int i = 1; i < imgs.size(); i++) {
            addMem(mean, mean, imgs[i]);
        }
        divElm(mean, mean, imgs.size());
        
        Mat mat(imgs.size(), mean.size());
        for (int i = 0; i < imgs.size(); i++) {
            for (int j = 0; j < mean.size(); j++) {
                mat(i, j) = imgs[i][j] - mean[j];
            }
        }

        Mat eigVec, eigVal;
        eigMat(eigVec, eigVal, covMat(mat) / imgs.size(), false);

        // sample
        {
            for(int i = 0; i < 50; i++){
                Mem2<Byte> dst(mean.dsize);
                for (int j = 0; j < dst.size(); j++) {
                    dst[j] = static_cast<Byte>(imgs[i][j] * 255 + 0.5);
                }
        
                char path[256];
                sprintf(path, "sample%02d.bmp", i);
                saveBMP(path, dst);
            }
        }

        // mean
        {
            Mem2<Byte> dst(mean.dsize);
            {
                for (int i = 0; i < dst.size(); i++) {
                    dst[i] = static_cast<Byte>(mean[i] * 255 + 0.5);
                }
            }
            saveBMP("mean.bmp", dst);
        }

        // principal component
        {
            for (int i = 0; i < 10; i++) {
                Mem2<Byte> dst(mean.dsize);
                {
                    Mem2<double> tmp(mean.dsize);

                    for (int j = 0; j < dst.size(); j++) {
                        tmp[j] = eigVec(j, i);
                    }
                    const double maxv = maxVal(tmp);
                    const double minv = minVal(tmp);
                    cnvMem(dst, tmp, 255 / (maxv - minv), minv);
                }

                char path[256];
                sprintf(path, "principal%02d.bmp", i);
                saveBMP(path, dst);
            }
        }

    }
    return 0;
}
