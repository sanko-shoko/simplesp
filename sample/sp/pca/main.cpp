#include "simplesp.h"

using namespace sp;


int main() {

	{
		//--------------------------------------------------------------------------------
		// principal component analysis (2D)
		//--------------------------------------------------------------------------------

		Graph2D graph(100, 0);

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
				const Vec2 vec = mat * randVecGauss(20.0, 5.0) + getVec(50.0, 50.0);
				data.push(vec);
			}

			graph.renderPoint(data, getCol(0, 200, 0), 3);
			graph.saveBMP("graphA.bmp");
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

			print(eigVec);
			print(eigVal);

			// first principal component
			{
				const double val = ::sqrt(eigVal(0, 0));
				const Vec2 vec = getVec(eigVec(0, 0), eigVec(1, 0));
				graph.renderLine(mean - vec * val * 2, mean + vec * val * 2, getCol(200, 0, 0), 2);
			}

			// second principal component
			{
				const double val = ::sqrt(eigVal(1, 1));
				const Vec2 vec = getVec(eigVec(0, 1), eigVec(1, 1));
				graph.renderLine(mean - vec * val * 2, mean + vec * val * 2, getCol(0, 0, 200), 2);
			}
			
			graph.renderPoint(mean, getCol(0, 0, 0), 5);

			graph.saveBMP("graphB.bmp");
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
				saveBMP(dst, path);
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
			saveBMP(dst, "mean.bmp");
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
				saveBMP(dst, path);
			}
		}

	}
	return 0;
}
