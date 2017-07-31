#include "simplesp.h"

using namespace sp;

void mnist(const char *path);
void cifar(const char *path);

int main(){

	mnist(SP_DATA_DIR "/mnist");
	//cifar(SP_DATA_DIR "/cifar");
	return 0;
}

void mnist(const char *path) {

	// Mem1 image num  : train 60k, test 10k 
	// Mem<double>  image size : 28x28
	Mem1<Mem<double> > trainImages, testImages;
	Mem1<int> trainLabels, testLabels;

	SP_ASSERT(loadMNIST(trainImages, trainLabels, testImages, testLabels, path));

	NetworkModel model;
	{
		model.addLayer(new AffineLayer(50));
		model.addLayer(new ActivationLayer);

		model.addLayer(new AffineLayer(50));
		model.addLayer(new ActivationLayer);

		model.addLayer(new AffineLayer(10));
		model.addLayer(new SoftMaxLayer);
	}

	const int epoch = 10;
	const int batch = 100;

	for (int e = 0; e < epoch; e++) {
		const Mem1<Mem<double> > images = shuffle(trainImages, e);
		const Mem1<int> labels = shuffle(trainLabels, e);

		// train
		for (int i = 0; i < trainImages.size(); i += batch) {
			const Mem1<Mem<double> > X = images.slice(0, i, i + batch);
			const Mem1<int> T = labels.slice(0, i, i + batch);

			model.train(X, T);

			printf("\r%02d train [%s] ", e, progressBar((i + batch), trainImages.size()));
			fflush(stdout);
		}

		// test
		{
			model.forward(testImages);
			printf("test accuracy %.3lf\n", testAccuracy(model.getResult(), testLabels));
		}
	}

}

void cifar(const char *path) {

	// Mem1 image num  : train 50k, test 10k 
	// Mem<double>  image size : 32x32x3
	Mem1<Mem<double> > trainImages, testImages;
	Mem1<int> trainLabels, testLabels;

	SP_ASSERT(loadCIFAR10(trainImages, trainLabels, testImages, testLabels, path));

	NetworkModel model;
	{
		model.addLayer(new ConvolutionLayer(10, 7, 2));
		model.addLayer(new BatchNormLayer(10));
		model.addLayer(new ActivationLayer);
		model.addLayer(new MaxPoolingLayer(3, 2));

		model.addLayer(new ConvolutionLayer(20, 5, 1));
		model.addLayer(new BatchNormLayer(20));
		model.addLayer(new ActivationLayer);
		model.addLayer(new MaxPoolingLayer(3, 2));

		model.addLayer(new AffineLayer(500));
		model.addLayer(new BatchNormLayer(500));
		model.addLayer(new ActivationLayer);

		model.addLayer(new AffineLayer(10));
		model.addLayer(new SoftMaxLayer);
	}


	const int epoch = 30;
	const int batch = 100;

	for (int e = 0; e < epoch; e++) {
		const Mem1<Mem<double> > images = shuffle(trainImages, e);
		const Mem1<int> labels = shuffle(trainLabels, e);

		// train
		for (int i = 0; i < trainImages.size(); i += batch) {
			const Mem1<Mem<double> > X = images.slice(0, i, i + batch);
			const Mem1<int> T = labels.slice(0, i, i + batch);

			model.train(X, T);

			printf("\r%02d train [%s] ", e, progressBar((i + batch), trainImages.size()));
			fflush(stdout);
		}

		// test
		{
			model.forward(testImages);
			printf("test accuracy %.3lf\n", testAccuracy(model.getResult(), testLabels));
		}
	}

}
