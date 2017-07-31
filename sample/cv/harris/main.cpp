#include "simplesp.h"
#include "spex/spcv.h"

using namespace sp;

void sample(cv::Mat &cvimg);

int main(){
	cv::VideoCapture cap(0);

	if (cap.isOpened() == false){
		printf("could not find USB camera\n");
		exit(0);
	}

	printf("'ESC' key : exit\n");

	int key = 0;

	// 27 = 'ESC' key
	while ((key = cv::waitKey(1)) != 27){

		// capture
		cv::Mat cvimg;
		cap >> cvimg;

		sample(cvimg);

		cv::imshow("harris", cvimg);
	}

	cv::destroyAllWindows();
	return 0;
}

void sample(cv::Mat &cvimg){
	Mem2<Col3> img;

	// convert data type
	cvCnvImg(img, cvimg);

	Mem1<Vec2> pixs;
	harris(pixs, img);

	renderCircle(img, pixs, 4, getCol(0, 255, 0), 1);

	cvCnvImg(cvimg, img);
}

