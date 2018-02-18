#include <opencv2/opencv.hpp>

#define SP_DATA_DIR "../../../../data"

using namespace cv;

int main(){

    {
        Mat img = imread(SP_DATA_DIR "/image/Lenna.bmp");
        // 画像の表示
        imshow("img", img);
        waitKey(0);

        imwrite("test.bmp", img);
    }


    {
        Mat img = imread(SP_DATA_DIR "/image/Lenna.bmp", 0);

        cv::GaussianBlur(img, img, Size(11, 11), 3.0);

        imshow("img", img);
        waitKey(0);
    }

    {
        Mat img(240, 320, CV_8UC3);

        for (int v = 0; v < img.size().height; v++) {
            for (int u = 0; u < img.size().width; u++) {
                img.at<Vec3b>(v, u) = Vec3b(100, 0, 0);
            }
        }

        imshow("img", img);
        waitKey(0);

        Vec3b *ptr = (Vec3b*)img.ptr();
        const int step = img.step / sizeof(Vec3b);

        for (int v = 0; v < img.size().height; v++) {
            for (int u = 0; u < img.size().width; u++) {
                ptr[v * step + u] = Vec3b(0, 100, 0);
            }
        }

        imshow("img", img);
        waitKey(0);
    }

    {
        Mat mat0 = Mat::zeros(3, 3, CV_32FC1);
        Mat mat1 = Mat::ones(3, 3, CV_32FC1);
        Mat mat2 = Mat::eye(3, 3, CV_32FC1);

        std::cout << mat0 << std::endl;
        std::cout << mat1 << std::endl;
        std::cout << mat2 << std::endl;

        Mat mat3 = mat1 * (mat2);
        std::cout << mat3 << std::endl;

    }
    return 0;
}

