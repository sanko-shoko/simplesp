#include <vector>
#include <string>
#include <fstream>
#include <opencv2/opencv.hpp>

#define SP_DATA_DIR "../../../../data"

using namespace std;
using namespace cv;

bool loadCSV(Mat &mat, const char *path, int type);

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
    {
        Mat mat0 = Mat::zeros(2, 2, CV_8UC1);
        mat0.at<unsigned char>(0, 0) = 100;
        mat0.at<unsigned char>(1, 1) = 100;

        Mat mat1;
        mat0.convertTo(mat1, CV_32FC1, 1.0 / 255.0);

        cout << mat0 << endl;
        cout << mat1 << endl;
    }
    {
        Mat mat0 = Mat::eye(3, 3, CV_32FC1);

        ofstream ofs("mat.csv");
        ofs << format(mat0, Formatter::FMT_CSV) << endl;

        Mat mat1;
        loadCSV(mat1, "mat.csv", CV_32FC1);
        cout << mat1 << endl;
    }
    return 0;
}

bool loadCSV(Mat &mat, const char *path, int type) {
    ifstream ifs(path);
    if (ifs.fail()) return false;

    vector<vector<double> > data;
    
    string str;
    while (getline(ifs, str)){
        vector<double> tmp;
    
        stringstream ss{ str };
        string buf;
        while (getline(ss, buf, ',')) {
            double val = 0.0;
            sscanf(buf.c_str(), "%lf", &val);

            tmp.push_back(val);
        }

        if (tmp.size() > 0) {
            data.push_back(tmp);
        }
    }
    if (data.size() == 0) return false;

    Mat mat64 = Mat::zeros(data.size(), data[0].size(), CV_64FC1);
    for (int r = 0; r < data.size(); r++) {
        for (int c = 0; c < data[r].size(); c++) {
            mat64.at<double>(r, c) = data[r][c];
        }
    }

    mat64.convertTo(mat, type);

    return true;
}
