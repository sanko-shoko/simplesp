#include "simplesp.h"
#include <vector>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

int main() {
    const Size bsize(10, 7);
    const double bdistance = 33.875;

    vector<Point3f> grid;
    for (int y = 0; y < bsize.height; y++) {
        for (int x = 0; x < bsize.width; x++) {
            grid.push_back(Point3f(x - (bsize.width - 1) / 2.0, y - (bsize.height - 1) / 2.0, 0.0f) * bdistance);
        }
    }

    int key = 0;
    Mat img;
    bool flag = true;

    vector<vector<Point2f> > pixs;
    vector<vector<Point3f> > objs;

    // 27 = 'ESC' key
    Mat cam, dist;

    cam = (cv::Mat_<double>(3, 3) << 455.21719, 0.0, 645.74530, 0.0, 455.15898, 501.04069, 0.0, 0.0, 1.0);
    dist = (cv::Mat_<double>(1, 4) << -0.0411811, -0.00832799, 0.00488491, -0.00250858);

    char tmp[256];
    sp::makeDir(sp::timestamp(tmp));
    while ((key = waitKey(1)) != 27) {

        if (flag == true || key == 'a' || img.size().area() == 0) {
            static int c = 0;
            char path[256];
            //sprintf(path, "C:\\Users\\sanko-shoko\\Desktop\\capdata\\20190715_215028\\%04d.png", c++);
            //sprintf(path, "C:\\Users\\sanko-shoko\\Desktop\\capdata\\20190716_180947\\%04d.png", c++);
            sprintf(path, "C:\\Users\\sanko-shoko\\Desktop\\capdata\\20190715_155140\\%04d.png", c++);
            {
                sp::File file(path, "rb");
                if (file.fp() == NULL) {
                    flag = false;
                    break;
                    continue;
                }
            }

            img = cv::imread(path, 1);

            //vector<Point2f> pointbuf;
            //bool found = false;
            //found = findChessboardCorners(img, bsize, pointbuf);
            //if (found == true) {
            //    found = cv::find4QuadCornerSubpix(img, pointbuf, cv::Size(3, 3));
            //}
            //printf("found %d\n", found);
            //if (found == true) {
            //    drawChessboardCorners(img, bsize, pointbuf, found);

            //    static int cnt = 0;
            //    printf("add detected points (i = %d)\n", cnt++);
            //    //if (cnt % 3 == 1) 
            //    {
            //        pixs.push_back(pointbuf);
            //        objs.push_back(grid);
            //    }
            //    //if (cnt == 20) {
            //    //    flag = false;
            //    //}
            //}

        }

        const double f = 300.0;
        Mat K = (cv::Mat_<double>(3, 3) << f, 0.0, (img.size().width - 1.0) / 2.0, 0.0, f, (img.size().height - 1.0) / 2.0, 0.0, 0.0, 1.0);
        Mat udimg;
        cv::fisheye::undistortImage(img, udimg, cam, dist, K);
        img = udimg;

        if (key == 'c') {
            {
                vector<Mat> rvecs, tvecs;
                std::cout << cam << dist << img.size() << std::endl;
                const double rms = cv::fisheye::calibrate(objs, pixs, cv::Size(img.size().height, img.size().width), cam, dist, rvecs, tvecs, fisheye::CALIB_FIX_SKEW | fisheye::CALIB_RECOMPUTE_EXTRINSIC);
                printf("opencv rms error: %g\n", rms);
            }
            cout << "cam = " << endl << cam << endl;
            cout << "dist = " << endl << dist << endl;

            //std::cout << cam << dist << img.size() << std::endl;
            vector<Mat> rvecs, tvecs;
            //cam.at<float>(0, 0) = 100.0;
            //cam.at<float>(1, 1) = 100.0;
            //cam.at<float>(2, 2) = 1.0;
            //cam.at<float>(0, 2) = (img.size().width - 1) / 2.0;
            //cam.at<float>(1, 2) = (img.size().height - 1) / 2.0;
            //dist.at<float>(0, 0) = 0.0;
            //dist.at<float>(0, 1) = 0.0;
            //dist.at<float>(0, 2) = 0.0;
            //dist.at<float>(0, 3) = 0.0;
            //std::cout << cam << dist << img.size() << std::endl;
            {
                const double rms = cv::fisheye::calibrate(objs, pixs, cv::Size(img.size().height, img.size().width), cam, dist, rvecs, tvecs, fisheye::CALIB_FIX_SKEW | fisheye::CALIB_RECOMPUTE_EXTRINSIC | fisheye::CALIB_USE_INTRINSIC_GUESS);
                printf("opencv rms error: %g\n", rms);
            }
            cout << "cam = " << endl << cam << endl;
            cout << "dist = " << endl << dist << endl;

            sp::CamParam spcam = sp::getCamParam(img.size().width, img.size().height);
            spcam.fx = cam.at<double>(0, 0);
            spcam.fy = cam.at<double>(1, 1);
            spcam.cx = cam.at<double>(0, 2);
            spcam.cy = cam.at<double>(1, 2);

            sp::Mat spdist(1, 4);
            spdist[0] = dist.at<double>(0, 0);
            spdist[1] = dist.at<double>(0, 1);
            spdist[2] = dist.at<double>(0, 2);
            spdist[3] = dist.at<double>(0, 3);

            sp::print(spcam);
            sp::print(spdist);
        }


        imshow("calibration", img);

        {
            static int c = 0;
            char path[256];
            sprintf(path, "%s\\%04d.png", tmp, c++);
            imwrite(path, img);
        }
    }

    destroyAllWindows();
    return 0;
}