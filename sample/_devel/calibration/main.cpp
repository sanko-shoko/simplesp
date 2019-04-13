#define SP_USE_DEBUG 0

#include "simplesp.h"
#include "spex/spcv.h"

using namespace sp;

int main(){
    
    // test camera parameter
    CamParam cam = getCamParam(640, 480);

    // test marker pose
    Mem1<Pose> poses;

    const Pose stereo = getPose(getVec3(-1.0, 0.0, 0.0));

    // marker position
    const DotMarkerParam mrk(5, 5, 30);
    const Mem2<Vec2> mrkMap = mrk.map * mrk.distance;
    
    {
        cam.cx = 300;
        cam.cy = 250;
        cam.k1 = 0.1;
        cam.k2 = 0.1;
        cam.k3 = 0.1;
        cam.p1 = 0.1;
        cam.p2 = 0.1;

        printf("grand truth\n");
        print(cam);

        // generate test pose
        poses.push(getPose(getVec3(+00.0, 0.0, 400)));
        poses.push(getPose(getVec3(0.0, +10.0, 400)) * getRotAngleX(+30 * SP_PI / 180.0));
        poses.push(getPose(getVec3(0.0, -10.0, 400)) * getRotAngleX(-30 * SP_PI / 180.0));
        poses.push(getPose(getVec3(+10.0, 0.0, 400)) * getRotAngleY(+30 * SP_PI / 180.0));
        poses.push(getPose(getVec3(-10.0, 0.0, 400)) * getRotAngleY(-30 * SP_PI / 180.0));
    }

    // test (generate test points)
    {
        printf("\n\n");
        printf("--------------------------------------------------------------------------------\n");
        printf("cam test (generate test points)\n");
        printf("--------------------------------------------------------------------------------\n");

        Mem1<Mem1<Vec2> > pixs, objs;

        // generate test points
        for (int i = 0; i < poses.size(); i++){
            Mem1<Vec2> tpixs, tobjs;
            for (int n = 0; n < mrkMap.size(); n++){
                const Vec3 pos = poses[i] * getVec3(mrkMap[n].x, mrkMap[n].y, 0.0);
                const Vec2 pix = mulCamD(cam, prjVec(pos)) + randVecGauss(0.1, 0.1);

                tpixs.push(pix);
                tobjs.push(mrkMap[n]);
            }
            pixs.push(tpixs);
            objs.push(tobjs);
        }

        // simplesp calibration 
        {
            printf("\n\n");
            printf("simplesp\n");

            // calibration
            CamParam dst;
            const double rms = calibCam(dst, cam.dsize[0], cam.dsize[1], pixs, objs);
            printf("simplesp rms error: %g\n", rms);

            print(dst);
            saveText("cam0.txt", dst);
        }

        // opencv calibration 
        {
            using namespace std;
            using namespace cv;

            printf("\n\n");
            printf("opencv\n");

            // Point2d is not available in cv::calibrateCamera
            vector<vector<Point2f> > cvpixs;
            vector<vector<Point3f> > cvobjs;

            for (int i = 0; i < pixs.size(); i++){
                vector<Point2f> tpixs;
                vector<Point3f> tobjs;
                for (int j = 0; j < pixs[i].size(); j++){
                    tpixs.push_back(Point2f(static_cast<float>(pixs[i][j].x), static_cast<float>(pixs[i][j].y)));
                    tobjs.push_back(Point3f(static_cast<float>(objs[i][j].x), static_cast<float>(objs[i][j].y), 0.f));
                }
                cvpixs.push_back(tpixs);
                cvobjs.push_back(tobjs);
            }

            const Size imgSize(cam.dsize[0], cam.dsize[1]);
            cv::Mat camMat, dist;
            vector<cv::Mat> rvecs, tvecs;

            const double rms = cv::calibrateCamera(cvobjs, cvpixs, Size(cam.dsize[0], cam.dsize[1]), camMat, dist, rvecs, tvecs);
            printf("opencv rms error: %g\n", rms);
            cout << "camMat = " << endl << camMat << endl;
            cout << "dist = " << endl << dist << endl;
        }
    }

    // cam test (generate test images)
    {
        printf("\n\n");
        printf("--------------------------------------------------------------------------------\n");
        printf("cam test (generate test images)\n");
        printf("--------------------------------------------------------------------------------\n");

        Mem1<Mem2<Byte> > imgs(poses.size());

        // generate test images
        for (int i = 0; i < poses.size(); i++){
            renderMarker(imgs[i], cam, poses[i], mrkMap);
            char str[256];
            sprintf(str, "test%02d.bmp", i);
            saveBMP(str, imgs[i]);
        }

        // simplesp calibration 
        {
            printf("\n\n");
            printf("simplesp\n");

            DotMarker dotMarker;
            dotMarker.setMrk(mrk);

            Mem1<Mem1<Vec2> > pixs, objs;

            // detect points
            for (int i = 0; i < imgs.size(); i++){
                if (dotMarker.execute(imgs[i])){
                    pixs.push(*dotMarker.getCrspPixs());
                    objs.push(*dotMarker.getCrspObjs());
                }
            }

            // calibration
            CamParam dst;
            const double rms = calibCam(dst, cam.dsize[0], cam.dsize[1], pixs, objs);
            printf("simplesp rms error: %g\n", rms);

            print(dst);
            saveText("cam1.txt", dst);
        }

        // opencv calibration 
        {
            using namespace std;
            using namespace cv;

            printf("\n\n");
            printf("opencv\n");

            vector<vector<Point2f> > cvpixs;
            vector<vector<Point3f> > cvobjs;

            const Size boardSize(mrk.map.dsize[0], mrk.map.dsize[1]);
            for (int i = 0; i < imgs.size(); i++){
                cv::Mat img(cam.dsize[1], cam.dsize[0], CV_8UC1);
                memcpy(img.ptr(), imgs[i].ptr, imgs[i].size());
                imwrite("test.png", img);
                vector<cv::Point2f> pointbuf;
                const bool found = findCirclesGrid(img, boardSize, pointbuf);
                if (found == false) continue;
                
                vector<Point2f> tpixs;
                vector<Point3f> tobjs;

                for (int n = 0; n < pointbuf.size(); n++){
                    tpixs.push_back(pointbuf[n]);
                    tobjs.push_back(cv::Point3f(static_cast<float>(mrkMap[n].x), static_cast<float>(mrkMap[n].y), 0.f));
                }
                cvpixs.push_back(tpixs);
                cvobjs.push_back(tobjs);
            }

            cv::Mat camMat, dist;
            vector<cv::Mat> rvecs, tvecs;

            const double rms = cv::calibrateCamera(cvobjs, cvpixs, Size(cam.dsize[0], cam.dsize[1]), camMat, dist, rvecs, tvecs);
            printf("opencv rms error: %g\n", rms);
            cout << "camMat = " << endl << camMat << endl;
            cout << "dist = " << endl << dist << endl;
        }
    }

    // stereo test (generate test points)
    {
        printf("\n\n");
        printf("--------------------------------------------------------------------------------\n");
        printf("stereo test (generate test points)\n");
        printf("--------------------------------------------------------------------------------\n");

        Mem1<Mem1<Vec2> > pixs0, pixs1, objs0, objs1;

        // generate test points
        for (int i = 0; i < poses.size(); i++){
            Mem1<Vec2> tpixs0, tpixs1, tobjs;

            const double sigma = 0.1;
            for (int n = 0; n < mrkMap.size(); n++){
                const Vec3 pos = poses[i] * getVec3(mrkMap[n].x, mrkMap[n].y, 0.0);
                const Vec2 pix0 = mulCamD(cam, prjVec(pos)) + randVecGauss(sigma, sigma);
                const Vec2 pix1 = mulCamD(cam, prjVec(stereo * pos)) + randVecGauss(sigma, sigma);

                tpixs0.push(pix0);
                tpixs1.push(pix1);
                tobjs.push(mrkMap[n]);
            }
            pixs0.push(tpixs0);
            pixs1.push(tpixs1);
            objs0.push(tobjs);
            objs1.push(tobjs);
        }

        // simplesp calibration 
        {
            printf("\n\n");
            printf("simplesp\n");

            // calibration
            Pose pose;
            const double rms = calibStereo(pose, cam, pixs1, objs1, cam, pixs0, objs0);
            printf("simplesp rms error: %g\n", rms);

            print(pose);
            saveText("pose.txt", pose);
        }

        // opencv calibration 
        {
            using namespace std;
            using namespace cv;

            printf("\n\n");
            printf("opencv\n");

            // Point2d is not available in cv::calibrateCamera
            vector<vector<Point2f> > cvpixs0;
            vector<vector<Point2f> > cvpixs1;
            vector<vector<Point3f> > cvobjs;

            for (int i = 0; i < pixs0.size(); i++){
                vector<Point2f> tpixs0;
                vector<Point2f> tpixs1;
                vector<Point3f> tobjs;
                for (int j = 0; j < pixs0[i].size(); j++){
                    tpixs0.push_back(Point2f(static_cast<float>(pixs0[i][j].x), static_cast<float>(pixs0[i][j].y)));
                    tpixs1.push_back(Point2f(static_cast<float>(pixs1[i][j].x), static_cast<float>(pixs1[i][j].y)));
                    tobjs.push_back(Point3f(static_cast<float>(objs0[i][j].x), static_cast<float>(objs0[i][j].y), 0.f));
                }
                cvpixs0.push_back(tpixs0);
                cvpixs1.push_back(tpixs1);
                cvobjs.push_back(tobjs);
            }

            const Size imgSize(cam.dsize[0], cam.dsize[1]);
            cv::Mat camMat = cv::Mat::zeros(3, 3, CV_64FC1);
            cv::Mat dist = cv::Mat::zeros(1, 5, CV_64FC1);

            camMat.at<double>(0, 0) = cam.fx;
            camMat.at<double>(1, 1) = cam.fy;
            camMat.at<double>(0, 2) = cam.cx;
            camMat.at<double>(1, 2) = cam.cy;
            camMat.at<double>(2, 2) = 1.0;

            dist.at<double>(0) = cam.k1;
            dist.at<double>(1) = cam.k2;
            dist.at<double>(2) = cam.p1;
            dist.at<double>(3) = cam.p2;
            dist.at<double>(4) = cam.k3;
            cv::Mat R, T, E, F;

            const double rms = cv::stereoCalibrate(cvobjs, cvpixs0, cvpixs1, camMat, dist, camMat, dist, imgSize, R, T, E, F);
            printf("opencv rms error: %g\n", rms);
            cout << "R " << R << endl;
            cout << "T " << T << endl;
        }
    }

    return 0;
}

