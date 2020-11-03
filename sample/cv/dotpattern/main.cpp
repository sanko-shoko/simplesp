#include "simplesp.h"
#include "spex/spcv.h"

using namespace sp;

const DotMarkerParam mrk(5, 5, 30.0);
const DotPatternParam ptn(1280, 720);

void sample(cv::Mat &cvimg, const int key);

int main(){

    saveBMP("pattern.bmp", ptn.img);

    cv::VideoCapture cap(0);
    if (cap.isOpened() == false){
        printf("could not find USB camera\n");
        exit(0);
    }

    printf("'a' key : add detected points for calibration\n");
    printf("'c' key : execute calibration (i >= 3) \n");
    printf("'ESC' key : exit\n");

    int key = 0;
    int flag = 0;
    // 27 = 'ESC' key
    while ((key = cv::waitKey(1)) != 27) {

        // capture
        cv::Mat cvimg;
        cap >> cvimg;

        sample(cvimg, key);

        cv::imshow("dotmarker", cvimg);
    }

    cv::destroyAllWindows();
    return 0;
}

void sample(cv::Mat &cvimg, const int key){
    Mem2<Col3> img;

    // convert data type
    cvCnvImg(img, cvimg);

    Mem2<Col3> mrkImg, ptnImg;
    splitMarkerAndPattern(mrkImg, ptnImg, img);

    // detector class
    static DotMarker dotMarker;
    static DotPattern dotPattern;

    dotMarker.setMrk(mrk);
    dotMarker.execute(mrkImg);

    dotPattern.setPtn(ptn);
    dotPattern.execute(ptnImg);


    // render
    {
        if (dotMarker.getPose() != NULL) {
            const Vec2 size = getVec2(dotMarker.getMrk().map.dsize[0] + 1, dotMarker.getMrk().map.dsize[1] + 1) * 0.5 * dotMarker.getMrk().distance;
            renderRect(img, dotMarker.getCam(), *dotMarker.getPose(), size * -1.0, size * +1.0, getCol3(0, 100, 200), 2);
            renderPoint(img, *dotMarker.getCrspPixs(), getCol3(0, 100, 200), 3);
        }

        if (dotPattern.getCrspPix().size() != 0) {
            renderPoint(img, dotPattern.getCrspPix(), getCol3(0, 255, 0), 3);
            renderRect(img, dotPattern.getCrspHom(), getVec2(0.0, 0.0), getVec2(ptn.img.dsize[0], ptn.img.dsize[1]), getCol3(0, 255, 0), 2);
        }
    }

    // calibration
    if (dotMarker.getPose() != NULL && dotPattern.getCrspPix().size() != 0){
        static int cnt = 0;
        static Mem1<Mem1<Vec2> > mrkPixsList, mrkObjsList;
        static Mem1<Mem1<Vec2> > ptnPixsList, ptnPrjsList;
        if (key == 'a') {
            printf("add detected points (i = %d)\n", cnt++);
            mrkPixsList.push(*dotMarker.getCrspPixs());
            mrkObjsList.push(*dotMarker.getCrspObjs());

            ptnPixsList.push(dotPattern.getCrspPix());
            ptnPrjsList.push(dotPattern.getCrspPrj());
        }

        if (key == 'c') {
            CamParam cam;
            const double camRMS = calibCam(cam, img.dsize[0], img.dsize[1], mrkPixsList, mrkObjsList);

            Mem1<Mem1<Vec2> > ptnObjsList;

            for (int i = 0; i < cnt; i++) {
                Mem1<Vec2> mrkUndists;
                for (int j = 0; j < mrkPixsList[i].size(); j++) {
                    mrkUndists.push(pixUndist(cam, mrkPixsList[i][j]));
                }

                Mat hom;
                calcHMat(hom, mrkObjsList[i], mrkUndists);

                Mem1<Vec2> ptnObjs;
                for (int j = 0; j < ptnPixsList[i].size(); j++) {
                    ptnObjs.push(hom * pixUndist(cam, ptnPixsList[i][j]));
                }

                ptnObjsList.push(ptnObjs);
            }

            CamParam prj;
            const double prjRMS = calibCam(prj, ptn.img.dsize[0], ptn.img.dsize[1], ptnPrjsList, ptnObjsList);

            Pose stereo;
            const double stereoRMS = calibStereo(stereo, cam, ptnPixsList, prj, ptnPrjsList, ptnObjsList);
            stereo.pos *= dotMarker.getMrk().distance;

            printf("camRMS %.3lf\n", camRMS);
            printf("prjRMS %.3lf\n", prjRMS);
            printf("stereoRMS %.3lf\n", stereoRMS);

            print(cam);
            print(prj);
            print(stereo);

            saveText("cam.txt", cam);
            saveText("prj.txt", prj);
            saveText("stereo.txt", stereo);
        }
    }

    cvCnvImg(cvimg, img);
}

