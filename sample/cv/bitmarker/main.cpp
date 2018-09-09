#include "simplesp.h"
#include "spex/spcv.h"

using namespace sp;

void sample(cv::Mat &cvimg, const int key);

int main(){
    cv::VideoCapture cap(0);
    if (cap.isOpened() == false){
        printf("could not find USB camera\n");
        exit(0);
    }

    printf("'s' key : switch center pose / markers pose\n");
    printf("'ESC' key : exit\n");

    int key = 0;

    // 27 = 'ESC' key
    while ((key = cv::waitKey(1)) != 27){

        // capture
        cv::Mat cvimg;
        cap >> cvimg;

        sample(cvimg, key);

        cv::imshow("bitmarker", cvimg);
    }

    cv::destroyAllWindows();
    return 0;
}

void sample(cv::Mat &cvimg, const int key){
    Mem2<Col3> img;

    // convert data type
    cvCnvImg(img, cvimg);

    // markers parameter
    Mem1<BitMarkerParam> mrks;

    // set marker info
    {
        const int dsize[2] = { 4, 3 };
        const int block = 3;
        const double length = 60.0;
        const double interval = 5.0;
        
        //const int dsize[2] = { 8, 6 };
        //const int block = 3;
        //const double length = 28.0;
        //const double interval = 5.0;

        mrks = getBitMarkerParam(0, block, length, dsize[0], dsize[1], interval);

        SP_ONCE(saveBitMarkerParamSVG("mrks.svg", 0, block, length, dsize[0], dsize[1], interval));
    }


    // detector class
    BitMarker bitMarker;

    static bool flag = true;
    if (key == 's') flag ^= true;

    // detect marker
    if(flag){
        bitMarker.addMrks(mrks);

        bitMarker.execute(img);

        if (bitMarker.getPose(0) != NULL) {

            const Mem1<Vec2> &cpixs = *bitMarker.getCrspPixs(0);
            renderPoint(img, *bitMarker.getCrspPixs(0), getCol(0, 255, 255), 3);

            renderAxis(img, bitMarker.getCam(), *bitMarker.getPose(0), 50.0, 2);
        }
    }
    else {
        for (int i = 0; i < mrks.size(); i++) {
            mrks[i].offset = zeroPose();
            bitMarker.addMrks(mrks[i]);
        }

        bitMarker.execute(img);

        for (int i = 0; i < mrks.size(); i++) {
            if (bitMarker.getPose(i) == NULL) continue;

            const Mem1<Vec2> &cpixs = *bitMarker.getCrspPixs(i);
            renderPoint(img, cpixs, getCol(0, 255, 255), 3);

            renderAxis(img, bitMarker.getCam(), *bitMarker.getPose(i), 20.0, 2);
        }
    }


    cvCnvImg(cvimg, img);
}

