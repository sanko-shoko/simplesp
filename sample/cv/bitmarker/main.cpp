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

    printf("'s' key : switch center pose / each poses\n");
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

        static bool once = true;
        if (once) {
            once = false;
            saveBitMarkerParamSVG("mrks.svg", 0, block, length, dsize[0], dsize[1], interval);
        }
    }


    // detector class
    BitMarker bitMarker;

    //// if camra parameter exist
    //{
    //    static CamParam cam;
    //    SP_ONCE(loadText("cam.txt", cam));
    //    bitMarker.setCam(cam);
    //}

    static bool mode = true;
    if (key == 's') mode ^= true;

    // detect marker
    if(mode){
        // center pose

        bitMarker.addMrks(mrks);

        bitMarker.execute(img);
    }
    else {
        // each poses

        for (int i = 0; i < mrks.size(); i++) {
            mrks[i].offset = zeroPose();
            bitMarker.addMrks(mrks[i]);
        }

        bitMarker.execute(img);
    }

    // render
    {
        for (int i = 0; i < bitMarker.size(); i++) {
            if (bitMarker.getPose(i) == NULL) continue;

            const Mem1<Vec2> &cpixs = *bitMarker.getCrspPixs(i);
            renderPoint(img, cpixs, getCol3(0, 255, 255), 3);

            renderAxis(img, bitMarker.getCam(), *bitMarker.getPose(i), 30.0, 2);
        }
    }


    cvCnvImg(cvimg, img);
}

