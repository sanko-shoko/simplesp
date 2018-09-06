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

    printf("'s' key : switch marker pose / base pose\n");
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

    // detector class
    BitMarker bitMarker;

    // set marker info
    {
        const int dsize[2] = { 4, 3 };
        const double length = 50.0;
        const double distance = 60.0;
        const Mem1<BitMarkerParam> mrks = getBitMarkerArray(dsize[0], dsize[1], length, distance);

        bitMarker.addMrks(mrks);
    }

    // estimate marker pose
    bitMarker.execute(img);

    if (bitMarker.getPose(0) != NULL){
        renderAxis(img, bitMarker.getCam(), *bitMarker.getPose(0), bitMarker.getMrks(0)[0].length / 2.0, 2);

        const Mem1<Vec2> &cpixs = *bitMarker.getCrspPixs(0);
        for (int i = 0; i < cpixs.size(); i++) {
            renderPoint(img, cpixs[i], getCol(0, 255, 0), 3);
        }
    }

    cvCnvImg(cvimg, img);
}

