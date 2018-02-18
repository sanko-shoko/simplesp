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

        cv::imshow("opticalflow", cvimg);
    }

    cv::destroyAllWindows();
    return 0;
}

void sample(cv::Mat &cvimg){
    Mem2<Col3> crnt;

    // convert data type
    cvCnvImg(crnt, cvimg);

    static Mem2<Col3> prev;
    if (prev.size() == 0) {
        prev = crnt;
        return;
    }

    // detect corner
    Mem1<Vec2> pixs;
    {
        harris(pixs, crnt, 6);
    }

    // optical flow
    Mem1<Vec2> flows;
    Mem1<bool> masks;
    {
        opticalFlowLK(flows, masks, prev, crnt, pixs);
        prev = crnt;
    }

    for (int i = 0; i < flows.size(); i++) {
        if (masks[i] == false) continue;

        const Vec2 pix = pixs[i];
        const Vec2 flow = flows[i];

        const double angle = (flow.x != 0.0 || flow.y != 0.0) ? ::atan2(flow.x, flow.y) : 0.0;
        const double norm = normVec(flow) / 50.0;

        Col3 col;
        cnvHSVToCol(col, getVec(angle + SP_PI, minVal(1.0, norm), 1.0));

        renderLine(crnt, pix, pix + flow, col, 2);
    }

    cvCnvImg(cvimg, crnt);
}

