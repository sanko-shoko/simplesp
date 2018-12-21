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

    printf("'a' key : detect features (harris)\n");
    printf("'s' key : detect features (SIFT)\n");
    printf("'ESC' key : exit\n");

    int key = 0;

    // 27 = 'ESC' key
    while ((key = cv::waitKey(1)) != 27){

        // capture
        cv::Mat cvimg;
        cap >> cvimg;

        sample(cvimg, key);

        cv::imshow("opticalflow", cvimg);
    }

    cv::destroyAllWindows();
    return 0;
}

void sample(cv::Mat &cvimg, const int key){
    Mem2<Col3> crnt;

    // convert data type
    cvCnvImg(crnt, cvimg);

    static Mem2<Col3> prev;

    static Mem1<Vec2> pixs;
    static Mem1<double> scls;

    // detect features
    if (key == 'a' || key == 's') { 
        prev = crnt;
        pixs.clear();
        scls.clear();

        if (key == 'a') {
            harris(pixs, crnt, 6);
        }
        if (key == 's') {
            const Mem1<Ftr> ftrs = SIFT::getFtrs(crnt);
            for (int i = 0; i < ftrs.size(); i++) {
                pixs.push(ftrs[i].pix);
                scls.push(ftrs[i].scl);
            }
        }
    }

    if (pixs.size() == 0) return;

    static Mem1<Vec2> flows;
    Mem1<bool> mask;
    {
        opticalFlowLK(flows, mask, crnt, prev, pixs, scls);
    }

    for (int i = 0; i < flows.size(); i++) {
        if (mask[i] == false) continue;

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
