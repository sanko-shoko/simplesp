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

    printf("'s' key : start viewtrack\n");
    printf("'ESC' key : exit\n");

    int key = 0;

    // 27 = 'ESC' key
    while ((key = cv::waitKey(1)) != 27){

        // capture
        cv::Mat cvimg;
        cap >> cvimg;

        sample(cvimg, key);

        cv::imshow("viewtrack", cvimg);
    }

    cv::destroyAllWindows();
    return 0;
}

void sample(cv::Mat &cvimg, const int key){
    Mem2<Col3> img;

    // convert data type
    cvCnvImg(img, cvimg);

    const CamParam cam = getCamParam(img.dsize);

    static ViewTrack tracker;

    if (key == 's') {
        tracker.setView(cam, img, zeroPose());
        return;
    }

    if (tracker.getView() == NULL) return;

    tracker.execute(cam, img, zeroPose());

    if (tracker.getPose() == NULL) {
        for (int i = 0; i < tracker.m_crsps.size(); i++) {
            if (tracker.m_mask[i] == false) continue;

            const Vec2 pix0 = tracker.getView()->fts[i].pix;
            const Vec2 pix1 = tracker.m_crsps[i];
            const Vec2 flow = pix1 - pix0;

            const double angle = (flow.x != 0.0 || flow.y != 0.0) ? ::atan2(flow.x, flow.y) : 0.0;
            const double norm = normVec(flow) / 50.0;

            Col3 col;
            cnvHSVToCol(col, getVec(angle + SP_PI, minVal(1.0, norm), 1.0));

            renderLine(img, pix0, pix1, col, 2);
        }
    }
    else{

        const Pose base = getPose(getVec(0.0, 0.0, 20.0));

        renderAxis(img, cam, *tracker.getPose() * base, 2.0, 2);
        renderGrid3d(img, cam, *tracker.getPose() * base, 6.0, 2, getCol(100, 200, 200), 2);
    }

    cvCnvImg(cvimg, img);
}
