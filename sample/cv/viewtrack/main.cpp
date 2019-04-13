#include "simplesp.h"
#include "spex/spcv.h"

using namespace sp;

const DotMarkerParam mrk(5, 5, 30.0);

void sample(cv::Mat &cvimg, const int key);

int main(){
    cv::VideoCapture cap(0);

    if (cap.isOpened() == false){
        printf("could not find USB camera\n");
        exit(0);
    }
    printf("'s' key : start viewtrack \n");
    printf("'ESC' key : exit \n");
    printf("\n");

    printf("option \n");
    printf("'a' key : add detected points for calibration \n");
    printf("'c' key : execute calibration (i >= 3) \n");

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

    // calibration
    static CalibTool ctool;
    {
        if (key == 'a') {
            ctool.addImg(mrk, img);
        }
        if (key == 'c') {
            if (ctool.execute() == true) {
                ctool.save("cam.txt");
            }
        }
    }

    const CamParam cam = (ctool.getCam() != NULL) ? *ctool.getCam() : getCamParam(img.dsize);

    static ViewTrack tracker;
    if (key == 's') {
        tracker.setCam(cam);
        tracker.setBase(img, zeroPose());
        return;
    }

    tracker.execute(img);

    //if (tracker.getPose() == NULL) 
    {
        if (tracker.getMask() != NULL) {
            const Mem1<bool> &mask = *tracker.getMask();
            const Mem1<Vec2> &bases = *tracker.getBases();
            const Mem1<Vec2> &crsps = *tracker.getCrsps();

            for (int i = 0; i < mask.size(); i++) {
                if (mask[i] == false) continue;

                const Vec2 pix0 = bases[i];
                const Vec2 pix1 = crsps[i];
                const Vec2 flow = pix1 - pix0;

                const double angle = (flow.x != 0.0 || flow.y != 0.0) ? ::atan2(flow.x, flow.y) : 0.0;
                const double norm = normVec(flow) / 50.0;

                Col3 col;
                cnvHSVToCol(col, getVec3(angle + SP_PI, minVal(1.0, norm), 1.0));

                renderLine(img, pix0, pix1, col, 2);
            }
        }
    }

    if (tracker.getPose() != NULL) {

        const Pose base = getPose(getVec3(0.0, 0.0, 20.0));

        renderAxis(img, cam, *tracker.getPose() * base, 2.0, 2);
        renderGrid3d(img, cam, *tracker.getPose() * base, 6.0, 2, getCol(100, 200, 200), 2);
    }

    cvCnvImg(cvimg, img);

}
