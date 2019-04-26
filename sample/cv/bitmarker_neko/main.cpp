#include "simplesp.h"
#include "spex/spcv.h"

using namespace sp;

void sample(cv::Mat &cvimg, const cv::Mat &cvneko, const int key);

int main(){
    cv::VideoCapture cap(0);
    if (cap.isOpened() == false){
        printf("could not find USB camera\n");
        exit(0);
    }

    cv::Mat cvneko = cv::imread(SP_DATA_DIR  "/marker/neko.bmp");
    if (cvneko.data == NULL){
        printf("could not find data [simplesp/data/marker/neko.bmp]\n");
        exit(0);
    }

    printf("'ESC' key : exit\n");

    int key = 0;

    // 27 = 'ESC' key
    while ((key = cv::waitKey(1)) != 27){

        // capture
        cv::Mat cvimg;
        cap >> cvimg;

        sample(cvimg, cvneko, key);

        cv::imshow("bitmarker", cvimg);
    }

    cv::destroyAllWindows();
    return 0;
}

void sample(cv::Mat &cvimg, const cv::Mat &cvneko, const int key){
    Mem2<Col3> img, neko;

    // convert data type
    cvCnvImg(img, cvimg);
    cvCnvImg(neko, cvneko);

    // detector class
    BitMarker bitMarker;

    // set marker info
    {
        const double length = 50.0;
        const BitMarkerParam mrks(neko, length);

        bitMarker.addMrks(mrks);
    }   

    // estimate marker pose
    bitMarker.execute(img);

    // render
    if (bitMarker.getPose(0) != NULL){
        renderAxis(img, bitMarker.getCam(), *bitMarker.getPose(0), bitMarker.getMrks(0)[0].length / 2.0, 2);
        
        const Mem1<Vec2> &cpixs = *bitMarker.getCrspPixs(0);
        for (int i = 0; i < cpixs.size(); i++) {
            renderPoint(img, cpixs[i], getCol3(0, 255, 0), 3);
        }

        //const Vec3 offset = getVec3(0.0, 0.0, -bitMarker.getMrks(0)[0].length / 2.0);
        //const Pose pose = *bitMarker.getPose(0) * getPose(offset);
        //renderCube(img, bitMarker.getCam(), pose, bitMarker.getMrks(0)[0].length, getCol3(50, 50, 200), 2);
    }

    cvCnvImg(cvimg, img);
}

