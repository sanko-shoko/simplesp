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


    const double length = 50.0;
    const double distance = 60.0;
    Mem<BitMarkerParam> mrks = getBitMarkerArray(4, 3, length, distance);

    // render
    static bool flag = false;
    if (key == 's') flag ^= true;

    if (flag == false){
        // detector class
        BitMarker bitMarker;

        bitMarker.setMrks(mrks);

        // estimate marker pose
        bitMarker.execute(img);

        for (int i = 0; i < bitMarker.getMrks().size(); i++){
            if (bitMarker.getPose(i) == NULL) continue;
            const BitMarkerParam &mrk = bitMarker.getMrks()[i];

            //renderAxis(img, bitMarker.getCamParam(), *bitMarker.getPose(i), mrk.length / 2.0, 2);

            const Vec3 offset = getVec(0.0, 0.0, -mrk.length / 2);
            const Pose pose = *bitMarker.getPose(i) * getPose(offset);
            renderCube(img, bitMarker.getCam(), pose, mrk.length, getCol(50, 50, 200), 2);
        }
    }
    else{
        Pose pose;
        if (calcBitMarkerArrayPose(pose, img, getCamParam(img.dsize), mrks) == true){
            renderAxis(img, getCamParam(img.dsize), pose, length, 2);
        }
    }

    cvCnvImg(cvimg, img);
}

