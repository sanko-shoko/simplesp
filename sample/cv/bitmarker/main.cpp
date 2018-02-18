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

    {
        const int dsize[2] = { 4, 3 };
        const double length = 50.0;
        const double distance = 60.0;

        Mem1<BitMarkerParam> mrks;
        for (int y = 0; y < dsize[1]; y++){
            for (int x = 0; x < dsize[0]; x++){
                const int id = mrks.size();

                BitMarkerParam mrk(id, length);
                mrk.setOffset(id, dsize[0], dsize[1], distance);

                mrks.push(mrk);
            }
        }
        bitMarker.setMrk(mrks);
    }

    // estimate marker pose
    bitMarker.execute(img);

    // render
    static bool flag = false;
    if (key == 's') flag ^= true;

    if (flag == false){
        for (int i = 0; i < bitMarker.getMrk().size(); i++){
            if (bitMarker.getPose(i) == NULL) continue;
            const BitMarkerParam &mrk = bitMarker.getMrk()[i];

            //renderAxis(img, bitMarker.getCamParam(), *bitMarker.getPose(i), mrk.length / 2.0, 2);

            const Vec3 offset = getVec(0.0, 0.0, -mrk.length / 2);
            const Pose pose = *bitMarker.getPose(i) * getPose(offset);
            renderCube(img, bitMarker.getCam(), pose, mrk.length, getCol(50, 50, 200), 2);
        }
    }
    else{
        if (bitMarker.getBase() != NULL){
            renderAxis(img, bitMarker.getCam(), *bitMarker.getBase(), 50.0, 2);
        }
    }

    cvCnvImg(cvimg, img);
}

