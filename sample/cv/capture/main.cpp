#include "simplesp.h"
#include "spex/spcv.h"

using namespace sp;

void sample(cv::Mat &cvimg, const int key);

int main(){
    printf("'s' key : save single image\n");
    printf("'m' key : save multi images\n");
    printf("'ESC' key : exit\n");

    cv::VideoCapture cap(0);

    //cap.set(CV_CAP_PROP_FRAME_WIDTH, 800);
    //cap.set(CV_CAP_PROP_FRAME_HEIGHT, 600);

    if (cap.isOpened() == false){
        printf("could not find USB camera\n");
        exit(0);
    }

    int key = 0;

    // 27 = 'ESC' key
    while ((key = cv::waitKey(1)) != 27){

        // capture
        cv::Mat cvimg;
        cap >> cvimg;

        sample(cvimg, key);

        cv::imshow("capture", cvimg);
    }

    cv::destroyAllWindows();
    return 0;
}

void sample(cv::Mat &cvimg, const int key){
    Mem2<Col3> img;

    // convert data type
    cvCnvImg(img, cvimg);

    // save image
    {
        static int cnt = 0;
        const int limit = 300;

        if (key == 's'){
            cnt = limit;
        }
        if (key == 'm'){
            cnt = 1;
        }

        if (cnt > 0){
            static int id = 0;
            char str[256];
            sprintf(str, "image%03d.bmp", id++);

            saveBMP(img, str);
            cnt++;
        }
        if (cnt > limit){
            cnt = 0;
        }
    }

    cvCnvImg(cvimg, img);
}

