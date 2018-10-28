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

    printf("'p' key : render detected points\n");
    printf("'a' key : add detected points for calibration\n");
    printf("'c' key : execute calibration (i >= 3) \n");
    printf("'d' key : diminish marker\n");
    printf("'ESC' key : exit\n");

    int key = 0;

    // 27 = 'ESC' key
    while ((key = cv::waitKey(1)) != 27){

        // capture
        cv::Mat cvimg;
        cap >> cvimg;

        sample(cvimg, key);

        cv::imshow("dotmarker", cvimg);
    }

    cv::destroyAllWindows();
    return 0;
}

void sample(cv::Mat &cvimg, const int key){
    Mem2<Col3> img;

    // convert data type
    cvCnvImg(img, cvimg);

    // detector class
    static DotMarker dotMarker;
    dotMarker.setMrk(mrk);
    
    // estimate marker pose
    dotMarker.execute(img);

    // calibration
    {
        static CalibTool ctool;

        if (key == 'a') {
            ctool.addImg(mrk, img);

            //char str[256];
            //sprintf(str, "calib%03d.bmp", ctool.size());
            //saveBMP(img, str);
        }

        if (key == 'c') {
            if (ctool.execute() == true) {
                ctool.save("cam.txt");

                const CamParam cam = *ctool.getCam();
                dotMarker.setCam(cam);
            }
        }
    }

    // diminish marker
    {
        static bool flag = false;
        if (key == 'd') flag ^= true;
        
        if (flag == true && dotMarker.getHom() != NULL){
            diminishDotMarker(img, dotMarker.getMrk(), *dotMarker.getHom());
        }
    }

    // render detected points
    {
        static bool flag = false;
        if (key == 'p') flag ^= true;
        
        if (flag == true && dotMarker.getCrspPixs()){
            renderPoint(img, *dotMarker.getCrspPixs(), getCol(0, 255, 0), 4);
        }
    }

    // undistort image
    {
        Mem2<Vec2> table;
        makeRemapTable(table, dotMarker.getCam());

        remap<Col3, Byte>(img, img, table);
    }

    // render
    if (dotMarker.getPose() != NULL){
        const Vec2 size = getVec(mrk.map.dsize[0] + 1, mrk.map.dsize[1] + 1) * 0.5 * dotMarker.getMrk().distance;

        renderRect(img, dotMarker.getCam(), *dotMarker.getPose(), -size, +size, getCol(0, 100, 200), 2);
        renderAxis(img, dotMarker.getCam(), *dotMarker.getPose(), minVal(size.x, size.y), 2);
    }

    cvCnvImg(cvimg, img);
}

