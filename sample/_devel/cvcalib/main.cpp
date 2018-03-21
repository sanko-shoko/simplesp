#include <vector>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main() {
    const Size bsize(5, 5);
    const double bdistance = 30.0;

    vector<Point3f> grid;
    for (int y = 0; y < bsize.height; y++) {
        for (int x = 0; x < bsize.width; x++) {
            grid.push_back(Point3f(x, y, 0.0f) * bdistance);
        }
    }

    VideoCapture cap(0);
    if (cap.isOpened() == false) {
        printf("could not find USB camera\n");
        exit(0);
    }

    printf("'a' key : add detected points for calibration\n");
    printf("'c' key : execute calibration (i >= 3) \n");
    printf("'ESC' key : exit\n");

    int key = 0;

    // 27 = 'ESC' key
    while ((key = waitKey(1)) != 27) {

        // capture
        Mat img;
        cap >> img;

        vector<Point2f> pointbuf;
        const bool found = findCirclesGrid(img, bsize, pointbuf);

        if (found == true) {
            drawChessboardCorners(img, bsize, pointbuf, found);

            static int cnt = 0;
            static vector<vector<Point2f> > pixsList;
            static vector<vector<Point3f> > objsList;

            if (key == 'a') {
                printf("add detected points (i = %d)\n", cnt++);
                pixsList.push_back(pointbuf);
                objsList.push_back(grid);

            }

            if (key == 'c' && cnt >= 3) {
                Mat camMat, dist;
                vector<Mat> rvecs, tvecs;

                const double rms = calibrateCamera(objsList, pixsList, img.size(), camMat, dist, rvecs, tvecs);
                printf("opencv rms error: %g\n", rms);

                cout << "camMat = " << endl << camMat << endl;
                cout << "dist = " << endl << dist << endl;
            }
        }

        imshow("calibration", img);
    }

    destroyAllWindows();
    return 0;
}

