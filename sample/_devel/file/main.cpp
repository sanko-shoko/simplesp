
#include "simplesp.h"

using namespace sp;

int main(){ 

    CamParam cam = getCamParam(640, 480);
    saveText(cam, "cam.txt");
    cam = getCamParam(320, 240);
    loadText(cam, "cam.txt");
    print(cam);

    Pose pose = zeroPose();
    saveText(pose, "pose.txt");
    pose = getPose(getRot(1, 1, 1, 1));
    loadText(pose, "pose.txt");
    print(pose);

    Vec3 vec = getVec(0.0, 0.0, 0.0);
    saveText(vec, "vec.txt");
    vec = getVec(2.0, 2.0, 3.0);
    loadText(vec, "vec.txt");
    print(vec);

    Mat mat = zeroMat(3, 3);
    saveText(mat, "mat.txt");
    mat = eyeMat(3, 3);
    loadText(mat, "mat.txt");
    print(mat);

    return 0;
}