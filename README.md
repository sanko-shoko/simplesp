# simplesp

simple library for computer vision


## features
calibration (camera, projector, robot), rectification (stereo camera), marker detector (squrare, circle grid), model based object tracking (2d edge, 3d depth), optimization (belief propagation, graphcut), superpixels (SLIC), kdtree, opticalflow, randomforest, visual hull, stereo matching (passive, active), etc


## samples

simplesp\sample

- sample\sp : no third library
- sample\cv : OpenCV required
- sample\gl : OpenGL (GLFW) required


## compile
code
```
#include "simplesp.h"
using namespace sp;
     
int main(){
    printf("compile test\n");
    return 0;
}
```

command
```
$ cd simplesp
$ g++ compiletest.cpp -I ./ -std=c++11
$ ./a.exe
```

