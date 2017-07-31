#include "simplesp.h"

using namespace sp;

int main() {
	Mem1<Mem2<Col3> > imgs;
	Mem1<CamParam> cams;

	for (int i = 0; i < 4; i++) {
		char str[512];
		sprintf(str, SP_DATA_DIR "/image/fuzzy%03d.bmp", i * 4 + 30);

		Mem2<Col3> img;
		SP_ASSERT(loadBMP(img, str));

		imgs.push(img);
		cams.push(getCamParam(img.dsize));
	}


	SfM sfm;
	sfm.execute(imgs, cams);

	sfm.savePly("test.ply");
	return 0;
}

