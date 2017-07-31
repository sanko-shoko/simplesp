#include "simplesp.h"

using namespace sp;

int main(){

	Mem2<Col3> imgs[2];
	SP_ASSERT(loadBMP(imgs[0], SP_DATA_DIR  "/image/shiba02.bmp"));
	SP_ASSERT(loadBMP(imgs[1], SP_DATA_DIR  "/image/shiba04.bmp"));

	CamParam cam;
	SP_ASSERT(loadText(cam, SP_DATA_DIR  "/image/shiba.txt"));

	Mem2<Col3> rimgs[2];
	{
		SfM sfm;
		sfm.addData(imgs[0], cam);
		sfm.addData(imgs[1], cam);

		for (int i = 0; i < 10; i++) {
			sfm.update();
		}

		Pose stereo = sfm.m_views[1].pose * invPose(sfm.m_views[0].pose);
		print(stereo);

		RectParam rects[2];
		rectify(rects[0], rects[1], cam, cam, stereo);

		Mem2<Vec2> tables[2];

		for (int i = 0; i < 2; i++) {
			makeRemapTable(tables[i], rects[i]);
			remap<Col3, Byte>(rimgs[i], imgs[i], tables[i]);

			char str[SP_STRMAX];
			sprintf(str, "rect%d.bmp", i);
			saveBMP(rimgs[i], str);
		}
	}


	return 0;
}