#include "simplesp.h"

using namespace sp;

int main(){

	//--------------------------------------------------------------------------------
	// phase only correlation
	//--------------------------------------------------------------------------------

	Mem2<Byte> img;
	SP_ASSERT(loadBMP(img, SP_DATA_DIR  "/image/Lenna.bmp"));

	Mem2<Byte> img0, img1;
	{
		const Rect rect = adjustRect(getRect2(img.dsize), -128);

		Rect rect0 = rect;
		Rect rect1 = rect;

		rect1.dbase[0] += 12;
		rect1.dbase[1] += 34;

		crop(img0, img, rect0);
		crop(img1, img, rect1);
		
		saveBMP(img0, "input0.bmp");
		saveBMP(img1, "input1.bmp");
	}

	Mem2<double> re0, im0;
	Mem2<double> re1, im1;
	{
		printf("dft\n");
		dft(re0, im0, img0);
		dft(re1, im1, img1);
	}

	Mem2<double> r;
	{
		poc(r, re0, im0, re1, im1);
		saveText(r, "r.csv");

		Mem2<Byte> vis;
		const double maxv = maxVal(r);
		const double minv = minVal(r);
		cnvMem(vis, r, 255.0 / (maxv - minv), minv);
		saveBMP(vis, "vis.bmp");

		const int id = maxArg(r);
		
		int x = id % r.dsize[0];
		int y = id / r.dsize[0];

		if (x > r.dsize[0] / 2) x -= r.dsize[0];
		if (y > r.dsize[1] / 2) y -= r.dsize[1];

		printf("shift %d %d\n", x, y);
		printf("peak %lf\n", maxv);
	}

	return 0;
}