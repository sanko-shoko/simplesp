#include "simplesp.h"

using namespace sp;

int main() {

	//--------------------------------------------------------------------------------
	// opticalflow
	//--------------------------------------------------------------------------------

	Mem2<Col3> img0, img1;

	// load image
	{
		if (1) {
			SP_ASSERT(loadBMP(img0, SP_DATA_DIR  "/image/shiba03.bmp"));
			SP_ASSERT(loadBMP(img1, SP_DATA_DIR  "/image/shiba02.bmp"));
		}
		else {
			Mem2<Col3> img;
			SP_ASSERT(loadBMP(img, SP_DATA_DIR  "/image/Lenna.bmp"));
			const Rect rect = adjustRect(getRect2(img.dsize), -64);

			Rect rect0 = rect;
			Rect rect1 = rect;

			rect1.dbase[0] += 10;
			rect1.dbase[1] += 10;

			crop<Col3, Byte>(img0, img, rect0);
			crop<Col3, Byte>(img1, img, rect1);
		}
		saveBMP(img0, "input0.bmp");
		saveBMP(img1, "input1.bmp");
	}


	Mem1<Vec2> pixs;

	// detect corner
	{
		harris(pixs, img0);

		Mem2<Col3> img = img0;
		renderCircle(img, pixs, 4, getCol(0, 255, 0), 1);

		saveBMP(img, "corner.bmp");
	}

	// optical flow (Lucas Kanade method)
	{
		Mem1<Vec2> flows;
		Mem1<bool> masks;
		opticalFlowLK(flows, masks, img0, img1, pixs);

		Mem2<Col3> img = img1;

		for (int i = 0; i < flows.size(); i++) {
			if (masks[i] == false) continue;

			const Vec2 pix = pixs[i];
			const Vec2 flow = flows[i];

			const double angle = (flow.x != 0.0 || flow.y != 0.0) ? ::atan2(flow.x, flow.y) : 0.0;
			const double norm = normVec(flow) / 50.0;

			Col3 col;
			cnvHSVToCol(col, getVec(angle + SP_PI, minVal(1.0, norm), 1.0));

			renderLine(img, pix, pix + flow, col, 2);
		}
		saveBMP(img, "opticalflowLK.bmp");
	}

	// optical flow (Patch Match)
	{
		Mem2<Vec2> flows;
		Mem2<bool> masks;
		opticalFlowPM(flows, masks, img0, img1, 11);

		Mem2<Col3> img(img0.dsize);
		img.zero();

		for (int v = 0; v < img0.dsize[1]; v++) {
			for (int u = 0; u < img0.dsize[0]; u++) {
				const Vec2 pix = getVec(u, v);
				const Vec2 flow = flows(u, v);

				const double angle = (flow.x != 0.0 || flow.y != 0.0) ? ::atan2(flow.x, flow.y) : 0.0;
				const double norm = normVec(flow) / 50.0;

				Col3 col;
				cnvHSVToCol(col, getVec(angle + SP_PI, minVal(1.0, norm), 1.0));

				img(u, v) = col;
			}
		}
		saveBMP(img, "opticalflowPM.bmp");
	}
	return 0;
}