#include "simplesp.h"

using namespace sp;

int main(){

	//--------------------------------------------------------------------------------
	// sift (scale invariant feature transform)
	//--------------------------------------------------------------------------------

	// init input image
	Mem2<Col3> imgs[2];
	Mem2<Col3> imgM;
	{
		SP_ASSERT(loadBMP(imgs[0], SP_DATA_DIR  "/image/Lenna.bmp"));

		imgs[1].resize(imgs[0].dsize);
		setElm(imgs[1], getCol(127, 127, 127));

		double mat[3 * 3] = {
			+0.8000, -0.2000, +130.00,
			+0.2000, +0.8000, +50.000,
			+0.0002, +0.0002, +1.0000
		};
		const Mat hom(3, 3, mat);

		warp<Col3, Byte>(imgs[1], imgs[0], hom);
		
		merge(imgM, imgs[0], imgs[1]);
		saveBMP(imgM, "input.bmp");
	}

	SIFT sift[2];

	// detection and description
	{
		for (int i = 0; i < 2; i++) {
			sift[i].execute(imgs[i]);
		}
	}

	{
		for (int i = 0; i < 2; i++) {
			const Mem1<Feature> &fts = sift[i].getFeatrue();

			for (int j = 0; j < fts.size(); j++) {
				renderCircle(imgs[i], fts[j].pix, fts[j].scale, getCol(100, 255, 100), 1);
			}
		}

		merge(imgM, imgs[0], imgs[1]);
		saveBMP(imgM, "sift.bmp");
	}

	// matching
	{
		const Mem1<Feature> &fts0 = sift[0].getFeatrue();
		const Mem1<Feature> &fts1 = sift[1].getFeatrue();

		const Mem1<int> matches = findMatch(fts0, fts1);

		Mem1<Vec2> pixs0, pixs1;
		for (int i = 0; i < matches.size(); i++) {
			const int j = matches[i];
			if (j < 0) continue;

			pixs0.push(fts0[i].pix);
			pixs1.push(fts1[j].pix);
		}

		const int w = imgs[0].dsize[0];
		const int h = imgs[0].dsize[1];

		renderLine(imgM, pixs0, pixs1 + getVec(w, 0), getCol(50, 200, 200), 1);

		Mat hom;
		if (calcHMatRANSAC(hom, pixs1, pixs0) == true){
			const Vec2 pix[4] = { getVec(0.0, 0.0), getVec(w, 0.0), getVec(w, h), getVec(0.0, h) };
			for (int i = 0; i < 4; i++){
				const Vec2 p0 = pix[i] - getVec(0.5, 0.5);
				const Vec2 p1 = pix[(i + 1) % 4] - getVec(0.5, 0.5);
				renderLine(imgM, hom * p0 + getVec(w, 0), hom * p1 + getVec(w, 0), getCol(100, 200, 100), 2);
			}
			print(hom);
		}

		saveBMP(imgM, "match.bmp");
	}
	return 0;
}

