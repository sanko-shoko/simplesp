#include "simplesp.h"

using namespace sp;

int main(){

	Mem2<Col3> img0, img1;
	SP_ASSERT(loadBMP(img0, SP_DATA_DIR  "/image/image000.bmp"));
	SP_ASSERT(loadBMP(img1, SP_DATA_DIR  "/image/image001.bmp"));
	for (int i = 0; i < img0.size(); i++) {
		img0[i].b = char(img0[i].b * 190 / 130);
		img1[i].b = char(img1[i].b * 190 / 130);
	}
	saveBMP(img0, "image0.bmp");
	saveBMP(img1, "image1.bmp");

	Mem2<Byte> mask(img0.dsize);

	Mem2<Byte> gry0, gry1;
	cnvImgToGry(gry0, img0);
	cnvImgToGry(gry1, img1);

	Mem2<Vec3> hsv0, hsv1;
	cnvImgToHSV(hsv0, img0);
	cnvImgToHSV(hsv1, img1);

	{
		const double thresh = 5.0 / 100.0;

		for (int i = 0; i < mask.size(); i++) {
			const double dif = ::fabs(gry0[i] - gry1[i]) / SP_BYTEMAX;
			mask[i] = (dif >= thresh) ? 255 : 0;
		}

		saveBMP(mask, "gry.bmp");
	}
	{
		const double thresh = 3.0 / 100.0;

		Mem2<short> tmp(mask.dsize);
		for (int i = 0; i < mask.size(); i++) {
			tmp[i] = gry0[i] - gry1[i];
		}
		gaussianFilter3x3(tmp, tmp);
		for (int i = 0; i < mask.size(); i++) {
			const double dif = ::fabs(tmp[i]) / SP_BYTEMAX;
			mask[i] = (dif >= thresh) ? 255 : 0;
		}

		saveBMP(mask, "gry_smooth.bmp");
	}

	{
		const double thresh = 5.0 / 100.0;

		for (int i = 0; i < mask.size(); i++) {
			const double r = ::fabs(img0[i].r - img1[i].r);
			const double g = ::fabs(img0[i].g - img1[i].g);
			const double b = ::fabs(img0[i].b - img1[i].b);

			const double dif = (r + g + b) / (3.0 * SP_BYTEMAX);
			mask[i] = (dif >= thresh) ? 255 : 0;
		}

		saveBMP(mask, "col.bmp");
	}

	{
		const double thresh = 3.0 / 100.0;

		Mem2<Vec3> tmp(mask.dsize);
		for (int i = 0; i < mask.size(); i++) {
			tmp[i].x = img0[i].r - img1[i].r;
			tmp[i].y = img0[i].g - img1[i].g;
			tmp[i].z = img0[i].b - img1[i].b;
		}
		gaussianFilter3x3<Vec3, double>(tmp, tmp);

		for (int i = 0; i < mask.size(); i++) {
			const double dif = (::fabs(tmp[i].x) + ::fabs(tmp[i].y) + ::fabs(tmp[i].z)) / (3.0 * SP_BYTEMAX);
			mask[i] = (dif >= thresh) ? 255 : 0;
		}

		saveBMP(mask, "col_smooth.bmp");
	}

	{
		const double thresh = 10.0 / 100.0;

		for (int i = 0; i < mask.size(); i++) {
			const Vec2 vec0 = getVec(::cos(hsv0[i].x), ::sin(hsv0[i].x)) * hsv0[i].y;
			const Vec2 vec1 = getVec(::cos(hsv1[i].x), ::sin(hsv1[i].x)) * hsv1[i].y;

			const double dif0 = normVec(vec0 - vec1);
			const double dif1 = ::fabs(hsv0[i].z - hsv1[i].z);

			const double dif = (dif0 + dif1) / 1.0;

			mask[i] = (dif >= thresh) ? 255 : 0;
		}
		saveBMP(mask, "hsv.bmp");
	}

	{
		const double thresh = 11.0 / 100.0;

		Mem2<Vec2> tmp0(mask.dsize);
		Mem2<double> tmp1(mask.dsize);
		for (int i = 0; i < mask.size(); i++) {
			const Vec2 vec0 = getVec(::cos(hsv0[i].x), ::sin(hsv0[i].x)) * hsv0[i].y;
			const Vec2 vec1 = getVec(::cos(hsv1[i].x), ::sin(hsv1[i].x)) * hsv1[i].y;
		
			tmp0[i] = vec0 - vec1;
			tmp1[i] = hsv0[i].z - hsv1[i].z;
		}

		gaussianFilter3x3<Vec2, double>(tmp0, tmp0);
		gaussianFilter3x3(tmp1, tmp1);

		for (int i = 0; i < mask.size(); i++) {

			const double dif0 = normVec(tmp0[i]);
			const double dif1 = ::fabs(tmp1[1]);

			const double dif = (dif0 * (1 + dif1)) / 1.0;
			//const double dif = (dif0 + dif1) / 1.0;

			mask[i] = (dif >= thresh) ? 255 : 0;
		}
		saveBMP(mask, "hsv_smooth.bmp");
	}

	{
		const double thresh = 5.0 / 100.0;

		const int kappa = 40;

		GraphCut gc(mask.size(), mask.size() * 4);

		for (int v = 0; v < mask.dsize[1]; v++) {
			for (int u = 0; u < mask.dsize[0]; u++) {
				const double r = ::fabs(img0(u, v).r - img1(u, v).r);
				const double g = ::fabs(img0(u, v).g - img1(u, v).g);
				const double b = ::fabs(img0(u, v).b - img1(u, v).b);

				const double dif = (r + g + b) / (3.0 * SP_BYTEMAX);

				int val = sp::round((dif - thresh) * SP_BYTEMAX);
				gc.setNode(acsid2(mask.dsize, u, v), val, 0);

				const int link[][2] = { { -1, 0 },{ 0, -1 },{ -1, -1 },{ +1, -1 } };
				for (int d = 0; d < 4; d++) {
					const int x = u + link[d][0];
					const int y = v + link[d][1];

					if (isInRect2(mask.dsize, x, y) == true) {
						gc.setLink(acsid2(mask.dsize, u, v), acsid2(mask.dsize, x, y), kappa);
					}
				}
			}
		}
		gc.execute();

		for (int i = 0; i < mask.size(); i++) {
			mask[i] = (gc.getLabel(i) > 0) ? 0 : 255;
		}
		saveBMP(mask, "mask2.bmp");
	}

	{
		const double thresh = 15.0 / 100.0;

		const int kappa = 40;

		GraphCut gc(mask.size(), mask.size() * 4);

		for (int v = 0; v < mask.dsize[1]; v++) {
			for (int u = 0; u < mask.dsize[0]; u++) {
				const Vec2 tmp0 = getVec(::cos(hsv0(u, v).x), ::sin(hsv0(u, v).x)) * hsv0(u, v).y;
				const Vec2 tmp1 = getVec(::cos(hsv1(u, v).x), ::sin(hsv1(u, v).x)) * hsv1(u, v).y;

				const double dif0 = normVec(tmp0 - tmp1);
				const double dif1 = 0 * ::fabs(hsv0(u, v).z - hsv1(u, v).z);

				const double dif = (dif0 + dif1) / 1.0;

				int val = sp::round((dif - thresh) * SP_BYTEMAX);
				gc.setNode(acsid2(mask.dsize, u, v), val, 0);

				const int link[][2] = { { -1, 0 },{ 0, -1 },{ -1, -1 },{ +1, -1 } };
				for (int d = 0; d < 4; d++) {
					const int x = u + link[d][0];
					const int y = v + link[d][1];

					if (isInRect2(mask.dsize, x, y) == true) {
						gc.setLink(acsid2(mask.dsize, u, v), acsid2(mask.dsize, x, y), kappa);
					}
				}
			}
		}
		gc.execute();

		for (int i = 0; i < mask.size(); i++) {
			mask[i] = (gc.getLabel(i) > 0) ? 0 : 255;
		}
		saveBMP(mask, "mask4.bmp");
	}


	return 0;
}