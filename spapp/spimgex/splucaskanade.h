//--------------------------------------------------------------------------------
// Copyright (c) 2017, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

// [reference]
// BD.Lucas, T.Kanade,
// "An iterative image registration technique with an application to stereo vision",
// Proceedings of Imaging Understanding Workshop, 1981

#ifndef __SP_LUCASKANADE_H__
#define __SP_LUCASKANADE_H__

#include "spcore/spcore.h"
#include "spapp/spimg/spimage.h"
#include "spapp/spimg/spfilter.h"

namespace sp{

	SP_CPUCALL void opticalFlowLK(Mem1<Vec2> &flows, Mem1<bool> &masks, const Mem2<Byte> &img0, const Mem2<Byte> &img1, const Mem1<Vec2> &pixs) {
		SP_LOGGER_INSTANCE;
		SP_LOGGER_SET("opticalFlowLK");

		const int WIN_SIZE = 21;
		const double EIG_THRESH = 0.01;

		flows.resize(pixs.size());
		flows.zero();

		masks.resize(pixs.size());
		masks.zero();

		const int pysize = round(log2(minVal(img0.dsize[0], img0.dsize[1]) / 20.0));

		Mem1<Mem2<Byte> > pyimg0s(pysize);
		Mem1<Mem2<Byte> > pyimg1s(pysize);

		for (int p = 0; p < pysize; p++) {
			if (p == 0) {
				pyimg0s[p] = img0;
				pyimg1s[p] = img1;
			}
			else {
				pyrdown(pyimg0s[p], pyimg0s[p - 1]);
				pyrdown(pyimg1s[p], pyimg1s[p - 1]);
			}
		}

		for(int p = pysize - 1; p >= 0; p--){
			const Mem2<Byte> &pyimg0 = pyimg0s[p];
			const Mem2<Byte> &pyimg1 = pyimg1s[p];

			const double scale = pow(0.5, p);

			Mem2<float> scharrX, scharrY;
			scharrFilterX(scharrX, pyimg0);
			scharrFilterY(scharrY, pyimg0);

			const Rect rect = getRect2(img1.dsize);
			const int offset = WIN_SIZE / 2;

			for (int i = 0; i < pixs.size(); i++) {

				// Ai = [dI/dx, dI/dy], A = [A0, A1, ... An-1]^T
				Mat A(WIN_SIZE * WIN_SIZE, 2);
				Mat I(WIN_SIZE * WIN_SIZE, 1);
				Mat AtA(2, 2);
				Mat AtB(2, 1);

				{
					AtA.zero();
					
					const Vec2 pix0 = pixs[i] * scale;

					for (int y = 0; y < WIN_SIZE; y++) {
						for (int x = 0; x < WIN_SIZE; x++) {
							const int i = y * WIN_SIZE + x;
							const Vec2 v = getVec(x - offset, y - offset);

							const Vec2 p0 = pix0 + v;
							if (isInRect2(rect, p0.x, p0.y) == false) continue;

							const double gx = acs2(scharrX, p0.x, p0.y) / SP_BYTEMAX;
							const double gy = acs2(scharrY, p0.x, p0.y) / SP_BYTEMAX;
							
							A(i, 0) = gx;
							A(i, 1) = gy;

							I(i, 0) = acs2(pyimg0, p0.x, p0.y);

							AtA(0, 0) += gx * gx;
							AtA(0, 1) += gx * gy;
							AtA(1, 0) += gy * gx;
							AtA(1, 1) += gy * gy;
						}
					}

					const double a = AtA(0, 0);
					const double b = AtA(0, 1);
					const double c = AtA(1, 1);
					const double D = a * c - b * b;

					const double mineig = (a + c - sqrt((a - c) * (a - c) + 4.0 * b * b)) / 2.0;

					if (mineig / (WIN_SIZE * WIN_SIZE) < square(EIG_THRESH)) continue;
					if (fabs(D) < SP_SMALL) continue;
				}

				const Mat invAtA = invMat(AtA);

				const int maxit = 2;
				for (int it = 0; it < maxit; it++) {
					AtB.zero();

					const Vec2 pix0 = pixs[i] * scale;
					const Vec2 pix1 = (pixs[i] + flows[i]) * scale;

					for (int y = 0; y < WIN_SIZE; y++) {
						for (int x = 0; x < WIN_SIZE; x++) {
							const int i = y * WIN_SIZE + x;
							const Vec2 v = getVec(x - offset, y - offset);

							const Vec2 p0 = pix0 + v;
							const Vec2 p1 = pix1 + v;
							if (isInRect2(rect, p0.x, p0.y) == false) continue;

							const double gx = A(i, 0);
							const double gy = A(i, 1);

							const double d = (I(i, 0) - acs2(pyimg1, p1.x, p1.y)) / SP_BYTEMAX;
							AtB(0, 0) += gx * d;
							AtB(1, 0) += gy * d;
						}
					}

					const Mat result = invAtA * AtB;
					if (result.size() == 0) continue;

					const double limit = 2.0;
					Vec2 delta = getVec(result[0], result[1]);
					delta.x = (delta.x > 0) ? minVal(+limit, delta.x) : maxVal(-limit, delta.x);
					delta.y = (delta.y > 0) ? minVal(+limit, delta.y) : maxVal(-limit, delta.y);

					flows[i] += delta / scale;

					if (p == 0) {
						masks[i] = true;
					}
				}
			}
		}
	}

	SP_CPUCALL void opticalFlowLK(Mem1<Vec2> &flows, Mem1<bool> &masks, const Mem2<Col3> &img0, const Mem2<Col3> &img1, const Mem1<Vec2> &pixs) {
		Mem2<Byte> gry0, gry1;
		cnvImg(gry0, img0);
		cnvImg(gry1, img1);

		opticalFlowLK(flows, masks, gry0, gry1, pixs);
	}
}

#endif