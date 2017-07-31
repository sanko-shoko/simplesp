//--------------------------------------------------------------------------------
// Copyright (c) 2017, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

// [reference]
// Connelly Barnes, Eli Shechtman, Adam Finkelstein, and Dan B Goldman, 
// "PatchMatch: A Randomized Correspondence Algorithm for Structural Image Editing"
// ACM Transactions on Graphics, 2009

#ifndef __SP_PATCHMATCH_H__
#define __SP_PATCHMATCH_H__

#include "spcore/spcore.h"
#include "spapp/spimg/spimage.h"
#include "spapp/spimg/spblock.h"

namespace sp{

	SP_CPUCALL void opticalFlowPM(Mem2<Vec2> &flows, Mem2<bool> &masks, const Mem2<Byte> &img0, const Mem2<Byte> &img1, const int winSize, const int maxit = 6) {

		flows.resize(img0.dsize);
		flows.zero();

		masks.resize(img0.dsize);
		masks.zero();

		Mem2<double> evals(img0.dsize);
		evals.zero();

		srand(0);

		const Rect rect = getRect2(img0.dsize);

		double range = minVal(img0.dsize[0], img0.dsize[1]) * 0.5;

		for (int it = 0; it < maxit; it++) {

			// random search
			for (int v = 0; v < img0.dsize[1]; v++) {
				for (int u = 0; u < img0.dsize[0]; u++) {

					const Vec2 flow = flows(u, v) + randVecUnif(range, range);
					if (isInRect2(rect, u + round(flow.x), v + round(flow.y)) == false) continue;

					const double eval = calcSAD(img0, img1, u, v, flow, winSize);

					if (eval > evals(u, v)) {
						flows(u, v) = flow;
						evals(u, v) = eval;
					}
				}
			}

			const int prop = (it % 2 == 0) ? +1 : -1;

			// propagation
			for (int iv = 0; iv < img0.dsize[1]; iv++) {
				for (int iu = 0; iu < img0.dsize[0]; iu++) {
					const int u = (prop > 0) ? iu : img0.dsize[0] - iu - 1;
					const int v = (prop > 0) ? iv : img0.dsize[1] - iv - 1;

					if (evals(u - prop, v) > evals(u, v)) {
						const Vec2 &flow = flows(u - prop, v);
						if (isInRect2(rect, u + round(flow.x), v + round(flow.y)) == false) continue;
						const double eval = calcSAD(img0, img1, u, v, flow, winSize);
						if (eval > evals(u, v)) {
							flows(u, v) = flow;
							evals(u, v) = eval;
						}
					}
					if (evals(u, v - prop) > evals(u, v)) {
						const Vec2 &flow = flows(u, v - prop);
						if (isInRect2(rect, u + round(flow.x), v + round(flow.y)) == false) continue;
						const double eval = calcSAD(img0, img1, u, v, flow, winSize);
						if (eval > evals(u, v)) {
							flows(u, v) = flow;
							evals(u, v) = eval;
						}
					}
				}
			}
			range *= 0.5;
		}

	}

	SP_CPUCALL void opticalFlowPM(Mem2<Vec2> &flows, Mem2<bool> &masks, const Mem2<Col3> &img0, const Mem2<Col3> &img1, const int winSize, const int maxit = 6) {
		Mem2<Byte> gry0, gry1;
		cnvImg(gry0, img0);
		cnvImg(gry1, img1);

		opticalFlowPM(flows, masks, gry0, gry1, winSize, maxit);
	}
}

#endif