//--------------------------------------------------------------------------------
// Copyright (c) 2017, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_STEREO_H__
#define __SP_STEREO_H__

#include "spcore/spcore.h"
#include "spapp/spimg/spimage.h"

namespace sp{

	SP_CPUFUNC void calcDisparity(Mem2<float> &dspL, Mem2<float> &dspR, const Mem2<Byte> &imgL, const Mem2<Byte> &imgR, const double mindisp, const double maxdisp){

		dspL.resize(imgL.dsize);
		dspL.zero();

		dspR.resize(imgR.dsize);
		dspR.zero();


		//srand(0);

		//const Rect rect = getRect2(img0.dsize);

		//double range = minVal(img0.dsize[0], img0.dsize[1]) * 0.5;

		//for (int it = 0; it < maxit; it++) {

		//	// random search
		//	for (int v = 0; v < img0.dsize[1]; v++) {
		//		for (int u = 0; u < img0.dsize[0]; u++) {

		//			const Vec2 flow = flows(u, v) + randVecUnif(range, range);
		//			if (isInRect2(rect, u + round(flow.x), v + round(flow.y)) == false) continue;

		//			const double eval = calcSAD(img0, img1, u, v, flow, winSize);

		//			if (eval > evals(u, v)) {
		//				flows(u, v) = flow;
		//				evals(u, v) = eval;
		//			}
		//		}
		//	}

		//	const int prop = (it % 2 == 0) ? +1 : -1;

		//	// propagation
		//	for (int iv = 0; iv < img0.dsize[1]; iv++) {
		//		for (int iu = 0; iu < img0.dsize[0]; iu++) {
		//			const int u = (prop > 0) ? iu : img0.dsize[0] - iu - 1;
		//			const int v = (prop > 0) ? iv : img0.dsize[1] - iv - 1;

		//			if (evals(u - prop, v) > evals(u, v)) {
		//				const Vec2 &flow = flows(u - prop, v);
		//				if (isInRect2(rect, u + round(flow.x), v + round(flow.y)) == false) continue;
		//				const double eval = calcSAD(img0, img1, u, v, flow, winSize);
		//				if (eval > evals(u, v)) {
		//					flows(u, v) = flow;
		//					evals(u, v) = eval;
		//				}
		//			}
		//			if (evals(u, v - prop) > evals(u, v)) {
		//				const Vec2 &flow = flows(u, v - prop);
		//				if (isInRect2(rect, u + round(flow.x), v + round(flow.y)) == false) continue;
		//				const double eval = calcSAD(img0, img1, u, v, flow, winSize);
		//				if (eval > evals(u, v)) {
		//					flows(u, v) = flow;
		//					evals(u, v) = eval;
		//				}
		//			}
		//		}
		//	}
		//	range *= 0.5;
		//}
	}

}
#endif