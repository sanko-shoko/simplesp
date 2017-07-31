//--------------------------------------------------------------------------------
// Copyright (c) 2017, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_BLOCK_H__
#define __SP_BLOCK_H__

#include "spcore/spcore.h"
#include "spapp/spimg/spimage.h"

namespace sp {

	SP_CPUCALL double calcSAD(const Mem2<Byte> &img0, const Mem2<Byte> &img1, const int x, const int y, const Vec2 &flow, const int winSize) {

		const int offset = winSize / 2;
		const int wx0 = minVal(x, offset);
		const int wy0 = minVal(y, offset);
		const int wx1 = minVal(img0.dsize[0] - x, winSize - offset);
		const int wy1 = minVal(img0.dsize[1] - y, winSize - offset);

		double sad = 0;
		for (int wy = -wy0; wy <= wy1; wy++) {
			for (int wx = -wx0; wx <= wx1; wx++) {
				const double v0 = acs2(img0, x + wx, y + wy);
				const double v1 = acs2(img1, x + wx + flow.x, y + wy + flow.y);
				sad += fabs(v0 - v1);
			}
		}
		const int cnt = (wx1 + wx0 + 1) * (wy1 + wy0 + 1);
		const double eval = 1.0 - static_cast<double>(sad) / (SP_BYTEMAX * cnt);

		return eval;
	}

	SP_CPUCALL double calcZNCC(const Mem2<Byte> &img0, const Mem2<Byte> &img1, const int x, const int y, const Vec2 &flow, const int winSize) {

		const int offset = winSize / 2;
		const int wx0 = minVal(x, offset);
		const int wy0 = minVal(y, offset);
		const int wx1 = minVal(img0.dsize[0] - x, winSize - offset);
		const int wy1 = minVal(img0.dsize[1] - y, winSize - offset);

		double sum0 = 0.0;
		double sum1 = 0.0;
		for (int wy = -wy0; wy <= wy1; wy++) {
			for (int wx = -wx0; wx <= wx1; wx++) {
				const double v0 = acs2(img0, x + wx, y + wy);
				const double v1 = acs2(img1, x + wx + flow.x, y + wy + flow.y);
				sum0 += v0;
				sum1 += v1;
			}
		}

		const int cnt = (wx1 + wx0 + 1) * (wy1 + wy0 + 1);
		double mean0 = sum0 / cnt;
		double mean1 = sum1 / cnt;

		double m00 = 0.0;
		double m01 = 0.0;
		double m11 = 0.0;
		for (int wy = -wy0; wy <= wy1; wy++) {
			for (int wx = -wx0; wx <= wx1; wx++) {
				const double v0 = acs2(img0, x + wx, y + wy) - mean0;
				const double v1 = acs2(img1, x + wx + flow.x, y + wy + flow.y) - mean1;
				m00 += v0 * v0;
				m01 += v0 * v1;
				m11 += v1 * v1;
			}
		}
		const double div = sqrt(m00 * m11);
		const double zncc = (div > SP_SMALL) ? m01 / div : 0.0;

		return zncc;
	}

}

#endif