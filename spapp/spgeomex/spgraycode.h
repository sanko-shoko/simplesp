//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_GRAYCODE_H__
#define __SP_GRAYCODE_H__

#include "spcore/spcore.h"
#include "spapp/spimg/spimage.h"

namespace sp{

	class GrayCode {
		int m_dsize[2];

	public:
		GrayCode() {
			m_dsize[0] = 0;
			m_dsize[1] = 0;
		}

		void setSize(const int dsize0, const int dsize1) {
			m_dsize[0] = dsize0;
			m_dsize[1] = dsize1;
		}

		void genCodeH(Mem1<Mem2<Byte> > &himgs) {

		}

		void genCodeV(Mem1<Mem2<Byte> > &vimgs) {

		}

	};

}
#endif