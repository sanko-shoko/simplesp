//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_CONTOUR_H__
#define __SP_CONTOUR_H__

#include "spcore/spcore.h"


namespace sp{

    SP_CPUFUNC bool snake(Mem1<Vec2> &contour, const Mem2<Byte> &img, const Rect &rect, const Mem1<Vec2> &vtxs) {




        return true;
    }
    SP_CPUFUNC bool snake(Mem1<Vec2> &contour, const Mem2<Byte> &img, const Rect &rect) {

        Mem1<Vec2> vtxs;
        vtxs.push(getVec(rect.dbase[0], rect.dbase[1]));
        vtxs.push(getVec(rect.dbase[0], rect.dbase[1] + rect.dsize[1]));
        vtxs.push(getVec(rect.dbase[0] + rect.dsize[0], rect.dbase[1] + rect.dsize[1]));
        vtxs.push(getVec(rect.dbase[0] + rect.dsize[0], rect.dbase[1]));

        return true;
    }
}

#endif