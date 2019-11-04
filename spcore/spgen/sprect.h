//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_RECT_H__
#define __SP_RECT_H__

#include "spcore/spcom.h"
#include "spcore/spgen/spbase.h"

namespace sp{

    //--------------------------------------------------------------------------------
    // rect
    //--------------------------------------------------------------------------------

    SP_GENFUNC Rect2 getRect2(const int dbase0, const int dbase1, const int dsize0, const int dsize1) {
        Rect2 rect;
        rect.dbase[0] = dbase0;
        rect.dbase[1] = dbase1;
        rect.dsize[0] = dsize0;
        rect.dsize[1] = dsize1;
        return rect;
    }

    SP_GENFUNC Rect2 getRect2(const int *dbase, const int *dsize) {
        return getRect2(dbase[0], dbase[1], dsize[0], dsize[1]);
    }

    SP_GENFUNC Rect2 getRect2(const int *dsize) {
        return getRect2(0, 0, dsize[0], dsize[1]);
    }

    SP_GENFUNC Rect2 getRect2(const Vec2 &vec) {
        return getRect2(round(vec.x), round(vec.y), 1, 1);
    }

    SP_GENFUNC Rect3 getRect3(const int dbase0, const int dbase1, const int dbase2, const int dsize0, const int dsize1, const int dsize2) {
        Rect3 rect;
        rect.dbase[0] = dbase0;
        rect.dbase[1] = dbase1;
        rect.dbase[2] = dbase2;
        rect.dsize[0] = dsize0;
        rect.dsize[1] = dsize1;
        rect.dsize[2] = dsize2;
        return rect;
    }

    SP_GENFUNC Rect3 getRect3(const int *dbase, const int *dsize) {
        return getRect3(dbase[0], dbase[1], dbase[2], dsize[0], dsize[1], dsize[2]);
    }

    SP_GENFUNC Rect3 getRect3(const int *dsize) {
        return getRect3(0, 0, 0, dsize[0], dsize[1], dsize[2]);
    }

    SP_GENFUNC Rect3 getRect3(const Vec3 &vec) {
        return getRect3(round(vec.x), round(vec.y), round(vec.z), 1, 1, 1);
    }


    //--------------------------------------------------------------------------------
    // check in rect
    //--------------------------------------------------------------------------------

    template<typename TYPE>
    SP_GENFUNC bool inRect(const Rect2 &rect, const TYPE *d) {
        for (int i = 0; i < 2; i++) {
            if (d[i] < static_cast<TYPE>(rect.dbase[i])) return false;
            if (d[i] > static_cast<TYPE>(rect.dbase[i] + rect.dsize[i] - 1)) return false;
        }
        return true;
    }
    template<typename TYPE>
    SP_GENFUNC bool inRect(const Rect3 &rect, const TYPE *d) {
        for (int i = 0; i < 3; i++) {
            if (d[i] < static_cast<TYPE>(rect.dbase[i])) return false;
            if (d[i] > static_cast<TYPE>(rect.dbase[i] + rect.dsize[i] - 1)) return false;
        }
        return true;
    }
    
    SP_GENFUNC bool inRect(const Rect2 &rect, const Rect2 &test) {
        for (int i = 0; i < 2; i++) {
            if (test.dbase[i] < rect.dbase[i]) return false;
            if (test.dbase[i] + test.dsize[i] > rect.dbase[i] + rect.dsize[i]) return false;
        }
        return true;
    }
    SP_GENFUNC bool inRect(const Rect3 &rect, const Rect3 &test) {
        for (int i = 0; i < 3; i++) {
            if (test.dbase[i] < rect.dbase[i]) return false;
            if (test.dbase[i] + test.dsize[i] > rect.dbase[i] + rect.dsize[i]) return false;
        }
        return true;
    }

    SP_GENFUNC bool inRect(const Rect2 &rect, const double d0, const double d1) {
        const double d[] = { d0, d1 };
        return inRect(rect, d);
    }

    SP_GENFUNC bool inRect(const int *dsize, const double d0, const double d1) {
        const double d[] = { d0, d1 };
        return inRect(getRect2(dsize), d);
    }

    SP_GENFUNC bool inRect(const Rect2 &rect, const Vec2 &vec) {
        const double d[] = { vec.x, vec.y };
        return inRect(rect, d);
    }

    SP_GENFUNC bool inRect(const int *dsize, const Vec2 &vec) {
        const double d[] = { vec.x, vec.y };
        return inRect(getRect2(dsize), d);
    }

    SP_GENFUNC bool inRect(const Rect3 &rect, const double d0, const double d1, const double d2) {
        const double d[] = { d0, d1, d2 };
        return inRect(rect, d);
    }

    SP_GENFUNC bool inRect(const int *dsize, const double d0, const double d1, const double d2) {
        const double d[] = { d0, d1, d2 };
        return inRect(getRect3(dsize), d);
    }

    SP_GENFUNC bool inRect(const Rect3 &rect, const Vec3 &vec) {
        const double d[] = { vec.x, vec.y, vec.z };
        return inRect(rect, d);
    }

    SP_GENFUNC bool inRect(const int *dsize, const Vec3 &vec) {
        const double d[] = { vec.x, vec.y, vec.z };
        return inRect(getRect3(dsize), d);
    }

    //--------------------------------------------------------------------------------
    // rect util
    //--------------------------------------------------------------------------------

    SP_GENFUNC Rect2 andRect(const Rect2 &rect0, const Rect2 &rect1) {
        int dbase[2] = { 0 }, dsize[2] = { 0 };
        for (int i = 0; i < 2; i++) {
            dbase[i] = maxVal(rect0.dbase[i], rect1.dbase[i]);
            dsize[i] = minVal(rect0.dbase[i] + rect0.dsize[i], rect1.dbase[i] + rect1.dsize[i]) - dbase[i];
        }
        return getRect2(dbase, dsize);
    }

    SP_GENFUNC Rect3 andRect(const Rect3 &rect0, const Rect3 &rect1) {
        int dbase[3] = { 0 }, dsize[3] = { 0 };
        for (int i = 0; i < 3; i++) {
            dbase[i] = maxVal(rect0.dbase[i], rect1.dbase[i]);
            dsize[i] = minVal(rect0.dbase[i] + rect0.dsize[i], rect1.dbase[i] + rect1.dsize[i]) - dbase[i];
        }
        return getRect3(dbase, dsize);
    }

    SP_GENFUNC Rect2 orRect(const Rect2 &rect0, const Rect2 &rect1) {
        int dbase[2] = { 0 }, dsize[2] = { 0 };
        for (int i = 0; i < 2; i++) {
            dbase[i] = minVal(rect0.dbase[i], rect1.dbase[i]);
            dsize[i] = maxVal(rect0.dbase[i] + rect0.dsize[i], rect1.dbase[i] + rect1.dsize[i]) - dbase[i];
            dsize[i] = maxVal(0, dsize[i]);
        }
        return getRect2(dbase, dsize);
    }

    SP_GENFUNC Rect3 orRect(const Rect3 &rect0, const Rect3 &rect1) {
        int dbase[3] = { 0 }, dsize[3] = { 0 };
        for (int i = 0; i < 3; i++) {
            dbase[i] = minVal(rect0.dbase[i], rect1.dbase[i]);
            dsize[i] = maxVal(rect0.dbase[i] + rect0.dsize[i], rect1.dbase[i] + rect1.dsize[i]) - dbase[i];
            dsize[i] = maxVal(0, dsize[i]);
        }
        return getRect3(dbase, dsize);
    }

    SP_GENFUNC Rect2 extRect(const Rect2 &rect, const int val) {
        int dbase[2] = { 0 }, dsize[2] = { 0 };
        for (int i = 0; i < 2; i++) {
            const int t = maxVal(val, -rect.dsize[i] / 2);
            dbase[i] = rect.dbase[i] - t;
            dsize[i] = rect.dsize[i] + 2 * t;
        }
        return getRect2(dbase, dsize);
    }

    SP_GENFUNC Rect3 extRect(const Rect3 &rect, const int val) {
        int dbase[3] = { 0 }, dsize[3] = { 0 };
        for (int i = 0; i < 3; i++) {
            const int t = maxVal(val, -rect.dsize[i] / 2);
            dbase[i] = rect.dbase[i] - t;
            dsize[i] = rect.dsize[i] + 2 * t;
        }
        return getRect3(dbase, dsize);
    }

    // get center vector
    SP_GENFUNC Vec2 getRectCent(const Rect2 &rect) {
        Vec2 vec;
        vec.x = rect.dbase[0] + (rect.dsize[0] - 1) / 2.0;
        vec.y = rect.dbase[1] + (rect.dsize[1] - 1) / 2.0;
        return vec;
    }

    // get center vector
    SP_GENFUNC Vec3 getRectCent(const Rect3 &rect) {
        Vec3 vec;
        vec.x = rect.dbase[0] + (rect.dsize[0] - 1) / 2.0;
        vec.y = rect.dbase[1] + (rect.dsize[1] - 1) / 2.0;
        vec.z = rect.dbase[2] + (rect.dsize[2] - 1) / 2.0;
        return vec;
    }

    //--------------------------------------------------------------------------------
    // rect operator
    //--------------------------------------------------------------------------------
    
    SP_GENFUNC Rect2 operator + (const Rect2 &rect, const int val) {
        return extRect(rect, +val);
    }
    SP_GENFUNC Rect3 operator + (const Rect3 &rect, const int val) {
        return extRect(rect, +val);
    }

    SP_GENFUNC Rect2 operator - (const Rect2 &rect, const int val) {
        return extRect(rect, -val);
    }
    SP_GENFUNC Rect3 operator - (const Rect3 &rect, const int val) {
        return extRect(rect, -val);
    }

    SP_GENFUNC void operator += (Rect2 &rect, const int val) {
        rect = extRect(rect, +val);
    }
    SP_GENFUNC void operator += (Rect3 &rect, const int val) {
        rect = extRect(rect, +val);
    }

    SP_GENFUNC void operator -= (Rect2 &rect, const int val) {
        rect = extRect(rect, -val);
    }
    SP_GENFUNC void operator -= (Rect3 &rect, const int val) {
        rect = extRect(rect, -val);
    }


}

#endif