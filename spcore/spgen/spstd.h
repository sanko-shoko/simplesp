//--------------------------------------------------------------------------------
// Copyright (c) 2017-2020, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_STD_H__
#define __SP_STD_H__

#include "spcore/spcom.h"
#include "spcore/spgen/spbase.h"
#include "spcore/spgen/spmath.h"

//--------------------------------------------------------------------------------
// vertex
//--------------------------------------------------------------------------------

namespace sp {

    //--------------------------------------------------------------------------------
    // vector
    //--------------------------------------------------------------------------------

    // get vector
    SP_GENFUNC Vec2 getVec2(const double x, const double y) {
        Vec2 dst;
        dst.x = static_cast<SP_REAL>(x);
        dst.y = static_cast<SP_REAL>(y);
        return dst;
    }

    // get vector
    SP_GENFUNC Vec3 getVec3(const double x, const double y, const double z) {
        Vec3 dst;
        dst.x = static_cast<SP_REAL>(x);
        dst.y = static_cast<SP_REAL>(y);
        dst.z = static_cast<SP_REAL>(z);
        return dst;
    }

    // get vector
    SP_GENFUNC Vec3 getVec3(const Vec2 &vec, const double z) {
        return getVec3(vec.x, vec.y, z);
    }
    
    // random uniform
    SP_CPUFUNC Vec2 randuVec2(const double x, const double y) {
        return getVec2(randu() * x, randu() * y);
    }
    // random uniform
    SP_CPUFUNC Vec2 randuVec2(const double x, const double y, const int seed) {
        const unsigned int s0 = static_cast<unsigned int>(seed);
        const unsigned int s1 = _snext(s0);
        return getVec2(randu(s0) * x, randu(s1) * y);
    }

    // random gauss
    SP_CPUFUNC Vec2 randgVec2(const double x, const double y) {
        return getVec2(randg() * x, randg() * y);
    }
    // random gauss
    SP_CPUFUNC Vec2 randgVec2(const double x, const double y, const int seed) {
        const unsigned int s0 = static_cast<unsigned int>(seed);
        const unsigned int s1 = _snext(s0);
        return getVec2(randg(s0) * x, randg(s1) * y);
    }

    // random uniform
    SP_CPUFUNC Vec3 randuVec3(const double x, const double y, const double z) {
        return getVec3(randu() * x, randu() * y, randu() * z);
    }
    // random uniform
    SP_CPUFUNC Vec3 randuVec3(const double x, const double y, const double z, const int seed) {
        const unsigned int s0 = static_cast<unsigned int>(seed);
        const unsigned int s1 = _snext(s0);
        const unsigned int s2 = _snext(s1);
        return getVec3(randu(s0) * x, randu(s1) * y, randu(s2) * z);
    }
    
    // random gauss
    SP_CPUFUNC Vec3 randgVec3(const double x, const double y, const double z) {
        return getVec3(randg() * x, randg() * y, randg() * z);
    }
    // random gauss
    SP_CPUFUNC Vec3 randgVec3(const double x, const double y, const double z, const int seed) {
        const unsigned int s0 = static_cast<unsigned int>(seed);
        const unsigned int s1 = _snext(s0);
        const unsigned int s2 = _snext(s1);
        return getVec3(randg(s0) * x, randg(s1) * y, randg(s2) * z);
    }

    //--------------------------------------------------------------------------------
    // vector operator
    //--------------------------------------------------------------------------------
    
    SP_GENFUNC Vec2 addVec(const Vec2 &vec0, const Vec2 &vec1) { return getVec2(vec0.x + vec1.x, vec0.y + vec1.y); }
    SP_GENFUNC Vec3 addVec(const Vec3 &vec0, const Vec3 &vec1) { return getVec3(vec0.x + vec1.x, vec0.y + vec1.y, vec0.z + vec1.z);  }
    SP_GENFUNC Vec2 addVec(const Vec2 &vec, const double val) { return getVec2(vec.x + val, vec.y + val); }
    SP_GENFUNC Vec3 addVec(const Vec3 &vec, const double val) { return getVec3(vec.x + val, vec.y + val, vec.z + val); }
    SP_GENFUNC Vec2 addVec(const double val, const Vec2 &vec) { return getVec2(val + vec.x, val + vec.y); }
    SP_GENFUNC Vec3 addVec(const double val, const Vec3 &vec) { return getVec3(val + vec.x, val + vec.y, val + vec.z); }
    
    SP_GENFUNC Vec2 subVec(const Vec2 &vec0, const Vec2 &vec1) { return getVec2(vec0.x - vec1.x, vec0.y - vec1.y); }
    SP_GENFUNC Vec3 subVec(const Vec3 &vec0, const Vec3 &vec1) { return getVec3(vec0.x - vec1.x, vec0.y - vec1.y, vec0.z - vec1.z); }
    SP_GENFUNC Vec2 subVec(const Vec2 &vec, const double val) { return getVec2(vec.x - val, vec.y - val); }
    SP_GENFUNC Vec3 subVec(const Vec3 &vec, const double val) { return getVec3(vec.x - val, vec.y - val, vec.z - val); }
    SP_GENFUNC Vec2 subVec(const double val, const Vec2 &vec) { return getVec2(val - vec.x, val - vec.y); }
    SP_GENFUNC Vec3 subVec(const double val, const Vec3 &vec) { return getVec3(val - vec.x, val - vec.y, val - vec.z); }

    SP_GENFUNC Vec2 mulVec(const Vec2 &vec0, const Vec2 &vec1) { return getVec2(vec0.x * vec1.x, vec0.y * vec1.y); }
    SP_GENFUNC Vec3 mulVec(const Vec3 &vec0, const Vec3 &vec1) { return getVec3(vec0.x * vec1.x, vec0.y * vec1.y, vec0.z * vec1.z); }
    SP_GENFUNC Vec2 mulVec(const Vec2 &vec, const double val) { return getVec2(vec.x * val, vec.y * val); }
    SP_GENFUNC Vec3 mulVec(const Vec3 &vec, const double val) { return getVec3(vec.x * val, vec.y * val, vec.z * val); }
    SP_GENFUNC Vec2 mulVec(const double val, const Vec2 &vec) { return getVec2(val * vec.x, val * vec.y);  }
    SP_GENFUNC Vec3 mulVec(const double val, const Vec3 &vec) { return getVec3(val * vec.x, val * vec.y, val * vec.z); }

    SP_GENFUNC Vec2 divVec(const Vec2 &vec0, const Vec2 &vec1) { SP_ASSERT(fabs(vec1.x) > SP_SMALL && fabs(vec1.y) > SP_SMALL); return getVec2(vec0.x / vec1.x, vec0.y / vec1.y); }
    SP_GENFUNC Vec3 divVec(const Vec3 &vec0, const Vec3 &vec1) { SP_ASSERT(fabs(vec1.x) > SP_SMALL && fabs(vec1.y) > SP_SMALL && fabs(vec1.z) > SP_SMALL); return getVec3(vec0.x / vec1.x, vec0.y / vec1.y, vec0.z / vec1.z); }
    SP_GENFUNC Vec2 divVec(const Vec2 &vec, const double val) { SP_ASSERT(fabs(val) > SP_SMALL); return getVec2(vec.x / val, vec.y / val); }
    SP_GENFUNC Vec3 divVec(const Vec3 &vec, const double val) { SP_ASSERT(fabs(val) > SP_SMALL); return getVec3(vec.x / val, vec.y / val, vec.z / val); }
    SP_GENFUNC Vec2 divVec(const double val, const Vec2 &vec) { SP_ASSERT(fabs(vec.x) > SP_SMALL && fabs(vec.y) > SP_SMALL); return getVec2(val / vec.x, val / vec.y); }
    SP_GENFUNC Vec3 divVec(const double val, const Vec3 &vec) { SP_ASSERT(fabs(vec.x) > SP_SMALL && fabs(vec.y) > SP_SMALL && fabs(vec.z) > SP_SMALL); return getVec3(val / vec.x, val / vec.y, val / vec.z); }

    SP_GENFUNC Vec2 operator + (const Vec2 &vec0, const Vec2 &vec1) { return addVec(vec0, vec1); }
    SP_GENFUNC Vec3 operator + (const Vec3 &vec0, const Vec3 &vec1) { return addVec(vec0, vec1); }
    SP_GENFUNC Vec2 operator + (const Vec2 &vec, const double &val) { return addVec(vec, val); }
    SP_GENFUNC Vec3 operator + (const Vec3 &vec, const double &val) { return addVec(vec, val); }
    SP_GENFUNC Vec2 operator + (const double &val, const Vec2 &vec) { return addVec(val, vec); }
    SP_GENFUNC Vec3 operator + (const double &val, const Vec3 &vec) { return addVec(val, vec); }
    SP_GENFUNC void operator += (Vec2 &vec0, const Vec2 &vec1) { vec0 = addVec(vec0, vec1); }
    SP_GENFUNC void operator += (Vec3 &vec0, const Vec3 &vec1) { vec0 = addVec(vec0, vec1); }
    SP_GENFUNC Vec2 operator + (const Vec2 &vec) { return vec; }
    SP_GENFUNC Vec3 operator + (const Vec3 &vec) { return vec; }

    SP_GENFUNC Vec2 operator - (const Vec2 &vec0, const Vec2 &vec1) { return subVec(vec0, vec1); }
    SP_GENFUNC Vec3 operator - (const Vec3 &vec0, const Vec3 &vec1) { return subVec(vec0, vec1); }
    SP_GENFUNC Vec2 operator - (const Vec2 &vec, const double &val) { return subVec(vec, val); }
    SP_GENFUNC Vec3 operator - (const Vec3 &vec, const double &val) { return subVec(vec, val); }
    SP_GENFUNC Vec2 operator - (const double &val, const Vec2 &vec) { return subVec(val, vec); }
    SP_GENFUNC Vec3 operator - (const double &val, const Vec3 &vec) { return subVec(val, vec); }
    SP_GENFUNC void operator -= (Vec2 &vec0, const Vec2 &vec1) { vec0 = subVec(vec0, vec1); }
    SP_GENFUNC void operator -= (Vec3 &vec0, const Vec3 &vec1) { vec0 = subVec(vec0, vec1); }
    SP_GENFUNC Vec2 operator - (const Vec2 &vec) { return mulVec(vec, -1.0); }
    SP_GENFUNC Vec3 operator - (const Vec3 &vec) { return mulVec(vec, -1.0); }

    SP_GENFUNC Vec2 operator * (const Vec2 &vec0, const Vec2 &vec1) { return mulVec(vec0, vec1); }
    SP_GENFUNC Vec3 operator * (const Vec3 &vec0, const Vec3 &vec1) { return mulVec(vec0, vec1); }
    SP_GENFUNC Vec2 operator * (const Vec2 &vec, const double val) { return mulVec(vec, val); }
    SP_GENFUNC Vec3 operator * (const Vec3 &vec, const double val) { return mulVec(vec, val); }
    SP_GENFUNC Vec2 operator * (const double val, const Vec2 &vec) { return mulVec(vec, val); }
    SP_GENFUNC Vec3 operator * (const double val, const Vec3 &vec) { return mulVec(vec, val); }
    SP_GENFUNC void operator *= (Vec2 &vec, const double val) { vec = mulVec(vec, val); }
    SP_GENFUNC void operator *= (Vec3 &vec, const double val) { vec = mulVec(vec, val); }

    SP_GENFUNC Vec2 operator / (const Vec2 &vec0, const Vec2 &vec1) { return divVec(vec0, vec1); }
    SP_GENFUNC Vec3 operator / (const Vec3 &vec0, const Vec3 &vec1) { return divVec(vec0, vec1); }
    SP_GENFUNC Vec2 operator / (const Vec2 &vec, const double val) { return divVec(vec, val); }
    SP_GENFUNC Vec3 operator / (const Vec3 &vec, const double val) { return divVec(vec, val); }
    SP_GENFUNC Vec2 operator / (const double val, const Vec2 &vec) { return divVec(val, vec); }
    SP_GENFUNC Vec3 operator / (const double val, const Vec3 &vec) { return divVec(val, vec); }
    SP_GENFUNC void operator /= (Vec2 &vec, const double val) { vec = divVec(vec, val); }
    SP_GENFUNC void operator /= (Vec3 &vec, const double val) { vec = divVec(vec, val); }

    //--------------------------------------------------------------------------------
    // vector util
    //--------------------------------------------------------------------------------

    // dot production
    SP_GENFUNC SP_REAL dotVec(const Vec2 &vec0, const Vec2 &vec1) {
        return vec0.x * vec1.x + vec0.y * vec1.y;
    }
    // dot production
    SP_GENFUNC SP_REAL dotVec(const Vec3 &vec0, const Vec3 &vec1) {
        return vec0.x * vec1.x + vec0.y * vec1.y + vec0.z * vec1.z;
    }

    // cross production
    SP_GENFUNC SP_REAL crsVec(const Vec2 &vec0, const Vec2 &vec1) {
        return vec0.x * vec1.y - vec0.y * vec1.x;
    }
    // cross production
    SP_GENFUNC Vec3 crsVec(const Vec3 &vec0, const Vec3 &vec1) {
        return getVec3(vec0.y * vec1.z - vec0.z * vec1.y, vec0.z * vec1.x - vec0.x * vec1.z, vec0.x * vec1.y - vec0.y * vec1.x);
    }

    // projection vec3 to vec2
    SP_GENFUNC Vec2 prjVec(const Vec3 &vec, const bool pers = true) {
        return (pers == true) ? getVec2(vec.x, vec.y) / vec.z : getVec2(vec.x, vec.y);
    }
    // projection vec2 to vec3
    SP_GENFUNC Vec3 prjVec(const Vec2 &vec, const double z, const bool pers = true) {
        return (pers == true) ? getVec3(vec.x, vec.y, 1.0) * z : getVec3(vec.x, vec.y, z);
    }

    // sq
    SP_GENFUNC SP_REAL sqVec(const Vec2 &vec) {
        return dotVec(vec, vec);
    }
    // sq
    SP_GENFUNC SP_REAL sqVec(const Vec3 &vec) {
        return dotVec(vec, vec);
    }

    // norm
    SP_GENFUNC SP_REAL normVec(const Vec2 &vec) {
        return sqrt(dotVec(vec, vec));
    }
    // norm
    SP_GENFUNC SP_REAL normVec(const Vec3 &vec) {
        return sqrt(dotVec(vec, vec));
    }

    // unit vector
    SP_GENFUNC Vec2 unitVec(const Vec2 &vec) {
        const double norm = normVec(vec);
        return (norm > SP_SMALL) ? vec / norm : getVec2(0.0, 0.0);
    }
    // unit vector
    SP_GENFUNC Vec3 unitVec(const Vec3 &vec) {
        const double norm = normVec(vec);
        return (norm > SP_SMALL) ? vec / norm : getVec3(0.0, 0.0, 0.0);
    }

    // round
    SP_GENFUNC Vec2 roundVec(const Vec2 &vec) {
        return getVec2(round(vec.x), round(vec.y));
    }
    // round
    SP_GENFUNC Vec3 roundVec(const Vec3 &vec) {
        return getVec3(round(vec.x), round(vec.y), round(vec.z));
    }

    //--------------------------------------------------------------------------------
    // matrix * vector
    //--------------------------------------------------------------------------------

    SP_GENFUNC Vec2 mulMat(const SP_REAL *mat, const int rows, const int cols, const Vec2 &vec) {
        Vec2 dst = getVec2(0.0, 0.0);
        if (rows == 2 && cols == 2) {
            dst.x = static_cast<SP_REAL>(mat[0 * 2 + 0] * vec.x + mat[0 * 2 + 1] * vec.y);
            dst.y = static_cast<SP_REAL>(mat[1 * 2 + 0] * vec.x + mat[1 * 2 + 1] * vec.y);
        }
        if (rows == 2 && cols == 3) {
            dst.x = static_cast<SP_REAL>(mat[0 * 3 + 0] * vec.x + mat[0 * 3 + 1] * vec.y + mat[0 * 3 + 2]);
            dst.y = static_cast<SP_REAL>(mat[1 * 3 + 0] * vec.x + mat[1 * 3 + 1] * vec.y + mat[1 * 3 + 2]);
        }
        if (rows == 3 && cols == 3) {
            const double scale = mat[2 * 3 + 0] * vec.x + mat[2 * 3 + 1] * vec.y + mat[2 * 3 + 2];
            dst.x = static_cast<SP_REAL>((mat[0 * 3 + 0] * vec.x + mat[0 * 3 + 1] * vec.y + mat[0 * 3 + 2]) / scale);
            dst.y = static_cast<SP_REAL>((mat[1 * 3 + 0] * vec.x + mat[1 * 3 + 1] * vec.y + mat[1 * 3 + 2]) / scale);
        }
        if (rows == 3 && cols == 4) {
            dst.x = static_cast<SP_REAL>(mat[0 * 4 + 0] * vec.x + mat[0 * 4 + 1] * vec.y + mat[0 * 4 + 2] * 0.0 + mat[0 * 4 + 3]);
            dst.y = static_cast<SP_REAL>(mat[1 * 4 + 0] * vec.x + mat[1 * 4 + 1] * vec.y + mat[1 * 4 + 2] * 0.0 + mat[1 * 4 + 3]);
        }
        if (rows == 4 && cols == 4) {
            const double scale = mat[3 * 4 + 0] * vec.x + mat[3 * 4 + 1] * vec.y + mat[3 * 4 + 2] * 0.0 + mat[3 * 4 + 3];
            dst.x = static_cast<SP_REAL>((mat[0 * 4 + 0] * vec.x + mat[0 * 4 + 1] * vec.y + mat[0 * 4 + 2] * 0.0 + mat[0 * 4 + 3]) / scale);
            dst.y = static_cast<SP_REAL>((mat[1 * 4 + 0] * vec.x + mat[1 * 4 + 1] * vec.y + mat[1 * 4 + 2] * 0.0 + mat[1 * 4 + 3]) / scale);
        }
        return dst;
    }

    SP_GENFUNC Vec3 mulMat(const SP_REAL *mat, const int rows, const int cols, const Vec3 &vec) {
        Vec3 dst = getVec3(0.0, 0.0, 0.0);
        if (rows == 3 && cols == 3) {
            dst.x = static_cast<SP_REAL>(mat[0 * 3 + 0] * vec.x + mat[0 * 3 + 1] * vec.y + mat[0 * 3 + 2] * vec.z);
            dst.y = static_cast<SP_REAL>(mat[1 * 3 + 0] * vec.x + mat[1 * 3 + 1] * vec.y + mat[1 * 3 + 2] * vec.z);
            dst.z = static_cast<SP_REAL>(mat[2 * 3 + 0] * vec.x + mat[2 * 3 + 1] * vec.y + mat[2 * 3 + 2] * vec.z);
        }
        if (rows == 3 && cols == 4) {
            dst.x = static_cast<SP_REAL>(mat[0 * 4 + 0] * vec.x + mat[0 * 4 + 1] * vec.y + mat[0 * 4 + 2] * vec.z + mat[0 * 4 + 3]);
            dst.y = static_cast<SP_REAL>(mat[1 * 4 + 0] * vec.x + mat[1 * 4 + 1] * vec.y + mat[1 * 4 + 2] * vec.z + mat[1 * 4 + 3]);
            dst.z = static_cast<SP_REAL>(mat[2 * 4 + 0] * vec.x + mat[2 * 4 + 1] * vec.y + mat[2 * 4 + 2] * vec.z + mat[2 * 4 + 3]);
        }
        if (rows == 4 && cols == 4) {
            const double scale = mat[3 * 4 + 0] * vec.x + mat[3 * 4 + 1] * vec.y + mat[3 * 4 + 2] * vec.z + mat[3 * 4 + 3];
            dst.x = static_cast<SP_REAL>((mat[0 * 4 + 0] * vec.x + mat[0 * 4 + 1] * vec.y + mat[0 * 4 + 2] * vec.z + mat[0 * 4 + 3]) / scale);
            dst.y = static_cast<SP_REAL>((mat[1 * 4 + 0] * vec.x + mat[1 * 4 + 1] * vec.y + mat[1 * 4 + 2] * vec.z + mat[1 * 4 + 3]) / scale);
            dst.z = static_cast<SP_REAL>((mat[2 * 4 + 0] * vec.x + mat[2 * 4 + 1] * vec.y + mat[2 * 4 + 2] * vec.z + mat[2 * 4 + 3]) / scale);
        }
        return dst;
    }


    //--------------------------------------------------------------------------------
    // vector pn (position and direction)
    //--------------------------------------------------------------------------------

    // get vector pd
    SP_GENFUNC VecPD2 getVecPD2(const Vec2 &vtx, const Vec2 &drc) {
        VecPD2 dst;
        dst.pos = vtx;
        dst.drc = drc;
        return dst;
    }

    // get vector pd
    SP_GENFUNC VecPD3 getVecPD3(const Vec3 &vtx, const Vec3 &drc) {
        VecPD3 dst;
        dst.pos = vtx;
        dst.drc = drc;
        return dst;
    }

    //--------------------------------------------------------------------------------
    // matrix * vector pn
    //--------------------------------------------------------------------------------

    SP_GENFUNC VecPD2 mulMat(const SP_REAL *mat, const int rows, const int cols, const VecPD2 &vec) {
        VecPD2 dst;

        dst.pos = mulMat(mat, rows, cols, vec.pos);

        SP_REAL rot[2 * 2] = { 0 };
        {
            for (int r = 0; r < 2; r++) {
                for (int c = 0; c < 2; c++) {
                    rot[r * 2 + c] = mat[r * cols + c];
                }
            }
        }
        if (rows == 3 && cols == 3) {
            const SP_REAL pos[2] = { dst.pos.x, dst.pos.y };
            for (int r = 0; r < 2; r++) {
                for (int c = 0; c < 2; c++) {
                    rot[r * 2 + c] -= mat[2 * cols + c] * pos[r];
                }
            }
        }

        dst.drc = unitVec(mulMat(rot, 2, 2, vec.drc));

        return dst;
    }

    SP_GENFUNC VecPD3 mulMat(const SP_REAL *mat, const int rows, const int cols, const VecPD3 &vec) {
        VecPD3 dst;

        dst.pos = mulMat(mat, rows, cols, vec.pos);

        SP_REAL rot[3 * 3] = { 0 };
        {
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    rot[r * 3 + c] = mat[r * cols + c];
                }
            }
        }
        if (rows == 4 && cols == 4) {
            const SP_REAL pos[3] = { dst.pos.x, dst.pos.y, dst.pos.z };
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    rot[r * 3 + c] -= mat[3 * cols + c] * pos[r];
                }
            }
        }

        dst.drc = unitVec(mulMat(rot, 3, 3, vec.drc));

        return dst;
    }

    //--------------------------------------------------------------------------------
    // extract z element
    //--------------------------------------------------------------------------------

    SP_GENFUNC SP_REAL extractZ(const VecPD3 &src) {
        return src.pos.z;
    }

    SP_GENFUNC SP_REAL extractZ(const Vec3 &src) {
        return src.z;
    }

    SP_GENFUNC SP_REAL extractZ(const double src) {
        return src;
    }


    //--------------------------------------------------------------------------------
    // line
    //--------------------------------------------------------------------------------
    
    SP_GENFUNC Line2 getLine2(const Vec2 &vec0, const Vec2 &vec1) {
        Line2 dst;
        dst.pos[0] = vec0;
        dst.pos[1] = vec1;
        return dst;
    }

    SP_GENFUNC Line3 getLine3(const Vec3 &vec0, const Vec3 &vec1) {
        Line3 dst;
        dst.pos[0] = vec0;
        dst.pos[1] = vec1;
        return dst;
    }

    //--------------------------------------------------------------------------------
    // line operator
    //--------------------------------------------------------------------------------
    
    SP_GENFUNC Line2 addLine(const Line2 &line, const Vec2 &vec) { return getLine2(line.pos[0] + vec, line.pos[1] + vec); }
    SP_GENFUNC Line3 addLine(const Line3 &line, const Vec3 &vec) { return getLine3(line.pos[0] + vec, line.pos[1] + vec); }

    SP_GENFUNC Line2 subLine(const Line2 &line, const Vec2 &vec) { return getLine2(line.pos[0] - vec, line.pos[1] - vec); }
    SP_GENFUNC Line3 subLine(const Line3 &line, const Vec3 &vec) { return getLine3(line.pos[0] - vec, line.pos[1] - vec); }

    SP_GENFUNC Line2 mulLine(const Line2 &line, const double val) { return getLine2(line.pos[0] * val, line.pos[1] * val); }
    SP_GENFUNC Line3 mulLine(const Line3 &line, const double val) { return getLine3(line.pos[0] * val, line.pos[1] * val); }

    SP_GENFUNC Line2 divLine(const Line2 &line, const double val) { return getLine2(line.pos[0] / val, line.pos[1] / val); }
    SP_GENFUNC Line3 divLine(const Line3 &line, const double val) { return getLine3(line.pos[0] / val, line.pos[1] / val); }

    SP_GENFUNC Line2 operator + (const Line2 &line, const Vec2 &vec) { return addLine(line, vec); }
    SP_GENFUNC Line3 operator + (const Line3 &line, const Vec3 &vec) { return addLine(line, vec); }
    SP_GENFUNC Line2 operator + (const Line2 &line) { return line; }
    SP_GENFUNC Line3 operator + (const Line3 &line) { return line; }
    SP_GENFUNC void operator += (Line2 &line, const Vec2 &vec) { line = addLine(line, vec); }
    SP_GENFUNC void operator += (Line3 &line, const Vec3 &vec) { line = addLine(line, vec); }

    SP_GENFUNC Line2 operator - (const Line2 &line, const Vec2 &vec) { return subLine(line, vec); }
    SP_GENFUNC Line3 operator - (const Line3 &line, const Vec3 &vec) { return subLine(line, vec); }
    SP_GENFUNC Line2 operator - (const Line2 &line) { return mulLine(line, -1.0); }
    SP_GENFUNC Line3 operator - (const Line3 &line) { return mulLine(line, -1.0); }
    SP_GENFUNC void operator -= (Line2 &line, const Vec2 &vec) { line = subLine(line, vec); }
    SP_GENFUNC void operator -= (Line3 &line, const Vec3 &vec) { line = subLine(line, vec); }
    
    SP_GENFUNC Line2 operator * (const Line2 &line, const double val) { return mulLine(line, val); }
    SP_GENFUNC Line3 operator * (const Line3 &line, const double val) { return mulLine(line, val); }
    SP_GENFUNC Line2 operator * (const double val, const Line2 &line) { return mulLine(line, val); }
    SP_GENFUNC Line3 operator * (const double val, const Line3 &line) { return mulLine(line, val); }
    SP_GENFUNC void operator *= (Line2 &line, const double val) { line = mulLine(line, val); }
    SP_GENFUNC void operator *= (Line3 &line, const double val) { line = mulLine(line, val); }
    
    SP_GENFUNC Line2 operator / (const Line2 &line, const double val) { return divLine(line, val); }
    SP_GENFUNC Line3 operator / (const Line3 &line, const double val) { return divLine(line, val); }
    SP_GENFUNC void operator /= (Line2 &line, const double val) { line = divLine(line, val); }
    SP_GENFUNC void operator /= (Line3 &line, const double val) { line = divLine(line, val); }

    //--------------------------------------------------------------------------------
    // line util
    //--------------------------------------------------------------------------------
    
    // projection vec3 to vec2
    SP_GENFUNC Line2 prjVec(const Line3 &line, const bool pers = true) {
        return getLine2(prjVec(line.pos[0], pers), prjVec(line.pos[1], pers));
    }

    // norm
    SP_GENFUNC double normLine(const Line2 &line) {
        return normVec(line.pos[0] - line.pos[1]);
    }

    // norm
    SP_GENFUNC double normLine(const Line3 &line) {
        return normVec(line.pos[0] - line.pos[1]);
    }

    // norm
    SP_GENFUNC double normVecToLine(const Vec2 &vec, const Line2 &line) {
        double ret = 0.0;

        const Vec2 lvec = line.pos[1] - line.pos[0];
        const double len = normVec(lvec);

        if (len < SP_SMALL) {
            ret = normVec(vec - (line.pos[0] + line.pos[1]) * 0.5);
        }
        else {
            const double s = dotVec(lvec, vec - line.pos[0]) / len;
            if (s < 0.0) {
                ret = normVec(vec - line.pos[0]);
            }
            else if (s > len) {
                ret = normVec(vec - line.pos[1]);
            }
            else {
                ret = normVec(vec - (line.pos[0] + lvec * s / len));
            }
        }
        return ret;
    }

    // norm
    SP_GENFUNC double normVecToLine(const Vec3 &vec, const Line3 &line) {
        double ret = 0.0;

        const Vec3 lvec = line.pos[1] - line.pos[0];
        const double len = normVec(lvec);

        if (len < SP_SMALL) {
            ret = normVec(vec - (line.pos[0] + line.pos[1]) * 0.5);
        }
        else {
            const double s = dotVec(lvec, vec - line.pos[0]) / len;
            if (s < 0.0) {
                ret = normVec(vec - line.pos[0]);
            }
            else if (s > len) {
                ret = normVec(vec - line.pos[1]);
            }
            else {
                ret = normVec(vec - (line.pos[0] + lvec * s / len));
            }
        }
        return ret;
    }

    //--------------------------------------------------------------------------------
    // matrix * line
    //--------------------------------------------------------------------------------

    SP_GENFUNC Line2 mulMat(const SP_REAL *mat, const int rows, const int cols, const Line2 &line) {
        Line2 dst;
        dst.pos[0] = mulMat(mat, rows, cols, line.pos[0]);
        dst.pos[1] = mulMat(mat, rows, cols, line.pos[1]);
        return dst;
    }

    SP_GENFUNC Line3 mulMat(const SP_REAL *mat, const int rows, const int cols, const Line3 &line) {
        Line3 dst;
        dst.pos[0] = mulMat(mat, rows, cols, line.pos[0]);
        dst.pos[1] = mulMat(mat, rows, cols, line.pos[1]);
        return dst;
    }


    //--------------------------------------------------------------------------------
    // mesh
    //--------------------------------------------------------------------------------

    // get mesh
    SP_GENFUNC Mesh2 getMesh2(const Vec2 &vec0, const Vec2 &vec1, const Vec2 &vec2) {
        Mesh2 dst;
        dst.pos[0] = vec0;
        dst.pos[1] = vec1;
        dst.pos[2] = vec2;
        return dst;
    }

    // get mesh
    SP_GENFUNC Mesh3 getMesh3(const Vec3 &vec0, const Vec3 &vec1, const Vec3 &vec2) {
        Mesh3 dst;
        dst.pos[0] = vec0;
        dst.pos[1] = vec1;
        dst.pos[2] = vec2;
        return dst;
    }

    //--------------------------------------------------------------------------------
    // mesh operator
    //--------------------------------------------------------------------------------

    SP_GENFUNC Mesh2 addMesh(const Mesh2 &mesh, const Vec2 vec) { return getMesh2(mesh.pos[0] + vec, mesh.pos[1] + vec, mesh.pos[2] + vec); }
    SP_GENFUNC Mesh3 addMesh(const Mesh3 &mesh, const Vec3 vec) { return getMesh3(mesh.pos[0] + vec, mesh.pos[1] + vec, mesh.pos[2] + vec); }
    
    SP_GENFUNC Mesh2 subMesh(const Mesh2 &mesh, const Vec2 vec) { return getMesh2(mesh.pos[0] - vec, mesh.pos[1] - vec, mesh.pos[2] - vec); }
    SP_GENFUNC Mesh3 subMesh(const Mesh3 &mesh, const Vec3 vec) { return getMesh3(mesh.pos[0] - vec, mesh.pos[1] - vec, mesh.pos[2] - vec); }
    
    SP_GENFUNC Mesh2 mulMesh(const Mesh2 &mesh, const double val) { return getMesh2(mesh.pos[0] * val, mesh.pos[1] * val, mesh.pos[2] * val); }
    SP_GENFUNC Mesh3 mulMesh(const Mesh3 &mesh, const double val) { return getMesh3(mesh.pos[0] * val, mesh.pos[1] * val, mesh.pos[2] * val); }

    SP_GENFUNC Mesh2 divMesh(const Mesh2 &mesh, const double val) { return getMesh2(mesh.pos[0] / val, mesh.pos[1] / val, mesh.pos[2] / val); }
    SP_GENFUNC Mesh3 divMesh(const Mesh3 &mesh, const double val) { return getMesh3(mesh.pos[0] / val, mesh.pos[1] / val, mesh.pos[2] / val); }

    SP_GENFUNC Mesh2 operator + (const Mesh2 &mesh, const Vec2 vec) { return addMesh(mesh, vec); }
    SP_GENFUNC Mesh3 operator + (const Mesh3 &mesh, const Vec3 vec) { return addMesh(mesh, vec); }
    SP_GENFUNC void operator += (Mesh2 &mesh, const Vec2 vec) { mesh = addMesh(mesh, vec); }
    SP_GENFUNC void operator += (Mesh3 &mesh, const Vec3 vec) { mesh = addMesh(mesh, vec); }

    SP_GENFUNC Mesh2 operator - (const Mesh2 &mesh, const Vec2 vec) { return subMesh(mesh, vec); }
    SP_GENFUNC Mesh3 operator - (const Mesh3 &mesh, const Vec3 vec) { return subMesh(mesh, vec); }
    SP_GENFUNC void operator -= (Mesh2 &mesh, const Vec2 vec) { mesh = subMesh(mesh, vec); }
    SP_GENFUNC void operator -= (Mesh3 &mesh, const Vec3 vec) { mesh = subMesh(mesh, vec); }

    SP_GENFUNC Mesh2 operator * (const Mesh2 &mesh, const double val) { return mulMesh(mesh, val); }
    SP_GENFUNC Mesh3 operator * (const Mesh3 &mesh, const double val) { return mulMesh(mesh, val); }
    SP_GENFUNC void operator *= (Mesh2 &mesh, const double val) { mesh = mulMesh(mesh, val); }
    SP_GENFUNC void operator *= (Mesh3 &mesh, const double val) { mesh = mulMesh(mesh, val); }

    SP_GENFUNC Mesh2 operator / (const Mesh2 &mesh, const double val) { return divMesh(mesh, val); }
    SP_GENFUNC Mesh3 operator / (const Mesh3 &mesh, const double val) { return divMesh(mesh, val); }
    SP_GENFUNC void operator /= (Mesh2 &mesh, const double val) { mesh = divMesh(mesh, val); }
    SP_GENFUNC void operator /= (Mesh3 &mesh, const double val) { mesh = divMesh(mesh, val); }

    //--------------------------------------------------------------------------------
    // matrix * mesh
    //--------------------------------------------------------------------------------

    SP_GENFUNC Mesh2 mulMat(const SP_REAL *mat, const int rows, const int cols, const Mesh2 &mesh) {
        Mesh2 dst;
        dst.pos[0] = mulMat(mat, rows, cols, mesh.pos[0]);
        dst.pos[1] = mulMat(mat, rows, cols, mesh.pos[1]);
        dst.pos[2] = mulMat(mat, rows, cols, mesh.pos[2]);
        return dst;
    }

    SP_GENFUNC Mesh3 mulMat(const SP_REAL *mat, const int rows, const int cols, const Mesh3 &mesh) {
        Mesh3 dst;
        dst.pos[0] = mulMat(mat, rows, cols, mesh.pos[0]);
        dst.pos[1] = mulMat(mat, rows, cols, mesh.pos[1]);
        dst.pos[2] = mulMat(mat, rows, cols, mesh.pos[2]);
        return dst;
    }

    //--------------------------------------------------------------------------------
    // mesh util
    //--------------------------------------------------------------------------------

    // get normal vector
    SP_GENFUNC Vec3 getMeshNrm(const Mesh3 &mesh) {
        return unitVec(crsVec(mesh.pos[1] - mesh.pos[0], mesh.pos[2] - mesh.pos[0]));
    }

    // get center vector
    SP_GENFUNC Vec2 getMeshCent(const Mesh2 &mesh) {
        return (mesh.pos[0] + mesh.pos[1] + mesh.pos[2]) / 3.0;
    }

    // get center vector
    SP_GENFUNC Vec3 getMeshCent(const Mesh3 &mesh) {
        return (mesh.pos[0] + mesh.pos[1] + mesh.pos[2]) / 3.0;
    }
    
    //--------------------------------------------------------------------------------
    // geodesic dorm
    //--------------------------------------------------------------------------------

    // geodesic mesh num
    SP_GENFUNC int getGeodesicMeshNum(const int level) {
        int ret = 20;
        for (int i = 0; i < level; i++) {
            ret *= 4;
        }
        return ret;
    }

    SP_GENFUNC Mesh3 getGeodesicMesh(const int level, const int id) {
        Mesh3 model[20];

        // init regular geodesic dorm (vertex num 12)
        {
            int cnt;

            Vec3 p[12];
            const double u = (1.0 + sqrt(5.0)) / 2.0;
            // vertex (0, ±1, ±u), (±u, 0, ±1), (±1, ±u, 0)

            cnt = 0;
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 2; j++) {
                    for (int k = 0; k < 2; k++) {
                        const double s[2] = { -1.0, +1.0 };
                        const double v[3] = { 0.0, s[j], s[k] * u };
                        p[cnt++] = getVec3(v[(i + 0) % 3], v[(i + 1) % 3], v[(i + 2) % 3]);
                    }
                }
            }

            cnt = 0;
            for (int i = 0; i < 12; i++) {
                for (int j = i + 1; j < 12; j++) {
                    for (int k = j + 1; k < 12; k++) {
                        if (cmp(dotVec(p[i], p[j]), u, 0.001) == false) continue;
                        if (cmp(dotVec(p[j], p[k]), u, 0.001) == false) continue;
                        if (cmp(dotVec(p[k], p[i]), u, 0.001) == false) continue;
                        model[cnt++] = getMesh3(unitVec(p[i]), unitVec(p[j]), unitVec(p[k]));
                    }
                }
            }

            for (int i = 0; i < 20; i++) {
                Mesh3 &m = model[i];
                if (dotVec(getMeshNrm(m), getMeshCent(m)) < 0.0) {
                    swap(m.pos[1], m.pos[2]);
                }
            }
        }

        // div regular geodesic dorm
        int num = getGeodesicMeshNum(level) / 20;
        int tmp = id;

        Mesh3 dst = model[tmp / num];
        for (int d = 0; d < level; d++) {
            const Vec3 p0 = unitVec(dst.pos[0] + dst.pos[1]);
            const Vec3 p1 = unitVec(dst.pos[1] + dst.pos[2]);
            const Vec3 p2 = unitVec(dst.pos[2] + dst.pos[0]);

            Mesh3 mesh[4];
            mesh[0] = getMesh3(p0, p1, p2);
            mesh[1] = getMesh3(dst.pos[0], p0, p2);
            mesh[2] = getMesh3(dst.pos[1], p1, p0);
            mesh[3] = getMesh3(dst.pos[2], p2, p1);

            tmp %= num;
            num /= 4;
            dst = mesh[tmp / num];
        }
        return dst;
    }

    
    //--------------------------------------------------------------------------------
    // box
    //--------------------------------------------------------------------------------

    // get box
    SP_GENFUNC Box2 getBox2(const Vec2 &vec0, const Vec2 &vec1) {
        Box2 dst;
        dst.pos[0] = vec0;
        dst.pos[1] = vec1;
        return dst;
    }

    // get box
    SP_GENFUNC Box3 getBox3(const Vec3 &vec0, const Vec3 &vec1) {
        Box3 dst;
        dst.pos[0] = vec0;
        dst.pos[1] = vec1;
        return dst;
    }

    // get box
    SP_GENFUNC Box3 nullBox3() {
        return getBox3(getVec3(1.0, 1.0, 1.0) * (+SP_INFINITY), getVec3(1.0, 1.0, 1.0) * (-SP_INFINITY));
    }

    // get box
    SP_GENFUNC Box3 getBox3(const Mesh3 &mesh) {
        Box3 dst = nullBox3();
        for (int p = 0; p < 3; p++) {
            const Vec3 &pos = mesh.pos[p];
            for (int i = 0; i < 3; i++) {
                acsv(dst.pos[0], i) = min(acsv(dst.pos[0], i), acsv(pos, i));
                acsv(dst.pos[1], i) = max(acsv(dst.pos[1], i), acsv(pos, i));
            }
        }
        return dst;
    }


    //--------------------------------------------------------------------------------
    // box util
    //--------------------------------------------------------------------------------

    SP_GENFUNC Box2 orBox(const Box2 &box0, const Box2 &box1) {
        Box2 dst = box0;
        for (int i = 0; i < 2; i++) {
            acsv(dst.pos[0], i) = min(acsv(dst.pos[0], i), acsv(box1.pos[0], i));
            acsv(dst.pos[1], i) = max(acsv(dst.pos[1], i), acsv(box1.pos[1], i));
        }
        return dst;
    }

    SP_GENFUNC Box3 orBox(const Box3 &box0, const Box3 &box1) {
        Box3 dst = box0;
        for (int i = 0; i < 3; i++) {
            acsv(dst.pos[0], i) = min(acsv(dst.pos[0], i), acsv(box1.pos[0], i));
            acsv(dst.pos[1], i) = max(acsv(dst.pos[1], i), acsv(box1.pos[1], i));
        }
        return dst;
    }

    SP_GENFUNC Box3 orBox(const Box3 &box, const Vec3 &vec) {
        Box3 dst = box;
        for (int i = 0; i < 3; i++) {
            acsv(dst.pos[0], i) = min(acsv(dst.pos[0], i), acsv(vec, i));
            acsv(dst.pos[1], i) = max(acsv(dst.pos[1], i), acsv(vec, i));
        }
        return dst;
    }

    SP_GENFUNC Box3 orBox(const Box3 &box, const Mesh3 &mesh) {
        Box3 dst = box;
        for (int p = 0; p < 3; p++) {
            const Vec3 &pos = mesh.pos[p];
            for (int i = 0; i < 3; i++) {
                acsv(dst.pos[0], i) = min(acsv(dst.pos[0], i), acsv(pos, i));
                acsv(dst.pos[1], i) = max(acsv(dst.pos[1], i), acsv(pos, i));
            }
        }
        return dst;
    }

    // get box area
    SP_GENFUNC SP_REAL getBoxArea(const Box2 &box) {
        const Vec2 d = box.pos[1] - box.pos[0];
        return d.x * d.y;
    }
    // get box area
    SP_GENFUNC SP_REAL getBoxArea(const Box3 &box) {
        const Vec3 d = box.pos[1] - box.pos[0];
        return (d.x * d.y + d.y * d.z + d.z * d.x) * 2.0;
    }

    // get box center
    SP_GENFUNC Vec2 getBoxCent(const Box2 &box) {
        return (box.pos[0] + box.pos[1]) * 0.5;
    }
    // get box center
    SP_GENFUNC Vec3 getBoxCent(const Box3 &box) {
        return (box.pos[0] + box.pos[1]) * 0.5;
    }
}


//--------------------------------------------------------------------------------
// color
//--------------------------------------------------------------------------------

namespace sp {
   
    // get color
    SP_GENFUNC Col3 getCol3(const Byte r, const Byte g, const Byte b) {
        Col3 dst;
        dst.r = r;
        dst.g = g;
        dst.b = b;
        return dst;
    }

    // get color
    SP_GENFUNC Col4 getCol4(const Byte r, const Byte g, const Byte b, const Byte a) {
        Col4 dst;
        dst.r = r;
        dst.g = g;
        dst.b = b;
        dst.a = a;
        return dst;
    }

    // get color
    SP_GENFUNC Col4 getCol4(const Col3 &col, const Byte a) {
        return getCol4(col.r, col.g, col.b, a);
    }

    // get color
    SP_GENFUNC Col3f getCol3f(const double r, const double g, const double b) {
        Col3f dst;
        dst.r = static_cast<float>(r);
        dst.g = static_cast<float>(g);
        dst.b = static_cast<float>(b);
        return dst;
    }

    // get color
    SP_GENFUNC Col4f getCol4f(const double r, const double g, const double b, const double a) {
        Col4f dst;
        dst.r = static_cast<float>(r);
        dst.g = static_cast<float>(g);
        dst.b = static_cast<float>(b);
        dst.a = static_cast<float>(a);
        return dst;
    }

    // get color
    SP_GENFUNC Col4f getCol4f(const Col3f &col, const double a) {
        return getCol4f(col.r, col.g, col.b, a);
    }

    //--------------------------------------------------------------------------------
    // color operator
    //--------------------------------------------------------------------------------

    SP_GENFUNC Col3f addCol(const Col3f &col0, const Col3f &col1) { return getCol3f(col0.r + col1.r, col0.g + col1.g, col0.b + col1.b); }
    SP_GENFUNC Col4f addCol(const Col4f &col0, const Col4f &col1) { return getCol4f(col0.r + col1.r, col0.g + col1.g, col0.b + col1.b, col0.a + col1.a); }

    SP_GENFUNC Col3f subCol(const Col3f &col0, const Col3f &col1) { return getCol3f(col0.r - col1.r, col0.g - col1.g, col0.b - col1.b); }
    SP_GENFUNC Col4f subCol(const Col4f &col0, const Col4f &col1) { return getCol4f(col0.r - col1.r, col0.g - col1.g, col0.b - col1.b, col0.a - col1.a); }

    SP_GENFUNC Col3f mulCol(const Col3f &col, const double val) { return getCol3f(col.r * val, col.g * val, col.b * val); }
    SP_GENFUNC Col4f mulCol(const Col4f &col, const double val) { return getCol4f(col.r * val, col.g * val, col.b * val, col.a * val); }

    SP_GENFUNC Col3f divCol(const Col3f &col, const double val) { SP_ASSERT(fabs(val) > SP_SMALL); return getCol3f(col.r / val, col.g / val, col.b / val); }
    SP_GENFUNC Col4f divCol(const Col4f &col, const double val) { SP_ASSERT(fabs(val) > SP_SMALL); return getCol4f(col.r / val, col.g / val, col.b / val, col.a / val); }

    SP_GENFUNC Col3f operator + (const Col3f &col0, const Col3f &col1) { return addCol(col0, col1); }
    SP_GENFUNC Col4f operator + (const Col4f &col0, const Col4f &col1) { return addCol(col0, col1); }
    SP_GENFUNC void operator += (Col3f &col0, const Col3f &col1) { col0 = addCol(col0, col1); }
    SP_GENFUNC void operator += (Col4f &col0, const Col4f &col1) { col0 = addCol(col0, col1); }

    SP_GENFUNC Col3f operator * (const Col3f &col, const double val) { return mulCol(col, val); }
    SP_GENFUNC Col4f operator * (const Col4f &col, const double val) { return mulCol(col, val); }
    SP_GENFUNC void operator *= (Col3f &col, const double val) { col = mulCol(col, val); }
    SP_GENFUNC void operator *= (Col4f &col, const double val) { col = mulCol(col, val); }

    SP_GENFUNC Col3f operator / (const Col3f &col, const double val) { return divCol(col, val); }
    SP_GENFUNC Col4f operator / (const Col4f &col, const double val) { return divCol(col, val); }
    SP_GENFUNC void operator /= (Col3f &col, const double val) { col = divCol(col, val); }
    SP_GENFUNC void operator /= (Col4f &col, const double val) { col = divCol(col, val); }

    //--------------------------------------------------------------------------------
    // color space
    //--------------------------------------------------------------------------------

    // convert phase to col3(rainbow), phase = [0, 1]
    SP_GENFUNC void cnvPhaseToCol(Col3 &col, const double phase) {
        const double p = max(0.0, min(phase, 1.0));

        col.r = static_cast<Byte>(255 * (sin(1.5 * SP_PI * p + SP_PI * (9.0 / 4.0)) + 1.0) / 2.0);
        col.g = static_cast<Byte>(255 * (sin(1.5 * SP_PI * p + SP_PI * (7.0 / 4.0)) + 1.0) / 2.0);
        col.b = static_cast<Byte>(255 * (sin(1.5 * SP_PI * p + SP_PI * (5.0 / 4.0)) + 1.0) / 2.0);
    }

    // convert hsv to col3, hsv = Vec3(h = [0, 2 * PI], s = [0, 1], v = [0, 1])
    SP_GENFUNC void cnvHSVToCol(Col3 &col, const Vec3 &hsv) {
        const double h = hsv.x;
        const double s = hsv.y;
        const double v = hsv.z;

        const double r = (h < 0 || h >= 2 * SP_PI) ? 0 : h;
        const double D = (r * 180.0 / SP_PI) / 60.0;

        const double f = D - floor(D);
        const Byte uv = static_cast<Byte>(v * 255.0 + 0.5);
        const Byte ua = static_cast<Byte>(v * 255.0 * (1.0 - s) + 0.5);
        const Byte ub = static_cast<Byte>(v * 255.0 * (1.0 - s * f) + 0.5);
        const Byte uc = static_cast<Byte>(v * 255.0 * (1.0 - s * (1.0 - f)) + 0.5);

        switch (floor(D) % 6) {
        case 0: col = getCol3(uv, uc, ua); break;
        case 1: col = getCol3(ub, uv, ua); break;
        case 2: col = getCol3(ua, uv, uc); break;
        case 3: col = getCol3(ua, ub, uv); break;
        case 4: col = getCol3(uc, ua, uv); break;
        case 5: col = getCol3(uv, ua, ub); break;
        }
    }

    // convert col3 to hsv, hsv = Vec3(h = [0, 2 * PI], s = [0, 1], v = [0, 1])
    SP_GENFUNC void cnvColToHSV(Vec3 &hsv, const Col3 &col) {
        const double maxv = max(col.r, max(col.g, col.b));
        const double minv = min(col.r, min(col.g, col.b));
        const double subv = maxv - minv;

        double h, s, v;
        {
            h = 0.0;
            v = maxv / 255.0;
            s = subv / max(maxv, 1.0);
        }
        if (subv == 0.0) {
            h = 0.0;
        }
        else if (col.r == maxv) {
            h = (col.g - col.b) / subv + 0.0;
        }
        else if (col.g == maxv) {
            h = (col.b - col.r) / subv + 2.0;
        }
        else if (col.b == maxv) {
            h = (col.r - col.g) / subv + 4.0;
        }
        h *= SP_PI / 3.0;
        if (h < 0.0) {
            h += 2 * SP_PI;
        }

        hsv = getVec3(h, s, v);
    }


    SP_GENFUNC void cnvXYZToLab(Vec3 &lab, const Vec3 &xyz) {

        const Vec3 w = getVec3(0.95047, 1.00000, 1.0883); // D65

        auto f = [](const double v)-> SP_REAL {
            return static_cast<SP_REAL>((v > 0.008856) ? pow(v, 1.0 / 3.0) : (7.787 * v) + (16.0 / 116.0));
        };

        Vec3 val;
        val.x = f(xyz.x / w.x);
        val.y = f(xyz.y / w.y);
        val.z = f(xyz.z / w.z);

        const double l = (116.0 * val.y) - 16.0;
        const double a = 500.0 * (val.x - val.y);
        const double b = 200.0 * (val.y - val.z);

        lab = getVec3(l, a, b);
    }


    SP_GENFUNC void cnvLabToXYZ(Vec3 &xyz, const Vec3 &lab) {

        const Vec3 w = getVec3(0.95047, 1.00000, 1.0883); // D65

        auto f = [](const SP_REAL v)-> SP_REAL {
            return static_cast<SP_REAL>((v > 0.206897) ? pow(v, 3.0) : 0.001107 * (116.0 * v - 16.0));
        };

        Vec3 val;
        val.y = static_cast<SP_REAL>((lab.x + 16.0) / 116.0);
        val.x = static_cast<SP_REAL>(val.y + lab.y / 500.0);
        val.z = static_cast<SP_REAL>(val.y - lab.z / 200.0);

        xyz.x = f(val.x) * w.x;
        xyz.y = f(val.y) * w.y;
        xyz.z = f(val.z) * w.z;
    }

    SP_GENFUNC void cnvColToXYZ(Vec3 &xyz, const Col3 &col) {
        auto f = [](const double v)-> SP_REAL {
            return static_cast<SP_REAL>((v > 0.040450) ? pow((v + 0.055) / 1.055, 2.4) : v / 12.92);
        };

        Vec3 val;
        val.x = f(col.r / 255.0);
        val.y = f(col.g / 255.0);
        val.z = f(col.b / 255.0);

        // D65
        xyz.x = static_cast<SP_REAL>(+0.412391 * val.x + 0.357584 * val.y + 0.180481 * val.z);
        xyz.y = static_cast<SP_REAL>(+0.212639 * val.x + 0.715169 * val.y + 0.072192 * val.z);
        xyz.z = static_cast<SP_REAL>(+0.019331 * val.x + 0.119195 * val.y + 0.950532 * val.z);
    }

    SP_GENFUNC void cnvXYZToCol(Col3 &col, const Vec3 &xyz) {
        auto f = [](const double v)-> SP_REAL {
            return (v > 0.0031308) ? 1.055 * pow(v, 1.0 / 2.4) - 0.055 : 12.92 * v;
        };

        Vec3 val;

        // D65
        val.x = static_cast<SP_REAL>(+3.240970 * xyz.x - 1.537383 * xyz.y - 0.498611 * xyz.z);
        val.y = static_cast<SP_REAL>(-0.969244 * xyz.x + 1.875968 * xyz.y + 0.041555 * xyz.z);
        val.z = static_cast<SP_REAL>(0.055630 * xyz.x - 0.203977 * xyz.y + 1.056972 * xyz.z);

        val.x = min(1.0, f(val.x));
        val.y = min(1.0, f(val.y));
        val.z = min(1.0, f(val.z));

        col = cast<Col3>(val);
    }

    SP_GENFUNC void cnvColToLab(Vec3 &lab, const Col3 &col) {
        Vec3 xyz;
        cnvColToXYZ(xyz, col);
        cnvXYZToLab(lab, xyz);
    }

    SP_GENFUNC void cnvLabToCol(Col3 &col, const Vec3 &lab) {
        Vec3 xyz;
        cnvLabToXYZ(xyz, lab);
        cnvXYZToCol(col, xyz);
    }

    //--------------------------------------------------------------------------------
    // convert geom to image
    //--------------------------------------------------------------------------------

    SP_GENFUNC void cnvDepthToCol(Byte &dst, const double depth, const double nearPlane, const double farPlane) {
        const double rate = 1.0 - (depth - nearPlane) / (farPlane - nearPlane);
        dst = static_cast<Byte>(255 * rate + 0.5);
    }

    SP_GENFUNC void cnvDepthToCol(Col3 &dst, const double depth, const double nearPlane, const double farPlane) {
        const double rate = 1.0 - (depth - nearPlane) / (farPlane - nearPlane);
        cnvPhaseToCol(dst, rate);
    }

    SP_GENFUNC void cnvNormalToCol(Byte &dst, const Vec3 &nrm) {
        dst = (nrm.z < 0) ? static_cast<Byte>(-255 * nrm.z) : 0;
    }

    SP_GENFUNC void cnvNormalToCol(Col3 &dst, const Vec3 &nrm) {
        dst.r = static_cast<Byte>(255 * (1.0 - nrm.x) / 2);
        dst.g = static_cast<Byte>(255 * (1.0 - nrm.y) / 2);
        dst.b = static_cast<Byte>(255 * (1.0 - nrm.z) / 2);
    }

    SP_GENFUNC void cnvDispToCol(Byte &dst, const float &disp, const int maxDisp, const int minDisp) {
        const double rate = 1.0 - (disp - minDisp) / (maxDisp - minDisp);
        dst = static_cast<Byte>(255 * rate + 0.5);
    }

    SP_GENFUNC void cnvDispToCol(Col3 &dst, const float &disp, const int maxDisp, const int minDisp) {
        const double rate = 1.0 - (disp - minDisp) / (maxDisp - minDisp);
        cnvPhaseToCol(dst, rate);
    }

    //--------------------------------------------------------------------------------
    // color util
    //--------------------------------------------------------------------------------

    SP_GENFUNC Byte blendCol(const Byte &col0, const double r0, const Byte &col1, const double r1) {
        if (r0 + r1 == 0.0f) return 0;
        if (r0 == 0.0) return col1;
        if (r1 == 0.0) return col0;

        Byte col;
        col = static_cast<Byte>((col0 * r0 + col1 * r1) / (r0 + r1));
        return col;
    }

    SP_GENFUNC Col3 blendCol(const Col3 &col0, const double r0, const Col3 &col1, const double r1) {
        if (r0 + r1 == 0.0f) return getCol3(0, 0, 0);
        if (r0 == 0.0) return col1;
        if (r1 == 0.0) return col0;

        Col3 col;
        col.r = static_cast<Byte>((col0.r * r0 + col1.r * r1) / (r0 + r1));
        col.g = static_cast<Byte>((col0.g * r0 + col1.g * r1) / (r0 + r1));
        col.b = static_cast<Byte>((col0.b * r0 + col1.b * r1) / (r0 + r1));
        return col;
    }

    SP_GENFUNC Col3f blendCol(const Col3f &col0, const double r0, const Col3f &col1, const double r1) {
        if (r0 + r1 == 0.0) return getCol4f(0.0, 0.0, 0.0, 0.0);
        if (r0 == 0.0) return col1;
        if (r1 == 0.0) return col0;

        Col4f dst;
        dst.r = (col0.r * r0 + col1.r * r1) / (r0 + r1);
        dst.g = (col0.g * r0 + col1.g * r1) / (r0 + r1);
        dst.b = (col0.b * r0 + col1.b * r1) / (r0 + r1);
        return dst;
    }

    SP_GENFUNC Col4f blendCol(const Col4f &col0, const double r0, const Col4f &col1, const double r1) {
        if (r0 + r1 == 0.0) return getCol4f(0.0, 0.0, 0.0, 0.0);
        if (r0 == 0.0) return col1;
        if (r1 == 0.0) return col0;

        Col4f dst;
        const float t0 = col0.a * r0;
        const float t1 = col1.a * r1;

        if (t0 + t1 > 0.0f) {
            dst.r = (col0.r * t0 + col1.r * t1) / (t0 + t1);
            dst.g = (col0.g * t0 + col1.g * t1) / (t0 + t1);
            dst.b = (col0.b * t0 + col1.b * t1) / (t0 + t1);
            dst.a = (t0 + t1) / (r0 + r1);
        }
        else {
            dst.r = (col0.r * r0 + col1.r * r1) / (r0 + r1);
            dst.g = (col0.g * r0 + col1.g * r1) / (r0 + r1);
            dst.b = (col0.b * r0 + col1.b * r1) / (r0 + r1);
            dst.a = 0.0f;
        }
        return dst;
    }

    // color id
    SP_GENFUNC Col3 getCol3FromId(const int id) {
        Col3 col = getCol3(0, 0, 0);
        col.r = (id) % 256;
        col.g = (id / (256)) % 256;
        col.b = (id / (256 * 256)) % 256;
        return col;
    }

    // color id
    SP_GENFUNC int getIdFromCol3(const Col3 &col) {
        int id = 0;
        id += col.r;
        id += col.g * (256);
        id += col.b * (256 * 256);
        if (id == 256 * 256 * 256 - 1) id = -1;
        return id;
    }

    // standord color i (0-12) v (0-7)
    SP_GENFUNC Col3 stdcol(const int i, const int v) {
        Col3 col;
        if (i == 0) {
            const float vlist[] = { 1.00f, 0.90f, 0.80f, 0.70f, 0.60f, 0.50f, 0.40f, 0.30f };
            cnvHSVToCol(col, sp::getVec3(0.0f, 0.0f, vlist[v]));
        }
        else {
            const float h = (i - 1) * 2.0f * SP_PI / 12.0f;
            const float slist[] = { 0.16f, 0.26f, 0.38f, 0.50f, 0.62f, 0.74f, 0.86f, 0.98f };
            const float vlist[] = { 0.98f, 0.95f, 0.91f, 0.86f, 0.80f, 0.73f, 0.65f, 0.56f };
            cnvHSVToCol(col, sp::getVec3(h, slist[v], vlist[v]));
        }
        return col;
    }

    SP_GENFUNC Col3 getCol3(const int label) {
        srand(max(label + 1, 0));
        Col3 col;
        cnvHSVToCol(col, getVec3((randu() + 1.0) * SP_PI, 1.0, 1.0));
        return col;
    }

    SP_GENFUNC Col3 revCol(const Col3 &col) {
        Vec3 hsv;
        cnvColToHSV(hsv, col);
        hsv.z = static_cast<SP_REAL>((hsv.z > 0.5) ? hsv.z - 0.5 : hsv.z + 0.5);

        Col3 tmp;
        cnvHSVToCol(tmp, hsv);

        return tmp;
    }

    SP_GENFUNC Col4 revCol(const Col4 &col) {
        Col3 c3 = revCol(getCol3(col.r, col.g, col.b));
        return getCol4(c3.r, c3.g, c3.b, col.a);
    }

}


//--------------------------------------------------------------------------------
// rect
//--------------------------------------------------------------------------------

namespace sp {

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
            dbase[i] = max(rect0.dbase[i], rect1.dbase[i]);
            dsize[i] = min(rect0.dbase[i] + rect0.dsize[i], rect1.dbase[i] + rect1.dsize[i]) - dbase[i];
        }
        return getRect2(dbase, dsize);
    }

    SP_GENFUNC Rect3 andRect(const Rect3 &rect0, const Rect3 &rect1) {
        int dbase[3] = { 0 }, dsize[3] = { 0 };
        for (int i = 0; i < 3; i++) {
            dbase[i] = max(rect0.dbase[i], rect1.dbase[i]);
            dsize[i] = min(rect0.dbase[i] + rect0.dsize[i], rect1.dbase[i] + rect1.dsize[i]) - dbase[i];
        }
        return getRect3(dbase, dsize);
    }

    SP_GENFUNC Rect2 orRect(const Rect2 &rect0, const Rect2 &rect1) {
        const bool b0 = (rect0.dsize[0] * rect0.dsize[1] > 0);
        const bool b1 = (rect1.dsize[0] * rect1.dsize[1] > 0);
        if (b0 && b1) {
            int dbase[2] = { 0 }, dsize[2] = { 0 };
            for (int i = 0; i < 2; i++) {
                dbase[i] = min(rect0.dbase[i], rect1.dbase[i]);
                dsize[i] = max(rect0.dbase[i] + rect0.dsize[i], rect1.dbase[i] + rect1.dsize[i]) - dbase[i];
                dsize[i] = max(0, dsize[i]);
            }
            return getRect2(dbase, dsize);
        }
        else {
            return (b0) ? rect0 : rect1;
        }
    }

    SP_GENFUNC Rect3 orRect(const Rect3 &rect0, const Rect3 &rect1) {
        const bool b0 = (rect0.dsize[0] * rect0.dsize[1] * rect0.dsize[2] > 0);
        const bool b1 = (rect1.dsize[0] * rect1.dsize[1] * rect1.dsize[2] > 0);
        if (b0 && b1) {
            int dbase[3] = { 0 }, dsize[3] = { 0 };
            for (int i = 0; i < 3; i++) {
                dbase[i] = min(rect0.dbase[i], rect1.dbase[i]);
                dsize[i] = max(rect0.dbase[i] + rect0.dsize[i], rect1.dbase[i] + rect1.dsize[i]) - dbase[i];
                dsize[i] = max(0, dsize[i]);
            }
            return getRect3(dbase, dsize);
        }
        else {
            return (b0) ? rect0 : rect1;
        }
    }

    SP_GENFUNC Rect2 extRect(const Rect2 &rect, const int val) {
        int dbase[2] = { 0 }, dsize[2] = { 0 };
        for (int i = 0; i < 2; i++) {
            const int t = max(val, -rect.dsize[i] / 2);
            dbase[i] = rect.dbase[i] - t;
            dsize[i] = rect.dsize[i] + 2 * t;
        }
        return getRect2(dbase, dsize);
    }

    SP_GENFUNC Rect3 extRect(const Rect3 &rect, const int val) {
        int dbase[3] = { 0 }, dsize[3] = { 0 };
        for (int i = 0; i < 3; i++) {
            const int t = max(val, -rect.dsize[i] / 2);
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

    SP_GENFUNC Rect2 operator + (const Rect2 &rect, const int val) { return extRect(rect, +val); }
    SP_GENFUNC Rect3 operator + (const Rect3 &rect, const int val) { return extRect(rect, +val); }
    SP_GENFUNC void operator += (Rect2 &rect, const int val) { rect = extRect(rect, +val); }
    SP_GENFUNC void operator += (Rect3 &rect, const int val) { rect = extRect(rect, +val); }

    SP_GENFUNC Rect2 operator - (const Rect2 &rect, const int val) { return extRect(rect, -val); }
    SP_GENFUNC Rect3 operator - (const Rect3 &rect, const int val) { return extRect(rect, -val); }
    SP_GENFUNC void operator -= (Rect2 &rect, const int val) { rect = extRect(rect, -val); }
    SP_GENFUNC void operator -= (Rect3 &rect, const int val) { rect = extRect(rect, -val); }

}


//--------------------------------------------------------------------------------
// camera parameter
//--------------------------------------------------------------------------------

namespace sp {

    SP_GENFUNC CamParam getCamParam(const int dsize0, const int dsize1, const double fx, const double fy, const double cx, const double cy) {
        CamParam dst;
        dst.type = CamParam_Pers;
        dst.dsize[0] = dsize0;
        dst.dsize[1] = dsize1;

        dst.fx = static_cast<SP_REAL>(fx);
        dst.fy = static_cast<SP_REAL>(fy);
        dst.cx = static_cast<SP_REAL>(cx);
        dst.cy = static_cast<SP_REAL>(cy);

        dst.k1 = static_cast<SP_REAL>(0.0);
        dst.k2 = static_cast<SP_REAL>(0.0);
        dst.k3 = static_cast<SP_REAL>(0.0);
        dst.k4 = static_cast<SP_REAL>(0.0);
        dst.p1 = static_cast<SP_REAL>(0.0);
        dst.p2 = static_cast<SP_REAL>(0.0);
        return dst;
    }

    SP_GENFUNC CamParam getCamParam(const int dsize0, const int dsize1, const double fx, const double fy) {
        return getCamParam(dsize0, dsize1, fx, fy, (dsize0 - 1) * 0.5, (dsize1 - 1) * 0.5);
    }

    SP_GENFUNC CamParam getCamParam(const int dsize0, const int dsize1) {
        // groundless camera parameter, but in many cases empirically, no big difference
        const SP_REAL f = 0.8 * (dsize0 + dsize1);
        return getCamParam(dsize0, dsize1, f, f);
    }

    SP_GENFUNC CamParam getCamParam(const int *dsize, const double fx, const double fy, const double cx, const double cy) {
        return getCamParam(dsize[0], dsize[1], fx, fy, cx, cy);
    }

    SP_GENFUNC CamParam getCamParam(const int *dsize, const double fx, const double fy) {
        return getCamParam(dsize[0], dsize[1], fx, fy);
    }

    SP_GENFUNC CamParam getCamParam(const int *dsize) {
        return getCamParam(dsize[0], dsize[1]);
    }

    SP_GENFUNC void getMat(SP_REAL *dst, const int rows, const int cols, const CamParam &cam) {
        dst[0 * cols + 0] = cam.fx;
        dst[0 * cols + 1] = 0.0;
        dst[0 * cols + 2] = cam.cx;

        dst[1 * cols + 0] = 0.0;
        dst[1 * cols + 1] = cam.fy;
        dst[1 * cols + 2] = cam.cy;

        dst[2 * cols + 0] = 0.0;
        dst[2 * cols + 1] = 0.0;
        dst[2 * cols + 2] = 1.0;
    }


    //--------------------------------------------------------------------------------
    // jacob
    //--------------------------------------------------------------------------------

    SP_GENFUNC void jacobCamToPix(SP_REAL *jacob, const CamParam &cam, const Vec2 &npx) {
        const double x2 = npx.x * npx.x;
        const double y2 = npx.y * npx.y;
        const double xy = npx.x * npx.y;

        const double r2 = x2 + y2;
        const double r4 = r2 * r2;
        const double r6 = r4 * r2;

        const double k = 1.0 + cam.k1 * r2 + cam.k2 * r4 + cam.k3 * r6;

        Vec2 dist;
        dist.x = static_cast<SP_REAL>(npx.x * k + cam.p1 * (2.0 * xy) + cam.p2 * (2.0 * x2 + r2));
        dist.y = static_cast<SP_REAL>(npx.y * k + cam.p1 * (2.0 * y2 + r2) + cam.p2 * (2.0 * xy));

        jacob[0 * 9 + 0] = static_cast<SP_REAL>(dist.x);
        jacob[0 * 9 + 1] = static_cast<SP_REAL>(0.0);
        jacob[0 * 9 + 2] = static_cast<SP_REAL>(1.0);
        jacob[0 * 9 + 3] = static_cast<SP_REAL>(0.0);
        jacob[0 * 9 + 4] = static_cast<SP_REAL>(cam.fx * (npx.x * r2));
        jacob[0 * 9 + 5] = static_cast<SP_REAL>(cam.fx * (npx.x * r4));
        jacob[0 * 9 + 6] = static_cast<SP_REAL>(cam.fx * (npx.x * r6));
        jacob[0 * 9 + 7] = static_cast<SP_REAL>(cam.fx * (2.0 * xy));
        jacob[0 * 9 + 8] = static_cast<SP_REAL>(cam.fx * (2.0 * x2 + r2));

        jacob[1 * 9 + 0] = static_cast<SP_REAL>(0.0);
        jacob[1 * 9 + 1] = static_cast<SP_REAL>(dist.y);
        jacob[1 * 9 + 2] = static_cast<SP_REAL>(0.0);
        jacob[1 * 9 + 3] = static_cast<SP_REAL>(1.0);
        jacob[1 * 9 + 4] = static_cast<SP_REAL>(cam.fy * (npx.y * r2));
        jacob[1 * 9 + 5] = static_cast<SP_REAL>(cam.fy * (npx.y * r4));
        jacob[1 * 9 + 6] = static_cast<SP_REAL>(cam.fy * (npx.y * r6));
        jacob[1 * 9 + 7] = static_cast<SP_REAL>(cam.fy * (2.0 * y2 + r2));
        jacob[1 * 9 + 8] = static_cast<SP_REAL>(cam.fy * (2.0 * xy));
    }

    SP_GENFUNC void jacobNpxToDist(SP_REAL *jacob, const CamParam &cam, const Vec2 &npx) {
        const double x2 = npx.x * npx.x;
        const double y2 = npx.y * npx.y;
        const double xy = npx.x * npx.y;

        const double r2 = x2 + y2;
        const double r4 = r2 * r2;
        const double r6 = r4 * r2;

        const double k1 = 1.0 + cam.k1 * r2 + cam.k2 * r4 + cam.k3 * r6;
        const double k2 = 2.0 * cam.k1 + 4.0 * cam.k2 * r2 + 6.0 * cam.k3 * r4;

        jacob[0 * 2 + 0] = static_cast<SP_REAL>(x2 * k2 + k1 + 2.0 * cam.p1 * npx.y + 6.0 * cam.p2 * npx.x);
        jacob[1 * 2 + 1] = static_cast<SP_REAL>(y2 * k2 + k1 + 6.0 * cam.p1 * npx.y + 2.0 * cam.p2 * npx.x);

        jacob[0 * 2 + 1] = static_cast<SP_REAL>(xy * k2 + 2.0 * cam.p1 * npx.x + 2.0 * cam.p2 * npx.y);
        jacob[1 * 2 + 0] = static_cast<SP_REAL>(xy * k2 + 2.0 * cam.p1 * npx.x + 2.0 * cam.p2 * npx.y);
    }

    SP_GENFUNC void jacobNpxToPix(SP_REAL *jacob, const CamParam &cam, const Vec2 &npx) {

        SP_REAL jNpxToDist[2 * 2] = { 0 };
        jacobNpxToDist(jNpxToDist, cam, npx);

        SP_REAL jDistToPix[2 * 2] = { 0 };
        jDistToPix[0 * 2 + 0] = cam.fx;
        jDistToPix[1 * 2 + 1] = cam.fy;

        mulMat(jacob, 2, 2, jDistToPix, 2, 2, jNpxToDist, 2, 2);
    }


    //--------------------------------------------------------------------------------
    // camera util
    //--------------------------------------------------------------------------------

    SP_GENFUNC Vec2 mulCam(const CamParam &cam, const Vec2 &npx) {
        Vec2 pix;
        pix.x = npx.x * cam.fx + cam.cx;
        pix.y = npx.y * cam.fy + cam.cy;
        return pix;
    }

    SP_GENFUNC Line2 mulCam(const CamParam &cam, const Line2 &npxline) {
        Line2 pixline;
        pixline.pos[0] = mulCam(cam, npxline.pos[0]);
        pixline.pos[1] = mulCam(cam, npxline.pos[1]);
        return pixline;
    }

    SP_GENFUNC Vec2 invCam(const CamParam &cam, const Vec2 &pix) {
        Vec2 npx;
        npx.x = (pix.x - cam.cx) / cam.fx;
        npx.y = (pix.y - cam.cy) / cam.fy;
        return npx;
    }

    SP_GENFUNC Line2 invCam(const CamParam &cam, const Line2 &pixline) {
        Line2 npxline;
        npxline.pos[0] = invCam(cam, pixline.pos[0]);
        npxline.pos[1] = invCam(cam, pixline.pos[1]);
        return npxline;
    }

    // distiortion
    SP_GENFUNC Vec2 npxDist(const CamParam &cam, const Vec2 &npx) {
        const double x2 = npx.x * npx.x;
        const double y2 = npx.y * npx.y;
        const double xy = npx.x * npx.y;

        const double r2 = x2 + y2;
        const double r4 = r2 * r2;
        const double r6 = r4 * r2;

        const double k = 1.0 + cam.k1 * r2 + cam.k2 * r4 + cam.k3 * r6;

        Vec2 dist;
        dist.x = static_cast<SP_REAL>(npx.x * k + cam.p1 * (2.0 * xy) + cam.p2 * (2.0 * x2 + r2));
        dist.y = static_cast<SP_REAL>(npx.y * k + cam.p1 * (2.0 * y2 + r2) + cam.p2 * (2.0 * xy));

        return dist;
    }

    // distiortion
    SP_GENFUNC Vec2 pixDist(const CamParam &cam, const Vec2 &pix) {
        return mulCam(cam, npxDist(cam, invCam(cam, pix)));
    }

    // undistortion
    SP_GENFUNC Vec2 npxUndist(const CamParam &cam, const Vec2 &npx) {
        const int maxit = 10;

        Vec2 undist = npx;
        for (int it = 0; it < maxit; it++) {
            const Vec2 err = npx - npxDist(cam, undist);
            if (normVec(err) < SP_SMALL) break;

            SP_REAL J[2 * 2], inv[2 * 2];
            jacobNpxToDist(J, cam, undist);

            if (invMat22(inv, J) == false) break;

            undist += mulMat(inv, 2, 2, err);
        }

        return undist;
    }

    // undistortion
    SP_GENFUNC Vec2 pixUndist(const CamParam &cam, const Vec2 &pix) {
        return mulCam(cam, npxUndist(cam, invCam(cam, pix)));
    }

    // ideal to pix
    SP_GENFUNC Vec2 mulCamD(const CamParam &cam, const Vec2 &npx) {
        return mulCam(cam, npxDist(cam, npx));
    }

    // pix to ideal
    SP_GENFUNC Vec2 invCamD(const CamParam &cam, const Vec2 &pix) {
        return npxUndist(cam, invCam(cam, pix));
    }

    // undistortion x
    SP_GENFUNC Vec2 npxUndistX(const CamParam &cam, const Vec3 &epi, const SP_REAL &npxx) {
        const int maxit = 10;

        const double a = -epi.x / epi.y;
        const double b = -epi.z / epi.y;

        double x = npxx;
        for (int i = 0; i < maxit; i++) {
            const double y = a * x + b;

            const double x2 = x * x;
            const double y2 = y * y;
            const double xy = x * y;

            const double r2 = x2 + y2;
            const double r4 = r2 * r2;
            const SP_REAL r6 = r4 * r2;

            const double k1 = 1.0 + cam.k1 * r2 + cam.k2 * r4 + cam.k3 * r6;
            const double k2 = 2.0 * cam.k1 + 4.0 * cam.k2 * r2 + 6.0 * cam.k3 * r4;

            const double err = npxx - (x * k1 + cam.p1 * (2.0 * xy) + cam.p2 * (2.0 * x2 + r2));

            // dist.x = npx.x * k + cam.p1 * (2.0 * xy) + cam.p2 * (2.0 * x2 + r2);
            // dist.y = npx.y * k + cam.p1 * (2.0 * y2 + r2) + cam.p2 * (2.0 * xy);
            // j = d(err) / d(npx.x)

            const double j = x * k2 + k1 + 2.0 * cam.p1 * y + 6.0 * cam.p2 * x;

            if (fabs(j) < SP_SMALL) break;
            x += err / j;
        }
        return getVec2(x, a * x + b);
    }

    // undistortion y
    SP_GENFUNC Vec2 npxUndistY(const CamParam &cam, const Vec3 &epi, const SP_REAL &npxy) {
        const int maxit = 10;

        const double a = -epi.y / epi.x;
        const double b = -epi.z / epi.x;

        double y = npxy;
        for (int i = 0; i < maxit; i++) {
            const double x = a * y + b;

            const double x2 = x * x;
            const double y2 = y * y;
            const double xy = x * y;

            const double r2 = x2 + y2;
            const double r4 = r2 * r2;
            const double r6 = r4 * r2;

            const double k1 = 1.0 + cam.k1 * r2 + cam.k2 * r4 + cam.k3 * r6;
            const double k2 = 2.0 * cam.k1 + 4.0 * cam.k2 * r2 + 6.0 * cam.k3 * r4;

            const double err = npxy - (y * k1 + cam.p1 * (2.0 * y2 + r2) + cam.p2 * (2.0 * xy));

            // dist.x = npx.x * k + cam.p1 * (2.0 * xy) + cam.p2 * (2.0 * x2 + r2);
            // dist.y = npx.y * k + cam.p1 * (2.0 * y2 + r2) + cam.p2 * (2.0 * xy);
            // j = d(err) / d(npx.y)

            const double j = y * k2 + k1 + 6.0 * cam.p1 * y + 2.0 * cam.p2 * x;

            if (fabs(j) < SP_SMALL) break;
            y += err / j;
        }
        return getVec2(a * y + b, y);
    }

    // update
    SP_GENFUNC CamParam updateCam(const CamParam &cam, const SP_REAL *delta) {
        CamParam dst = cam;
        dst.fx += delta[0];
        dst.fy += delta[1];
        dst.cx += delta[2];
        dst.cy += delta[3];

        dst.k1 += delta[4];
        dst.k2 += delta[5];
        dst.k3 += delta[6];
        dst.p1 += delta[7];
        dst.p2 += delta[8];
        return dst;
    }

    //--------------------------------------------------------------------------------
    // rescale 
    //--------------------------------------------------------------------------------

    SP_CPUFUNC void rescale(CamParam &dst, const CamParam &cam, const double dscale0, const double dscale1) {
        dst = cam;

        dst.dsize[0] = round(cam.dsize[0] * dscale0);
        dst.dsize[1] = round(cam.dsize[1] * dscale1);

        dst.fx = static_cast<SP_REAL>(dst.fx * dscale0);
        dst.fy = static_cast<SP_REAL>(dst.fy * dscale1);

        dst.cx = static_cast<SP_REAL>(dst.cx * dscale0);
        dst.cy = static_cast<SP_REAL>(dst.cy * dscale1);
    }

    //--------------------------------------------------------------------------------
    // pyramid down 
    //--------------------------------------------------------------------------------

    SP_CPUFUNC void pyrdown(CamParam &dst, const CamParam &cam) {
        rescale(dst, cam, 0.5, 0.5);
    }

}

//--------------------------------------------------------------------------------
// transform
//--------------------------------------------------------------------------------

namespace sp {

    //--------------------------------------------------------------------------------
    // rotation
    //--------------------------------------------------------------------------------

    SP_GENFUNC Rot nrmRot(const Rot &rot) {
        Rot dst;

        const double div = sqrt(rot.qx * rot.qx + rot.qy * rot.qy + rot.qz * rot.qz + rot.qw * rot.qw);
        if (div > SP_SMALL) {
            const double s = (sign(rot.qw) >= 0.0) ? +1 : -1;

            dst.qx = rot.qx / div * s;
            dst.qy = rot.qy / div * s;
            dst.qz = rot.qz / div * s;
            dst.qw = rot.qw / div * s;
        }
        else {
            dst.qx = 0.0;
            dst.qy = 0.0;
            dst.qz = 0.0;
            dst.qw = 1.0;
        }
        return dst;
    }

    SP_GENFUNC Rot getRot(const SP_REAL qx, const SP_REAL qy, const SP_REAL qz, const SP_REAL qw) {
        Rot dst;
        dst.qx = qx;
        dst.qy = qy;
        dst.qz = qz;
        dst.qw = qw;
        return nrmRot(dst);
    }

    SP_GENFUNC Rot getRot(const SP_REAL *mat, const int rows, const int cols) {
        Rot dst;
        dst.qx = sqrt(max(0.0, 1 + mat[0 * cols + 0] - mat[1 * cols + 1] - mat[2 * cols + 2])) / 2;
        dst.qy = sqrt(max(0.0, 1 - mat[0 * cols + 0] + mat[1 * cols + 1] - mat[2 * cols + 2])) / 2;
        dst.qz = sqrt(max(0.0, 1 - mat[0 * cols + 0] - mat[1 * cols + 1] + mat[2 * cols + 2])) / 2;
        dst.qw = sqrt(max(0.0, 1 + mat[0 * cols + 0] + mat[1 * cols + 1] + mat[2 * cols + 2])) / 2;

        dst.qx *= sign(dst.qx * (mat[2 * cols + 1] - mat[1 * cols + 2]));
        dst.qy *= sign(dst.qy * (mat[0 * cols + 2] - mat[2 * cols + 0]));
        dst.qz *= sign(dst.qz * (mat[1 * cols + 0] - mat[0 * cols + 1]));

        return nrmRot(dst);
    }

    SP_GENFUNC void getMat(SP_REAL *dst, const int rows, const int cols, const Rot &rot) {
        {
            const double qx2 = rot.qx * rot.qx;
            const double qy2 = rot.qy * rot.qy;
            const double qz2 = rot.qz * rot.qz;
            const double qw2 = rot.qw * rot.qw;

            dst[0 * cols + 0] = static_cast<SP_REAL>(qw2 + qx2 - qy2 - qz2);
            dst[1 * cols + 1] = static_cast<SP_REAL>(qw2 - qx2 + qy2 - qz2);
            dst[2 * cols + 2] = static_cast<SP_REAL>(qw2 - qx2 - qy2 + qz2);
        }
        {
            const double qxy = rot.qx * rot.qy;
            const double qzw = rot.qz * rot.qw;
            dst[0 * cols + 1] = static_cast<SP_REAL>(2 * (qxy - qzw));
            dst[1 * cols + 0] = static_cast<SP_REAL>(2 * (qxy + qzw));

            const double qxz = rot.qx * rot.qz;
            const double qyw = rot.qy * rot.qw;
            dst[0 * cols + 2] = static_cast<SP_REAL>(2 * (qxz + qyw));
            dst[2 * cols + 0] = static_cast<SP_REAL>(2 * (qxz - qyw));

            const double qyz = rot.qy * rot.qz;
            const double qxw = rot.qx * rot.qw;
            dst[1 * cols + 2] = static_cast<SP_REAL>(2 * (qyz - qxw));
            dst[2 * cols + 1] = static_cast<SP_REAL>(2 * (qyz + qxw));
        }
    }

    SP_GENFUNC Rot zeroRot() {
        return getRot(0.0, 0.0, 0.0, 1.0);
    }

    SP_GENFUNC Rot invRot(const Rot &rot) {
        return getRot(-rot.qx, -rot.qy, -rot.qz, rot.qw);
    }

    SP_GENFUNC Rot mulRot(const Rot &rot0, const Rot &rot1) {
        Rot dst;
        dst.qx = static_cast<SP_REAL>((rot0.qw * rot1.qx) + (rot0.qx * rot1.qw) + (rot0.qy * rot1.qz) - (rot0.qz * rot1.qy));
        dst.qy = static_cast<SP_REAL>((rot0.qw * rot1.qy) + (rot0.qy * rot1.qw) + (rot0.qz * rot1.qx) - (rot0.qx * rot1.qz));
        dst.qz = static_cast<SP_REAL>((rot0.qw * rot1.qz) + (rot0.qz * rot1.qw) + (rot0.qx * rot1.qy) - (rot0.qy * rot1.qx));

        dst.qw = static_cast<SP_REAL>((rot0.qw * rot1.qw) - (rot0.qx * rot1.qx) - (rot0.qy * rot1.qy) - (rot0.qz * rot1.qz));

        return nrmRot(dst);
    }

    SP_GENFUNC Vec3 mulRot(const Rot &rot, const Vec3 &vec) {
        SP_REAL rotMat[3 * 3];
        getMat(rotMat, 3, 3, rot);

        return mulMat(rotMat, 3, 3, vec);
    }

    SP_GENFUNC Vec3 mulRot(const Rot &rot, const Vec2 &vec) {
        return mulRot(rot, getVec3(vec.x, vec.y, 0.0));
    }


    //--------------------------------------------------------------------------------
    // rotation operator
    //--------------------------------------------------------------------------------

    SP_GENFUNC Vec3 operator * (const Rot &rot, const Vec3 &vec) {
        return mulRot(rot, vec);
    }

    SP_GENFUNC Vec3 operator * (const Rot &rot, const Vec2 &vec) {
        return mulRot(rot, vec);
    }

    SP_GENFUNC Rot operator * (const Rot &rot0, const Rot &rot1) {
        return mulRot(rot0, rot1);
    }

    SP_GENFUNC void operator *= (Rot &rot0, const Rot &rot1) {
        rot0 = mulRot(rot0, rot1);
    }

    //--------------------------------------------------------------------------------
    // rotation util
    //--------------------------------------------------------------------------------

    SP_GENFUNC void getMatAngleX(SP_REAL *dst, const int rows, const int cols, const SP_REAL angle) {
        dst[0 * cols + 0] = 1.0;
        dst[0 * cols + 1] = 0.0;
        dst[0 * cols + 2] = 0.0;

        dst[1 * cols + 0] = 0.0;
        dst[1 * cols + 1] = +cos(angle);
        dst[1 * cols + 2] = -sin(angle);

        dst[2 * cols + 0] = 0.0;
        dst[2 * cols + 1] = +sin(angle);
        dst[2 * cols + 2] = +cos(angle);
    }

    SP_GENFUNC void getMatAngleY(SP_REAL *dst, const int rows, const int cols, const SP_REAL angle) {
        dst[0 * cols + 0] = +cos(angle);
        dst[0 * cols + 1] = 0.0;
        dst[0 * cols + 2] = +sin(angle);

        dst[1 * cols + 0] = 0.0;
        dst[1 * cols + 1] = 1.0;
        dst[1 * cols + 2] = 0.0;

        dst[2 * cols + 0] = -sin(angle);
        dst[2 * cols + 1] = 0.0;
        dst[2 * cols + 2] = +cos(angle);
    }

    SP_GENFUNC void getMatAngleZ(SP_REAL *dst, const int rows, const int cols, const SP_REAL angle) {
        dst[0 * cols + 0] = +cos(angle);
        dst[0 * cols + 1] = -sin(angle);
        dst[0 * cols + 2] = 0.0;

        dst[1 * cols + 0] = +sin(angle);
        dst[1 * cols + 1] = +cos(angle);
        dst[1 * cols + 2] = 0.0;

        dst[2 * cols + 0] = 0.0;
        dst[2 * cols + 1] = 0.0;
        dst[2 * cols + 2] = 1.0;
    }

    SP_GENFUNC void getMatRodrigues(SP_REAL *dst, const int rows, const int cols, const Vec3 &vec) {
        const SP_REAL angle = normVec(vec);
        const Vec3 nrm = unitVec(vec);

        const SP_REAL c = cos(angle);
        const SP_REAL s = sin(angle);

        dst[0 * 3 + 0] = static_cast<SP_REAL>(nrm.x * nrm.x * (1.0 - c) + c);
        dst[0 * 3 + 1] = static_cast<SP_REAL>(nrm.x * nrm.y * (1.0 - c) - nrm.z * s);
        dst[0 * 3 + 2] = static_cast<SP_REAL>(nrm.x * nrm.z * (1.0 - c) + nrm.y * s);

        dst[1 * 3 + 0] = static_cast<SP_REAL>(nrm.y * nrm.x * (1.0 - c) + nrm.z * s);
        dst[1 * 3 + 1] = static_cast<SP_REAL>(nrm.y * nrm.y * (1.0 - c) + c);
        dst[1 * 3 + 2] = static_cast<SP_REAL>(nrm.y * nrm.z * (1.0 - c) - nrm.x * s);

        dst[2 * 3 + 0] = static_cast<SP_REAL>(nrm.z * nrm.x * (1.0 - c) - nrm.y * s);
        dst[2 * 3 + 1] = static_cast<SP_REAL>(nrm.z * nrm.y * (1.0 - c) + nrm.x * s);
        dst[2 * 3 + 2] = static_cast<SP_REAL>(nrm.z * nrm.z * (1.0 - c) + c);
    }

    SP_GENFUNC void getMatRodrigues(SP_REAL *dst, const int rows, const int cols, const Vec3 &vec, const SP_REAL angle) {
        getMatRodrigues(dst, rows, cols, unitVec(vec) * angle);
    }

    SP_GENFUNC Rot getRotAxis(const Vec3 &x, const Vec3 &y, const Vec3 &z) {
        const Vec3 nx = unitVec(x);
        const Vec3 ny = unitVec(y);
        const Vec3 nz = unitVec(z);
        SP_REAL mat[3 * 3];
        mat[0 * 3 + 0] = nx.x; mat[0 * 3 + 1] = ny.x; mat[0 * 3 + 2] = nz.x;
        mat[1 * 3 + 0] = nx.y; mat[1 * 3 + 1] = ny.y; mat[1 * 3 + 2] = nz.y;
        mat[2 * 3 + 0] = nx.z; mat[2 * 3 + 1] = ny.z; mat[2 * 3 + 2] = nz.z;

        return getRot(mat, 3, 3);
    }

    SP_GENFUNC Rot getRotAngle(const Vec3 &vec) {
        const SP_REAL angle = normVec(vec);
        if (angle > SP_SMALL) {
            const Vec3 nrm = unitVec(vec);

            const SP_REAL s = sin(angle * 0.5);
            const SP_REAL c = cos(angle * 0.5);
            return getRot(s * nrm.x, s * nrm.y, s * nrm.z, c);
        }
        else {
            return zeroRot();
        }
    }

    SP_GENFUNC Rot getRotAngle(const Vec3 &vec, const double angle) {
        return getRotAngle(unitVec(vec) * angle);
    }

    SP_GENFUNC Rot getRotAngleX(const double angle) {
        return getRotAngle(getVec3(1.0, 0.0, 0.0), angle);
    }

    SP_GENFUNC Rot getRotAngleY(const double angle) {
        return getRotAngle(getVec3(0.0, 1.0, 0.0), angle);
    }

    SP_GENFUNC Rot getRotAngleZ(const double angle) {
        return getRotAngle(getVec3(0.0, 0.0, 1.0), angle);
    }

    SP_GENFUNC Vec3 getAngle(const Rot &rot) {
        Vec3 vec = getVec3(0.0, 0.0, 0.0);

        if (cmp(rot, getRot(0.0, 0.0, 0.0, 1.0)) == false) {
            const SP_REAL angle = acos(rot.qw) * 2.0;

            if (cmp(angle, 0.0) == false) {
                const SP_REAL s = sin(angle * 0.5);
                vec.x = rot.qx / s * angle;
                vec.y = rot.qy / s * angle;
                vec.z = rot.qz / s * angle;
            }
        }
        return vec;
    }

    SP_GENFUNC Rot getRotDirection(const Vec3 &vec) {
        const Vec3 nrm = unitVec(vec);

        if (fabs(nrm.z) == 1.0) {
            const SP_REAL angle = (nrm.z > 0) ? 0.0 : SP_PI;
            return getRotAngleX(angle);
        }
        else {
            const Vec3 v0 = crsVec(getVec3(0.0, 1.0, 0.0), getVec3(nrm.x, nrm.y, 0.0));
            const SP_REAL a0 = acos(nrm.y / sqrt(nrm.x * nrm.x + nrm.y * nrm.y));
            const Rot rot0 = getRotAngle(v0, a0);

            const Vec3 v1 = crsVec(getVec3(0.0, 0.0, 1.0), nrm);
            const SP_REAL a1 = acos(nrm.z);
            const Rot rot1 = getRotAngle(v1, a1);

            return invRot(rot1 * rot0);
        }
    }

    // zyx eulter
    SP_GENFUNC Rot getRotEuler(const Vec3 &euler) {
        const Rot rotx = getRotAngleX(euler.x);
        const Rot roty = getRotAngleY(euler.y);
        const Rot rotz = getRotAngleZ(euler.z);
        return rotz * roty * rotx;
    }

    // zyx eulter
    SP_GENFUNC Vec3 getEuler(const SP_REAL *mat, const int rows, const int cols) {

        Vec3 euler;
        euler.y = asin(-mat[2 * 3 + 0]);

        if (fabs(euler.y) < SP_PI / 2.0) {
            euler.z = atan2(mat[1 * 3 + 0], mat[0 * 3 + 0]);
            euler.x = atan2(mat[2 * 3 + 1], mat[2 * 3 + 2]);
        }
        else {
            euler.z = atan2(-mat[1 * 3 + 2], mat[0 * 3 + 2]);
            euler.x = 0.0;
        }

        return euler;
    }

    // zyx eulter
    SP_GENFUNC Vec3 getEuler(const Rot &rot) {
        SP_REAL mat[3 * 3];
        getMat(mat, 3, 3, rot);

        return getEuler(mat, 3, 3);
    }

    // random unif
    SP_CPUFUNC Rot randuRot(const double max) {
        return getRotAngle(randuVec3(1.0, 1.0, 1.0), randu() * max);
    }

    // random gauss
    SP_CPUFUNC Rot randgRot(const double max) {
        return getRotAngle(randuVec3(1.0, 1.0, 1.0), randg() * max);
    }

    // update
    SP_GENFUNC Rot updateRot(const Rot &rot, const SP_REAL *delta) {
        return getRotAngle(getVec3(delta[0], delta[1], delta[2])) * rot;
    }

    // angle
    SP_GENFUNC SP_REAL getAngle(const Rot &rot, const int axis) {
        SP_ASSERT(axis >= 0 && axis < 3);

        const Vec3 v0 = getVec3(axis == 0 ? 1.0 : 0.0, axis == 1 ? 1.0 : 0.0, axis == 2 ? 1.0 : 0.0);
        const Vec3 v1 = rot * v0;
        const SP_REAL angle = acos(dotVec(v0, v1));
        return angle;
    }

    // angle
    SP_GENFUNC SP_REAL getAngle(const Vec2 &vec0, const Vec2 &vec1) {
        double ret = 0.0;
        const double a = normVec(vec0);
        const double b = normVec(vec1);
        if (a > SP_SMALL && b > SP_SMALL) {
            ret = acos(dotVec(vec0, vec1) / (a * b));
        }
        return static_cast<SP_REAL>(ret);
    }

    // angle
    SP_GENFUNC SP_REAL getAngle(const Vec3 &vec0, const Vec3 &vec1) {
        double ret = 0.0;
        const double a = normVec(vec0);
        const double b = normVec(vec1);
        if (a > SP_SMALL && b > SP_SMALL) {
            ret = acos(dotVec(vec0, vec1) / (a * b));
        }
        return static_cast<SP_REAL>(ret);
    }

    // dif
    SP_GENFUNC SP_REAL difRot(const Rot &rot0, const Rot &rot1) {
        return normVec(getAngle(rot0 * invRot(rot1)));
    }

    // dif
    SP_GENFUNC SP_REAL difRot(const Rot &rot0, const Rot &rot1, const int axis) {
        return getAngle(rot0 * invRot(rot1), axis);
    }


    //--------------------------------------------------------------------------------
    // pose
    //--------------------------------------------------------------------------------

    SP_GENFUNC Pose getPose(const Rot &rot, const Vec3 &trn) {
        Pose dst;
        dst.rot = nrmRot(rot);
        dst.pos = trn;

        return dst;
    }

    SP_GENFUNC Pose getPose(const Rot &rot) {
        return getPose(rot, getVec3(0.0, 0.0, 0.0));
    }

    SP_GENFUNC Pose getPose(const Vec3 &trn) {
        return getPose(zeroRot(), trn);
    }

    SP_GENFUNC Pose getPose(const SP_REAL *mat, const int rows, const int cols) {
        Pose dst;
        if ((rows == 3 || rows == 4) && cols == 4) {
            dst.rot = getRot(mat, rows, cols);
            dst.pos = getVec3(mat[0 * cols + 3], mat[1 * cols + 3], mat[2 * cols + 3]);
        }
        if ((rows == 6 && cols == 1) || (rows == 1 && cols == 6)) {
            Vec3 euler = getVec3(mat[0], mat[1], mat[2]);
            dst.rot = getRotEuler(euler);
            dst.pos = getVec3(mat[3], mat[4], mat[5]);
        }
        return dst;
    }

    SP_GENFUNC void getMat(SP_REAL *dst, const int rows, const int cols, const Pose &pose) {
        if ((rows == 3 || rows == 4) && cols == 4) {
            eyeMat(dst, rows, cols);
            getMat(dst, rows, cols, pose.rot);

            dst[0 * cols + 3] = pose.pos.x;
            dst[1 * cols + 3] = pose.pos.y;
            dst[2 * cols + 3] = pose.pos.z;
        }
        if ((rows == 6 && cols == 1) || (rows == 1 && cols == 6)) {
            const Vec3 euler = getEuler(pose.rot);
            dst[0] = euler.x;
            dst[1] = euler.y;
            dst[2] = euler.z;
            dst[3] = pose.pos.x;
            dst[4] = pose.pos.y;
            dst[5] = pose.pos.z;
        }
    }

    SP_GENFUNC Pose zeroPose() {
        return getPose(zeroRot(), getVec3(0.0, 0.0, 0.0));
    }

    SP_GENFUNC Pose invPose(const Pose &pose) {
        Pose dst;
        dst.rot = invRot(pose.rot);
        dst.pos = mulVec(mulRot(dst.rot, pose.pos), -1.0);

        return dst;
    }

    SP_GENFUNC Pose mulPose(const Pose &pose0, const Pose &pose1) {
        Pose dst;
        dst.rot = mulRot(pose0.rot, pose1.rot);
        dst.pos = addVec(mulRot(pose0.rot, pose1.pos), pose0.pos);

        return dst;
    }

    SP_GENFUNC Vec3 mulPose(const Pose &pose, const Vec3 &vec) {
        SP_REAL poseMat[3 * 4];
        getMat(poseMat, 3, 4, pose);

        return mulMat(poseMat, 3, 4, vec);
    }

    SP_GENFUNC Vec3 mulPose(const Pose &pose, const Vec2 &vec) {
        return mulPose(pose, getVec3(vec.x, vec.y, 0.0));
    }

    SP_GENFUNC VecPD3 mulPose(const Pose &pose, const VecPD3 &vec) {
        SP_REAL poseMat[3 * 4];
        getMat(poseMat, 3, 4, pose);

        return mulMat(poseMat, 3, 4, vec);
    }

    SP_GENFUNC Line3 mulPose(const Pose &pose, const Line3 &line) {
        SP_REAL poseMat[3 * 4];
        getMat(poseMat, 3, 4, pose);

        return mulMat(poseMat, 3, 4, line);
    }

    SP_GENFUNC Mesh3 mulPose(const Pose &pose, const Mesh3 &mesh) {
        SP_REAL poseMat[3 * 4];
        getMat(poseMat, 3, 4, pose);

        return mulMat(poseMat, 3, 4, mesh);
    }


    //--------------------------------------------------------------------------------
    // pose operator
    //--------------------------------------------------------------------------------

    SP_GENFUNC Vec3 operator * (const Pose &pose, const Vec3 &vec) {
        return mulPose(pose, vec);
    }

    SP_GENFUNC Vec3 operator * (const Pose &pose, const Vec2 &vec) {
        return mulPose(pose, vec);
    }

    SP_GENFUNC VecPD3 operator * (const Pose &pose, const VecPD3 &vec) {
        return mulPose(pose, vec);
    }

    SP_GENFUNC Line3 operator * (const Pose &pose, const Line3 &line) {
        return mulPose(pose, line);
    }

    SP_GENFUNC Mesh3 operator * (const Pose &pose, const Mesh3 &mesh) {
        return mulPose(pose, mesh);
    }

    SP_GENFUNC Pose operator * (const Pose &pose0, const Pose &pose1) {
        return mulPose(pose0, pose1);
    }

    SP_GENFUNC void operator *= (Pose &pose0, const Pose &pose1) {
        pose0 = mulPose(pose0, pose1);
    }

    //--------------------------------------------------------------------------------
    // pose util
    //--------------------------------------------------------------------------------

    // random unif
    SP_CPUFUNC Pose randuPose(const double rmax, const double tmax) {
        return getPose(randuRot(rmax), randuVec3(tmax, tmax, tmax));
    }

    // random gauss
    SP_CPUFUNC Pose randgPose(const double rmax, const double tmax) {
        return getPose(randgRot(rmax), randgVec3(tmax, tmax, tmax));
    }

    // update
    SP_GENFUNC Pose updatePose(const Pose &pose, const SP_REAL *delta) {
        Pose dst;
        dst.rot = updateRot(pose.rot, &delta[0]);
        dst.pos = pose.pos + getVec3(delta[3], delta[4], delta[5]);
        return dst;
    }


    SP_GENFUNC Pose getGeodesicPose(const int level, const int id, const double distance = 0.0) {
        const Vec3 v = getMeshCent(getGeodesicMesh(level, id)) * (-1.0);
        const Pose pose = getPose(getRotDirection(v), getVec3(0.0, 0.0, distance));
        return pose;
    }

    //--------------------------------------------------------------------------------
    // rotation / pose operator
    //--------------------------------------------------------------------------------

    SP_GENFUNC Pose operator * (const Rot &rot, const Pose &pose) {
        return mulPose(getPose(rot), pose);
    }

    SP_GENFUNC Pose operator * (const Pose &pose, const Rot &rot) {
        return mulPose(pose, getPose(rot));
    }

    SP_GENFUNC void operator *= (Pose &pose, const Rot &rot) {
        pose = mulPose(pose, getPose(rot));
    }

    //--------------------------------------------------------------------------------
    // jacob
    //--------------------------------------------------------------------------------

    SP_GENFUNC void jacobPosToNpx(SP_REAL *jacob, const Vec3 &pos) {
        SP_REAL divz = (pos.z != 0) ? 1.0 / pos.z : 0.0;

        jacob[0 * 3 + 0] = divz; jacob[0 * 3 + 1] = 0.0; jacob[0 * 3 + 2] = -pos.x * divz * divz;
        jacob[1 * 3 + 0] = 0.0; jacob[1 * 3 + 1] = divz; jacob[1 * 3 + 2] = -pos.y * divz * divz;
    }

    SP_GENFUNC void jacobPosToPix(SP_REAL *jacob, const CamParam &cam, const Vec3 &pos) {

        SP_REAL jPosToNpz[2 * 3] = { 0 };
        jacobPosToNpx(jPosToNpz, pos);

        SP_REAL jNpxToPix[2 * 2];
        jacobNpxToPix(jNpxToPix, cam, prjVec(pos));

        mulMat(jacob, 2, 3, jNpxToPix, 2, 2, jPosToNpz, 2, 3);
    }

    SP_GENFUNC void jacobPoseToPos(SP_REAL *jacob, const Pose &pose, const Vec3 &pos) {
        SP_REAL rmat[3 * 3];
        getMat(rmat, 3, 3, pose.rot);
        const Vec3 v = mulMat(rmat, 3, 3, pos);
        jacob[0 * 6 + 0] = +0.0; jacob[0 * 6 + 1] = +v.z; jacob[0 * 6 + 2] = -v.y;
        jacob[1 * 6 + 0] = -v.z; jacob[1 * 6 + 1] = +0.0; jacob[1 * 6 + 2] = +v.x;
        jacob[2 * 6 + 0] = +v.y; jacob[2 * 6 + 1] = -v.x; jacob[2 * 6 + 2] = +0.0;

        jacob[0 * 6 + 3] = 1.0; jacob[0 * 6 + 4] = 0.0; jacob[0 * 6 + 5] = 0.0;
        jacob[1 * 6 + 3] = 0.0; jacob[1 * 6 + 4] = 1.0; jacob[1 * 6 + 5] = 0.0;
        jacob[2 * 6 + 3] = 0.0; jacob[2 * 6 + 4] = 0.0; jacob[2 * 6 + 5] = 1.0;

    }

    SP_GENFUNC void jacobPoseToNpx(SP_REAL *jacob, const Pose &pose, const Vec3 &pos) {
        SP_REAL pmat[3 * 4];
        getMat(pmat, 3, 4, pose);

        SP_REAL jPoseToPos[3 * 6] = { 0 };
        jacobPoseToPos(jPoseToPos, pose, pos);

        SP_REAL jPosToNpx[2 * 3] = { 0 };
        jacobPosToNpx(jPosToNpx, mulMat(pmat, 3, 4, pos));

        mulMat(jacob, 2, 6, jPosToNpx, 2, 3, jPoseToPos, 3, 6);
    }

    SP_GENFUNC void jacobPoseToPix(SP_REAL *jacob, const CamParam &cam, const Pose &pose, const Vec3 &pos) {
        SP_REAL pmat[3 * 4];
        getMat(pmat, 3, 4, pose);

        SP_REAL jPoseToPos[3 * 6] = { 0 };
        jacobPoseToPos(jPoseToPos, pose, pos);

        SP_REAL jPosToPix[2 * 3];
        jacobPosToPix(jPosToPix, cam, mulMat(pmat, 3, 4, pos));

        mulMat(jacob, 2, 6, jPosToPix, 2, 3, jPoseToPos, 3, 6);
    }

}


#endif