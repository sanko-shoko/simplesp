//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_VEC_H__
#define __SP_VEC_H__

#include "spcore/spcom.h"
#include "spcore/spgen/spbase.h"

namespace sp {

    //--------------------------------------------------------------------------------
    // vector
    //--------------------------------------------------------------------------------

     // get vector
    SP_GENFUNC Vec2 getVec2(const double x, const double y) {
        Vec2 dst;
        dst.x = SP_RCAST(x);
        dst.y = SP_RCAST(y);
        return dst;
    }
    // get vector
    template<typename TYPE>
    SP_GENFUNC Vec2 getVec2(const TYPE *v) {
        return getVec2(v[0], v[1]);
    }

    // get vector
    SP_GENFUNC Vec3 getVec3(const double x, const double y, const double z) {
        Vec3 dst;
        dst.x = SP_RCAST(x);
        dst.y = SP_RCAST(y);
        dst.z = SP_RCAST(z);
        return dst;
    }
    // get vector
    template<typename TYPE>
    SP_GENFUNC Vec3 getVec3(const TYPE *v) {
        return getVec3(v[0], v[1], v[2]);
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
    SP_CPUFUNC Vec2 randuVec2(const double x, const double y, const unsigned int seed) {
        const unsigned int s0 = seed;
        const unsigned int s1 = snext(s0);
        return getVec2(randu(s0) * x, randu(s1) * y);
    }
    // random gauss
    SP_CPUFUNC Vec2 randgVec2(const double x, const double y) {
        return getVec2(randg() * x, randg() * y);
    }
    // random gauss
    SP_CPUFUNC Vec2 randgVec2(const double x, const double y, const unsigned int seed) {
        const unsigned int s0 = seed;
        const unsigned int s1 = snext(s0);
        return getVec2(randg(s0) * x, randg(s1) * y);
    }

    // random uniform
    SP_CPUFUNC Vec3 randuVec3(const double x, const double y, const double z) {
        return getVec3(randu() * x, randu() * y, randu() * z);
    }
    // random uniform
    SP_CPUFUNC Vec3 randuVec3(const double x, const double y, const double z, const unsigned int seed) {
        const unsigned int s0 = seed;
        const unsigned int s1 = snext(s0);
        const unsigned int s2 = snext(s1);
        return getVec3(randu(s0) * x, randu(s1) * y, randu(s2) * z);
    }
    // random gauss
    SP_CPUFUNC Vec3 randgVec3(const double x, const double y, const double z) {
        return getVec3(randg() * x, randg() * y, randg() * z);
    }
    // random gauss
    SP_CPUFUNC Vec3 randgVec3(const double x, const double y, const double z, const unsigned int seed) {
        const unsigned int s0 = seed;
        const unsigned int s1 = snext(s0);
        const unsigned int s2 = snext(s1);
        return getVec3(randg(s0) * x, randg(s1) * y, randg(s2) * z);
    }

    //--------------------------------------------------------------------------------
    // vector operator (function)
    //--------------------------------------------------------------------------------
    
    // addition
    SP_GENFUNC Vec2 addVec(const Vec2 &vec0, const Vec2 &vec1) {
        return getVec2(vec0.x + vec1.x, vec0.y + vec1.y);
    }
    // addition
    SP_GENFUNC Vec3 addVec(const Vec3 &vec0, const Vec3 &vec1) {
        return getVec3(vec0.x + vec1.x, vec0.y + vec1.y, vec0.z + vec1.z);
    }
    // addition
    SP_GENFUNC Vec2 addVec(const Vec2 &vec, const double val) {
        return getVec2(vec.x + val, vec.y + val);
    }
    // addition
    SP_GENFUNC Vec3 addVec(const Vec3 &vec, const double val) {
        return getVec3(vec.x + val, vec.y + val, vec.z + val);
    }
    // addition
    SP_GENFUNC Vec2 addVec(const double val, const Vec2 &vec) {
        return getVec2(val + vec.x, val + vec.y);
    }
    // addition
    SP_GENFUNC Vec3 addVec(const double val, const Vec3 &vec) {
        return getVec3(val + vec.x, val + vec.y, val + vec.z);
    }

    // subtraction
    SP_GENFUNC Vec2 subVec(const Vec2 &vec0, const Vec2 &vec1) {
        return getVec2(vec0.x - vec1.x, vec0.y - vec1.y);
    }
    // subtraction
    SP_GENFUNC Vec3 subVec(const Vec3 &vec0, const Vec3 &vec1) {
        return getVec3(vec0.x - vec1.x, vec0.y - vec1.y, vec0.z - vec1.z);
    }
    // subtraction
    SP_GENFUNC Vec2 subVec(const Vec2 &vec, const double val) {
        return getVec2(vec.x - val, vec.y - val);
    }
    // subtraction
    SP_GENFUNC Vec3 subVec(const Vec3 &vec, const double val) {
        return getVec3(vec.x - val, vec.y - val, vec.z - val);
    }
    // subtraction
    SP_GENFUNC Vec2 subVec(const double val, const Vec2 &vec) {
        return getVec2(val - vec.x, val - vec.y);
    }
    // subtraction
    SP_GENFUNC Vec3 subVec(const double val, const Vec3 &vec) {
        return getVec3(val - vec.x, val - vec.y, val - vec.z);
    }

    // multiple
    SP_GENFUNC Vec2 mulVec(const Vec2 &vec0, const Vec2 &vec1) {
        return getVec2(vec0.x * vec1.x, vec0.y * vec1.y);
    }
    // multiple
    SP_GENFUNC Vec3 mulVec(const Vec3 &vec0, const Vec3 &vec1) {
        return getVec3(vec0.x * vec1.x, vec0.y * vec1.y, vec0.z * vec1.z);
    }
    // multiple
    SP_GENFUNC Vec2 mulVec(const Vec2 &vec, const double val) {
        return getVec2(vec.x * val, vec.y * val);
    }
    // multiple
    SP_GENFUNC Vec3 mulVec(const Vec3 &vec, const double val) {
        return getVec3(vec.x * val, vec.y * val, vec.z * val);
    }
    // multiple
    SP_GENFUNC Vec2 mulVec(const double val, const Vec2 &vec) {
        return getVec2(val * vec.x, val * vec.y);
    }
    // multiple
    SP_GENFUNC Vec3 mulVec(const double val, const Vec3 &vec) {
        return getVec3(val * vec.x, val * vec.y, val * vec.z);
    }

    // division
    SP_GENFUNC Vec2 divVec(const Vec2 &vec0, const Vec2 &vec1) {
        SP_ASSERT(fabs(vec1.x) > SP_SMALL && fabs(vec1.y) > SP_SMALL);
        return getVec2(vec0.x / vec1.x, vec0.y / vec1.y);
    }
    // division
    SP_GENFUNC Vec3 divVec(const Vec3 &vec0, const Vec3 &vec1) {
        SP_ASSERT(fabs(vec1.x) > SP_SMALL && fabs(vec1.y) > SP_SMALL && fabs(vec1.z) > SP_SMALL);
        return getVec3(vec0.x / vec1.x, vec0.y / vec1.y, vec0.z / vec1.z);
    }
    // division
    SP_GENFUNC Vec2 divVec(const Vec2 &vec, const double val) {
        SP_ASSERT(fabs(val) > SP_SMALL);
        return getVec2(vec.x / val, vec.y / val);
    }
    // division
    SP_GENFUNC Vec3 divVec(const Vec3 &vec, const double val) {
        SP_ASSERT(fabs(val) > SP_SMALL);
        return getVec3(vec.x / val, vec.y / val, vec.z / val);
    }
    // division
    SP_GENFUNC Vec2 divVec(const double val, const Vec2 &vec) {
        SP_ASSERT(fabs(vec.x) > SP_SMALL && fabs(vec.y) > SP_SMALL);
        return getVec2(val / vec.x, val / vec.y);
    }
    // division
    SP_GENFUNC Vec3 divVec(const double val, const Vec3 &vec) {
        SP_ASSERT(fabs(vec.x) > SP_SMALL && fabs(vec.y) > SP_SMALL && fabs(vec.z) > SP_SMALL);
        return getVec3(val / vec.x, val / vec.y, val / vec.z);
    }

    //--------------------------------------------------------------------------------
    // vector operator
    //--------------------------------------------------------------------------------

    SP_GENFUNC Vec2 operator + (const Vec2 &vec0, const Vec2 &vec1) {
        return addVec(vec0, vec1);
    }
    SP_GENFUNC Vec3 operator + (const Vec3 &vec0, const Vec3 &vec1) {
        return addVec(vec0, vec1);
    }
    SP_GENFUNC Vec2 operator + (const Vec2 &vec, const double &val) {
        return addVec(vec, val);
    }
    SP_GENFUNC Vec3 operator + (const Vec3 &vec, const double &val) {
        return addVec(vec, val);
    }
    SP_GENFUNC Vec2 operator + (const double &val, const Vec2 &vec) {
        return addVec(val, vec);
    }
    SP_GENFUNC Vec3 operator + (const double &val, const Vec3 &vec) {
        return addVec(val, vec);
    }
    SP_GENFUNC void operator += (Vec2 &vec0, const Vec2 &vec1) {
        vec0 = addVec(vec0, vec1);
    }
    SP_GENFUNC void operator += (Vec3 &vec0, const Vec3 &vec1) {
        vec0 = addVec(vec0, vec1);
    }
    SP_GENFUNC Vec2 operator + (const Vec2 &vec) {
        return vec;
    }
    SP_GENFUNC Vec3 operator + (const Vec3 &vec) {
        return vec;
    }

    SP_GENFUNC Vec2 operator - (const Vec2 &vec0, const Vec2 &vec1) {
        return subVec(vec0, vec1);
    }
    SP_GENFUNC Vec3 operator - (const Vec3 &vec0, const Vec3 &vec1) {
        return subVec(vec0, vec1);
    }
    SP_GENFUNC Vec2 operator - (const Vec2 &vec, const double &val) {
        return subVec(vec, val);
    }
    SP_GENFUNC Vec3 operator - (const Vec3 &vec, const double &val) {
        return subVec(vec, val);
    }
    SP_GENFUNC Vec2 operator - (const double &val, const Vec2 &vec) {
        return subVec(val, vec);
    }
    SP_GENFUNC Vec3 operator - (const double &val, const Vec3 &vec) {
        return subVec(val, vec);
    }
    SP_GENFUNC void operator -= (Vec2 &vec0, const Vec2 &vec1) {
        vec0 = subVec(vec0, vec1);
    }
    SP_GENFUNC void operator -= (Vec3 &vec0, const Vec3 &vec1) {
        vec0 = subVec(vec0, vec1);
    }
    SP_GENFUNC Vec2 operator - (const Vec2 &vec) {
        return mulVec(vec, -1.0);
    }
    SP_GENFUNC Vec3 operator - (const Vec3 &vec) {
        return mulVec(vec, -1.0);
    }

    SP_GENFUNC Vec2 operator * (const Vec2 &vec0, const Vec2 &vec1) {
        return mulVec(vec0, vec1);
    }
    SP_GENFUNC Vec3 operator * (const Vec3 &vec0, const Vec3 &vec1) {
        return mulVec(vec0, vec1);
    }
    SP_GENFUNC Vec2 operator * (const Vec2 &vec, const double val) {
        return mulVec(vec, val);
    }
    SP_GENFUNC Vec3 operator * (const Vec3 &vec, const double val) {
        return mulVec(vec, val);
    }
    SP_GENFUNC Vec2 operator * (const double val, const Vec2 &vec) {
        return mulVec(vec, val);
    }
    SP_GENFUNC Vec3 operator * (const double val, const Vec3 &vec) {
        return mulVec(vec, val);
    }
    SP_GENFUNC void operator *= (Vec2 &vec, const double val) {
        vec = mulVec(vec, val);
    }
    SP_GENFUNC void operator *= (Vec3 &vec, const double val) {
        vec = mulVec(vec, val);
    }

    SP_GENFUNC Vec2 operator / (const Vec2 &vec0, const Vec2 &vec1) {
        return divVec(vec0, vec1);
    }
    SP_GENFUNC Vec3 operator / (const Vec3 &vec0, const Vec3 &vec1) {
        return divVec(vec0, vec1);
    }
    SP_GENFUNC Vec2 operator / (const Vec2 &vec, const double val){
        return divVec(vec, val);
    }
    SP_GENFUNC Vec3 operator / (const Vec3 &vec, const double val) {
        return divVec(vec, val);
    }
    SP_GENFUNC Vec2 operator / (const double val, const Vec2 &vec) {
        return divVec(val, vec);
    }
    SP_GENFUNC Vec3 operator / (const double val, const Vec3 &vec) {
        return divVec(val, vec);
    }
    SP_GENFUNC void operator /= (Vec2 &vec, const double val) {
        vec = divVec(vec, val);
    }
    SP_GENFUNC void operator /= (Vec3 &vec, const double val) {
        vec = divVec(vec, val);
    }

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
            dst.x = SP_RCAST(mat[0 * 2 + 0] * vec.x + mat[0 * 2 + 1] * vec.y);
            dst.y = SP_RCAST(mat[1 * 2 + 0] * vec.x + mat[1 * 2 + 1] * vec.y);
        }
        if (rows == 2 && cols == 3) {
            dst.x = SP_RCAST(mat[0 * 3 + 0] * vec.x + mat[0 * 3 + 1] * vec.y + mat[0 * 3 + 2]);
            dst.y = SP_RCAST(mat[1 * 3 + 0] * vec.x + mat[1 * 3 + 1] * vec.y + mat[1 * 3 + 2]);
        }
        if (rows == 3 && cols == 3) {
            const double scale = mat[2 * 3 + 0] * vec.x + mat[2 * 3 + 1] * vec.y + mat[2 * 3 + 2];
            dst.x = SP_RCAST((mat[0 * 3 + 0] * vec.x + mat[0 * 3 + 1] * vec.y + mat[0 * 3 + 2]) / scale);
            dst.y = SP_RCAST((mat[1 * 3 + 0] * vec.x + mat[1 * 3 + 1] * vec.y + mat[1 * 3 + 2]) / scale);
        }
        if (rows == 3 && cols == 4) {
            dst.x = SP_RCAST(mat[0 * 4 + 0] * vec.x + mat[0 * 4 + 1] * vec.y + mat[0 * 4 + 2] * 0.0 + mat[0 * 4 + 3]);
            dst.y = SP_RCAST(mat[1 * 4 + 0] * vec.x + mat[1 * 4 + 1] * vec.y + mat[1 * 4 + 2] * 0.0 + mat[1 * 4 + 3]);
        }
        if (rows == 4 && cols == 4) {
            const double scale = mat[3 * 4 + 0] * vec.x + mat[3 * 4 + 1] * vec.y + mat[3 * 4 + 2] * 0.0 + mat[3 * 4 + 3];
            dst.x = SP_RCAST((mat[0 * 4 + 0] * vec.x + mat[0 * 4 + 1] * vec.y + mat[0 * 4 + 2] * 0.0 + mat[0 * 4 + 3]) / scale);
            dst.y = SP_RCAST((mat[1 * 4 + 0] * vec.x + mat[1 * 4 + 1] * vec.y + mat[1 * 4 + 2] * 0.0 + mat[1 * 4 + 3]) / scale);
        }

        return dst;
    }

    SP_GENFUNC Vec3 mulMat(const SP_REAL *mat, const int rows, const int cols, const Vec3 &vec) {
        Vec3 dst = getVec3(0.0, 0.0, 0.0);
        if (rows == 3 && cols == 3) {
            dst.x = SP_RCAST(mat[0 * 3 + 0] * vec.x + mat[0 * 3 + 1] * vec.y + mat[0 * 3 + 2] * vec.z);
            dst.y = SP_RCAST(mat[1 * 3 + 0] * vec.x + mat[1 * 3 + 1] * vec.y + mat[1 * 3 + 2] * vec.z);
            dst.z = SP_RCAST(mat[2 * 3 + 0] * vec.x + mat[2 * 3 + 1] * vec.y + mat[2 * 3 + 2] * vec.z);
        }
        if (rows == 3 && cols == 4) {
            dst.x = SP_RCAST(mat[0 * 4 + 0] * vec.x + mat[0 * 4 + 1] * vec.y + mat[0 * 4 + 2] * vec.z + mat[0 * 4 + 3]);
            dst.y = SP_RCAST(mat[1 * 4 + 0] * vec.x + mat[1 * 4 + 1] * vec.y + mat[1 * 4 + 2] * vec.z + mat[1 * 4 + 3]);
            dst.z = SP_RCAST(mat[2 * 4 + 0] * vec.x + mat[2 * 4 + 1] * vec.y + mat[2 * 4 + 2] * vec.z + mat[2 * 4 + 3]);
        }
        if (rows == 4 && cols == 4) {
            const double scale = mat[3 * 4 + 0] * vec.x + mat[3 * 4 + 1] * vec.y + mat[3 * 4 + 2] * vec.z + mat[3 * 4 + 3];
            dst.x = SP_RCAST((mat[0 * 4 + 0] * vec.x + mat[0 * 4 + 1] * vec.y + mat[0 * 4 + 2] * vec.z + mat[0 * 4 + 3]) / scale);
            dst.y = SP_RCAST((mat[1 * 4 + 0] * vec.x + mat[1 * 4 + 1] * vec.y + mat[1 * 4 + 2] * vec.z + mat[1 * 4 + 3]) / scale);
            dst.z = SP_RCAST((mat[2 * 4 + 0] * vec.x + mat[2 * 4 + 1] * vec.y + mat[2 * 4 + 2] * vec.z + mat[2 * 4 + 3]) / scale);
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

    // addition
    SP_GENFUNC Line2 addLine(const Line2 &line, const Vec2 &vec) {
        return getLine2(line.pos[0] + vec, line.pos[1] + vec);
    }
    // addition
    SP_GENFUNC Line3 addLine(const Line3 &line, const Vec3 &vec) {
        return getLine3(line.pos[0] + vec, line.pos[1] + vec);
    }

    // subtraction
    SP_GENFUNC Line2 subLine(const Line2 &line, const Vec2 &vec) {
        return getLine2(line.pos[0] - vec, line.pos[1] - vec);
    }
    // subtraction
    SP_GENFUNC Line3 subLine(const Line3 &line, const Vec3 &vec) {
        return getLine3(line.pos[0] - vec, line.pos[1] - vec);
    }

    // multiple
    SP_GENFUNC Line2 mulLine(const Line2 &line, const double val) {
        return getLine2(line.pos[0] * val, line.pos[1] * val);
    }
    // multiple
    SP_GENFUNC Line3 mulLine(const Line3 &line, const double val) {
        return getLine3(line.pos[0] * val, line.pos[1] * val);
    }

    // division
    SP_GENFUNC Line2 divLine(const Line2 &line, const double val) {
        return getLine2(line.pos[0] / val, line.pos[1] / val);
    }
    // division
    SP_GENFUNC Line3 divLine(const Line3 &line, const double val) {
        return getLine3(line.pos[0] / val, line.pos[1] / val);
    }

    //--------------------------------------------------------------------------------
    // line operator
    //--------------------------------------------------------------------------------

    SP_GENFUNC Line2 operator + (const Line2 &line, const Vec2 &vec) {
        return addLine(line, vec);
    }
    SP_GENFUNC Line3 operator + (const Line3 &line, const Vec3 &vec) {
        return addLine(line, vec);
    }

    SP_GENFUNC Line2 operator + (const Line2 &line) {
        return line;
    }
    SP_GENFUNC Line3 operator + (const Line3 &line) {
        return line;
    }

    SP_GENFUNC Line2 operator - (const Line2 &line, const Vec2 &vec) {
        return subLine(line, vec);
    }
    SP_GENFUNC Line3 operator - (const Line3 &line, const Vec3 &vec) {
        return subLine(line, vec);
    }

    SP_GENFUNC Line2 operator - (const Line2 &line) {
        return mulLine(line, -1.0);
    }
    SP_GENFUNC Line3 operator - (const Line3 &line) {
        return mulLine(line, -1.0);
    }

    SP_GENFUNC Line2 operator * (const Line2 &line, const double val) {
        return mulLine(line, val);
    }
    SP_GENFUNC Line3 operator * (const Line3 &line, const double val) {
        return mulLine(line, val);
    }

    SP_GENFUNC Line2 operator * (const double val, const Line2 &line) {
        return mulLine(line, val);
    }
    SP_GENFUNC Line3 operator * (const double val, const Line3 &line) {
        return mulLine(line, val);
    }

    SP_GENFUNC Line2 operator / (const Line2 &line, const double val) {
        return divLine(line, val);
    }
    SP_GENFUNC Line3 operator / (const Line3 &line, const double val) {
        return divLine(line, val);
    }

    SP_GENFUNC void operator += (Line2 &line, const Vec2 &vec) {
        line = addLine(line, vec);
    }
    SP_GENFUNC void operator += (Line3 &line, const Vec3 &vec) {
        line = addLine(line, vec);
    }

    SP_GENFUNC void operator -= (Line2 &line, const Vec2 &vec) {
        line = subLine(line, vec);
    }
    SP_GENFUNC void operator -= (Line3 &line, const Vec3 &vec) {
        line = subLine(line, vec);
    }

    SP_GENFUNC void operator *= (Line2 &line, const double val) {
        line = mulLine(line, val);
    }
    SP_GENFUNC void operator *= (Line3 &line, const double val) {
        line = mulLine(line, val);
    }

    SP_GENFUNC void operator /= (Line2 &line, const double val) {
        line = divLine(line, val);
    }
    SP_GENFUNC void operator /= (Line3 &line, const double val) {
        line = divLine(line, val);
    }

    //--------------------------------------------------------------------------------
    // line util
    //--------------------------------------------------------------------------------
    
    // projection vec3 to vec2
    SP_GENFUNC Line2 prjVec(const Line3 &line, const bool pers = true) {
        return getLine2(prjVec(line.pos[0], pers), prjVec(line.pos[1], pers));
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
            if(s < 0.0){
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

    // addition
    SP_GENFUNC Mesh2 addMesh(const Mesh2 &mesh, const Vec2 vec) {
        Mesh2 dst;
        dst.pos[0] = mesh.pos[0] + vec;
        dst.pos[1] = mesh.pos[1] + vec;
        dst.pos[2] = mesh.pos[2] + vec;
        return dst;
    }

    // addition
    SP_GENFUNC Mesh3 addMesh(const Mesh3 &mesh, const Vec3 vec) {
        Mesh3 dst;
        dst.pos[0] = mesh.pos[0] + vec;
        dst.pos[1] = mesh.pos[1] + vec;
        dst.pos[2] = mesh.pos[2] + vec;
        return dst;
    }

    // subtraction
    SP_GENFUNC Mesh2 subMesh(const Mesh2 &mesh, const Vec2 vec) {
        Mesh2 dst;
        dst.pos[0] = mesh.pos[0] - vec;
        dst.pos[1] = mesh.pos[1] - vec;
        dst.pos[2] = mesh.pos[2] - vec;
        return dst;
    }

    // subtraction
    SP_GENFUNC Mesh3 subMesh(const Mesh3 &mesh, const Vec3 vec) {
        Mesh3 dst;
        dst.pos[0] = mesh.pos[0] - vec;
        dst.pos[1] = mesh.pos[1] - vec;
        dst.pos[2] = mesh.pos[2] - vec;
        return dst;
    }

    // multiple
    SP_GENFUNC Mesh2 mulMesh(const Mesh2 &mesh, const double val) {
        Mesh2 dst;
        dst.pos[0] = mesh.pos[0] * val;
        dst.pos[1] = mesh.pos[1] * val;
        dst.pos[2] = mesh.pos[2] * val;
        return dst;
    }

    // multiple
    SP_GENFUNC Mesh3 mulMesh(const Mesh3 &mesh, const double val) {
        Mesh3 dst;
        dst.pos[0] = mesh.pos[0] * val;
        dst.pos[1] = mesh.pos[1] * val;
        dst.pos[2] = mesh.pos[2] * val;
        return dst;
    }

    // division
    SP_GENFUNC Mesh2 divMesh(const Mesh2 &mesh, const double val) {
        Mesh2 dst;
        dst.pos[0] = mesh.pos[0] / val;
        dst.pos[1] = mesh.pos[1] / val;
        dst.pos[2] = mesh.pos[2] / val;
        return dst;
    }

    // division
    SP_GENFUNC Mesh3 divMesh(const Mesh3 &mesh, const double val) {
        Mesh3 dst;
        dst.pos[0] = mesh.pos[0] / val;
        dst.pos[1] = mesh.pos[1] / val;
        dst.pos[2] = mesh.pos[2] / val;
        return dst;
    }


    //--------------------------------------------------------------------------------
    // mesh operator
    //--------------------------------------------------------------------------------

    SP_GENFUNC Mesh2 operator + (const Mesh2 &mesh, const Vec2 vec) {
        return addMesh(mesh, vec);
    }

    SP_GENFUNC Mesh3 operator + (const Mesh3 &mesh, const Vec3 vec) {
        return addMesh(mesh, vec);
    }

    SP_GENFUNC Mesh2 operator - (const Mesh2 &mesh, const Vec2 vec) {
        return subMesh(mesh, vec);
    }

    SP_GENFUNC Mesh3 operator - (const Mesh3 &mesh, const Vec3 vec) {
        return subMesh(mesh, vec);
    }

    SP_GENFUNC Mesh2 operator * (const Mesh2 &mesh, const double val) {
        return mulMesh(mesh, val);
    }

    SP_GENFUNC Mesh3 operator * (const Mesh3 &mesh, const double val) {
        return mulMesh(mesh, val);
    }

    SP_GENFUNC Mesh2 operator / (const Mesh2 &mesh, const double val) {
        return divMesh(mesh, val);
    }

    SP_GENFUNC Mesh3 operator / (const Mesh3 &mesh, const double val) {
        return divMesh(mesh, val);
    }

    SP_GENFUNC void operator += (Mesh2 &mesh, const Vec2 vec) {
        mesh = addMesh(mesh, vec);
    }

    SP_GENFUNC void operator += (Mesh3 &mesh, const Vec3 vec) {
        mesh = addMesh(mesh, vec);
    }

    SP_GENFUNC void operator -= (Mesh2 &mesh, const Vec2 vec) {
        mesh = subMesh(mesh, vec);
    }

    SP_GENFUNC void operator -= (Mesh3 &mesh, const Vec3 vec) {
        mesh = subMesh(mesh, vec);
    }

    SP_GENFUNC void operator *= (Mesh2 &mesh, const double val) {
        mesh = mulMesh(mesh, val);
    }

    SP_GENFUNC void operator *= (Mesh3 &mesh, const double val) {
        mesh = mulMesh(mesh, val);
    }

    SP_GENFUNC void operator /= (Mesh2 &mesh, const double val) {
        mesh = divMesh(mesh, val);
    }

    SP_GENFUNC void operator /= (Mesh3 &mesh, const double val) {
        mesh = divMesh(mesh, val);
    }


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
                acsv(dst.pos[0], i) = minval(acsv(dst.pos[0], i), acsv(pos, i));
                acsv(dst.pos[1], i) = maxval(acsv(dst.pos[1], i), acsv(pos, i));
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
            acsv(dst.pos[0], i) = minval(acsv(dst.pos[0], i), acsv(box1.pos[0], i));
            acsv(dst.pos[1], i) = maxval(acsv(dst.pos[1], i), acsv(box1.pos[1], i));
        }
        return dst;
    }

    SP_GENFUNC Box3 orBox(const Box3 &box0, const Box3 &box1) {
        Box3 dst = box0;
        for (int i = 0; i < 3; i++) {
            acsv(dst.pos[0], i) = minval(acsv(dst.pos[0], i), acsv(box1.pos[0], i));
            acsv(dst.pos[1], i) = maxval(acsv(dst.pos[1], i), acsv(box1.pos[1], i));
        }
        return dst;
    }

    SP_GENFUNC Box3 orBox(const Box3 &box, const Vec3 &vec) {
        Box3 dst = box;
        for (int i = 0; i < 3; i++) {
            acsv(dst.pos[0], i) = minval(acsv(dst.pos[0], i), acsv(vec, i));
            acsv(dst.pos[1], i) = maxval(acsv(dst.pos[1], i), acsv(vec, i));
        }
        return dst;
    }

    SP_GENFUNC Box3 orBox(const Box3 &box, const Mesh3 &mesh) {
        Box3 dst = box;
        for (int p = 0; p < 3; p++) {
            const Vec3 &pos = mesh.pos[p];
            for (int i = 0; i < 3; i++) {
                acsv(dst.pos[0], i) = minval(acsv(dst.pos[0], i), acsv(pos, i));
                acsv(dst.pos[1], i) = maxval(acsv(dst.pos[1], i), acsv(pos, i));
            }
        }
        return dst;
    }

    SP_GENFUNC SP_REAL getBoxArea(const Box3 &box) {
        const Vec3 d = box.pos[1] - box.pos[0];
        return (d.x * d.y + d.y * d.z + d.z * d.x) * 2.0;
    }


}


#endif