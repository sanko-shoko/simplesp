﻿//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_VEC_H__
#define __SP_VEC_H__

#include "spcore/spcom.h"
#include "spcore/spgen/spbase.h"
#include "spcore/spgen/spmath.h"

namespace sp {

    //--------------------------------------------------------------------------------
    // vector
    //--------------------------------------------------------------------------------

     // get vector
    SP_GENFUNC Vec2 getVec2(const double x, const double y) {
        Vec2 dst;
        dst.x = SP_CAST(x);
        dst.y = SP_CAST(y);
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
        dst.x = SP_CAST(x);
        dst.y = SP_CAST(y);
        dst.z = SP_CAST(z);
        return dst;
    }

    // get vector
    template<typename TYPE>
    SP_GENFUNC Vec3 getVec3(const TYPE *v) {
        return getVec3(v[0], v[1], v[2]);
    }

    // get vector
    SP_GENFUNC Vec3 getVec3(const Vec2 &vec, const double z) {
        Vec3 dst;
        dst.x = SP_CAST(vec.x);
        dst.y = SP_CAST(vec.y);
        dst.z = SP_CAST(z);
        return dst;
    }

    // get vector
    SP_GENFUNC Vec3 getVec3(const Col3 &col) {
        Vec3 dst;
        dst.x = static_cast<SP_REAL>(col.r) / SP_BYTEMAX;
        dst.y = static_cast<SP_REAL>(col.g) / SP_BYTEMAX;
        dst.z = static_cast<SP_REAL>(col.b) / SP_BYTEMAX;
        return dst;
    }

    // addition
    SP_GENFUNC Vec2 addVec2(const Vec2 &vec0, const Vec2 &vec1) {
        return getVec2(vec0.x + vec1.x, vec0.y + vec1.y);
    }

    // addition
    SP_GENFUNC Vec3 addVec3(const Vec3 &vec0, const Vec3 &vec1) {
        return getVec3(vec0.x + vec1.x, vec0.y + vec1.y, vec0.z + vec1.z);
    }

    // subtraction
    SP_GENFUNC Vec2 subVec2(const Vec2 &vec0, const Vec2 &vec1) {
        return getVec2(vec0.x - vec1.x, vec0.y - vec1.y);
    }

    // subtraction
    SP_GENFUNC Vec3 subVec3(const Vec3 &vec0, const Vec3 &vec1) {
        return getVec3(vec0.x - vec1.x, vec0.y - vec1.y, vec0.z - vec1.z);
    }

    // multiple
    SP_GENFUNC Vec2 mulVec2(const Vec2 &vec, const double val) {
        return getVec2(vec.x * val, vec.y * val);
    }

    // multiple
    SP_GENFUNC Vec3 mulVec3(const Vec3 &vec, const double val) {
        return getVec3(vec.x * val, vec.y * val, vec.z * val);
    }

    // division
    SP_GENFUNC Vec2 divVec2(const Vec2 &vec, const double val) {
        return (val != 0.0) ? mulVec2(vec, 1.0 / val) : vec;
    }

    // division
    SP_GENFUNC Vec3 divVec3(const Vec3 &vec, const double val) {
        return (val != 0.0) ? mulVec3(vec, 1.0 / val) : vec;
    }

    // compare
    SP_GENFUNC bool cmpVec2(const Vec2 &vec0, const Vec2 &vec1, const double t = 1.0e-10) {
        return cmpVal(vec0.x, vec1.x, t) & cmpVal(vec0.y, vec1.y, t);
    }
    // compare
    SP_GENFUNC bool cmpVec3(const Vec3 &vec0, const Vec3 &vec1, const double t = 1.0e-10) {
        return cmpVal(vec0.x, vec1.x, t) & cmpVal(vec0.y, vec1.y, t) & cmpVal(vec0.z, vec1.z, t);
    }

    //--------------------------------------------------------------------------------
    // vector operator
    //--------------------------------------------------------------------------------

    SP_GENFUNC Vec2 operator + (const Vec2 &vec0, const Vec2 &vec1) {
        return addVec2(vec0, vec1);
    }

    SP_GENFUNC Vec3 operator + (const Vec3 &vec0, const Vec3 &vec1) {
        return addVec3(vec0, vec1);
    }

    SP_GENFUNC Vec2 operator + (const Vec2 &vec) {
        return vec;
    }

    SP_GENFUNC Vec3 operator + (const Vec3 &vec) {
        return vec;
    }

    SP_GENFUNC Vec2 operator - (const Vec2 &vec0, const Vec2 &vec1) {
        return subVec2(vec0, vec1);
    }

    SP_GENFUNC Vec3 operator - (const Vec3 &vec0, const Vec3 &vec1) {
        return subVec3(vec0, vec1);
    }

    SP_GENFUNC Vec2 operator - (const Vec2 &vec) {
        return mulVec2(vec, -1.0);
    }

    SP_GENFUNC Vec3 operator - (const Vec3 &vec) {
        return mulVec3(vec, -1.0);
    }

    SP_GENFUNC Vec2 operator * (const Vec2 &vec, const double val) {
        return mulVec2(vec, val);
    }

    SP_GENFUNC Vec3 operator * (const Vec3 &vec, const double val) {
        return mulVec3(vec, val);
    }

    SP_GENFUNC Vec2 operator * (const double val, const Vec2 &vec) {
        return mulVec2(vec, val);
    }

    SP_GENFUNC Vec3 operator * (const double val, const Vec3 &vec) {
        return mulVec3(vec, val);
    }

    SP_GENFUNC Vec2 operator / (const Vec2 &vec, const double val){
        return divVec2(vec, val);
    }

    SP_GENFUNC Vec3 operator / (const Vec3 &vec, const double val){
        return divVec3(vec, val);
    }

    SP_GENFUNC void operator += (Vec2 &vec0, const Vec2 &vec1) {
        vec0 = addVec2(vec0, vec1);
    }

    SP_GENFUNC void operator += (Vec3 &vec0, const Vec3 &vec1) {
        vec0 = addVec3(vec0, vec1);
    }

    SP_GENFUNC void operator -= (Vec2 &vec0, const Vec2 &vec1) {
        vec0 = subVec2(vec0, vec1);
    }

    SP_GENFUNC void operator -= (Vec3 &vec0, const Vec3 &vec1) {
        vec0 = subVec3(vec0, vec1);
    }

    SP_GENFUNC void operator *= (Vec2 &vec, const double val) {
        vec = mulVec2(vec, val);
    }

    SP_GENFUNC void operator *= (Vec3 &vec, const double val) {
        vec = mulVec3(vec, val);
    }

    SP_GENFUNC void operator /= (Vec2 &vec, const double val) {
        vec = divVec2(vec, val);
    }

    SP_GENFUNC void operator /= (Vec3 &vec, const double val) {
        vec = divVec3(vec, val);
    }

    SP_GENFUNC bool operator == (const Vec2 &vec0, const Vec2 &vec1) {
        return cmpVec2(vec0, vec1);
    }

    SP_GENFUNC bool operator != (const Vec2 &vec0, const Vec2 &vec1) {
        return !cmpVec2(vec0, vec1);
    }

    SP_GENFUNC bool operator == (const Vec3 &vec0, const Vec3 &vec1) {
        return cmpVec3(vec0, vec1);
    }

    SP_GENFUNC bool operator != (const Vec3 &vec0, const Vec3 &vec1) {
        return !cmpVec3(vec0, vec1);
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
    SP_GENFUNC Vec3 crsVec(const Vec2 &vec0, const Vec2 &vec1) {
        return getVec3(0.0, 0.0, vec0.x * vec1.y - vec0.y * vec1.x);
    }

    // cross production
    SP_GENFUNC Vec3 crsVec(const Vec3 &vec0, const Vec3 &vec1) {
        return getVec3(vec0.y * vec1.z - vec0.z * vec1.y, vec0.z * vec1.x - vec0.x * vec1.z, vec0.x * vec1.y - vec0.y * vec1.x);
    }

    // projection vec3 to vec2
    SP_GENFUNC Vec2 prjVec(const Vec3 &vec, const bool pers = true) {
        return (pers == true) ? getVec2(vec.x, vec.y) / vec.z : getVec2(vec.x, vec.y);
    }

    // square
    SP_GENFUNC SP_REAL sqVec(const Vec2 &vec) {
        return dotVec(vec, vec);
    }

    // square
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
        return vec / normVec(vec);
    }

    // unit vector
    SP_GENFUNC Vec3 unitVec(const Vec3 &vec) {
        return vec / normVec(vec);
    }

    // random uniform
    SP_CPUFUNC Vec2 randuVec2(const double x, const double y) {
        return getVec2(randu() * x, randu() * y);
    }
    
    // random gauss
    SP_CPUFUNC Vec2 randgVec2(const double x, const double y) {
        return getVec2(randg() * x, randg() * y);
    }

    // random uniform
    SP_CPUFUNC Vec3 randuVec3(const double x, const double y, const double z) {
        return getVec3(randu() * x, randu() * y, randu() * z);
    }

    // random gauss
    SP_CPUFUNC Vec3 randgVec3(const double x, const double y, const double z) {
        return getVec3(randg() * x, randg() * y, randg() * z);
    }

    // round
    SP_GENFUNC Vec2 round(const Vec2 &vec) {
        return getVec2(round(vec.x), round(vec.y));
    }

    // round
    SP_GENFUNC Vec3 round(const Vec3 &vec) {
        return getVec3(round(vec.x), round(vec.y), round(vec.z));
    }

    // angle
    SP_GENFUNC SP_REAL getAngle(const Vec2 &vec0, const Vec2 &vec1) {
        double ret = 0.0;
        const double a = normVec(vec0);
        const double b = normVec(vec1);
        if (a > SP_SMALL && b > SP_SMALL) {
            ret = acos(dotVec(vec0, vec1) / (a * b));
        }
        return SP_CAST(ret);
    }

    // angle
    SP_GENFUNC SP_REAL getAngle(const Vec3 &vec0, const Vec3 &vec1) {
        double ret = 0.0;
        const double a = normVec(vec0);
        const double b = normVec(vec1);
        if (a > SP_SMALL && b > SP_SMALL) {
            ret = acos(dotVec(vec0, vec1) / (a * b));
        }
        return SP_CAST(ret);
    }


    //--------------------------------------------------------------------------------
    // matrix * vector
    //--------------------------------------------------------------------------------

    SP_GENFUNC Vec2 mulMat(const SP_REAL *mat, const int rows, const int cols, const Vec2 &vec) {
        Vec2 dst = getVec2(0.0, 0.0);
        if (rows == 2 && cols == 2) {
            dst.x = SP_CAST(mat[0 * 2 + 0] * vec.x + mat[0 * 2 + 1] * vec.y);
            dst.y = SP_CAST(mat[1 * 2 + 0] * vec.x + mat[1 * 2 + 1] * vec.y);
        }
        if (rows == 2 && cols == 3) {
            dst.x = SP_CAST(mat[0 * 3 + 0] * vec.x + mat[0 * 3 + 1] * vec.y + mat[0 * 3 + 2]);
            dst.y = SP_CAST(mat[1 * 3 + 0] * vec.x + mat[1 * 3 + 1] * vec.y + mat[1 * 3 + 2]);
        }
        if (rows == 3 && cols == 3) {
            const double scale = mat[2 * 3 + 0] * vec.x + mat[2 * 3 + 1] * vec.y + mat[2 * 3 + 2];
            dst.x = SP_CAST((mat[0 * 3 + 0] * vec.x + mat[0 * 3 + 1] * vec.y + mat[0 * 3 + 2]) / scale);
            dst.y = SP_CAST((mat[1 * 3 + 0] * vec.x + mat[1 * 3 + 1] * vec.y + mat[1 * 3 + 2]) / scale);
        }
        if (rows == 3 && cols == 4) {
            dst.x = SP_CAST(mat[0 * 4 + 0] * vec.x + mat[0 * 4 + 1] * vec.y + mat[0 * 4 + 2] * 0.0 + mat[0 * 4 + 3]);
            dst.y = SP_CAST(mat[1 * 4 + 0] * vec.x + mat[1 * 4 + 1] * vec.y + mat[1 * 4 + 2] * 0.0 + mat[1 * 4 + 3]);
        }
        if (rows == 4 && cols == 4) {
            const double scale = mat[3 * 4 + 0] * vec.x + mat[3 * 4 + 1] * vec.y + mat[3 * 4 + 2] * 0.0 + mat[3 * 4 + 3];
            dst.x = SP_CAST((mat[0 * 4 + 0] * vec.x + mat[0 * 4 + 1] * vec.y + mat[0 * 4 + 2] * 0.0 + mat[0 * 4 + 3]) / scale);
            dst.y = SP_CAST((mat[1 * 4 + 0] * vec.x + mat[1 * 4 + 1] * vec.y + mat[1 * 4 + 2] * 0.0 + mat[1 * 4 + 3]) / scale);
        }

        return dst;
    }

    SP_GENFUNC Vec3 mulMat(const SP_REAL *mat, const int rows, const int cols, const Vec3 &vec) {
        Vec3 dst = getVec3(0.0, 0.0, 0.0);
        if (rows == 3 && cols == 3) {
            dst.x = SP_CAST(mat[0 * 3 + 0] * vec.x + mat[0 * 3 + 1] * vec.y + mat[0 * 3 + 2] * vec.z);
            dst.y = SP_CAST(mat[1 * 3 + 0] * vec.x + mat[1 * 3 + 1] * vec.y + mat[1 * 3 + 2] * vec.z);
            dst.z = SP_CAST(mat[2 * 3 + 0] * vec.x + mat[2 * 3 + 1] * vec.y + mat[2 * 3 + 2] * vec.z);
        }
        if (rows == 3 && cols == 4) {
            dst.x = SP_CAST(mat[0 * 4 + 0] * vec.x + mat[0 * 4 + 1] * vec.y + mat[0 * 4 + 2] * vec.z + mat[0 * 4 + 3]);
            dst.y = SP_CAST(mat[1 * 4 + 0] * vec.x + mat[1 * 4 + 1] * vec.y + mat[1 * 4 + 2] * vec.z + mat[1 * 4 + 3]);
            dst.z = SP_CAST(mat[2 * 4 + 0] * vec.x + mat[2 * 4 + 1] * vec.y + mat[2 * 4 + 2] * vec.z + mat[2 * 4 + 3]);
        }
        if (rows == 4 && cols == 4) {
            const double scale = mat[3 * 4 + 0] * vec.x + mat[3 * 4 + 1] * vec.y + mat[3 * 4 + 2] * vec.z + mat[3 * 4 + 3];
            dst.x = SP_CAST((mat[0 * 4 + 0] * vec.x + mat[0 * 4 + 1] * vec.y + mat[0 * 4 + 2] * vec.z + mat[0 * 4 + 3]) / scale);
            dst.y = SP_CAST((mat[1 * 4 + 0] * vec.x + mat[1 * 4 + 1] * vec.y + mat[1 * 4 + 2] * vec.z + mat[1 * 4 + 3]) / scale);
            dst.z = SP_CAST((mat[2 * 4 + 0] * vec.x + mat[2 * 4 + 1] * vec.y + mat[2 * 4 + 2] * vec.z + mat[2 * 4 + 3]) / scale);
        }

        return dst;
    }


    //--------------------------------------------------------------------------------
    // vector pn (position and direction)
    //--------------------------------------------------------------------------------

    // get vector pd
    SP_GENFUNC VecPN2 getVecPN2(const Vec2 &vtx, const Vec2 &nrm) {
        VecPN2 dst;
        dst.pos = vtx;
        dst.nrm = nrm;
        return dst;
    }

    // get vector pd
    SP_GENFUNC VecPN3 getVecPN3(const Vec3 &vtx, const Vec3 &nrm) {
        VecPN3 dst;
        dst.pos = vtx;
        dst.nrm = nrm;
        return dst;
    }


    //--------------------------------------------------------------------------------
    // matrix * vector pn
    //--------------------------------------------------------------------------------

    SP_GENFUNC VecPN2 mulMat(const SP_REAL *mat, const int rows, const int cols, const VecPN2 &vec) {
        VecPN2 dst;

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

        dst.nrm = unitVec(mulMat(rot, 2, 2, vec.nrm));

        return dst;
    }


    SP_GENFUNC VecPN3 mulMat(const SP_REAL *mat, const int rows, const int cols, const VecPN3 &vec) {
        VecPN3 dst;

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

        dst.nrm = unitVec(mulMat(rot, 3, 3, vec.nrm));

        return dst;
    }

    //--------------------------------------------------------------------------------
    // extract z element
    //--------------------------------------------------------------------------------

    SP_GENFUNC SP_REAL extractZ(const VecPN3 &src) {
        return src.pos.z;
    }

    SP_GENFUNC SP_REAL extractZ(const Vec3 &src) {
        return src.z;
    }

    SP_GENFUNC SP_REAL extractZ(const SP_REAL &src) {
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
    // line util
    //--------------------------------------------------------------------------------
    
    // projection vec3 to vec2
    SP_GENFUNC Line2 prjVec(const Line3 &line, const bool pers = true) {
        return getLine2(prjVec(line.pos[0], pers), prjVec(line.pos[1], pers));
    }

    // norm from line to
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
    SP_GENFUNC Mesh2 addMesh2(const Mesh2 &mesh, const Vec2 vec) {
        Mesh2 dst;
        dst.pos[0] = mesh.pos[0] + vec;
        dst.pos[1] = mesh.pos[1] + vec;
        dst.pos[2] = mesh.pos[2] + vec;
        return dst;
    }

    // addition
    SP_GENFUNC Mesh3 addMesh3(const Mesh3 &mesh, const Vec3 vec) {
        Mesh3 dst;
        dst.pos[0] = mesh.pos[0] + vec;
        dst.pos[1] = mesh.pos[1] + vec;
        dst.pos[2] = mesh.pos[2] + vec;
        return dst;
    }

    // subtraction
    SP_GENFUNC Mesh2 subMesh2(const Mesh2 &mesh, const Vec2 vec) {
        Mesh2 dst;
        dst.pos[0] = mesh.pos[0] - vec;
        dst.pos[1] = mesh.pos[1] - vec;
        dst.pos[2] = mesh.pos[2] - vec;
        return dst;
    }

    // subtraction
    SP_GENFUNC Mesh3 subMesh3(const Mesh3 &mesh, const Vec3 vec) {
        Mesh3 dst;
        dst.pos[0] = mesh.pos[0] - vec;
        dst.pos[1] = mesh.pos[1] - vec;
        dst.pos[2] = mesh.pos[2] - vec;
        return dst;
    }

    // multiple
    SP_GENFUNC Mesh2 mulMesh2(const Mesh2 &mesh, const double val) {
        Mesh2 dst;
        dst.pos[0] = mesh.pos[0] * val;
        dst.pos[1] = mesh.pos[1] * val;
        dst.pos[2] = mesh.pos[2] * val;
        return dst;
    }

    // multiple
    SP_GENFUNC Mesh3 mulMesh3(const Mesh3 &mesh, const double val) {
        Mesh3 dst;
        dst.pos[0] = mesh.pos[0] * val;
        dst.pos[1] = mesh.pos[1] * val;
        dst.pos[2] = mesh.pos[2] * val;
        return dst;
    }

    // division
    SP_GENFUNC Mesh2 divMesh2(const Mesh2 &mesh, const double val) {
        Mesh2 dst;
        dst.pos[0] = mesh.pos[0] / val;
        dst.pos[1] = mesh.pos[1] / val;
        dst.pos[2] = mesh.pos[2] / val;
        return dst;
    }

    // division
    SP_GENFUNC Mesh3 divMesh3(const Mesh3 &mesh, const double val) {
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
        return addMesh2(mesh, vec);
    }

    SP_GENFUNC Mesh3 operator + (const Mesh3 &mesh, const Vec3 vec) {
        return addMesh3(mesh, vec);
    }

    SP_GENFUNC Mesh2 operator - (const Mesh2 &mesh, const Vec2 vec) {
        return subMesh2(mesh, vec);
    }

    SP_GENFUNC Mesh3 operator - (const Mesh3 &mesh, const Vec3 vec) {
        return subMesh3(mesh, vec);
    }

    SP_GENFUNC Mesh2 operator * (const Mesh2 &mesh, const double val) {
        return mulMesh2(mesh, val);
    }

    SP_GENFUNC Mesh3 operator * (const Mesh3 &mesh, const double val) {
        return mulMesh3(mesh, val);
    }

    SP_GENFUNC Mesh2 operator / (const Mesh2 &mesh, const double val) {
        return divMesh2(mesh, val);
    }

    SP_GENFUNC Mesh3 operator / (const Mesh3 &mesh, const double val) {
        return divMesh3(mesh, val);
    }

    SP_GENFUNC void operator += (Mesh2 &mesh, const Vec2 vec) {
        mesh = addMesh2(mesh, vec);
    }

    SP_GENFUNC void operator += (Mesh3 &mesh, const Vec3 vec) {
        mesh = addMesh3(mesh, vec);
    }

    SP_GENFUNC void operator -= (Mesh2 &mesh, const Vec2 vec) {
        mesh = subMesh2(mesh, vec);
    }

    SP_GENFUNC void operator -= (Mesh3 &mesh, const Vec3 vec) {
        mesh = subMesh3(mesh, vec);
    }

    SP_GENFUNC void operator *= (Mesh2 &mesh, const double val) {
        mesh = mulMesh2(mesh, val);
    }

    SP_GENFUNC void operator *= (Mesh3 &mesh, const double val) {
        mesh = mulMesh3(mesh, val);
    }

    SP_GENFUNC void operator /= (Mesh2 &mesh, const double val) {
        mesh = divMesh2(mesh, val);
    }

    SP_GENFUNC void operator /= (Mesh3 &mesh, const double val) {
        mesh = divMesh3(mesh, val);
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
    SP_GENFUNC Vec2 getMeshPos(const Mesh2 &mesh) {
        return (mesh.pos[0] + mesh.pos[1] + mesh.pos[2]) / 3.0;
    }

    // get center vector
    SP_GENFUNC Vec3 getMeshPos(const Mesh3 &mesh) {
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
                        if (cmpVal(dotVec(p[i], p[j]), u, SP_CAST(0.001)) == false) continue;
                        if (cmpVal(dotVec(p[j], p[k]), u, SP_CAST(0.001)) == false) continue;
                        if (cmpVal(dotVec(p[k], p[i]), u, SP_CAST(0.001)) == false) continue;
                        model[cnt++] = getMesh3(unitVec(p[i]), unitVec(p[j]), unitVec(p[k]));
                    }
                }
            }

            for (int i = 0; i < 20; i++) {
                Mesh3 &m = model[i];
                if (dotVec(getMeshNrm(m), getMeshPos(m)) < 0.0) {
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

}


#endif