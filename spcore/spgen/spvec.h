//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_VEC_H__
#define __SP_VEC_H__

#include "spcore/spcom.h"
#include "spcore/spwrap.h"
#include "spcore/spgen/spbase.h"

namespace sp{

    //--------------------------------------------------------------------------------
    // vector
    //--------------------------------------------------------------------------------

    // get vector
    SP_GENFUNC Vec2 getVec(const double x, const double y){
        Vec2 dst;
        dst.x = x, dst.y = y;
        return dst;
    }

    // get vector
    SP_GENFUNC Vec3 getVec(const double x, const double y, const double z){
        Vec3 dst;
        dst.x = x, dst.y = y, dst.z = z;
        return dst;
    }

    // get vector
    SP_GENFUNC Vec3 getVec(const Col3 &col) {
        Vec3 dst;
        dst.x = col.r, dst.y = col.g, dst.z = col.b;
        return dst;
    }

    // addition
    SP_GENFUNC Vec2 addVec(const Vec2 &vec0, const Vec2 &vec1){
        return getVec(vec0.x + vec1.x, vec0.y + vec1.y);
    }

    // addition
    SP_GENFUNC Vec3 addVec(const Vec3 &vec0, const Vec3 &vec1){
        return getVec(vec0.x + vec1.x, vec0.y + vec1.y, vec0.z + vec1.z);
    }

    // subtraction
    SP_GENFUNC Vec2 subVec(const Vec2 &vec0, const Vec2 &vec1){
        return getVec(vec0.x - vec1.x, vec0.y - vec1.y);
    }

    // subtraction
    SP_GENFUNC Vec3 subVec(const Vec3 &vec0, const Vec3 &vec1){
        return getVec(vec0.x - vec1.x, vec0.y - vec1.y, vec0.z - vec1.z);
    }

    // multiple
    SP_GENFUNC Vec2 mulVec(const Vec2 &vec, const double val){
        return getVec(vec.x * val, vec.y * val);
    }

    // multiple
    SP_GENFUNC Vec3 mulVec(const Vec3 &vec, const double val){
        return getVec(vec.x * val, vec.y * val, vec.z * val);
    }

    // division
    SP_GENFUNC Vec2 divVec(const Vec2 &vec, const double val){
        return (val != 0.0) ? mulVec(vec, 1.0 / val) : vec;
    }

    // division
    SP_GENFUNC Vec3 divVec(const Vec3 &vec, const double val){
        return (val != 0.0) ? mulVec(vec, 1.0 / val) : vec;
    }

    // extend
    SP_GENFUNC Vec3 extVec(const Vec2 &vec, const double z) {
        return getVec(vec.x, vec.y, z);
    }


    //--------------------------------------------------------------------------------
    // vector operator
    //--------------------------------------------------------------------------------

    SP_GENFUNC Vec2 operator + (const Vec2 &vec0, const Vec2 &vec1){
        return addVec(vec0, vec1);
    }

    SP_GENFUNC Vec3 operator + (const Vec3 &vec0, const Vec3 &vec1){
        return addVec(vec0, vec1);
    }

    SP_GENFUNC Vec2 operator - (const Vec2 &vec0, const Vec2 &vec1){
        return subVec(vec0, vec1);
    }

    SP_GENFUNC Vec3 operator - (const Vec3 &vec0, const Vec3 &vec1){
        return subVec(vec0, vec1);
    }

    SP_GENFUNC Vec2 operator * (const Vec2 &vec, const double val){
        return mulVec(vec, val);
    }

    SP_GENFUNC Vec3 operator * (const Vec3 &vec, const double val){
        return mulVec(vec, val);
    }

    SP_GENFUNC Vec2 operator / (const Vec2 &vec, const double val){
        return divVec(vec, val);
    }

    SP_GENFUNC Vec3 operator / (const Vec3 &vec, const double val){
        return divVec(vec, val);
    }

    SP_GENFUNC void operator += (Vec2 &vec0, const Vec2 &vec1){
        vec0 = addVec(vec0, vec1);
    }

    SP_GENFUNC void operator += (Vec3 &vec0, const Vec3 &vec1){
        vec0 = addVec(vec0, vec1);
    }

    SP_GENFUNC void operator -= (Vec2 &vec0, const Vec2 &vec1){
        vec0 = subVec(vec0, vec1);
    }

    SP_GENFUNC void operator -= (Vec3 &vec0, const Vec3 &vec1){
        vec0 = subVec(vec0, vec1);
    }

    SP_GENFUNC void operator *= (Vec2 &vec, const double val){
        vec = mulVec(vec, val);
    }

    SP_GENFUNC void operator *= (Vec3 &vec, const double val){
        vec = mulVec(vec, val);
    }

    SP_GENFUNC void operator /= (Vec2 &vec, const double val){
        vec = divVec(vec, val);
    }

    SP_GENFUNC void operator /= (Vec3 &vec, const double val){
        vec = divVec(vec, val);
    }


    //--------------------------------------------------------------------------------
    // vector util
    //--------------------------------------------------------------------------------

    // dot production
    SP_GENFUNC double dotVec(const Vec2 &vec0, const Vec2 &vec1){
        return vec0.x * vec1.x + vec0.y * vec1.y;
    }

    // dot production
    SP_GENFUNC double dotVec(const Vec3 &vec0, const Vec3 &vec1){
        return vec0.x * vec1.x + vec0.y * vec1.y + vec0.z * vec1.z;
    }

    // cross production
    SP_GENFUNC Vec3 crsVec(const Vec2 &vec0, const Vec2 &vec1){
        return getVec(0.0, 0.0, vec0.x * vec1.y - vec0.y * vec1.x);
    }

    // cross production
    SP_GENFUNC Vec3 crsVec(const Vec3 &vec0, const Vec3 &vec1){
        return getVec(vec0.y * vec1.z - vec0.z * vec1.y, vec0.z * vec1.x - vec0.x * vec1.z, vec0.x * vec1.y - vec0.y * vec1.x);
    }

    // projection vec3 to vec2
    SP_GENFUNC Vec2 prjVec(const Vec3 &vec) {
        return getVec(vec.x, vec.y) / vec.z;
    }

    // projection vec2 to vec3
    SP_GENFUNC Vec3 prjVec(const Vec2 &vec) {
        return getVec(vec.x, vec.y, 1.0);
    }

    SP_GENFUNC double sqVec(const Vec2 &vec){
        return dotVec(vec, vec);
    }

    // square
    SP_GENFUNC double sqVec(const Vec3 &vec){
        return dotVec(vec, vec);
    }

    // norm
    SP_GENFUNC double normVec(const Vec2 &vec){
        return sqrt(dotVec(vec, vec));
    }

    // norm
    SP_GENFUNC double normVec(const Vec3 &vec){
        return sqrt(dotVec(vec, vec));
    }

    // unit vector
    SP_GENFUNC Vec2 unitVec(const Vec2 &vec){
        return vec / normVec(vec);
    }

    // unit vector
    SP_GENFUNC Vec3 unitVec(const Vec3 &vec){
        return vec / normVec(vec);
    }

    // random uniform
    SP_CPUFUNC Vec2 randVecUnif(const double x, const double y){
        return getVec(randValUnif() * x, randValUnif() * y);
    }

    // random uniform
    SP_CPUFUNC Vec3 randVecUnif(const double x, const double y, const double z){
        return getVec(randValUnif() * x, randValUnif() * y, randValUnif() * z);
    }

    // random gauss
    SP_CPUFUNC Vec2 randVecGauss(const double x, const double y){
        return getVec(randValGauss() * x, randValGauss() * y);
    }

    // random gauss
    SP_CPUFUNC Vec3 randVecGauss(const double x, const double y, const double z){
        return getVec(randValGauss() * x, randValGauss() * y, randValGauss() * z);
    }


    //--------------------------------------------------------------------------------
    // matrix * vector
    //--------------------------------------------------------------------------------
    
    SP_GENFUNC Vec2 mulMat(const double *mat, const int rows, const int cols, const Vec2 &vec){
        Vec2 dst = getVec(0.0, 0.0);
        if (rows == 2 && cols == 2){
            dst.x = mat[0 * 2 + 0] * vec.x + mat[0 * 2 + 1] * vec.y;
            dst.y = mat[1 * 2 + 0] * vec.x + mat[1 * 2 + 1] * vec.y;
        }
        if (rows == 2 && cols == 3){
            dst.x = mat[0 * 3 + 0] * vec.x + mat[0 * 3 + 1] * vec.y + mat[0 * 3 + 2];
            dst.y = mat[1 * 3 + 0] * vec.x + mat[1 * 3 + 1] * vec.y + mat[1 * 3 + 2];
        }
        if (rows == 3 && cols == 3) {
            const double scale = mat[2 * 3 + 0] * vec.x + mat[2 * 3 + 1] * vec.y + mat[2 * 3 + 2];
            dst.x = (mat[0 * 3 + 0] * vec.x + mat[0 * 3 + 1] * vec.y + mat[0 * 3 + 2]) / scale;
            dst.y = (mat[1 * 3 + 0] * vec.x + mat[1 * 3 + 1] * vec.y + mat[1 * 3 + 2]) / scale;
        }
        if (rows == 3 && cols == 4) {
            dst.x = mat[0 * 4 + 0] * vec.x + mat[0 * 4 + 1] * vec.y + mat[0 * 4 + 2] * 0.0 + mat[0 * 4 + 3];
            dst.y = mat[1 * 4 + 0] * vec.x + mat[1 * 4 + 1] * vec.y + mat[1 * 4 + 2] * 0.0 + mat[1 * 4 + 3];
        }
        if (rows == 4 && cols == 4) {
            const double scale = mat[3 * 4 + 0] * vec.x + mat[3 * 4 + 1] * vec.y + mat[3 * 4 + 2] * 0.0 + mat[3 * 4 + 3];
            dst.x = (mat[0 * 4 + 0] * vec.x + mat[0 * 4 + 1] * vec.y + mat[0 * 4 + 2] * 0.0 + mat[0 * 4 + 3]) / scale;
            dst.y = (mat[1 * 4 + 0] * vec.x + mat[1 * 4 + 1] * vec.y + mat[1 * 4 + 2] * 0.0 + mat[1 * 4 + 3]) / scale;
        }

        return dst;
    }

    SP_GENFUNC Vec3 mulMat(const double *mat, const int rows, const int cols, const Vec3 &vec){
        Vec3 dst = getVec(0.0, 0.0, 0.0);
        if (rows == 3 && cols == 3){
            dst.x = mat[0 * 3 + 0] * vec.x + mat[0 * 3 + 1] * vec.y + mat[0 * 3 + 2] * vec.z;
            dst.y = mat[1 * 3 + 0] * vec.x + mat[1 * 3 + 1] * vec.y + mat[1 * 3 + 2] * vec.z;
            dst.z = mat[2 * 3 + 0] * vec.x + mat[2 * 3 + 1] * vec.y + mat[2 * 3 + 2] * vec.z;
        }
        if (rows == 3 && cols == 4){
            dst.x = mat[0 * 4 + 0] * vec.x + mat[0 * 4 + 1] * vec.y + mat[0 * 4 + 2] * vec.z + mat[0 * 4 + 3];
            dst.y = mat[1 * 4 + 0] * vec.x + mat[1 * 4 + 1] * vec.y + mat[1 * 4 + 2] * vec.z + mat[1 * 4 + 3];
            dst.z = mat[2 * 4 + 0] * vec.x + mat[2 * 4 + 1] * vec.y + mat[2 * 4 + 2] * vec.z + mat[2 * 4 + 3];
        }
        if (rows == 4 && cols == 4){
            const double scale = mat[3 * 4 + 0] * vec.x + mat[3 * 4 + 1] * vec.y + mat[3 * 4 + 2] * vec.z + mat[3 * 4 + 3];
            dst.x = (mat[0 * 4 + 0] * vec.x + mat[0 * 4 + 1] * vec.y + mat[0 * 4 + 2] * vec.z + mat[0 * 4 + 3]) / scale;
            dst.y = (mat[1 * 4 + 0] * vec.x + mat[1 * 4 + 1] * vec.y + mat[1 * 4 + 2] * vec.z + mat[1 * 4 + 3]) / scale;
            dst.z = (mat[2 * 4 + 0] * vec.x + mat[2 * 4 + 1] * vec.y + mat[2 * 4 + 2] * vec.z + mat[2 * 4 + 3]) / scale;
        }

        return dst;
    }


    //--------------------------------------------------------------------------------
    // vector pn (position and direction)
    //--------------------------------------------------------------------------------
    
    // get vector pd
    SP_GENFUNC VecPN2 getVecPN(const Vec2 &vtx, const Vec2 &nrm){
        VecPN2 dst;
        dst.pos = vtx;
        dst.nrm = nrm;
        return dst;
    }

    // get vector pd
    SP_GENFUNC VecPN3 getVecPN(const Vec3 &vtx, const Vec3 &nrm){
        VecPN3 dst;
        dst.pos = vtx;
        dst.nrm = nrm;
        return dst;
    }


    //--------------------------------------------------------------------------------
    // matrix * vector pn
    //--------------------------------------------------------------------------------

    SP_GENFUNC VecPN2 mulMat(const double *mat, const int rows, const int cols, const VecPN2 &vec){
        VecPN2 dst;

        dst.pos = mulMat(mat, rows, cols, vec.pos);

        double rot[2 * 2] = { 0 };
        {
            for (int r = 0; r < 2; r++){
                for (int c = 0; c < 2; c++){
                    rot[r * 2 + c] = mat[r * cols + c];
                }
            }
        }
        if (rows == 3 && cols == 3){
            const double pos[2] = { dst.pos.x, dst.pos.y };
            for (int r = 0; r < 2; r++){
                for (int c = 0; c < 2; c++){
                    rot[r * 2 + c] -= mat[2 * cols + c] * pos[r];
                }
            }
        }

        dst.nrm = unitVec(mulMat(rot, 2, 2, vec.nrm));

        return dst;
    }


    SP_GENFUNC VecPN3 mulMat(const double *mat, const int rows, const int cols, const VecPN3 &vec){
        VecPN3 dst;

        dst.pos = mulMat(mat, rows, cols, vec.pos);

        double rot[3 * 3] = { 0 };
        {
            for (int r = 0; r < 3; r++){
                for (int c = 0; c < 3; c++){
                    rot[r * 3 + c] = mat[r * cols + c];
                }
            }
        }
        if (rows == 4 && cols == 4){
            const double pos[3] = { dst.pos.x, dst.pos.y, dst.pos.z };
            for (int r = 0; r < 3; r++){
                for (int c = 0; c < 3; c++){
                    rot[r * 3 + c] -= mat[3 * cols + c] * pos[r];
                }
            }
        }

        dst.nrm = unitVec(mulMat(rot, 3, 3, vec.nrm));

        return dst;
    }

    //--------------------------------------------------------------------------------
    // extract depth element
    //--------------------------------------------------------------------------------

    SP_GENFUNC double extractDepth(const VecPN3 &src){
        return src.pos.z;
    }

    SP_GENFUNC double extractDepth(const Vec3 &src){
        return src.z;
    }

    SP_GENFUNC double extractDepth(const double &src){
        return src;
    }


    //--------------------------------------------------------------------------------
    // mesh
    //--------------------------------------------------------------------------------

    // get mesh
    SP_GENFUNC Mesh getMesh(const Vec3 &vec0, const Vec3 &vec1, const Vec3 &vec2){
        Mesh dst;
        dst.pos[0] = vec0;
        dst.pos[1] = vec1;
        dst.pos[2] = vec2;
        return dst;
    }

    // addition
    SP_GENFUNC Mesh addMesh(const Mesh &mesh, const Vec3 vec){
        Mesh dst;
        dst.pos[0] = mesh.pos[0] + vec;
        dst.pos[1] = mesh.pos[1] + vec;
        dst.pos[2] = mesh.pos[2] + vec;
        return dst;
    }

    // subtraction
    SP_GENFUNC Mesh subMesh(const Mesh &mesh, const Vec3 vec){
        Mesh dst;
        dst.pos[0] = mesh.pos[0] - vec;
        dst.pos[1] = mesh.pos[1] - vec;
        dst.pos[2] = mesh.pos[2] - vec;
        return dst;
    }

    // multiple
    SP_GENFUNC Mesh mulMesh(const Mesh &mesh, const double val){
        Mesh dst;
        dst.pos[0] = mesh.pos[0] * val;
        dst.pos[1] = mesh.pos[1] * val;
        dst.pos[2] = mesh.pos[2] * val;
        return dst;
    }

    // division
    SP_GENFUNC Mesh divMesh(const Mesh &mesh, const double val){
        Mesh dst;
        dst.pos[0] = mesh.pos[0] / val;
        dst.pos[1] = mesh.pos[1] / val;
        dst.pos[2] = mesh.pos[2] / val;
        return dst;
    }


    //--------------------------------------------------------------------------------
    // mesh operator
    //--------------------------------------------------------------------------------

    SP_GENFUNC Mesh operator + (const Mesh &mesh, const Vec3 vec){
        return addMesh(mesh, vec);
    }

    SP_GENFUNC Mesh operator - (const Mesh &mesh, const Vec3 vec){
        return subMesh(mesh, vec);
    }

    SP_GENFUNC Mesh operator * (const Mesh &mesh, const double val){
        return mulMesh(mesh, val);
    }

    SP_GENFUNC Mesh operator / (const Mesh &mesh, const double val){
        return divMesh(mesh, val);
    }

    SP_GENFUNC void operator += (Mesh &mesh, const Vec3 vec){
        mesh = addMesh(mesh, vec);
    }

    SP_GENFUNC void operator -= (Mesh &mesh, const Vec3 vec){
        mesh = subMesh(mesh, vec);
    }

    SP_GENFUNC void operator *= (Mesh &mesh, const double val){
        mesh = mulMesh(mesh, val);
    }

    SP_GENFUNC void operator /= (Mesh &mesh, const double val){
        mesh = divMesh(mesh, val);
    }


    //--------------------------------------------------------------------------------
    // matrix * mesh
    //--------------------------------------------------------------------------------

    SP_GENFUNC Mesh mulMat(const double *mat, const int rows, const int cols, const Mesh &mesh){
        Mesh dst;
        dst.pos[0] = mulMat(mat, rows, cols, mesh.pos[0]);
        dst.pos[1] = mulMat(mat, rows, cols, mesh.pos[1]);
        dst.pos[2] = mulMat(mat, rows, cols, mesh.pos[2]);
        return dst;
    }


    //--------------------------------------------------------------------------------
    // mesh util
    //--------------------------------------------------------------------------------

    // get normal vector
    SP_GENFUNC Vec3 getMeshNrm(const Mesh &mesh){
        return unitVec(crsVec(mesh.pos[1] - mesh.pos[0], mesh.pos[2] - mesh.pos[0]));
    }

    // get center vector
    SP_GENFUNC Vec3 getMeshPos(const Mesh &mesh){
        return (mesh.pos[0] + mesh.pos[1] + mesh.pos[2]) / 3.0;
    }


    //--------------------------------------------------------------------------------
    // line
    //--------------------------------------------------------------------------------

    // get line normal vector
    SP_GENFUNC Vec2 getLineNrm(const Vec2 &vec) {
        return unitVec(getVec(-vec.y, vec.x));
    }


    //--------------------------------------------------------------------------------
    // geodesic dorm
    //--------------------------------------------------------------------------------

    // geodesic mesh num
    SP_GENFUNC int getGeodesicMeshNum(const int level){
        int ret = 20;
        for (int i = 0; i < level; i++){
            ret *= 4;
        }
        return ret;
    }

    SP_GENFUNC Mesh getGeodesicMesh(const int level, const int id){
        Mesh model[20];

        // init regular geodesic dorm (vertex num 12)
        {
            int cnt;

            Vec3 p[12];
            const double u = (1.0 + sqrt(5.0)) / 2.0;
            // vertex (0, ±1, ±u), (±u, 0, ±1), (±1, ±u, 0)

            cnt = 0;
            for (int i = 0; i < 3; i++){
                for (int j = 0; j < 2; j++){
                    for (int k = 0; k < 2; k++){
                        const double s[2] = { -1.0, +1.0 };
                        const double v[3] = { 0.0, s[j], s[k] * u };
                        p[cnt++] = getVec(v[(i + 0) % 3], v[(i + 1) % 3], v[(i + 2) % 3]);
                    }
                }
            }

            cnt = 0;
            for (int i = 0; i < 12; i++){
                for (int j = i + 1; j < 12; j++){
                    for (int k = j + 1; k < 12; k++){
                        if (cmpVal(dotVec(p[i], p[j]), u) == false) continue;
                        if (cmpVal(dotVec(p[j], p[k]), u) == false) continue;
                        if (cmpVal(dotVec(p[k], p[i]), u) == false) continue;
                        model[cnt++] = getMesh(unitVec(p[i]), unitVec(p[j]), unitVec(p[k]));
                    }
                }
            }

            for (int i = 0; i < 20; i++){
                Mesh &m = model[i];
                if (dotVec(getMeshNrm(m), getMeshPos(m)) < 0.0){
                    swap(m.pos[1], m.pos[2]);
                }
            }
        }

        // div regular geodesic dorm
        int num = getGeodesicMeshNum(level) / 20;
        int tmp = id;

        Mesh dst = model[tmp / num];
        for (int d = 0; d < level; d++){
            const Vec3 p0 = unitVec(dst.pos[0] + dst.pos[1]);
            const Vec3 p1 = unitVec(dst.pos[1] + dst.pos[2]);
            const Vec3 p2 = unitVec(dst.pos[2] + dst.pos[0]);

            Mesh mesh[4];
            mesh[0] = getMesh(p0, p1, p2);
            mesh[1] = getMesh(dst.pos[0], p0, p2);
            mesh[2] = getMesh(dst.pos[1], p1, p0);
            mesh[3] = getMesh(dst.pos[2], p2, p1);

            tmp %= num;
            num /= 4;
            dst = mesh[tmp / num];
        }
        return dst;
    }

}


#endif