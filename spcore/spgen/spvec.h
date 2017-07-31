//--------------------------------------------------------------------------------
// Copyright (c) 2017, sanko-shoko. All rights reserved.
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
	SP_GENCALL Vec2 getVec(const double x, const double y){
		Vec2 dst;
		dst.x = x, dst.y = y;
		return dst;
	}

	// get vector
	SP_GENCALL Vec3 getVec(const double x, const double y, const double z){
		Vec3 dst;
		dst.x = x, dst.y = y, dst.z = z;
		return dst;
	}

	// get vector
	SP_GENCALL Vec3 getVec(const Col3 &col) {
		Vec3 dst;
		dst.x = col.r, dst.y = col.g, dst.z = col.b;
		return dst;
	}

	// get vector
	SP_GENCALL Vec3 getVec(const Vec2 &vec, const double z) {
		return getVec(vec.x, vec.y, z);
	}

	// addition
	SP_GENCALL Vec2 addVec(const Vec2 &vec0, const Vec2 &vec1){
		return getVec(vec0.x + vec1.x, vec0.y + vec1.y);
	}

	// addition
	SP_GENCALL Vec3 addVec(const Vec3 &vec0, const Vec3 &vec1){
		return getVec(vec0.x + vec1.x, vec0.y + vec1.y, vec0.z + vec1.z);
	}

	// subtraction
	SP_GENCALL Vec2 subVec(const Vec2 &vec0, const Vec2 &vec1){
		return getVec(vec0.x - vec1.x, vec0.y - vec1.y);
	}

	// subtraction
	SP_GENCALL Vec3 subVec(const Vec3 &vec0, const Vec3 &vec1){
		return getVec(vec0.x - vec1.x, vec0.y - vec1.y, vec0.z - vec1.z);
	}

	// multiple
	SP_GENCALL Vec2 mulVec(const Vec2 &vec, const double val){
		return getVec(vec.x * val, vec.y * val);
	}

	// multiple
	SP_GENCALL Vec3 mulVec(const Vec3 &vec, const double val){
		return getVec(vec.x * val, vec.y * val, vec.z * val);
	}

	// division
	SP_GENCALL Vec2 divVec(const Vec2 &vec, const double val){
		return (val != 0.0) ? getVec(vec.x / val, vec.y / val) : vec;
	}

	// division
	SP_GENCALL Vec3 divVec(const Vec3 &vec, const double val){
		return (val != 0.0) ? getVec(vec.x / val, vec.y / val, vec.z / val) : vec;
	}


	//--------------------------------------------------------------------------------
	// vector operator
	//--------------------------------------------------------------------------------

	SP_GENCALL Vec2 operator + (const Vec2 &vec0, const Vec2 &vec1){
		return addVec(vec0, vec1);
	}

	SP_GENCALL Vec3 operator + (const Vec3 &vec0, const Vec3 &vec1){
		return addVec(vec0, vec1);
	}

	SP_GENCALL Vec2 operator - (const Vec2 &vec0, const Vec2 &vec1){
		return subVec(vec0, vec1);
	}

	SP_GENCALL Vec3 operator - (const Vec3 &vec0, const Vec3 &vec1){
		return subVec(vec0, vec1);
	}

	SP_GENCALL Vec2 operator * (const Vec2 &vec, const double val){
		return mulVec(vec, val);
	}

	SP_GENCALL Vec3 operator * (const Vec3 &vec, const double val){
		return mulVec(vec, val);
	}

	SP_GENCALL Vec2 operator / (const Vec2 &vec, const double val){
		return divVec(vec, val);
	}

	SP_GENCALL Vec3 operator / (const Vec3 &vec, const double val){
		return divVec(vec, val);
	}

	SP_GENCALL void operator += (Vec2 &vec0, const Vec2 &vec1){
		vec0 = addVec(vec0, vec1);
	}

	SP_GENCALL void operator += (Vec3 &vec0, const Vec3 &vec1){
		vec0 = addVec(vec0, vec1);
	}

	SP_GENCALL void operator -= (Vec2 &vec0, const Vec2 &vec1){
		vec0 = subVec(vec0, vec1);
	}

	SP_GENCALL void operator -= (Vec3 &vec0, const Vec3 &vec1){
		vec0 = subVec(vec0, vec1);
	}

	SP_GENCALL void operator *= (Vec2 &vec, const double val){
		vec = mulVec(vec, val);
	}

	SP_GENCALL void operator *= (Vec3 &vec, const double val){
		vec = mulVec(vec, val);
	}

	SP_GENCALL void operator /= (Vec2 &vec, const double val){
		vec = divVec(vec, val);
	}

	SP_GENCALL void operator /= (Vec3 &vec, const double val){
		vec = divVec(vec, val);
	}


	//--------------------------------------------------------------------------------
	// vector util
	//--------------------------------------------------------------------------------

	// dot production
	SP_GENCALL double dotVec(const Vec2 &vec0, const Vec2 &vec1){
		return vec0.x * vec1.x + vec0.y * vec1.y;
	}

	// dot production
	SP_GENCALL double dotVec(const Vec3 &vec0, const Vec3 &vec1){
		return vec0.x * vec1.x + vec0.y * vec1.y + vec0.z * vec1.z;
	}

	// cross production
	SP_GENCALL Vec3 crsVec(const Vec2 &vec0, const Vec2 &vec1){
		return getVec(0.0, 0.0, vec0.x * vec1.y - vec0.y * vec1.x);
	}

	// cross production
	SP_GENCALL Vec3 crsVec(const Vec3 &vec0, const Vec3 &vec1){
		return getVec(vec0.y * vec1.z - vec0.z * vec1.y, vec0.z * vec1.x - vec0.x * vec1.z, vec0.x * vec1.y - vec0.y * vec1.x);
	}

	// projection vec3 to vec2
	SP_GENCALL Vec2 prjVec(const Vec3 &vec) {
		return getVec(vec.x, vec.y) / vec.z;
	}

	// projection vec2 to vec3
	SP_GENCALL Vec3 prjVec(const Vec2 &vec) {
		return getVec(vec.x, vec.y, 1.0);
	}

	SP_GENCALL double sqVec(const Vec2 &vec){
		return dotVec(vec, vec);
	}

	// square
	SP_GENCALL double sqVec(const Vec3 &vec){
		return dotVec(vec, vec);
	}

	// norm
	SP_GENCALL double normVec(const Vec2 &vec){
		return sqrt(dotVec(vec, vec));
	}

	// norm
	SP_GENCALL double normVec(const Vec3 &vec){
		return sqrt(dotVec(vec, vec));
	}

	// unit vector
	SP_GENCALL Vec2 unitVec(const Vec2 &vec){
		return vec / normVec(vec);
	}

	// unit vector
	SP_GENCALL Vec3 unitVec(const Vec3 &vec){
		return vec / normVec(vec);
	}

	// random uniform
	SP_CPUCALL Vec2 randVecUnif(const double x, const double y){
		return getVec(randValUnif() * x, randValUnif() * y);
	}

	// random uniform
	SP_CPUCALL Vec3 randVecUnif(const double x, const double y, const double z){
		return getVec(randValUnif() * x, randValUnif() * y, randValUnif() * z);
	}

	// random gauss
	SP_CPUCALL Vec2 randVecGauss(const double x, const double y){
		return getVec(randValGauss() * x, randValGauss() * y);
	}

	// random gauss
	SP_CPUCALL Vec3 randVecGauss(const double x, const double y, const double z){
		return getVec(randValGauss() * x, randValGauss() * y, randValGauss() * z);
	}


	//--------------------------------------------------------------------------------
	// matrix * vector
	//--------------------------------------------------------------------------------
	
	SP_GENCALL Vec2 mulMat(const double *mat, const int rows, const int cols, const Vec2 &vec){
		Vec2 dst = getVec(0.0, 0.0);
		if (rows == 2 && cols == 2){
			dst.x = mat[0 * 2 + 0] * vec.x + mat[0 * 2 + 1] * vec.y;
			dst.y = mat[1 * 2 + 0] * vec.x + mat[1 * 2 + 1] * vec.y;
		}
		if (rows == 2 && cols == 3){
			dst.x = mat[0 * 3 + 0] * vec.x + mat[0 * 3 + 1] * vec.y + mat[0 * 3 + 2];
			dst.y = mat[1 * 3 + 0] * vec.x + mat[1 * 3 + 1] * vec.y + mat[1 * 3 + 2];
		}
		if (rows == 3 && cols == 3){
			const double scale = mat[2 * 3 + 0] * vec.x + mat[2 * 3 + 1] * vec.y + mat[2 * 3 + 2];
			dst.x = (mat[0 * 3 + 0] * vec.x + mat[0 * 3 + 1] * vec.y + mat[0 * 3 + 2]) / scale;
			dst.y = (mat[1 * 3 + 0] * vec.x + mat[1 * 3 + 1] * vec.y + mat[1 * 3 + 2]) / scale;
		}

		return dst;
	}

	SP_GENCALL Vec3 mulMat(const double *mat, const int rows, const int cols, const Vec3 &vec){
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
	// vector pd (position and direction)
	//--------------------------------------------------------------------------------
	
	// get vector pd
	SP_GENCALL VecVN2 getVecVN(const Vec2 &vtx, const Vec2 &nrm){
		VecVN2 dst;
		dst.vtx = vtx;
		dst.nrm = nrm;
		return dst;
	}

	// get vector pd
	SP_GENCALL VecVN3 getVecVN(const Vec3 &vtx, const Vec3 &nrm){
		VecVN3 dst;
		dst.vtx = vtx;
		dst.nrm = nrm;
		return dst;
	}


	//--------------------------------------------------------------------------------
	// matrix * vector vn
	//--------------------------------------------------------------------------------

	SP_GENCALL VecVN2 mulMat(const double *mat, const int rows, const int cols, const VecVN2 &vec){
		VecVN2 dst;

		dst.vtx = mulMat(mat, rows, cols, vec.vtx);

		double rot[2 * 2] = { 0 };
		{
			for (int r = 0; r < 2; r++){
				for (int c = 0; c < 2; c++){
					rot[r * 2 + c] = mat[r * cols + c];
				}
			}
		}
		if (rows == 3 && cols == 3){
			const double pos[2] = { dst.vtx.x, dst.vtx.y };
			for (int r = 0; r < 2; r++){
				for (int c = 0; c < 2; c++){
					rot[r * 2 + c] -= mat[2 * cols + c] * pos[r];
				}
			}
		}

		dst.nrm = unitVec(mulMat(rot, 2, 2, vec.nrm));

		return dst;
	}


	SP_GENCALL VecVN3 mulMat(const double *mat, const int rows, const int cols, const VecVN3 &vec){
		VecVN3 dst;

		dst.vtx = mulMat(mat, rows, cols, vec.vtx);

		double rot[3 * 3] = { 0 };
		{
			for (int r = 0; r < 3; r++){
				for (int c = 0; c < 3; c++){
					rot[r * 3 + c] = mat[r * cols + c];
				}
			}
		}
		if (rows == 4 && cols == 4){
			const double pos[3] = { dst.vtx.x, dst.vtx.y, dst.vtx.z };
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

	SP_GENCALL double extractDepth(const VecVN3 &src){
		return src.vtx.z;
	}

	SP_GENCALL double extractDepth(const Vec3 &src){
		return src.z;
	}

	SP_GENCALL double extractDepth(const double &src){
		return src;
	}


	//--------------------------------------------------------------------------------
	// mesh
	//--------------------------------------------------------------------------------

	// get mesh
	SP_GENCALL Mesh getMesh(const Vec3 &vec0, const Vec3 &vec1, const Vec3 &vec2){
		Mesh dst;
		dst.vtx[0] = vec0;
		dst.vtx[1] = vec1;
		dst.vtx[2] = vec2;
		return dst;
	}

	// addition
	SP_GENCALL Mesh addMesh(const Mesh &mesh, const Vec3 vec){
		Mesh dst;
		dst.vtx[0] = mesh.vtx[0] + vec;
		dst.vtx[1] = mesh.vtx[1] + vec;
		dst.vtx[2] = mesh.vtx[2] + vec;
		return dst;
	}

	// subtraction
	SP_GENCALL Mesh subMesh(const Mesh &mesh, const Vec3 vec){
		Mesh dst;
		dst.vtx[0] = mesh.vtx[0] - vec;
		dst.vtx[1] = mesh.vtx[1] - vec;
		dst.vtx[2] = mesh.vtx[2] - vec;
		return dst;
	}

	// multiple
	SP_GENCALL Mesh mulMesh(const Mesh &mesh, const double val){
		Mesh dst;
		dst.vtx[0] = mesh.vtx[0] * val;
		dst.vtx[1] = mesh.vtx[1] * val;
		dst.vtx[2] = mesh.vtx[2] * val;
		return dst;
	}

	// division
	SP_GENCALL Mesh divMesh(const Mesh &mesh, const double val){
		Mesh dst;
		dst.vtx[0] = mesh.vtx[0] / val;
		dst.vtx[1] = mesh.vtx[1] / val;
		dst.vtx[2] = mesh.vtx[2] / val;
		return dst;
	}


	//--------------------------------------------------------------------------------
	// mesh operator
	//--------------------------------------------------------------------------------

	SP_GENCALL Mesh operator + (const Mesh &mesh, const Vec3 vec){
		return addMesh(mesh, vec);
	}

	SP_GENCALL Mesh operator - (const Mesh &mesh, const Vec3 vec){
		return subMesh(mesh, vec);
	}

	SP_GENCALL Mesh operator * (const Mesh &mesh, const double val){
		return mulMesh(mesh, val);
	}

	SP_GENCALL Mesh operator / (const Mesh &mesh, const double val){
		return divMesh(mesh, val);
	}

	SP_GENCALL void operator += (Mesh &mesh, const Vec3 vec){
		mesh = addMesh(mesh, vec);
	}

	SP_GENCALL void operator -= (Mesh &mesh, const Vec3 vec){
		mesh = subMesh(mesh, vec);
	}

	SP_GENCALL void operator *= (Mesh &mesh, const double val){
		mesh = mulMesh(mesh, val);
	}

	SP_GENCALL void operator /= (Mesh &mesh, const double val){
		mesh = divMesh(mesh, val);
	}


	//--------------------------------------------------------------------------------
	// matrix * mesh
	//--------------------------------------------------------------------------------

	SP_GENCALL Mesh mulMat(const double *mat, const int rows, const int cols, const Mesh &mesh){
		Mesh dst;
		dst.vtx[0] = mulMat(mat, rows, cols, mesh.vtx[0]);
		dst.vtx[1] = mulMat(mat, rows, cols, mesh.vtx[1]);
		dst.vtx[2] = mulMat(mat, rows, cols, mesh.vtx[2]);
		return dst;
	}


	//--------------------------------------------------------------------------------
	// mesh util
	//--------------------------------------------------------------------------------

	// get normal vector
	SP_GENCALL Vec3 getMeshNrm(const Mesh &mesh){
		return unitVec(crsVec(mesh.vtx[1] - mesh.vtx[0], mesh.vtx[2] - mesh.vtx[0]));
	}

	// get center vector
	SP_GENCALL Vec3 getMeshPos(const Mesh &mesh){
		return (mesh.vtx[0] + mesh.vtx[1] + mesh.vtx[2]) / 3.0;
	}


	//--------------------------------------------------------------------------------
	// geodesic dorm
	//--------------------------------------------------------------------------------

	// geodesic mesh num
	SP_GENCALL int getGeodesicMeshNum(const int div){
		int ret = 20;
		for (int i = 0; i < div; i++){
			ret *= 4;
		}
		return ret;
	}

	SP_GENCALL Mesh getGeodesicMesh(const int div, const int id){
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
					swap(m.vtx[1], m.vtx[2]);
				}
			}
		}

		// div regular geodesic dorm
		int num = getGeodesicMeshNum(div) / 20;
		int tmp = id;

		Mesh dst = model[tmp / num];
		for (int d = 0; d < div; d++){
			const Vec3 p0 = unitVec(addVec(dst.vtx[0], dst.vtx[1]));
			const Vec3 p1 = unitVec(addVec(dst.vtx[1], dst.vtx[2]));
			const Vec3 p2 = unitVec(addVec(dst.vtx[2], dst.vtx[0]));

			Mesh mesh[4];
			mesh[0] = getMesh(p0, p1, p2);
			mesh[1] = getMesh(dst.vtx[0], p0, p2);
			mesh[2] = getMesh(dst.vtx[1], p1, p0);
			mesh[3] = getMesh(dst.vtx[2], p2, p1);

			tmp %= num;
			num /= 4;
			dst = mesh[tmp / num];
		}
		return dst;
	}
}


#endif