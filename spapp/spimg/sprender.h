//--------------------------------------------------------------------------------
// Copyright (c) 2017, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_RENDER_H__
#define __SP_RENDER_H__

#include "spcore/spcore.h"
#include "spapp/spimg/spimage.h"

namespace sp{

	//--------------------------------------------------------------------------------
	// render pixel
	//--------------------------------------------------------------------------------

	template<typename TYPE>
	SP_CPUFUNC void renderPoint(Mem<TYPE> &dst, const Vec2 &pix, const TYPE &val, const int radius = 1){
		SP_ASSERT(isValid(2, dst));
		
		const int w = radius - 1;
		for (int y = -w; y <= w; y++){
			for (int x = -w; x <= w; x++){
				if (pythag(x, y) > w + 0.5) continue;

				const int u = round(pix.x + x);
				const int v = round(pix.y + y);
				if (isInRect2(dst.dsize, u, v) == true){
					acs2(dst, u, v) = val;
				}
			}
		}
	}

	template<typename TYPE>
	SP_CPUFUNC void renderPoint(Mem<TYPE> &dst, const CamParam &cam, const Pose &pose, const Vec3 &pnt, const TYPE &val, const int radius = 1){
		SP_ASSERT(isValid(2, dst));

		const Vec3 p = pose * pnt;
		if (p.z > 0){
			renderPoint(dst, cam * prjVec(p), val, radius);
		}
	}

	template<typename TYPE>
	SP_CPUFUNC void renderPoint(Mem<TYPE> &dst, const Mem1<Vec2> &pixs, const TYPE &val, const int radius = 1){
		SP_ASSERT(isValid(2, dst));

		for (int i = 0; i < pixs.size(); i++){
			renderPoint(dst, pixs[i], val, radius);
		}
	}

	template<typename TYPE>
	SP_CPUFUNC void renderPoint(Mem<TYPE> &dst, const CamParam &cam, const Pose &pose, const Mem1<Vec3> &pnts, const TYPE &val, const int radius = 1){
		SP_ASSERT(isValid(2, dst));

		for (int i = 0; i < pnts.size(); i++){
			renderPoint(dst, cam, pose, pnts[i], val, radius);
		}
	}


	//--------------------------------------------------------------------------------
	// render line
	//--------------------------------------------------------------------------------

	template<typename TYPE>
	SP_CPUFUNC void renderLine(Mem<TYPE> &dst, const Vec2 &pix0, const Vec2 &pix1, const TYPE &val, const int thick = 1){
		SP_ASSERT(isValid(2, dst));

		const double div = maxVal(fabs(pix1.x - pix0.x), fabs(pix1.y - pix0.y));
		
		const Vec2 step = (round(div) > 0) ? (pix1 - pix0) / div : getVec(0.0, 0.0);
		for (int i = 0; i <= round(div); i++) {
			renderPoint(dst, pix0 + step * i, val, thick);
		}
	}

	template<typename TYPE>
	SP_CPUFUNC void renderLine(Mem<TYPE> &dst, const Mem1<Vec2> &pixs0, const Mem1<Vec2> &pixs1, const TYPE &val, const int thick = 1){
		SP_ASSERT(isValid(2, dst));
		if (pixs0.size() != pixs1.size()) return;

		for (int i = 0; i < pixs0.size(); i++){
			renderLine(dst, pixs0[i], pixs1[i], val, thick);
		}
	}

	template<typename TYPE>
	SP_CPUFUNC void renderLine(Mem<TYPE> &dst, const CamParam &cam, const Pose &pose, const Vec3 &pnt0, const Vec3 &pnt1, const TYPE &val, const int thick = 1){
		SP_ASSERT(isValid(2, dst));

		const Vec3 p0 = pose * pnt0;
		const Vec3 p1 = pose * pnt1;
		if (p0.z > 0 && p1.z > 0){
			renderLine(dst, mulCam(cam, prjVec(p0)), mulCam(cam, prjVec(p1)), val, thick);
		}
	}

	template<typename TYPE>
	SP_CPUFUNC void renderLine(Mem<TYPE> &dst, const CamParam &cam, const Pose &pose, const Mem1<Vec3> &pnts0, const Mem1<Vec3> &pnts1, const TYPE &val, const int thick = 1){
		SP_ASSERT(isValid(2, dst));
		if (pnts0.size() != pnts1.size()) return;

		for (int i = 0; i < pnts0.size(); i++){
			renderLine(dst, cam, pose, pnts0[i], pnts1[i], val, thick);
		}
	}

	
	//--------------------------------------------------------------------------------
	// render circle
	//--------------------------------------------------------------------------------

	template<typename TYPE>
	SP_CPUFUNC void renderCircle(Mem<TYPE> &dst, const Vec2 &pix, const double radius, const TYPE &val, const int thick = 1){
		SP_ASSERT(isValid(2, dst));

		for (int r = 0; r < 36; r++){
			const double r0 = (r + 0) * SP_PI / 18.0;
			const double r1 = (r + 1) * SP_PI / 18.0;
			const Vec2 pix0 = pix + getVec(cos(r0), sin(r0)) * radius;
			const Vec2 pix1 = pix + getVec(cos(r1), sin(r1)) * radius;
			renderLine(dst, pix0, pix1, val, thick);
		}
	}

	template<typename TYPE>
	SP_CPUFUNC void renderCircle(Mem<TYPE> &dst, const Mem1<Vec2> &pixs, const double radius, const TYPE &val, const int size = 1){
		SP_ASSERT(isValid(2, dst));

		for (int i = 0; i < pixs.size(); i++){
			renderCircle(dst, pixs[i], radius, val, size);
		}
	}


	//--------------------------------------------------------------------------------
	// render mesh
	//--------------------------------------------------------------------------------

	template<typename TYPE>
	SP_CPUFUNC void renderMesh(Mem<TYPE> &dst, const CamParam &cam, const Pose &pose, const Mesh &mesh, const TYPE &val, const int thick = 1){
		SP_ASSERT(isValid(2, dst));

		renderLine(dst, cam, pose, mesh.vtx[0], mesh.vtx[1], val, thick);
		renderLine(dst, cam, pose, mesh.vtx[1], mesh.vtx[2], val, thick);
		renderLine(dst, cam, pose, mesh.vtx[2], mesh.vtx[0], val, thick);
	}

	template<typename TYPE>
	SP_CPUFUNC void renderMesh(Mem<TYPE> &dst, const CamParam &cam, const Pose &pose, const Mem1<Mesh> &mesh, const TYPE &val, const int thick = 1){
		SP_ASSERT(isValid(2, dst));

		for (int i = 0; i < mesh.size(); i++){
			renderMesh(dst, cam, pose, mesh[i], val, thick);
		}
	}


	//--------------------------------------------------------------------------------
	// render util
	//--------------------------------------------------------------------------------

	template<typename TYPE>
	SP_CPUFUNC void renderAxis(Mem<TYPE> &dst, const CamParam &cam, const Pose &pose, const double length, const int thick = 1){
		SP_ASSERT(isValid(2, dst));

		TYPE r, g, b;
		cnvImg(r, getCol(255, 50, 50));
		cnvImg(g, getCol(50, 255, 50));
		cnvImg(b, getCol(50, 50, 255));

		renderLine(dst, cam, pose, getVec(0.0, 0.0, 0.0), getVec(length, 0.0, 0.0), r, thick);
		renderLine(dst, cam, pose, getVec(0.0, 0.0, 0.0), getVec(0.0, length, 0.0), g, thick);
		renderLine(dst, cam, pose, getVec(0.0, 0.0, 0.0), getVec(0.0, 0.0, length), b, thick);
	}

	template<typename TYPE>
	SP_CPUFUNC void renderCube(Mem<TYPE> &dst, const CamParam &cam, const Pose &pose, const double size, const TYPE &val, const int thick = 1){
		SP_ASSERT(isValid(2, dst));

		const double s = size / 2.0;
		const Vec2 xyloop[4] = { getVec(-s, -s), getVec(+s, -s), getVec(+s, +s), getVec(-s, +s) };
		for (int i = 0; i < 4; i++){
			const Vec2 a = xyloop[(i + 0) % 4];
			const Vec2 b = xyloop[(i + 1) % 4];

			renderLine(dst, cam, pose, getVec(a.x, a.y, -s), getVec(b.x, b.y, -s), val, thick);
			renderLine(dst, cam, pose, getVec(a.x, a.y, +s), getVec(b.x, b.y, +s), val, thick);
			renderLine(dst, cam, pose, getVec(a.x, a.y, -s), getVec(a.x, a.y, +s), val, thick);
		}
	}

	template<typename TYPE>
	SP_CPUFUNC void renderGrid(Mem<TYPE> &dst, const CamParam &cam, const Pose &pose, const double length, const int num, const TYPE &val, const int thick = 1){
		SP_ASSERT(isValid(2, dst));

		for (int i = 0; i < num; i++){
			const double p = i * 2 * length / (num - 1);
			renderLine(dst, cam, pose, getVec(-length, -length + p, 0.0), getVec(+length, -length + p, 0.0), val, thick);
			renderLine(dst, cam, pose, getVec(-length + p, -length, 0.0), getVec(-length + p, +length, 0.0), val, thick);
		}
	}

	template<typename TYPE>
	SP_CPUFUNC void renderCam(Mem<TYPE> &dst, const CamParam &cam, const Pose &pose, const CamParam &trg, const double size, const TYPE &val, const int thick = 1) {
		const double f = (trg.fx + trg.fy) / 2.0;
		const double w = (trg.dsize[0] / 2.0) / f * size;
		const double h = (trg.dsize[1] / 2.0) / f * size;

		const Vec2 loop[4] = { getVec(-w, -h), getVec(+w, -h), getVec(+w, +h), getVec(-w, +h) };
		for (int i = 0; i < 4; i++) {
			const Vec2 a = loop[(i + 0) % 4];
			const Vec2 b = loop[(i + 1) % 4];
			renderLine(dst, cam, pose, getVec(0.0, 0.0, 0.0), getVec(a.x, a.y, size), val, thick);
			renderLine(dst, cam, pose, getVec(a.x, a.y, size), getVec(b.x, b.y, size), val, thick);
		}
	}

	template<typename TYPE>
	SP_CPUFUNC void renderRect(Mem<TYPE> &dst, const CamParam &cam, const Pose &pose, const Vec2 &obj0, const Vec2 &obj1, const TYPE &val, const int thick = 1) {
		SP_ASSERT(isValid(2, dst));

		Mem1<Vec3> objs;
		objs.push(getVec(obj0.x, obj0.y, 0.0));
		objs.push(getVec(obj1.x, obj0.y, 0.0));
		objs.push(getVec(obj1.x, obj1.y, 0.0));
		objs.push(getVec(obj0.x, obj1.y, 0.0));

		for (int i = 0; i < 4; i++) {
			renderLine(dst, cam, pose, objs[i], objs[(i + 1) % 4], val, thick);
		}
	}

	template<typename TYPE>
	SP_CPUFUNC void renderRect(Mem<TYPE> &dst, const Mat &hom, const Vec2 &obj0, const Vec2 &obj1, const TYPE &val, const int thick = 1) {
		SP_ASSERT(isValid(2, dst));

		Mem1<Vec2> objs;
		objs.push(getVec(obj0.x, obj0.y));
		objs.push(getVec(obj1.x, obj0.y));
		objs.push(getVec(obj1.x, obj1.y));
		objs.push(getVec(obj0.x, obj1.y));

		for (int i = 0; i < 4; i++) {
			renderLine(dst, hom * objs[i], hom * objs[(i + 1) % 4], val, thick);
		}
	}

	template<typename TYPE>
	SP_CPUFUNC void renderEpipolar(Mem<TYPE> &dst, const Mat F, const Vec2 &pix, const TYPE &val, const int thick = 1){
		SP_ASSERT(isValid(2, dst));

		const int w = dst.dsize[0];
		const int h = dst.dsize[1];

		const Vec3 line = F * getVec(pix, 1.0);
		if (fabs(line.y) > fabs(line.x)){
			const Vec2 pix0 = getVec(0, -(0 * line.x + line.z) / line.y);
			const Vec2 pix1 = getVec(w, -(w * line.x + line.z) / line.y);
			renderLine(dst, pix0, pix1, val, thick);
		}
		if (fabs(line.x) > fabs(line.y)){
			const Vec2 pix0 = getVec(-(0 * line.y + line.z) / line.x, 0);
			const Vec2 pix1 = getVec(-(h * line.y + line.z) / line.x, h);
			renderLine(dst, pix0, pix1, val, thick);
		}
	}

	template<typename TYPE>
	SP_CPUFUNC void renderEpipolar(Mem<TYPE> &dst, const Mat F, const Mem1<Vec2> &pixs, const TYPE &val, const int thick = 1){
		SP_ASSERT(isValid(2, dst));

		for (int i = 0; i < pixs.size(); i++){
			renderEpipolar(dst, F, pixs[i], val, thick);
		}
	}


	//--------------------------------------------------------------------------------
	// render calibration marker
	//--------------------------------------------------------------------------------

	SP_CPUFUNC void renderMarker(Mem2<Byte> &dst, const CamParam &cam, const Pose &pose, const Mem2<Vec2> &mrkMap) {
		dst.resize(cam.dsize);
		setElm(dst, 255);

		const double radius = normVec(mrkMap[0] - mrkMap[1]) * 0.1;

		const Vec3 base = pose * getVec(0.0, 0.0, 0.0);
		const Vec3 A = pose * getVec(1.0, 0.0, 0.0) - base;
		const Vec3 B = pose * getVec(0.0, 1.0, 0.0) - base;

		double mat[3 * 3] = { -A.x, -B.x, 0.0, -A.y, -B.y, 0.0, -A.z, -B.z, 0.0 };
		double val[3] = { base.x, base.y, base.z };

		for (int v = 0; v < dst.dsize[1]; v++) {
			for (int u = 0; u < dst.dsize[0]; u++) {
				const Vec2 prj = npxUndist(cam, invCam(cam, getVec(u, v)));
				const Vec3 vec = getVec(prj.x, prj.y, 1.0);

				mat[0 * 3 + 2] = vec.x;
				mat[1 * 3 + 2] = vec.y;
				mat[2 * 3 + 2] = vec.z;

				double inv[3 * 3];
				if (invMat33(inv, mat) == false) continue;

				double result[3];
				mulMat(result, 3, 1, inv, 3, 3, val, 3, 1);

				const Vec2 pos = getVec(result[0], result[1]);

				for (int i = 0; i < mrkMap.size(); i++) {
					if (normVec(pos - mrkMap[i]) < radius) {
						dst(u, v) = 0;
						break;
					}
				}
			}
		}
	}


	//--------------------------------------------------------------------------------
	// render vector vn
	//--------------------------------------------------------------------------------

	SP_CPUFUNC void renderVecVN(Mem<VecVN3> &map, const CamParam &cam, const Pose &pose, const Mesh &mesh) {

		if (cmpSize(2, map.dsize, cam.dsize) == false) {
			map.resize(2, cam.dsize);
			map.zero();
		}

		Rect rect;

		const Mesh pm = pose * mesh;
		{
			int xs = map.dsize[0];
			int xe = 0;

			int ys = map.dsize[1];
			int ye = 0;

			bool valid = false;
			for (int i = 0; i < 3; i++) {
				if (pm.vtx[i].z < SP_SMALL) continue;

				valid |= true;

				const Vec2 pix = mulCamD(cam, prjVec(pm.vtx[i]));

				xs = minVal(xs, floor(pix.x + 1));
				xe = maxVal(xe, floor(pix.x + 1));

				ys = minVal(ys, floor(pix.y + 1));
				ye = maxVal(ye, floor(pix.y + 1));
			}
			if (valid == false) return;

			rect = andRect(getRect2(map.dsize), getRect2(xs, ys, xe - xs, ye - ys));
		}

		const Vec3 nrm = getMeshNrm(pm);

		const Vec3 base = pm.vtx[0];
		const Vec3 A = pm.vtx[1] - base;
		const Vec3 B = pm.vtx[2] - base;

		double mat[3 * 3] = { -A.x, -B.x, 0.0, -A.y, -B.y, 0.0, -A.z, -B.z, 0.0 };
		double val[3] = { base.x, base.y, base.z };

		for (int v = rect.dbase[1]; v < rect.dbase[1] + rect.dsize[1]; v++) {
			for (int u = rect.dbase[0]; u < rect.dbase[0] + rect.dsize[0]; u++) {
				const Vec2 prj = npxUndist(cam, invCam(cam, getVec(u, v)));
				const Vec3 vec = getVec(prj.x, prj.y, 1.0);

				mat[0 * 3 + 2] = vec.x;
				mat[1 * 3 + 2] = vec.y;
				mat[2 * 3 + 2] = vec.z;

				double inv[3 * 3];
				if (invMat33(inv, mat) == false) continue;

				double result[3];
				mulMat(result, 3, 1, inv, 3, 3, val, 3, 1);

				if (result[0] < 0.0 || result[1] < 0.0 || result[0] + result[1] > 1.0) continue;

				const double depth = result[2];
				if (depth < SP_SMALL) continue;

				const double ref = extractDepth(acs2(map, u, v));
				if (ref == 0.0 || depth < ref) {
					acs2(map, u, v) = getVecVN(vec * depth, nrm);
				}
			}
		}

	}

	SP_CPUFUNC void renderVecVN(Mem<VecVN3> &map, const CamParam &cam, const Pose &pose, const Mem<Mesh> &meshes) {

		for (int i = 0; i < meshes.size(); i++) {
			renderVecVN(map, cam, pose, meshes[i]);
		}
	}

	SP_CPUFUNC void renderDepth(Mem<double> &map, const CamParam &cam, const Pose &pose, const Mem<Mesh> &meshes) {

		Mem<VecVN3> vnmap;
		for (int i = 0; i < meshes.size(); i++) {
			renderVecVN(vnmap, cam, pose, meshes[i]);
		}

		map.resize(2, cam.dsize);
		for (int i = 0; i < map.size(); i++) {
			map[i] = extractDepth(vnmap[i]);
		}
	}


	SP_CPUFUNC void renderImage(Mem2<Byte> &dst, const CamParam &cam, const Pose &pose, const Mem1<Mesh> &meshes) {

		dst.resize(cam.dsize);
		dst.zero();

		Mem<VecVN3> map;
		renderVecVN(map, cam, pose, meshes);

		for (int i = 0; i < dst.size(); i++) {
			const Vec3 nrm = map[i].nrm;
			if (nrm.z < 0.0) {
				cnvVal(dst[i], -nrm.z * SP_BYTEMAX);
			}
		}
	}


	SP_CPUFUNC void renderPattern(Mem2<Byte> &dst, const CamParam &cam, const Pose &pose, const Mem1<Mesh> &meshes,
		const CamParam &prj, const Pose &cam2prj, const Mem2<Byte> &ptn) {

		dst.resize(cam.dsize);
		dst.zero();

		Mem2<VecVN3> cmap;
		renderVecVN(cmap, cam, pose, meshes);

		Mem2<VecVN3> pmap;
		renderVecVN(pmap, prj, cam2prj * pose, meshes);

		Mem2<double> pdepth(pmap.dsize);
		Mem2<double> pmask(pmap.dsize);
		for (int i = 0; i < pmap.size(); i++) {
			pdepth[i] = pmap[i].vtx.z;
			pmask[i] = (pmap[i].vtx.z > 0.0) ? 1.0 : 0.0;
		}

		for (int v = 0; v < cam.dsize[1]; v++) {
			for (int u = 0; u < cam.dsize[0]; u++) {
				if (cmap(u, v).vtx.z == 0.0) continue;

				const Vec2 cpix = getVec(u, v);
				const Vec2 cnpx = invCam(cam, cpix);
				const Vec3 cpos = getVec(cnpx.x, cnpx.y, 1.0) * cmap(u, v).vtx.z;

				const Vec3 ppos = cam2prj * cpos;
				const Vec2 pnpx = prjVec(ppos);
				const Vec2 ppix = mulCamD(prj, pnpx);

				const double div = acs2(pmask, ppix.x, ppix.y);
				if (div < SP_SMALL) continue;
				
				const double depth = acs2(pdepth, ppix.x, ppix.y) / div;
				if (depth < ppos.z - 1.0) continue;

				if (isInRect2(ptn.dsize, round(ppix.x), round(ppix.y)) == false) continue;
				const double val = acs2(ptn, ppix.x, ppix.y);

				const Vec3 nrm = cmap(u, v).nrm;
				if (nrm.z < 0.0) {
					cnvVal(dst(u, v), -nrm.z * val);
				}

			}
		}

	}

}

#endif