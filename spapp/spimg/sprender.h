﻿//--------------------------------------------------------------------------------
// Copyright (c) 2017-2020, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_RENDER_H__
#define __SP_RENDER_H__

#include "spcore/spcore.h"
#include "spapp/spimg/spimg.h"

namespace sp{

    //--------------------------------------------------------------------------------
    // render pixel
    //--------------------------------------------------------------------------------

    template<typename TYPE>
    SP_CPUFUNC void renderPoint(Mem<TYPE> &dst, const Vec2 &pix, const TYPE &val, const double radius = 1.0){
        
        const int w = ceil(radius - 0.1);
        for (int y = -w; y <= w; y++){
            for (int x = -w; x <= w; x++){
                if (pythag(x, y) >= radius) continue;

                const int u = round(pix.x + x);
                const int v = round(pix.y + y);
                if (inRect(dst.dsize, u, v) == true){
                    acs2(dst, u, v) = val;
                }
            }
        }
    }

    template<typename TYPE>
    SP_CPUFUNC void renderPoint(Mem<TYPE> &dst, const CamParam &cam, const Pose &pose, const Vec3 &pnt, const TYPE &val, const double radius = 1.0){

        const Vec3 p = pose * pnt;
        if (p.z > 0){
            renderPoint(dst, mulCam(cam, prjVec(p)), val, radius);
        }
    }

    template<typename TYPE>
    SP_CPUFUNC void renderPoint(Mem<TYPE> &dst, const Mem1<Vec2> &pixs, const TYPE &val, const double radius = 1.0){

        for (int i = 0; i < pixs.size(); i++){
            renderPoint(dst, pixs[i], val, radius);
        }
    }

    template<typename TYPE>
    SP_CPUFUNC void renderPoint(Mem<TYPE> &dst, const CamParam &cam, const Pose &pose, const Mem1<Vec3> &pnts, const TYPE &val, const double radius = 1.0){

        for (int i = 0; i < pnts.size(); i++){
            renderPoint(dst, cam, pose, pnts[i], val, radius);
        }
    }


    //--------------------------------------------------------------------------------
    // render line
    //--------------------------------------------------------------------------------

    template<typename TYPE>
    SP_CPUFUNC void renderLine(Mem<TYPE> &dst, const Vec2 &pix0, const Vec2 &pix1, const TYPE &val, const double thick = 1.0){

        const double div = max(fabs(pix1.x - pix0.x), fabs(pix1.y - pix0.y));
        
        const Vec2 step = (round(div) > 0) ? (pix1 - pix0) / div : getVec2(0.0, 0.0);
        for (int i = 0; i <= round(div); i++) {
            renderPoint(dst, pix0 + step * i, val, thick);
        }
    }

    template<typename TYPE>
    SP_CPUFUNC void renderLine(Mem<TYPE> &dst, const Mem1<Vec2> &pixs0, const Mem1<Vec2> &pixs1, const TYPE &val, const double thick = 1.0){
        if (pixs0.size() != pixs1.size()) return;

        for (int i = 0; i < pixs0.size(); i++){
            renderLine(dst, pixs0[i], pixs1[i], val, thick);
        }
    }

    template<typename TYPE>
    SP_CPUFUNC void renderLine(Mem<TYPE> &dst, const CamParam &cam, const Pose &pose, const Vec3 &pnt0, const Vec3 &pnt1, const TYPE &val, const double thick = 1.0){

        const Vec3 p0 = pose * pnt0;
        const Vec3 p1 = pose * pnt1;
        if (p0.z > 0 && p1.z > 0){
            renderLine(dst, mulCam(cam, prjVec(p0)), mulCam(cam, prjVec(p1)), val, thick);
        }
    }

    template<typename TYPE>
    SP_CPUFUNC void renderLine(Mem<TYPE> &dst, const CamParam &cam, const Pose &pose, const Mem1<Vec3> &pnts0, const Mem1<Vec3> &pnts1, const TYPE &val, const double thick = 1.0){
        if (pnts0.size() != pnts1.size()) return;

        for (int i = 0; i < pnts0.size(); i++){
            renderLine(dst, cam, pose, pnts0[i], pnts1[i], val, thick);
        }
    }

    
    //--------------------------------------------------------------------------------
    // render circle
    //--------------------------------------------------------------------------------

    template<typename TYPE>
    SP_CPUFUNC void renderCircle(Mem<TYPE> &dst, const Vec2 &pix, const double radius, const TYPE &val, const double thick = 1.0){

        for (int r = 0; r < 36; r++){
            const double r0 = (r + 0) * SP_PI / 18.0;
            const double r1 = (r + 1) * SP_PI / 18.0;
            const Vec2 pix0 = pix + getVec2(cos(r0), sin(r0)) * radius;
            const Vec2 pix1 = pix + getVec2(cos(r1), sin(r1)) * radius;
            renderLine(dst, pix0, pix1, val, thick);
        }
    }

    template<typename TYPE>
    SP_CPUFUNC void renderCircle(Mem<TYPE> &dst, const Mem1<Vec2> &pixs, const double radius, const TYPE &val, const double thick = 1.0){

        for (int i = 0; i < pixs.size(); i++){
            renderCircle(dst, pixs[i], radius, val, thick);
        }
    }


    //--------------------------------------------------------------------------------
    // render mesh
    //--------------------------------------------------------------------------------

    template<typename TYPE>
    SP_CPUFUNC void renderMesh(Mem<TYPE> &dst, const CamParam &cam, const Pose &pose, const Mesh3 &mesh, const TYPE &val, const double thick = 1.0){

        renderLine(dst, cam, pose, mesh.pos[0], mesh.pos[1], val, thick);
        renderLine(dst, cam, pose, mesh.pos[1], mesh.pos[2], val, thick);
        renderLine(dst, cam, pose, mesh.pos[2], mesh.pos[0], val, thick);
    }

    template<typename TYPE>
    SP_CPUFUNC void renderMesh(Mem<TYPE> &dst, const CamParam &cam, const Pose &pose, const Mem1<Mesh3> &mesh, const TYPE &val, const double thick = 1.0){

        for (int i = 0; i < mesh.size(); i++){
            renderMesh(dst, cam, pose, mesh[i], val, thick);
        }
    }


    //--------------------------------------------------------------------------------
    // render util
    //--------------------------------------------------------------------------------

    template<typename TYPE>
    SP_CPUFUNC void renderAxis(Mem<TYPE> &dst, const CamParam &cam, const Pose &pose, const double length, const double thick = 1.0){

        const TYPE r = cast<TYPE>(getCol3(255, 50, 50));
        const TYPE g = cast<TYPE>(getCol3(50, 255, 50));
        const TYPE b = cast<TYPE>(getCol3(50, 50, 255));

        renderLine(dst, cam, pose, getVec3(0.0, 0.0, 0.0), getVec3(length, 0.0, 0.0), r, thick);
        renderLine(dst, cam, pose, getVec3(0.0, 0.0, 0.0), getVec3(0.0, length, 0.0), g, thick);
        renderLine(dst, cam, pose, getVec3(0.0, 0.0, 0.0), getVec3(0.0, 0.0, length), b, thick);
    }

    template<typename TYPE>
    SP_CPUFUNC void renderCube(Mem<TYPE> &dst, const CamParam &cam, const Pose &pose, const double size, const TYPE &val, const double thick = 1.0){

        const double s = size / 2.0;
        const Vec2 xyloop[4] = { getVec2(-s, -s), getVec2(+s, -s), getVec2(+s, +s), getVec2(-s, +s) };
        for (int i = 0; i < 4; i++){
            const Vec2 a = xyloop[(i + 0) % 4];
            const Vec2 b = xyloop[(i + 1) % 4];

            renderLine(dst, cam, pose, getVec3(a.x, a.y, -s), getVec3(b.x, b.y, -s), val, thick);
            renderLine(dst, cam, pose, getVec3(a.x, a.y, +s), getVec3(b.x, b.y, +s), val, thick);
            renderLine(dst, cam, pose, getVec3(a.x, a.y, -s), getVec3(a.x, a.y, +s), val, thick);
        }
    }

    template<typename TYPE>
    SP_CPUFUNC void renderGrid2d(Mem<TYPE> &dst, const CamParam &cam, const Pose &pose, const double length, const int num, const TYPE &val, const double thick = 1.0){

        for (int i = 0; i < num; i++){
            const double half = length / 2.0;
            const double p = i * length / (num - 1);
            renderLine(dst, cam, pose, getVec3(-half, -half + p, 0.0), getVec3(+half, -half + p, 0.0), val, thick);
            renderLine(dst, cam, pose, getVec3(-half + p, -half, 0.0), getVec3(-half + p, +half, 0.0), val, thick);
        }
    }

    template<typename TYPE>
    SP_CPUFUNC void renderGrid3d(Mem<TYPE> &dst, const CamParam &cam, const Pose &pose, const double length, const int num, const TYPE &val, const double thick = 1.0) {

        for (int i = 0; i < num; i++) {
            for (int j = 0; j < num; j++) {
                const double half = length / 2.0;
                const double p = i * length / (num - 1);
                const double q = j * length / (num - 1);
                renderLine(dst, cam, pose, getVec3(-half, -half + p, -half + q), getVec3(+half, -half + p, -half + q), val, thick);
                renderLine(dst, cam, pose, getVec3(-half + p, -half, -half + q), getVec3(-half + p, +half, -half + q), val, thick);
                renderLine(dst, cam, pose, getVec3(-half + p, -half + q, -half), getVec3(-half + p, -half + q, +half), val, thick);
            }
        }
    }

    template<typename TYPE>
    SP_CPUFUNC void renderCam(Mem<TYPE> &dst, const CamParam &cam, const Pose &pose, const CamParam &trg, const double size, const TYPE &val, const double thick = 1.0) {
        const double f = (trg.fx + trg.fy) / 2.0;
        const double w = (trg.dsize[0] / 2.0) / f * size;
        const double h = (trg.dsize[1] / 2.0) / f * size;

        const Vec2 loop[4] = { getVec2(-w, -h), getVec2(+w, -h), getVec2(+w, +h), getVec2(-w, +h) };
        for (int i = 0; i < 4; i++) {
            const Vec2 a = loop[(i + 0) % 4];
            const Vec2 b = loop[(i + 1) % 4];
            renderLine(dst, cam, pose, getVec3(0.0, 0.0, 0.0), getVec3(a.x, a.y, size), val, thick);
            renderLine(dst, cam, pose, getVec3(a.x, a.y, size), getVec3(b.x, b.y, size), val, thick);
        }
    }

    template<typename TYPE>
    SP_CPUFUNC void renderRect(Mem<TYPE> &dst, const CamParam &cam, const Pose &pose, const Vec2 &obj0, const Vec2 &obj1, const TYPE &val, const double thick = 1.0) {

        Mem1<Vec3> objs;
        objs.push(getVec3(obj0.x, obj0.y, 0.0));
        objs.push(getVec3(obj1.x, obj0.y, 0.0));
        objs.push(getVec3(obj1.x, obj1.y, 0.0));
        objs.push(getVec3(obj0.x, obj1.y, 0.0));

        for (int i = 0; i < 4; i++) {
            renderLine(dst, cam, pose, objs[i], objs[(i + 1) % 4], val, thick);
        }
    }

    template<typename TYPE>
    SP_CPUFUNC void renderRect(Mem<TYPE> &dst, const Mat &hom, const Vec2 &obj0, const Vec2 &obj1, const TYPE &val, const double thick = 1.0) {

        Mem1<Vec2> objs;
        objs.push(getVec2(obj0.x, obj0.y));
        objs.push(getVec2(obj1.x, obj0.y));
        objs.push(getVec2(obj1.x, obj1.y));
        objs.push(getVec2(obj0.x, obj1.y));

        for (int i = 0; i < 4; i++) {
            renderLine(dst, hom * objs[i], hom * objs[(i + 1) % 4], val, thick);
        }
    }

    template<typename TYPE>
    SP_CPUFUNC void renderEpipolar(Mem<TYPE> &dst, const Mat F, const Vec2 &pix, const TYPE &val, const double thick = 1.0){

        const int w = dst.dsize[0];
        const int h = dst.dsize[1];

        const Vec3 line = F * getVec3(pix.x, pix.y, 1.0);
        if (fabs(line.y) > fabs(line.x)){
            const Vec2 pix0 = getVec2(0, -(0 * line.x + line.z) / line.y);
            const Vec2 pix1 = getVec2(w, -(w * line.x + line.z) / line.y);
            renderLine(dst, pix0, pix1, val, thick);
        }
        if (fabs(line.x) > fabs(line.y)){
            const Vec2 pix0 = getVec2(-(0 * line.y + line.z) / line.x, 0);
            const Vec2 pix1 = getVec2(-(h * line.y + line.z) / line.x, h);
            renderLine(dst, pix0, pix1, val, thick);
        }
    }

    template<typename TYPE>
    SP_CPUFUNC void renderEpipolar(Mem<TYPE> &dst, const Mat F, const Mem1<Vec2> &pixs, const TYPE &val, const double thick = 1.0){

        for (int i = 0; i < pixs.size(); i++){
            renderEpipolar(dst, F, pixs[i], val, thick);
        }
    }


    //--------------------------------------------------------------------------------
    // render calibration marker
    //--------------------------------------------------------------------------------

    SP_CPUFUNC void renderMarker(Mem2<Byte> &dst, const CamParam &cam, const Pose &pose, const Mem2<Vec2> &mrkMap) {
        dst.resize(cam.dsize);
        dst.zero();

        const double radius = normVec(mrkMap[0] - mrkMap[1]) * 0.1;

        const Vec3 base = pose * getVec3(0.0, 0.0, 0.0);
        const Vec3 A = pose * getVec3(1.0, 0.0, 0.0) - base;
        const Vec3 B = pose * getVec3(0.0, 1.0, 0.0) - base;

        SP_REAL mat[3 * 3] = { -A.x, -B.x, 0.0, -A.y, -B.y, 0.0, -A.z, -B.z, 0.0 };
        SP_REAL val[3] = { base.x, base.y, base.z };
        
        //const Vec2 cent = getVec3(cam.cx, cam.cy);

        for (int v = 0; v < dst.dsize[1]; v++) {
            for (int u = 0; u < dst.dsize[0]; u++) {
                const Vec2 npx = npxUndist(cam, invCam(cam, getVec2(u, v)));
                const Vec3 vec = getVec3(npx.x, npx.y, 1.0);

                dst(u, v) = 255;

                mat[0 * 3 + 2] = vec.x;
                mat[1 * 3 + 2] = vec.y;
                mat[2 * 3 + 2] = vec.z;

                SP_REAL inv[3 * 3];
                if (invMat33(inv, mat) == false) continue;

                SP_REAL result[3];
                mulMat(result, 3, 1, inv, 3, 3, val, 3, 1);

                const Vec2 pos = getVec2(result[0], result[1]);

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
    // render geom
    //--------------------------------------------------------------------------------
    
    SP_CPUFUNC void renderVecPD(Mem<VecPD3> &dst, const CamParam &cam, const Pose &pose, const Mesh3 &mesh) {

        if (cmp(dst.dsize, cam.dsize, 2) == false) {
            dst.resize(2, cam.dsize);
            dst.zero();
        }

        Rect2 rect;

        const Mesh3 pm = pose * mesh;
        {
            int xs = dst.dsize[0];
            int xe = 0;

            int ys = dst.dsize[1];
            int ye = 0;

            bool valid = false;
            for (int i = 0; i < 3; i++) {
                if (pm.pos[i].z < SP_SMALL) continue;

                valid |= true;

                const Vec2 pix = mulCamD(cam, prjVec(pm.pos[i]));

                xs = min(xs, floor(pix.x + 1));
                xe = max(xe, floor(pix.x + 1));

                ys = min(ys, floor(pix.y + 1));
                ye = max(ye, floor(pix.y + 1));
            }
            if (valid == false) return;

            rect = andRect(getRect2(dst.dsize), getRect2(xs, ys, xe - xs, ye - ys));
        }

        const Vec3 nrm = getMeshNrm(pm);

        const Vec3 base = pm.pos[0];
        const Vec3 A = pm.pos[1] - base;
        const Vec3 B = pm.pos[2] - base;

        SP_REAL mat[3 * 3] = { -A.x, -B.x, 0.0, -A.y, -B.y, 0.0, -A.z, -B.z, 0.0 };
        SP_REAL val[3] = { base.x, base.y, base.z };

        for (int v = rect.dbase[1]; v < rect.dbase[1] + rect.dsize[1]; v++) {
            for (int u = rect.dbase[0]; u < rect.dbase[0] + rect.dsize[0]; u++) {
                const Vec2 prj = npxUndist(cam, invCam(cam, getVec2(u, v)));
                const Vec3 vec = getVec3(prj.x, prj.y, 1.0);

                mat[0 * 3 + 2] = vec.x;
                mat[1 * 3 + 2] = vec.y;
                mat[2 * 3 + 2] = vec.z;

                SP_REAL inv[3 * 3];
                if (invMat33(inv, mat) == false) continue;

                SP_REAL result[3];
                mulMat(result, 3, 1, inv, 3, 3, val, 3, 1);

                if (result[0] < 0.0 || result[1] < 0.0 || result[0] + result[1] > 1.0) continue;

                const SP_REAL depth = result[2];
                if (depth < SP_SMALL) continue;

                const SP_REAL ref = extractZ(acs2(dst, u, v));
                if (ref == 0.0 || depth < ref) {
                    acs2(dst, u, v) = getVecPD3(vec * depth, nrm);
                }
            }
        }

    }

    SP_CPUFUNC void renderVecPD(Mem<VecPD3> &dst, const CamParam &cam, const Pose &pose, const Mem<Mesh3> &meshes) {

        for (int i = 0; i < meshes.size(); i++) {
            renderVecPD(dst, cam, pose, meshes[i]);
        }
    }

    template<typename DEPTH>
    SP_CPUFUNC void renderDepth(Mem<DEPTH> &dst, const CamParam &cam, const Pose &pose, const Mem<Mesh3> &meshes) {

        Mem<VecPD3> pnmap;
        for (int i = 0; i < meshes.size(); i++) {
            renderVecPD(pnmap, cam, pose, meshes[i]);
        }

        dst.resize(2, cam.dsize);
        for (int i = 0; i < dst.size(); i++) {
            dst[i] = extractZ(pnmap[i]);
        }
    }


    SP_CPUFUNC void renderNormal(Mem2<Byte> &dst, const CamParam &cam, const Pose &pose, const Mem1<Mesh3> &meshes) {

        dst.resize(cam.dsize);
        dst.zero();

        Mem<VecPD3> map;
        renderVecPD(map, cam, pose, meshes);

        for (int i = 0; i < dst.size(); i++) {
            cnvNormalToCol(dst[i], map[i].drc);
        }
    }


    SP_CPUFUNC void renderPattern(Mem2<Byte> &dst, const CamParam &cam, const Pose &pose, const Mem1<Mesh3> &meshes,
        const Pose &cam2prj, const CamParam &prj, const Mem2<Byte> &ptn) {

        dst.resize(cam.dsize);
        dst.zero();

        Mem2<VecPD3> cmap;
        renderVecPD(cmap, cam, pose, meshes);

        Mem2<VecPD3> pmap;
        renderVecPD(pmap, prj, cam2prj * pose, meshes);

        typedef MemA<SP_REAL, 2> Double2;
        Mem2<Double2> ptmp(pmap.dsize);
        for (int i = 0; i < pmap.size(); i++) {
            ptmp[i][0] = pmap[i].pos.z > 0.0 ? 1.0 : 0.0;
            ptmp[i][1] = pmap[i].pos.z;
        }

        for (int v = 0; v < cam.dsize[1]; v++) {
            for (int u = 0; u < cam.dsize[0]; u++) {
                if (cmap(u, v).pos.z == 0.0) continue;

                const Vec2 cpix = getVec2(u, v);
                const Vec2 cnpx = invCam(cam, cpix);
                const Vec3 cpos = getVec3(cnpx.x, cnpx.y, 1.0) * cmap(u, v).pos.z;

                const Vec3 ppos = cam2prj * cpos;
                const Vec2 pnpx = prjVec(ppos);
                const Vec2 ppix = mulCamD(prj, pnpx);

                const SP_REAL div = acs2<Double2, SP_REAL>(ptmp, ppix.x, ppix.y, 0);
                if (div < SP_SMALL) continue;
                
                const SP_REAL depth = acs2<Double2, SP_REAL>(ptmp, ppix.x, ppix.y, 1) / div;
                if (depth < ppos.z - 1.0) continue;

                if (inRect(prj.dsize, round(ppix.x), round(ppix.y)) == false) continue;
                const SP_REAL val = acs2(ptn, ppix.x, ppix.y);

                const Vec3 nrm = cam2prj.rot * cmap(u, v).drc;
                if (nrm.z < 0.0) {
                    dst(u, v) = cast<Byte>(-nrm.z * val * 0.9);
                }

            }
        }

    }

    SP_CPUFUNC void renderCrsp(Mem2<Vec2> &dst, const CamParam &cam, const Pose &pose, const Mem1<Mesh3> &meshes,
        const Pose &cam2prj, const CamParam &prj) {

        dst.resize(cam.dsize);
        setElm(dst, getVec2(-1.0, -1.0));

        Mem2<VecPD3> cmap;
        renderVecPD(cmap, cam, pose, meshes);

        Mem2<VecPD3> pmap;
        renderVecPD(pmap, prj, cam2prj * pose, meshes);

        typedef MemA<SP_REAL, 2> Double2;
        Mem2<Double2> ptmp(pmap.dsize);
        for (int i = 0; i < pmap.size(); i++) {
            ptmp[i][0] = pmap[i].pos.z > 0.0 ? 1.0 : 0.0;
            ptmp[i][1] = pmap[i].pos.z;
        }

        for (int v = 0; v < cam.dsize[1]; v++) {
            for (int u = 0; u < cam.dsize[0]; u++) {
                if (cmap(u, v).pos.z == 0.0) continue;

                const Vec2 cpix = getVec2(u, v);
                const Vec2 cnpx = invCam(cam, cpix);
                const Vec3 cpos = getVec3(cnpx.x, cnpx.y, 1.0) * cmap(u, v).pos.z;

                const Vec3 ppos = cam2prj * cpos;
                const Vec2 pnpx = prjVec(ppos);
                const Vec2 ppix = mulCamD(prj, pnpx);

                const SP_REAL div = acs2<Double2, SP_REAL>(ptmp, ppix.x, ppix.y, 0);
                if (div < SP_SMALL) continue;

                const SP_REAL depth = acs2<Double2, SP_REAL>(ptmp, ppix.x, ppix.y, 1) / div;
                if (depth < ppos.z - 1.0) continue;

                if (inRect(prj.dsize, round(ppix.x), round(ppix.y)) == false) continue;
                dst(u, v) = ppix;

            }
        }

    }
}

#endif
