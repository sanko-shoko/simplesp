//--------------------------------------------------------------------------------
// Copyright (c) 2017-2020, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_CAM_H__
#define __SP_CAM_H__

#include "spcore/spcom.h"
#include "spcore/spgen/spbase.h"
#include "spcore/spgen/spvec.h"
#include "spcore/spgen/spmath.h"

namespace sp{

    //--------------------------------------------------------------------------------
    // camera parameter
    //--------------------------------------------------------------------------------

    SP_GENFUNC CamParam getCamParam(const int dsize0, const int dsize1, const double fx, const double fy, const double cx, const double cy){
        CamParam dst;
        dst.type = CamParam_Pers;
        dst.dsize[0] = dsize0;
        dst.dsize[1] = dsize1;

        dst.fx = SP_CAST_REAL(fx);
        dst.fy = SP_CAST_REAL(fy);
        dst.cx = SP_CAST_REAL(cx);
        dst.cy = SP_CAST_REAL(cy);

        dst.k1 = SP_CAST_REAL(0.0);
        dst.k2 = SP_CAST_REAL(0.0);
        dst.k3 = SP_CAST_REAL(0.0);
        dst.k4 = SP_CAST_REAL(0.0);
        dst.p1 = SP_CAST_REAL(0.0);
        dst.p2 = SP_CAST_REAL(0.0);
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
        dist.x = SP_CAST_REAL(npx.x * k + cam.p1 * (2.0 * xy) + cam.p2 * (2.0 * x2 + r2));
        dist.y = SP_CAST_REAL(npx.y * k + cam.p1 * (2.0 * y2 + r2) + cam.p2 * (2.0 * xy));

        jacob[0 * 9 + 0] = SP_CAST_REAL(dist.x);
        jacob[0 * 9 + 1] = SP_CAST_REAL(0.0);
        jacob[0 * 9 + 2] = SP_CAST_REAL(1.0);
        jacob[0 * 9 + 3] = SP_CAST_REAL(0.0);
        jacob[0 * 9 + 4] = SP_CAST_REAL(cam.fx * (npx.x * r2));
        jacob[0 * 9 + 5] = SP_CAST_REAL(cam.fx * (npx.x * r4));
        jacob[0 * 9 + 6] = SP_CAST_REAL(cam.fx * (npx.x * r6));
        jacob[0 * 9 + 7] = SP_CAST_REAL(cam.fx * (2.0 * xy));
        jacob[0 * 9 + 8] = SP_CAST_REAL(cam.fx * (2.0 * x2 + r2));

        jacob[1 * 9 + 0] = SP_CAST_REAL(0.0);
        jacob[1 * 9 + 1] = SP_CAST_REAL(dist.y);
        jacob[1 * 9 + 2] = SP_CAST_REAL(0.0);
        jacob[1 * 9 + 3] = SP_CAST_REAL(1.0);
        jacob[1 * 9 + 4] = SP_CAST_REAL(cam.fy * (npx.y * r2));
        jacob[1 * 9 + 5] = SP_CAST_REAL(cam.fy * (npx.y * r4));
        jacob[1 * 9 + 6] = SP_CAST_REAL(cam.fy * (npx.y * r6));
        jacob[1 * 9 + 7] = SP_CAST_REAL(cam.fy * (2.0 * y2 + r2));
        jacob[1 * 9 + 8] = SP_CAST_REAL(cam.fy * (2.0 * xy));
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

        jacob[0 * 2 + 0] = SP_CAST_REAL(x2 * k2 + k1 + 2.0 * cam.p1 * npx.y + 6.0 * cam.p2 * npx.x);
        jacob[1 * 2 + 1] = SP_CAST_REAL(y2 * k2 + k1 + 6.0 * cam.p1 * npx.y + 2.0 * cam.p2 * npx.x);

        jacob[0 * 2 + 1] = SP_CAST_REAL(xy * k2 + 2.0 * cam.p1 * npx.x + 2.0 * cam.p2 * npx.y);
        jacob[1 * 2 + 0] = SP_CAST_REAL(xy * k2 + 2.0 * cam.p1 * npx.x + 2.0 * cam.p2 * npx.y);
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
        dist.x = SP_CAST_REAL(npx.x * k + cam.p1 * (2.0 * xy) + cam.p2 * (2.0 * x2 + r2));
        dist.y = SP_CAST_REAL(npx.y * k + cam.p1 * (2.0 * y2 + r2) + cam.p2 * (2.0 * xy));

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


#endif