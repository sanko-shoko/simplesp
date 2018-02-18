//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_CAM_H__
#define __SP_CAM_H__

#include "spcore/spcom.h"
#include "spcore/spwrap.h"
#include "spcore/spgen/spbase.h"
#include "spcore/spgen/spvec.h"
#include "spcore/spgen/spmath.h"

namespace sp{

    //--------------------------------------------------------------------------------
    // camera parameter
    //--------------------------------------------------------------------------------

    SP_GENFUNC CamParam getCamParam(const int dsize0, const int dsize1, const double fx, const double fy, const double cx, const double cy){
        CamParam dst;
        dst.dsize[0] = dsize0;
        dst.dsize[1] = dsize1;

        dst.fx = fx;
        dst.fy = fy;
        dst.cx = cx;
        dst.cy = cy;

        dst.k1 = 0.0;
        dst.k2 = 0.0;
        dst.k3 = 0.0;
        dst.p1 = 0.0;
        dst.p2 = 0.0;
        return dst;
    }

    SP_GENFUNC CamParam getCamParam(const int dsize0, const int dsize1, const double fx, const double fy) {
        return getCamParam(dsize0, dsize1, fx, fy, (dsize0 - 1) * 0.5, (dsize1 - 1) * 0.5);

    }

    SP_GENFUNC CamParam getCamParam(const int dsize0, const int dsize1) {
        // groundless camera parameter, but in many cases empirically, no big difference
        const double f = 0.8 * (dsize0 + dsize1);
        return getCamParam(dsize0, dsize1, f, f, (dsize0 - 1) * 0.5, (dsize1 - 1) * 0.5);
    }

    SP_GENFUNC CamParam getCamParam(const int *dsize) {
        return getCamParam(dsize[0], dsize[1]);
    }

    SP_GENFUNC void getMat(double *dst, const int rows, const int cols, const CamParam &cam) {
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

    SP_GENFUNC void jacobCamToPix(double *jacob, const CamParam &cam, const Vec2 &npx) {
        const double x2 = npx.x * npx.x;
        const double y2 = npx.y * npx.y;
        const double xy = npx.x * npx.y;

        const double r2 = x2 + y2;
        const double r4 = r2 * r2;
        const double r6 = r4 * r2;

        const double k = 1.0 + cam.k1 * r2 + cam.k2 * r4 + cam.k3 * r6;

        Vec2 dist;
        dist.x = npx.x * k + cam.p1 * (2.0 * xy) + cam.p2 * (2.0 * x2 + r2);
        dist.y = npx.y * k + cam.p1 * (2.0 * y2 + r2) + cam.p2 * (2.0 * xy);

        jacob[0 * 9 + 0] = dist.x;
        jacob[0 * 9 + 1] = 0.0;
        jacob[0 * 9 + 2] = 1.0;
        jacob[0 * 9 + 3] = 0.0;
        jacob[0 * 9 + 4] = cam.fx * (npx.x * r2);
        jacob[0 * 9 + 5] = cam.fx * (npx.x * r4);
        jacob[0 * 9 + 6] = cam.fx * (npx.x * r6);
        jacob[0 * 9 + 7] = cam.fx * (2.0 * xy);
        jacob[0 * 9 + 8] = cam.fx * (2.0 * x2 + r2);

        jacob[1 * 9 + 0] = 0.0;
        jacob[1 * 9 + 1] = dist.y;
        jacob[1 * 9 + 2] = 0.0;
        jacob[1 * 9 + 3] = 1.0;
        jacob[1 * 9 + 4] = cam.fy * (npx.y * r2);
        jacob[1 * 9 + 5] = cam.fy * (npx.y * r4);
        jacob[1 * 9 + 6] = cam.fy * (npx.y * r6);
        jacob[1 * 9 + 7] = cam.fy * (2.0 * y2 + r2);
        jacob[1 * 9 + 8] = cam.fy * (2.0 * xy);
    }

    SP_GENFUNC void jacobNpxToDist(double *jacob, const CamParam &cam, const Vec2 &npx) {
        const double x2 = npx.x * npx.x;
        const double y2 = npx.y * npx.y;
        const double xy = npx.x * npx.y;

        const double r2 = x2 + y2;
        const double r4 = r2 * r2;
        const double r6 = r4 * r2;

        const double k1 = 1.0 + cam.k1 * r2 + cam.k2 * r4 + cam.k3 * r6;
        const double k2 = 2.0 * cam.k1 + 4.0 * cam.k2 * r2 + 6.0 * cam.k3 * r4;

        jacob[0 * 2 + 0] = x2 * k2 + k1 + 2.0 * cam.p1 * npx.y + 6.0 * cam.p2 * npx.x;
        jacob[1 * 2 + 1] = y2 * k2 + k1 + 6.0 * cam.p1 * npx.y + 2.0 * cam.p2 * npx.x;

        jacob[0 * 2 + 1] = xy * k2 + 2.0 * cam.p1 * npx.x + 2.0 * cam.p2 * npx.y;
        jacob[1 * 2 + 0] = xy * k2 + 2.0 * cam.p1 * npx.x + 2.0 * cam.p2 * npx.y;
    }

    SP_GENFUNC void jacobNpxToPix(double *jacob, const CamParam &cam, const Vec2 &npx) {

        double jNpxToDist[2 * 2] = { 0 };
        jacobNpxToDist(jNpxToDist, cam, npx);

        double jDistToPix[2 * 2] = { 0 };
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

    SP_GENFUNC Vec2 invCam(const CamParam &cam, const Vec2 &pix) {
        Vec2 npx;
        npx.x = (pix.x - cam.cx) / cam.fx;
        npx.y = (pix.y - cam.cy) / cam.fy;
        return npx;
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
        dist.x = npx.x * k + cam.p1 * (2.0 * xy) + cam.p2 * (2.0 * x2 + r2);
        dist.y = npx.y * k + cam.p1 * (2.0 * y2 + r2) + cam.p2 * (2.0 * xy);

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

            double J[2 * 2], inv[2 * 2];
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
    SP_GENFUNC Vec2 npxUndistX(const CamParam &cam, const Vec3 &epi, const double &npxx) {
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
            const double r6 = r4 * r2;

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
        return getVec(x, a * x + b);
    }

    // undistortion y
    SP_GENFUNC Vec2 npxUndistY(const CamParam &cam, const Vec3 &epi, const double &npxy) {
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
        return getVec(a * y + b, y);
    }
}


#endif