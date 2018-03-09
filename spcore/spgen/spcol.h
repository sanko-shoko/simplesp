//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_COL_H__
#define __SP_COL_H__

#include "spcore/spcore.h"
#include "spcore/spgen/spvec.h"

namespace sp{

    //--------------------------------------------------------------------------------
    // color
    //--------------------------------------------------------------------------------

    // get color
    SP_GENFUNC Col3 getCol(const Byte r, const Byte g, const Byte b){
        Col3 dst;
        dst.r = r; dst.g = g; dst.b = b;
        return dst;
    }

    // get color
    SP_GENFUNC Col4 getCol(const Byte r, const Byte g, const Byte b, const Byte a) {
        Col4 dst;
        dst.r = r; dst.g = g; dst.b = b; dst.a = a;
        return dst;
    }

    // get color
    SP_GENFUNC Col3 getCol(const Vec3 &vec) {
        Col3 dst;
        cnvVal(dst.r, vec.x);
        cnvVal(dst.g, vec.y);
        cnvVal(dst.b, vec.z);
        return dst;
    }

    // addition
    SP_GENFUNC Col3 addCol(const Col3 &col0, const Col3 &col1) {
        const Byte r = static_cast<Byte>(maxVal(0, minVal(255, col0.r + col1.r)));
        const Byte g = static_cast<Byte>(maxVal(0, minVal(255, col0.g + col1.g)));
        const Byte b = static_cast<Byte>(maxVal(0, minVal(255, col0.b + col1.b)));
        return getCol(r, g, b);
    }

    // subtraction
    SP_GENFUNC Col3 subCol(const Col3 &col0, const Col3 &col1) {
        const Byte r = static_cast<Byte>(maxVal(0, minVal(255, col0.r - col1.r)));
        const Byte g = static_cast<Byte>(maxVal(0, minVal(255, col0.g - col1.g)));
        const Byte b = static_cast<Byte>(maxVal(0, minVal(255, col0.b - col1.b)));
        return getCol(r, g, b);
    }

    // multiple
    SP_GENFUNC Col3 mulCol(const Col3 &col, const double val) {
        const Byte r = static_cast<Byte>(maxVal(0, minVal(255, static_cast<int>(col.r * val))));
        const Byte g = static_cast<Byte>(maxVal(0, minVal(255, static_cast<int>(col.g * val))));
        const Byte b = static_cast<Byte>(maxVal(0, minVal(255, static_cast<int>(col.b * val))));
        return getCol(r, g, b);
    }

    // division
    SP_GENFUNC Col3 divCol(const Col3 &col, const double val) {
        return (val != 0.0) ? mulCol(col, 1.0 / val) : col;
    }


    //--------------------------------------------------------------------------------
    // color operator
    //--------------------------------------------------------------------------------

    SP_GENFUNC Col3 operator + (const Col3 &col0, const Col3 &col1) {
        return addCol(col0, col1);
    }

    SP_GENFUNC Col3 operator - (const Col3 &col0, const Col3 &col1) {
        return subCol(col0, col1);
    }

    SP_GENFUNC Col3 operator * (const Col3 &col, const double val) {
        return mulCol(col, val);
    }

    SP_GENFUNC Col3 operator / (const Col3 &col, const double val) {
        return divCol(col, val);
    }

    SP_GENFUNC void operator += (Col3 &col0, const Col3 &col1) {
        col0 = addCol(col0, col1);
    }

    SP_GENFUNC void operator -= (Col3 &col0, const Col3 &col1) {
        col0 = subCol(col0, col1);
    }

    SP_GENFUNC void operator *= (Col3 &col, const double val) {
        col = mulCol(col, val);
    }

    SP_GENFUNC void operator /= (Col3 &col, const double val) {
        col = divCol(col, val);
    }


    //--------------------------------------------------------------------------------
    // blend
    //--------------------------------------------------------------------------------

    SP_GENFUNC Col3 blendCol(const Col3 &col0, const Col3 &col1, const double rate){
        Col3 col;
        cnvVal(col.r, col0.r * rate + col1.r * (1.0 - rate));
        cnvVal(col.g, col0.g * rate + col1.g * (1.0 - rate));
        cnvVal(col.b, col0.b * rate + col1.b * (1.0 - rate));
        return col;
    }


    //--------------------------------------------------------------------------------
    // color space
    //--------------------------------------------------------------------------------

    // convert phase to col3(rainbow), phase = [0, 1]
    SP_GENFUNC void cnvPhaseToCol(Col3 &col, const double phase){
        const double p = maxVal(0.0, minVal(phase, 1.0));
        const double s = SP_PI + SP_PI / 4;

        cnvVal(col.r, 255 * (sin(1.5 * SP_PI * p + SP_PI * (9.0 / 4.0)) + 1.0) / 2.0);
        cnvVal(col.g, 255 * (sin(1.5 * SP_PI * p + SP_PI * (7.0 / 4.0)) + 1.0) / 2.0);
        cnvVal(col.b, 255 * (sin(1.5 * SP_PI * p + SP_PI * (5.0 / 4.0)) + 1.0) / 2.0);
    }

    // convert hsv to col3, hsv = Vec3(h = [0, 2 * PI], s = [0, 1], v = [0, 1])
    SP_GENFUNC void cnvHSVToCol(Col3 &col, const Vec3 &hsv){
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
        
        switch (floor(D) % 6){
        case 0: col = getCol(uv, uc, ua); break;
        case 1: col = getCol(ub, uv, ua); break;
        case 2: col = getCol(ua, uv, uc); break;
        case 3: col = getCol(ua, ub, uv); break;
        case 4: col = getCol(uc, ua, uv); break;
        case 5: col = getCol(uv, ua, ub); break;
        }
    }

    // convert col3 to hsv, hsv = Vec3(h = [0, 2 * PI], s = [0, 1], v = [0, 1])
    SP_GENFUNC void cnvColToHSV(Vec3 &hsv, const Col3 &col){
        const double maxv = maxVal(col.r, maxVal(col.g, col.b));
        const double minv = minVal(col.r, minVal(col.g, col.b));
        const double subv = maxv - minv;

        double h, s, v;
        {
            v = maxv / 255.0;
            s = subv / maxVal(maxv, 1.0);
        }
        if (subv == 0.0) {
            h = 0.0;
        }
        else if (col.r == maxv) {
            h = (col.b - col.g) / subv + 0.0;
        }
        else if (col.g == maxv) {
            h = (col.r - col.b) / subv + 2.0;
        }
        else if (col.b == maxv) {
            h = (col.g - col.r) / subv + 4.0;
        }
        
        h *= SP_PI / 3.0;
        if (h < 0.0) {
            h += 2 * SP_PI;
        }

        hsv = getVec(h, s, v);
    }

    
    SP_GENFUNC void cnvXYZToLab(Vec3 &lab, const Vec3 &xyz){

        const Vec3 w = getVec(0.95047, 1.00000, 1.0883); // D65

        auto f = [](const double v)-> double {
            return (v > 0.008856) ? pow(v, 1.0 / 3.0) : (7.787 * v) + (16.0 / 116.0);
        };

        Vec3 val;
        val.x = f(xyz.x / w.x);
        val.y = f(xyz.y / w.y);
        val.z = f(xyz.z / w.z);

        const double l = (116.0 * val.y) - 16.0;
        const double a = 500.0 * (val.x - val.y);
        const double b = 200.0 * (val.y - val.z);

        lab = getVec(l, a, b);
    }

    
    SP_GENFUNC void cnvLabToXYZ(Vec3 &xyz, const Vec3 &lab) {

        const Vec3 w = getVec(0.95047, 1.00000, 1.0883); // D65

        auto f = [](const double v)-> double {
            return (v > 0.206897) ? pow(v, 3.0) : 0.001107 * (116.0 * v - 16.0);
        };

        Vec3 val;
        val.y = (lab.x + 16.0) / 116.0;
        val.x = val.y + lab.y / 500.0;
        val.z = val.y - lab.z / 200.0;

        xyz.x = f(val.x) * w.x;
        xyz.y = f(val.y) * w.y;
        xyz.z = f(val.z) * w.z;
    }

    SP_GENFUNC void cnvColToXYZ(Vec3 &xyz, const Col3 &col){
        auto f = [](const double v)-> double {
            return (v > 0.040450) ? pow((v + 0.055) / 1.055, 2.4) : v / 12.92;
        };

        Vec3 val;
        val.x = f(col.r / 255.0);
        val.y = f(col.g / 255.0);
        val.z = f(col.b / 255.0);

        // D65
        xyz.x = +0.412391 * val.x + 0.357584 * val.y + 0.180481 * val.z;
        xyz.y = +0.212639 * val.x + 0.715169 * val.y + 0.072192 * val.z;
        xyz.z = +0.019331 * val.x + 0.119195 * val.y + 0.950532 * val.z;
    }

    SP_GENFUNC void cnvXYZToCol(Col3 &col, const Vec3 &xyz) {
        auto f = [](const double v)-> double {
            return (v > 0.0031308) ? 1.055 * pow(v, 1.0 / 2.4) - 0.055 : 12.92 * v;
        };

        Vec3 val;

        // D65
        val.x = +3.240970 * xyz.x - 1.537383 * xyz.y - 0.498611 * xyz.z;
        val.y = -0.969244 * xyz.x + 1.875968 * xyz.y + 0.041555 * xyz.z;
        val.z = 0.055630 * xyz.x - 0.203977 * xyz.y + 1.056972 * xyz.z;
    
        val.x = minVal(1.0, f(val.x)) * 255.0;
        val.y = minVal(1.0, f(val.y)) * 255.0;
        val.z = minVal(1.0, f(val.z)) * 255.0;

        col = getCol(val);
    }

    SP_GENFUNC void cnvColToLab(Vec3 &lab, const Col3 &col){
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
    // convert color
    //--------------------------------------------------------------------------------

    template <typename TYPE>
    SP_GENFUNC void cnvCol(TYPE &dst, const TYPE &src){
        dst = src;
    }

    SP_GENFUNC void cnvCol(Col3 &dst, const Byte &src){
        dst = getCol(src, src, src);
    }

    SP_GENFUNC void cnvCol(Byte &dst, const Col3 &src){
        dst = static_cast<Byte>(0.299 * src.r + 0.587 * src.g + 0.114 * src.b + 0.5);
    }

    SP_GENFUNC void cnvCol(Byte &dst, const Col4 &src) {
        dst = static_cast<Byte>(0.299 * src.r + 0.587 * src.g + 0.114 * src.b + 0.5);
    }

    SP_GENFUNC void cnvCol(Col3 &dst, const Col4 &src) {
        dst.r = src.r;
        dst.g = src.g;
        dst.b = src.b;
    }

    SP_GENFUNC void cnvCol(Col4 &dst, const Col3 &src) {
        dst.r = src.r;
        dst.g = src.g;
        dst.b = src.b;
        dst.a = SP_BYTEMAX;
    }

    
    //--------------------------------------------------------------------------------
    // convert geom to image
    //--------------------------------------------------------------------------------

    SP_GENFUNC void cnvDepthToCol(Byte &dst, const double depth, const double nearPlane, const double farPlane){
        const double rate = 1.0 - (depth - nearPlane) / (farPlane - nearPlane);
        dst = static_cast<Byte>(255 * rate);
    }

    SP_GENFUNC void cnvDepthToCol(Col3 &dst, const double depth, const double nearPlane, const double farPlane){
        const double rate = 1.0 - (depth - nearPlane) / (farPlane - nearPlane);
        cnvPhaseToCol(dst, rate);
    }

    SP_GENFUNC void cnvNormalToCol(Byte &dst, const Vec3 &nrm){
        dst = (nrm.z < 0) ? static_cast<Byte>(-255 * nrm.z) : 0;
    }

    SP_GENFUNC void cnvNormalToCol(Col3 &dst, const Vec3 &nrm){
        dst.r = static_cast<Byte>(255 * (1.0 - nrm.x) / 2);
        dst.g = static_cast<Byte>(255 * (1.0 - nrm.y) / 2);
        dst.b = static_cast<Byte>(255 * (1.0 - nrm.z) / 2);
    }

    SP_GENFUNC void cnvDispToCol(Byte &dst, const Disp &disp, const int maxDisp, const int minDisp) {
        const double rate = 1.0 - (disp.disp - minDisp) / (maxDisp - minDisp);
        dst = static_cast<Byte>(255 * rate);
    }

    SP_GENFUNC void cnvDispToCol(Col3 &dst, const Disp &disp, const int maxDisp, const int minDisp) {
        const double rate = 1.0 - (disp.disp - minDisp) / (maxDisp - minDisp);
        cnvPhaseToCol(dst, rate);
    }

}

#endif