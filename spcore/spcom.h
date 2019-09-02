//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_COM_H__
#define __SP_COM_H__


//--------------------------------------------------------------------------------
// system
//--------------------------------------------------------------------------------

#ifndef SP_USE_OMP
#define SP_USE_OMP 1
#endif

#ifndef SP_USE_ASSERT
#define SP_USE_ASSERT 1
#endif

#ifndef SP_USE_WRAPPER
#define SP_USE_WRAPPER 1
#endif

#ifndef SP_USE_CONSOLE
#define SP_USE_CONSOLE 1
#endif

#ifndef SP_USE_DEBUG
#define SP_USE_DEBUG 0
#endif


//--------------------------------------------------------------------------------
// call
//--------------------------------------------------------------------------------

// cpu function
#ifndef SP_CPUFUNC
#define SP_CPUFUNC static
#endif

// cpu/gpu function
#ifndef SP_GENFUNC
#define SP_GENFUNC static
#endif


//--------------------------------------------------------------------------------
// const value
//--------------------------------------------------------------------------------

// pi
#define SP_PI static_cast<SP_REAL>(3.14159265358979323846)

// limit value regarded as zero
#define SP_SMALL SP_RCAST(1.0e-20)

// limit value regarded as infinity
#define SP_INFINITY SP_RCAST(1.0e+20)

// maximal value (unsigned char)
#define SP_BYTEMAX 255

// maximal value (int)
#define SP_INTMAX ((int)(((unsigned)-1)/2))

// maximal string
#define SP_STRMAX 512

// maximal dimension
#define SP_DIMMAX 3


//--------------------------------------------------------------------------------
// real number type
//--------------------------------------------------------------------------------

#ifndef SP_REAL
#define SP_REAL double
#endif


//--------------------------------------------------------------------------------
// struct
//--------------------------------------------------------------------------------

namespace sp{

    //--------------------------------------------------------------------------------
    // type
    //--------------------------------------------------------------------------------
    
    typedef unsigned char Byte;

    //--------------------------------------------------------------------------------
    // complex
    //--------------------------------------------------------------------------------

    struct Cmp {
        SP_REAL re, im;
    };

    //--------------------------------------------------------------------------------
    // memory ptr
    //--------------------------------------------------------------------------------

    template<typename TYPE> struct ExPtr{
        // pointer
        TYPE *ptr;

        // dimension
        int dim;

        // dimension size
        int dsize[SP_DIMMAX];
    };

    //--------------------------------------------------------------------------------
    // rectangle
    //--------------------------------------------------------------------------------

    struct Rect2 {
        // dimension base
        int dbase[2];

        // dimension size
        int dsize[2];
    };

    struct Rect3 {
        // dimension base
        int dbase[3];

        // dimension size
        int dsize[3];
    };


    //--------------------------------------------------------------------------------
    // vector
    //--------------------------------------------------------------------------------

    struct Vec2{
        SP_REAL x, y;
    };

    struct Vec3{
        SP_REAL x, y, z;
    };

    //--------------------------------------------------------------------------------
    // position and direction
    //--------------------------------------------------------------------------------
    
    struct VecPD2{
        Vec2 pos, drc;
    };

    struct VecPD3{
        Vec3 pos, drc;
    };

    //--------------------------------------------------------------------------------
    // line
    //--------------------------------------------------------------------------------

    struct Line2 {
        Vec2 pos[2];
    };

    struct Line3 {
        Vec3 pos[2];
    };

    //--------------------------------------------------------------------------------
    // triangle mesh
    //--------------------------------------------------------------------------------
    
    struct Mesh2{
        Vec2 pos[3];
    };

    struct Mesh3 {
        Vec3 pos[3];
    };

    //--------------------------------------------------------------------------------
    // Axis Aligned Bounding Box
    //--------------------------------------------------------------------------------

    struct Box2 {
        Vec2 pos[2];
    };

    struct Box3 {
        Vec3 pos[2];
    };


    //--------------------------------------------------------------------------------
    // 3d transform
    //--------------------------------------------------------------------------------

    struct Rot{
        // quaternion
        SP_REAL qx, qy, qz, qw;
    };

    struct Pose{
        // rotation
        Rot rot;

        // translation
        Vec3 trn;
    };


    //--------------------------------------------------------------------------------
    // camera parameter
    //--------------------------------------------------------------------------------

    struct CamParam{
        int dsize[2];
        SP_REAL fx, fy;
        SP_REAL cx, cy;
        SP_REAL k1, k2, k3, p1, p2;
    };


    //--------------------------------------------------------------------------------
    // color
    //--------------------------------------------------------------------------------

    struct Col3{
        Byte r, g, b;
    };

    struct Col4 : public Col3 {
        Byte a;
    };

    struct Material {
        Col4 amb;
        Col4 dif;
        Col4 spc;
        Col4 ems;
        Byte shn;
    };

    //--------------------------------------------------------------------------------
    // byte order
    //--------------------------------------------------------------------------------
    
    enum ByteOrder { BigEndian, LittleEndian };

}


//--------------------------------------------------------------------------------
// cast
//--------------------------------------------------------------------------------

#define SP_RCAST(V) static_cast<SP_REAL>(V)

namespace sp {

    //--------------------------------------------------------------------------------
    // basic type
    //--------------------------------------------------------------------------------
  
    template<typename TYPE> SP_GENFUNC void _cast(char &dst, const TYPE &src) {
        dst = static_cast<char>(src + 0.5 - (src < 0));
    }

    template<typename TYPE> SP_GENFUNC void _cast(short &dst, const TYPE &src) {
        dst = static_cast<short>(src + 0.5 - (src < 0));
    }

    template<typename TYPE> SP_GENFUNC void _cast(int &dst, const TYPE &src) {
        dst = static_cast<int>(src + 0.5 - (src < 0));
    }

    template<typename TYPE> SP_GENFUNC void _cast(float &dst, const TYPE &src) {
        dst = static_cast<float>(src);
    }

    template<typename TYPE> SP_GENFUNC void _cast(double &dst, const TYPE &src) {
        dst = static_cast<double>(src);
    }

    //--------------------------------------------------------------------------------
    // byte
    //--------------------------------------------------------------------------------

    SP_GENFUNC void _cast(Byte &dst, const int &src) {
        dst = static_cast<unsigned char>((src + 0.5) * (src > 0));
    }

    SP_GENFUNC void _cast(Byte &dst, const double &src) {
        dst = static_cast<unsigned char>((src + 0.5) * (src > 0));
    }

    SP_GENFUNC void _cast(Byte &dst, const Byte &src) {
        dst = src;
    }

    SP_GENFUNC void _cast(Byte &dst, const Col3 &src) {
        dst = static_cast<Byte>(0.299 * src.r + 0.587 * src.g + 0.114 * src.b + 0.5);
    }

    //--------------------------------------------------------------------------------
    // color 3
    //--------------------------------------------------------------------------------

    SP_GENFUNC void _cast(Col3 &dst, const Byte &src) {
        dst.r = src;
        dst.g = src;
        dst.b = src;
    }

    SP_GENFUNC void _cast(Col3 &dst, const Col3 &src) {
        dst.r = src.r;
        dst.g = src.g;
        dst.b = src.b;
    }

    SP_GENFUNC void _cast(Col3 &dst, const Vec3 &src) {
        dst.r = static_cast<Byte>(src.x * SP_BYTEMAX + 0.5);
        dst.g = static_cast<Byte>(src.y * SP_BYTEMAX + 0.5);
        dst.b = static_cast<Byte>(src.z * SP_BYTEMAX + 0.5);
    }
    
    //--------------------------------------------------------------------------------
    // color 4
    //--------------------------------------------------------------------------------
  
    SP_GENFUNC void _cast(Col4 &dst, const Byte &src) {
        dst.r = src;
        dst.g = src;
        dst.b = src;
        dst.a = SP_BYTEMAX;
    }

    SP_GENFUNC void _cast(Col4 &dst, const Col3 &src) {
        dst.r = src.r;
        dst.g = src.g;
        dst.b = src.b;
        dst.a = SP_BYTEMAX;
    }

    SP_GENFUNC void _cast(Col4 &dst, const Col4 &src) {
        dst.r = src.r;
        dst.g = src.g;
        dst.b = src.b;
        dst.a = src.a;
    }

    SP_GENFUNC void _cast(Col4 &dst, const Vec3 &src) {
        dst.r = static_cast<Byte>(src.x * SP_BYTEMAX + 0.5);
        dst.g = static_cast<Byte>(src.y * SP_BYTEMAX + 0.5);
        dst.b = static_cast<Byte>(src.z * SP_BYTEMAX + 0.5);
        dst.a = SP_BYTEMAX;
    }

    //--------------------------------------------------------------------------------
    // vector 3
    //--------------------------------------------------------------------------------

    SP_GENFUNC void _cast(Vec3 &dst, const Col3 &src) {
        dst.x = static_cast<SP_REAL>(src.r) / SP_BYTEMAX;
        dst.y = static_cast<SP_REAL>(src.g) / SP_BYTEMAX;
        dst.z = static_cast<SP_REAL>(src.b) / SP_BYTEMAX;
    }

  
    template<typename DST, typename SRC> SP_GENFUNC DST cast(const SRC &src) {
        DST dst;
        _cast(dst, src);
        return dst;
    }
}


//--------------------------------------------------------------------------------
// compare
//--------------------------------------------------------------------------------

namespace sp {

    // compare double
    SP_GENFUNC bool cmp(const double val0, const double val1, const double t = 1.0e-6) {
        return ((val0 - val1) < +t && (val0 - val1) > -t) ? true : false;
    }

    // compare complex
    SP_GENFUNC bool cmp(const Cmp &cmp0, const Cmp &cmp1, const double t = 1.0e-6) {
        return cmp(cmp0.re, cmp1.re, t) & cmp(cmp0.im, cmp1.im, t);
    }

    // compare memory
    SP_CPUFUNC bool cmp(const void *src0, const void *src1, const int size) {
        const char *p0 = reinterpret_cast<const char*>(src0);
        const char *p1 = reinterpret_cast<const char*>(src1);
        for (int i = 0; i < size; i++) {
            if(p0[i] != p1[i]) return false;
        }
        return true;
    }

    // compare size
    SP_GENFUNC bool cmp(const int dim, const int *dsize0, const int *dsize1) {
        for (int i = 0; i < dim; i++) {
            if (dsize0[i] != dsize1[i]) return false;
        }
        return true;
    }

    // compare size
    template<typename TYPE0, typename TYPE1>
    SP_GENFUNC bool cmp(const ExPtr<TYPE0> &mem0, const ExPtr<TYPE1> &mem1) {
        if (mem0.dim != mem1.dim) return false;
        return cmp(mem0.dim, mem0.dsize, mem1.dsize);
    }

    // compare rect
    SP_GENFUNC bool cmp(const Rect2 &rect0, const Rect2 &rect1) {
        for (int i = 0; i < 2; i++) {
            if (rect0.dbase[i] != rect1.dbase[i]) return false;
            if (rect0.dsize[i] != rect1.dsize[i]) return false;
        }
        return true;
    }

    // compare rect
    SP_GENFUNC bool cmp(const Rect3 &rect0, const Rect3 &rect1) {
        for (int i = 0; i < 3; i++) {
            if (rect0.dbase[i] != rect1.dbase[i]) return false;
            if (rect0.dsize[i] != rect1.dsize[i]) return false;
        }
        return true;
    }

    // compare vec
    SP_GENFUNC bool cmp(const Vec2 &vec0, const Vec2 &vec1, const double t = 1.0e-6) {
        return cmp(vec0.x, vec1.x, t) & cmp(vec0.y, vec1.y, t);
    }
    // compare vec
    SP_GENFUNC bool cmp(const Vec3 &vec0, const Vec3 &vec1, const double t = 1.0e-6) {
        return cmp(vec0.x, vec1.x, t) & cmp(vec0.y, vec1.y, t) & cmp(vec0.z, vec1.z, t);
    }

    // compare vec (position and normal)
    SP_GENFUNC bool cmp(const VecPD2 &vec0, const VecPD2 &vec1, const double t = 1.0e-6) {
        return cmp(vec0.pos, vec1.pos, t) & cmp(vec0.drc, vec1.drc, t);
    }
    // compare vec (position and normal)
    SP_GENFUNC bool cmp(const VecPD3 &vec0, const VecPD3 &vec1, const double t = 1.0e-6) {
        return cmp(vec0.pos, vec1.pos, t) & cmp(vec0.drc, vec1.drc, t);
    }
    
    // compare line
    SP_GENFUNC bool cmp(const Line2 &line0, const Line2 &line1, const double t = 1.0e-6) {
        return cmp(line0.pos[0], line1.pos[0], t) & cmp(line0.pos[1], line1.pos[1], t);
    }
    // compare line
    SP_GENFUNC bool cmp(const Line3 &line0, const Line3 &line1, const double t = 1.0e-6) {
        return cmp(line0.pos[0], line1.pos[0], t) & cmp(line0.pos[1], line1.pos[1], t);
    }

    // compare mesh
    SP_GENFUNC bool cmp(const Mesh2 &mesh0, const Mesh2 &mesh1, const double t = 1.0e-6) {
        return cmp(mesh0.pos[0], mesh0.pos[0], t) & cmp(mesh1.pos[1], mesh1.pos[1], t) & cmp(mesh1.pos[2], mesh1.pos[2], t);
    }
    // compare mesh
    SP_GENFUNC bool cmp(const Mesh3 &mesh0, const Mesh3 &mesh1, const double t = 1.0e-6) {
        return cmp(mesh0.pos[0], mesh0.pos[0], t) & cmp(mesh1.pos[1], mesh1.pos[1], t) & cmp(mesh1.pos[2], mesh1.pos[2], t);
    }

    // compare rotation
    SP_GENFUNC bool cmp(const Rot &rot0, const Rot &rot1, const double t = 1.0e-6) {
        bool ret = true;
        const double s0 = (rot0.qw > 0.0) ? +1.0 : -1.0;
        const double s1 = (rot1.qw > 0.0) ? +1.0 : -1.0;
        ret &= cmp(rot0.qx * s0, rot1.qx * s1, t);
        ret &= cmp(rot0.qy * s0, rot1.qy * s1, t);
        ret &= cmp(rot0.qz * s0, rot1.qz * s1, t);
        ret &= cmp(rot0.qw * s0, rot1.qw * s1, t);
        return ret;
    }

    // compare pose
    SP_GENFUNC bool cmp(const Pose &pose0, const Pose &pose1, const double tr = 1.0e-6, const double tt = 1.0e-6) {
        bool ret = true;
        ret &= cmp(pose0.rot, pose1.rot, tr);
        ret &= cmp(pose0.trn, pose1.trn, tt);
        return ret;
    }

    // compare color
    SP_GENFUNC bool cmp(const Col3 &col0, const Col3 &col1) {
        return (col0.r == col1.r) & (col0.g == col1.g) & (col0.b == col1.b);
    }

    // compare color
    SP_GENFUNC bool cmp(const Col4 &col0, const Col4 &col1) {
        return (col0.r == col1.r) & (col0.g == col1.g) & (col0.b == col1.b) & (col0.a == col1.a);
    }


#define SP_CMP_OPERATOR(TYPE) \
    SP_GENFUNC bool operator == (const TYPE &t0, const TYPE &t1) { return cmp(t0, t1); } \
    SP_GENFUNC bool operator != (const TYPE &t0, const TYPE &t1) { return !cmp(t0, t1); }

    SP_CMP_OPERATOR(Rect2);
    SP_CMP_OPERATOR(Rect3);
    SP_CMP_OPERATOR(Cmp);
    SP_CMP_OPERATOR(Vec2);
    SP_CMP_OPERATOR(Vec3);
    SP_CMP_OPERATOR(VecPD2);
    SP_CMP_OPERATOR(VecPD3);
    SP_CMP_OPERATOR(Line2);
    SP_CMP_OPERATOR(Line3);
    SP_CMP_OPERATOR(Mesh2);
    SP_CMP_OPERATOR(Mesh3);
    SP_CMP_OPERATOR(Rot);
    SP_CMP_OPERATOR(Pose);
    SP_CMP_OPERATOR(Col3);
    SP_CMP_OPERATOR(Col4);


}

#endif