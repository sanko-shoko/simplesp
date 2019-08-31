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
    // position and normal
    //--------------------------------------------------------------------------------
    
    struct VecPN2{
        Vec2 pos, nrm;
    };

    struct VecPN3{
        Vec3 pos, nrm;
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

#endif