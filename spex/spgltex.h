//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_GLTEX_H__
#define __SP_GLTEX_H__

#include "GLFW/glfw3.h"

#include "spapp/spapp.h"

namespace sp{

    //--------------------------------------------------------------------------------
    // texture
    //--------------------------------------------------------------------------------
    
    class Texture {

    private:

        Mem1<char> mem;

    public:

        // texture id
        GLuint id;

        Texture() {
            id = 0;
        }

        template <typename TYPE>
        Texture(const Mem<TYPE> &img) {
            setImg(img);
        }

        Texture(const Texture &texture) {
            *this = texture;
        }

        Texture& operator = (const Texture &texture) {
            mem = texture.mem;
            setImg(mem);
            return *this;
        }
        ~Texture() {
            if (id > 0) {
                glDeleteTextures(1, &id);
            }
        }

        template <typename TYPE>
        bool setImg(const Mem<TYPE> &img) {
            return setImg(img.ptr, img.dsize, sizeof(TYPE));
        }

        bool setImg(const void *img, const int *dsize, const int ch) {

            int format;
            switch (ch) {
            case 1: format = GL_LUMINANCE; break;
            case 3: format = GL_RGB; break;
            case 4: format = GL_RGBA; break;
            default: return false;
            }

            mem.resize(dsize[0] * dsize[1] * ch, img);
          
            if (id > 0) {
                glDeleteTextures(1, &id);
            }
            glGenTextures(1, &id);

            glBindTexture(GL_TEXTURE_2D, id);

            glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, dsize[0], dsize[1], 0, format, GL_UNSIGNED_BYTE, mem.ptr);

            glBindTexture(GL_TEXTURE_2D, 0);

            return (id > 0) ? true : false;
        }
    };

    template<typename TYPE>
    SP_CPUFUNC void glTexImg(const Mem<TYPE> &src) {
        if (src.size() == 0) return;

        Texture tex;
        if (tex.setImg(src) == false) return;

        glPushAttrib(GL_ENABLE_BIT);
        glEnable(GL_TEXTURE_2D);

        {
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

            glBindTexture(GL_TEXTURE_2D, tex.id);

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

            //glShadeModel(GL_FLAT);
            glColor3d(1.0, 1.0, 1.0);
            glColorMask(1, 1, 1, 1);

            const int w = src.dsize[0];
            const int h = src.dsize[1];

            glBegin(GL_QUADS);
            glTexCoord2i(0, 0); glVertex2d(0 - 0.5, 0 - 0.5);
            glTexCoord2i(0, 1); glVertex2d(0 - 0.5, h - 0.5);
            glTexCoord2i(1, 1); glVertex2d(w - 0.5, h - 0.5);
            glTexCoord2i(1, 0); glVertex2d(w - 0.5, 0 - 0.5);
            glEnd();

            glBindTexture(GL_TEXTURE_2D, 0);
        }
        glPopAttrib();
    }

    template<typename TYPE0 = Col3, typename TYPE1>
    SP_CPUFUNC void glTexDepth(const Mem<TYPE1> &src, const double nearPlane = 100.0, const double farPlane = 10000.0) {
        if (src.size() == 0) return;

        Mem2<TYPE0> img;
        cnvDepthToImg(img, src, nearPlane, farPlane);
        glTexImg(img);
    }

    template<typename TYPE0 = Col3, typename TYPE1>
    SP_CPUFUNC void glTexNormal(const Mem<TYPE1> &src) {
        if (src.size() == 0) return;

        Mem2<TYPE0> img;
        cnvNormalToImg(img, src);
        glTexImg(img);
    }

}

#endif
