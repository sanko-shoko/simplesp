﻿//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SPGLEW_H__
#define __SPGLEW_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define GLEW_STATIC
#include "GL/glew.h"

namespace sp {

    class VertexBufferObject {

    private:
        GLuint m_buffer;

        int m_size;

    private:
        VertexBufferObject(const VertexBufferObject &vbo) {}
        VertexBufferObject& operator =(const VertexBufferObject &vbo) {}

    public:

        VertexBufferObject() {
            m_size = 0;
            glGenBuffers(1, &m_buffer);
        }

        ~VertexBufferObject() {
            glDeleteBuffers(1, &m_buffer);
        }

        void data(const void *vtx, const int size) {
            bind();
            m_size = size;
            glBufferData(GL_ARRAY_BUFFER, size, vtx, GL_STATIC_DRAW);
            unbind();
        }

        void bind() {
            glBindBuffer(GL_ARRAY_BUFFER, m_buffer);
        }
        void unbind() {
            glBindBuffer(GL_ARRAY_BUFFER, 0);
        }
        int size() {
            return m_size;
        }
    };


    class FrameBufferObject {

    private:
        int samples;
        GLuint m_msfb;
        GLuint m_mstx[2];

    public:
        int dsize[2];
       
        GLuint m_fb;
        GLuint m_tx[2];
        bool m_bind;

    private:
        void reset() {
            memset(this, 0, sizeof(FrameBufferObject));
        }

    public:
        FrameBufferObject() {
            reset();
        }

        FrameBufferObject(const int *dsize) {
            reset();
            resize(dsize);
        }

        FrameBufferObject(const int dsize0, const int dsize1) {
            reset();
            resize(dsize0, dsize1);
        }

        ~FrameBufferObject() {
            free();
        }

        void free() {
            if (m_fb) {
                glDeleteFramebuffers(1, &m_fb);
            }
            if (m_msfb) {
                glDeleteFramebuffers(1, &m_msfb);
            }
            if (m_tx[0]) {
                glDeleteTextures(1, &m_tx[0]);
            }
            if (m_tx[1]) {
                glDeleteTextures(1, &m_tx[1]);
            }
            if (m_mstx[0]) {
                glDeleteTextures(1, &m_mstx[0]);
            }
            if (m_mstx[1]) {
                glDeleteTextures(1, &m_mstx[1]);
            }
            reset();
        }

        void resize(const int dsize[2], const int samples = 1) {
            const int s = maxval(1, samples);
            if (this->dsize[0] == dsize[0] && this->dsize[1] == dsize[1] && this->samples == s) return;

            free();
            this->dsize[0] = dsize[0];
            this->dsize[1] = dsize[1];
            this->samples = s;

            {
                glGenFramebuffers(1, &m_fb);
                glGenTextures(2, m_tx);

                // color
                glBindTexture(GL_TEXTURE_2D, m_tx[0]);
                glPixelStorei(GL_PACK_ALIGNMENT, 1);
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, dsize[0], dsize[1], 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);

                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

                // depth
                glBindTexture(GL_TEXTURE_2D, m_tx[1]);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
                glTexParameteri(GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE, GL_LUMINANCE);

                glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32, dsize[0], dsize[1], 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);
            }
            if (s > 1) {
                glGenFramebuffers(1, &m_msfb);
                glGenTextures(2, m_mstx);

                // color
                glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, m_mstx[0]);

                glPixelStorei(GL_PACK_ALIGNMENT, 1);
                glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, s, GL_RGBA, dsize[0], dsize[1], false);

                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

                // depth
                glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, m_mstx[1]);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
                glTexParameteri(GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE, GL_LUMINANCE);

                glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, s, GL_DEPTH_COMPONENT, dsize[0], dsize[1], false);
            }
            glBindTexture(GL_TEXTURE_2D, 0);
        }

        void resize(const int dsize0, const int dsize1, const int samples = 1) {
            int dsize[2] = { dsize0, dsize1 };
            resize(dsize, samples);
        }

        void bind() {
            glPushAttrib(GL_VIEWPORT_BIT);
            ::glViewport(0, 0, dsize[0], dsize[1]);

            if (m_msfb != 0) {
                glBindFramebuffer(GL_FRAMEBUFFER, m_msfb);
                glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE, m_mstx[0], 0);
                glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D_MULTISAMPLE, m_mstx[1], 0);
            }
            else {
                glBindFramebuffer(GL_FRAMEBUFFER, m_fb);
                glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_tx[0], 0);
                glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, m_tx[1], 0);
            }

            m_bind = true;
            //glClearColor(0.0, 0.0, 0.0, 1.0);
            //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        }

        void unbind() {
            if (m_msfb != 0) {
                glBindFramebuffer(GL_READ_FRAMEBUFFER, m_msfb);
                glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE, m_mstx[0], 0);
                glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D_MULTISAMPLE, m_mstx[1], 0);
                //glFramebufferRenderbuffer(GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);

                glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_fb);
                glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_tx[0], 0);
                glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, m_tx[1], 0);
                //glFramebufferRenderbuffer(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);

                glBlitFramebuffer(0, 0, dsize[0], dsize[1], 0, 0, dsize[0], dsize[1], GL_COLOR_BUFFER_BIT, GL_NEAREST);
                glBlitFramebuffer(0, 0, dsize[0], dsize[1], 0, 0, dsize[0], dsize[1], GL_DEPTH_BUFFER_BIT, GL_NEAREST);
            }
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
            glPopAttrib();
            m_bind = false;
        }

        void readi(void *img, const int ch) {
            if (dsize[0] == 0 || dsize[1] == 0) return;

            glBindFramebuffer(GL_FRAMEBUFFER, m_fb);

            unsigned char *tmp = new unsigned char[dsize[0] * dsize[1] * 4];
            glReadPixels(0, 0, dsize[0], dsize[1], GL_RGBA, GL_UNSIGNED_BYTE, tmp);

            unsigned char *dst = (unsigned char *)img;

#pragma omp parallel for
            for (int v = 0; v < dsize[1]; v++) {
                for (int u = 0; u < dsize[0]; u++) {
                    const int d = (v * dsize[0] + u) * ch;
                    const int s = ((dsize[1] - 1 - v) * dsize[0] + u) * 4;

                    dst[d + 0] = tmp[s + 0];
                    dst[d + 1] = tmp[s + 1];
                    dst[d + 2] = tmp[s + 2];

                    if (ch == 4) {
                        dst[d + 3] = tmp[s + 3];
                    }
                }
            }

            delete[]tmp;

            glBindFramebuffer(GL_FRAMEBUFFER, 0);
        }

        template<typename DEPTH>
        void readz(DEPTH *zbf, const bool pers, const double nearPlane = 1.0, const double farPlane = 10000.0) {
            if (dsize[0] == 0 || dsize[1] == 0) return;

            glBindFramebuffer(GL_FRAMEBUFFER, m_fb);

            float *tmp = new float[dsize[0] * dsize[1]];
            glReadPixels(0, 0, dsize[0], dsize[1], GL_DEPTH_COMPONENT, GL_FLOAT, tmp);

#pragma omp parallel for
            for (int v = 0; v < dsize[1]; v++) {
                for (int u = 0; u < dsize[0]; u++) {
                    const float t = tmp[(dsize[1] - 1 - v) * dsize[0] + u];
                    double d = 0.0;
                    if (pers == true) {
                        const double div = (farPlane - t * (farPlane - nearPlane));
                        if (div < 0.001) continue;

                        d = farPlane * nearPlane / div;
                    }
                    else {
                        const double p2 = 2.0 / (farPlane - nearPlane);
                        const double p3 = -(farPlane + nearPlane) / (farPlane - nearPlane);
                        d = (t * 2 - 1 - p3) / p2;
                    }
                    zbf[v * dsize[0] + u] = static_cast<DEPTH>((d > nearPlane && d < farPlane) ? d : 0.0);
                }
            }
            delete[]tmp;
            glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
        }

    };

    class Shader {
#define SHADER_BUFFER_MAX 100

    private:
        GLuint m_program;
        GLuint m_buffer[SHADER_BUFFER_MAX];

    private:
        void reset() {
            memset(this, 0, sizeof(Shader));
        }

        void free() {
            if (m_program) {
                glDeleteProgram(m_program);
            }
            for (int i = 0; i < SHADER_BUFFER_MAX; i++) {
                if (m_buffer[i] == 0) continue;
                glDisableVertexAttribArray(i);
                glDeleteBuffers(1, &m_buffer[i]);
            }
            reset();
        }

    public:
        Shader() {
            m_program = 0;
            memset(m_buffer, 0, sizeof(GLuint) * SHADER_BUFFER_MAX);
        }

        ~Shader() {
            free();
        }

        bool valid() {
            return m_program != 0 ? true : false;
        }

        //bool load(File &vert, File &frag, File &geom = File()) {
        //    File *flies[3] = { &vert, &frag, &geom };
        //    string texts[3];

        //    for (int i = 0; i < 3; i++) {
        //        File &file = *flies[i];
        //        string &text = texts[i];
        //        text = "";
        //        if (file.valid() == false) continue;

        //        char str[SP_STRMAX];

        //        const int pos = file.tell();
        //        file.seek(0);
        //        while (file.gets(str)) {
        //            text += str;
        //        }
        //        file.seek(pos);
        //    }
        //    return load(texts[0].c_str(), texts[1].c_str(), texts[2].c_str());
        //}
        bool load(const char *vert, const char *frag, const char *geom = NULL) {
            free();

            GLuint vertShader = 0;
            GLuint fragShader = 0;
            GLuint geomShader = 0;
            GLuint *pVertShader = NULL;
            GLuint *pFragShader = NULL;
            GLuint *pGeomShader = NULL;

            if (vert != NULL && vert[0] != '\0') {
                vertShader = getShader(vert, GL_VERTEX_SHADER);
                pVertShader = &vertShader;
            }
            if (frag != NULL && frag[0] != '\0') {
                fragShader = getShader(frag, GL_FRAGMENT_SHADER);
                pFragShader = &fragShader;
            }
            if (geom != NULL && geom[0] != '\0') {
                geomShader = getShader(geom, GL_GEOMETRY_SHADER_EXT);
                pGeomShader = &geomShader;
            }

            m_program = getProgram(pVertShader, pFragShader, pGeomShader);

            if (pVertShader != NULL) {
                glDeleteShader(*pVertShader);
            }
            if (pFragShader != NULL) {
                glDeleteShader(*pFragShader);
            }
            if (pGeomShader != NULL) {
                glDeleteShader(*pGeomShader);
            }

            return true;
        }

        void enable() {
            glUseProgram(m_program);


            //Mat proj(4, 4);
            //Mat view(4, 4);

            //glGetDoublev(GL_PROJECTION_MATRIX, proj.ptr);
            //glGetDoublev(GL_MODELVIEW_MATRIX, view.ptr);

            //proj = trnMat(proj);
            //view = trnMat(view);

            //const Mat tmat = trnMat(proj * view);

            //Mem2<float> tmatf(4, 4);
            //cnvMem(tmatf, tmat);

            //const GLint location = glGetUniformLocation(m_program, "mat");
            //glUniformMatrix4fv(location, 1, GL_FALSE, tmatf.ptr);
        }

        void disable() {
            glUseProgram(0);
        }

        template<typename TYPE>
        void setUniform1i(const char *name, const TYPE val) {
            const GLint location = glGetUniformLocation(m_program, name);
            glUniform1i(location, static_cast<int>(val));
        }

        template<typename TYPE>
        void setUniform1f(const char *name, const TYPE val) {
            const GLint location = glGetUniformLocation(m_program, name);
            glUniform1f(location, static_cast<float>(val));
        }

        template<typename TYPE>
        void setUniform2f(const char *name, const TYPE *vec) {
            const GLint location = glGetUniformLocation(m_program, name);
            glUniform2f(location, static_cast<float>(vec[0]), static_cast<float>(vec[1]));
        }
        template<typename TYPE>
        void setUniform3f(const char *name, const TYPE *vec) {
            const GLint location = glGetUniformLocation(m_program, name);
            glUniform3f(location, static_cast<float>(vec[0]), static_cast<float>(vec[1]), static_cast<float>(vec[2]));
        }

        template<typename TYPE>
        void setUniform3m(const char *name, const TYPE *mat) {
            float matf[3 * 3];
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    matf[r * 3 + c] = static_cast<float>(mat[c * 3 + r]);
                }
            }

            const GLint location = glGetUniformLocation(m_program, name);
            glUniformMatrix3fv(location, 1, GL_FALSE, matf);
        }

        template<typename TYPE>
        void setUniform4m(const char *name, const TYPE *mat) {
            float matf[4 * 4];
            for (int r = 0; r < 4; r++) {
                for (int c = 0; c < 4; c++) {
                    matf[r * 4 + c] = static_cast<float>(mat[c * 4 + r]);
                }
            }
            const GLint location = glGetUniformLocation(m_program, name);
            glUniformMatrix4fv(location, 1, GL_FALSE, tmatf.ptr);
        }


        //template<typename TYPE>
        //void setVertex3f(const int id, const TYPE *vtxs, const int size) {
        //    if (m_buffer[id] == 0) {
        //        glGenBuffers(1, &m_buffer[id]);
        //    }

        //    glBindBuffer(GL_ARRAY_BUFFER, m_buffer[id]);
        //    glBufferData(GL_ARRAY_BUFFER, sizeof(TYPE) * size, vtxs, GL_STATIC_DRAW);

        //    glEnableVertexAttribArray(id);
        //    glBindBuffer(GL_ARRAY_BUFFER, m_buffer[id]);
        //    glVertexAttribPointer(id, 3, GL_FLOAT, GL_FALSE, 0, NULL);
        //}

        //void setVertex(const int id, const Vec2 *vtxs, const int size) {
        //    GLuint buffer;
        //    glGenBuffers(1, &buffer);
        //    glBindBuffer(GL_ARRAY_BUFFER, buffer);
        //    glBufferData(GL_ARRAY_BUFFER, sizeof(Vec2) * size, vtxs, GL_STATIC_DRAW);

        //    glEnableVertexAttribArray(id);
        //    glBindBuffer(GL_ARRAY_BUFFER, buffer);
        //    glVertexAttribPointer(id, 2, GL_DOUBLE, GL_FALSE, 0, NULL);

        //    m_buffer.push(buffer);
        //}


    private:


        GLuint getProgram(GLuint *pVertShader, GLuint *pFragShader, GLuint *pGeomShader) {
            GLuint program = glCreateProgram();

            if (pVertShader != NULL) glAttachShader(program, *pVertShader);
            if (pFragShader != NULL) glAttachShader(program, *pFragShader);
            if (pGeomShader != NULL) glAttachShader(program, *pGeomShader);

            glLinkProgram(program);

            GLint result, length;
            glGetProgramiv(program, GL_LINK_STATUS, &result);
            glGetProgramiv(program, GL_INFO_LOG_LENGTH, &length);
            if (length > 0) {
                char *info = new char[length + 1];
                glGetProgramInfoLog(program, length, NULL, info);
                //printf("%s\n", info);
                delete[]info;
            }

            return program;
        }

        GLuint getShader(const char *code, const int type) {
            const GLuint shader = glCreateShader(type);

            glShaderSource(shader, 1, &code, NULL);
            glCompileShader(shader);

            GLint result, length;
            glGetShaderiv(shader, GL_COMPILE_STATUS, &result);
            glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);
            if (length > 0) {
                char *info = new char[length + 1];
                glGetShaderInfoLog(shader, length, NULL, info);
                //printf("%s\n", info);
                delete[]info;
            }

            return shader;
        }

    };
}

#endif