//--------------------------------------------------------------------------------
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
        GLuint m_vb;

    private:
        VertexBufferObject(const VertexBufferObject &vbo) {}
        VertexBufferObject& operator =(const VertexBufferObject &vbo) {}

    public:

        VertexBufferObject() {
            glGenBuffers(1, &m_vb);
        }

        ~VertexBufferObject() {
            glDeleteBuffers(1, &m_vb);
        }

        void set(const int size, const void *vtx) {
            bind();
            glBufferData(GL_ARRAY_BUFFER, size, vtx, GL_STATIC_DRAW);
            unbind();
        }

        void bind() {
            glBindBuffer(GL_ARRAY_BUFFER, m_vb);
        }
        void unbind() {
            glBindBuffer(GL_ARRAY_BUFFER, 0);
        }

        GLuint vbid() const {
            return m_vb;
        }
    };


    class FrameBufferObject {

    private:
        GLuint m_msfb;
        GLuint m_mstx[2];

        GLuint m_fb;
        GLuint m_tx[2];

    public:
        int samples;
        int dsize[2];
       
    private:
        void reset() {
            memset(this, 0, sizeof(FrameBufferObject));
        }

    public:
        FrameBufferObject() {
            reset();
        }

        FrameBufferObject(const int *dsize, const int samples = 1) {
            reset();
            resize(dsize, samples);
        }

        FrameBufferObject(const int dsize0, const int dsize1, const int samples = 1) {
            reset();
            resize(dsize0, dsize1, samples);
        }

        ~FrameBufferObject() {
            free();
        }

        void free() {
            {
                if (m_fb) {
                    glDeleteFramebuffers(1, &m_fb);
                }
                if (m_msfb) {
                    glDeleteFramebuffers(1, &m_msfb);
                }
            }
            for (int i = 0; i < 2; i++) {
                if (m_tx[i]) {
                    glDeleteTextures(1, &m_tx[i]);
                }
                if (m_mstx[i]) {
                    glDeleteTextures(1, &m_mstx[i]);
                }
            }
            reset();
        }

        void resize(const int dsize[2], const int samples = 1) {
            if (this->dsize[0] == dsize[0] && this->dsize[1] == dsize[1] && this->samples == samples) return;

            free();
            this->dsize[0] = dsize[0];
            this->dsize[1] = dsize[1];
            this->samples = samples;

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
            if (samples > 1) {
                glGenFramebuffers(1, &m_msfb);
                glGenTextures(2, m_mstx);

                // color
                glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, m_mstx[0]);

                glPixelStorei(GL_PACK_ALIGNMENT, 1);
                glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, samples, GL_RGBA, dsize[0], dsize[1], false);

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

                glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, samples, GL_DEPTH_COMPONENT, dsize[0], dsize[1], false);
            }
            glBindTexture(GL_TEXTURE_2D, 0);
        }

        void resize(const int dsize0, const int dsize1, const int samples = 1) {
            int dsize[2] = { dsize0, dsize1 };
            resize(dsize, samples);
        }

        void bind() {
            if (dsize[0] == 0 || dsize[1] == 0)return;
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

            //glClearColor(0.0, 0.0, 0.0, 1.0);
            //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        }

        void unbind() {
            if (dsize[0] == 0 || dsize[1] == 0)return;
            if (m_msfb != 0) {
                glBindFramebuffer(GL_READ_FRAMEBUFFER, m_msfb);
                glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE, m_mstx[0], 0);
                glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D_MULTISAMPLE, m_mstx[1], 0);

                glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_fb);
                glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_tx[0], 0);
                glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, m_tx[1], 0);

                glBlitFramebuffer(0, 0, dsize[0], dsize[1], 0, 0, dsize[0], dsize[1], GL_COLOR_BUFFER_BIT, GL_NEAREST);
                glBlitFramebuffer(0, 0, dsize[0], dsize[1], 0, 0, dsize[0], dsize[1], GL_DEPTH_BUFFER_BIT, GL_NEAREST);
            }
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
            glPopAttrib();
        }

        void readi(void *img, const int ch) {
            if (dsize[0] == 0 || dsize[1] == 0) return;

            glBindFramebuffer(GL_FRAMEBUFFER, m_fb);

            unsigned char *tmp = new unsigned char[dsize[0] * dsize[1] * 4];
            glReadPixels(0, 0, dsize[0], dsize[1], GL_RGBA, GL_UNSIGNED_BYTE, tmp);

            unsigned char *dst = (unsigned char *)img;

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

        GLuint fbid() const {
            return m_fb;
        }

        GLuint txid(const int i) const {
            return m_tx[i];
        }

    };

    class Shader {

    private:
        GLuint m_pgid;

    private:
        void reset() {
            m_pgid = 0;
        }

        void free() {
            if (m_pgid) {
                glDeleteProgram(m_pgid);
            }
            reset();
        }

    private:

        GLuint compile(const GLuint pgid, const int type, const char *code, char *log = NULL) {
            const GLuint shader = glCreateShader(type);

            glShaderSource(shader, 1, &code, NULL);
            glCompileShader(shader);

            GLint result;
            glGetShaderiv(shader, GL_COMPILE_STATUS, &result);

            GLint length;
            glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);

            if (length > 0 && log != NULL) {
                const int offset = strlen(log);
                glGetShaderInfoLog(shader, length, NULL, &log[offset]);
            }

            if (result == GL_TRUE) {
                return shader;
            }
            else {
                glDeleteShader(shader);
                return 0;
            }
        }

        GLuint link(const GLuint vertShader, const GLuint fragShader, const GLuint geomShader, char *log = NULL) {
            const GLuint pgid = glCreateProgram();
            
            if (vertShader != 0) glAttachShader(pgid, vertShader);
            if (fragShader != 0) glAttachShader(pgid, fragShader);
            if (geomShader != 0) glAttachShader(pgid, geomShader);

            glLinkProgram(pgid);

            GLint result;
            glGetProgramiv(pgid, GL_LINK_STATUS, &result);

            GLint length;
            glGetProgramiv(pgid, GL_INFO_LOG_LENGTH, &length);

            if (length > 0 && log != NULL) {
                const int offset = strlen(log);
                glGetProgramInfoLog(pgid, length, NULL, &log[offset]);
            }

            if (result == GL_TRUE) {
                return pgid;
            }
            else {
                glDeleteProgram(pgid);
                return 0;
            }
        }

    public:
        Shader() {
            reset();
        }

        ~Shader() {
            free();
        }

        bool pgid() {
            return m_pgid;
        }

        bool load(const char *vertCode, const char *fragCode, const char *geomCode, char *log = NULL) {
            free();

            GLuint vertShader = 0;
            GLuint fragShader = 0;
            GLuint geomShader = 0;

            if (vertCode != NULL && vertCode[0] != '\0') {
                vertShader = compile(m_pgid, GL_VERTEX_SHADER  , vertCode, log);
            }
            if (fragCode != NULL && fragCode[0] != '\0') {
                fragShader = compile(m_pgid, GL_FRAGMENT_SHADER, fragCode, log);
            }
            if (geomCode != NULL && geomCode[0] != '\0') {
                geomShader = compile(m_pgid, GL_GEOMETRY_SHADER, geomCode, log);
            }

            m_pgid = link(vertShader, fragShader, geomShader, log);

            if (vertShader != 0) {
                glDeleteShader(vertShader);
            }
            if (fragShader != 0) {
                glDeleteShader(fragShader);
            }
            if (geomShader != 0) {
                glDeleteShader(geomShader);
            }

            return (m_pgid != 0) ? true : false;
        }

        void enable() {
            glUseProgram(m_pgid);


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
            glActiveTexture(GL_TEXTURE0);
            glUseProgram(0);
        }

        void setUniformTx(const char *name, const int index, const GLuint txid) {
            switch (index) {
            case 0: glActiveTexture(GL_TEXTURE0); break;
            case 1: glActiveTexture(GL_TEXTURE1); break;
            case 2: glActiveTexture(GL_TEXTURE2); break;
            case 3: glActiveTexture(GL_TEXTURE3); break;
            case 4: glActiveTexture(GL_TEXTURE4); break;
            default: return;
            }

            glBindTexture(GL_TEXTURE_2D, txid);
            const GLint location = glGetUniformLocation(m_pgid, name);
            glUniform1i(location, index);
        }

        template<typename TYPE>
        void setUniform1i(const char *name, const TYPE val) {
            const GLint location = glGetUniformLocation(m_pgid, name);
            glUniform1i(location, static_cast<int>(val));
        }

        template<typename TYPE>
        void setUniform1f(const char *name, const TYPE val) {
            const GLint location = glGetUniformLocation(m_pgid, name);
            glUniform1f(location, static_cast<float>(val));
        }

        template<typename TYPE>
        void setUniform2f(const char *name, const TYPE *vec) {
            const GLint location = glGetUniformLocation(m_pgid, name);
            glUniform2f(location, static_cast<float>(vec[0]), static_cast<float>(vec[1]));
        }
        template<typename TYPE>
        void setUniform3f(const char *name, const TYPE *vec) {
            const GLint location = glGetUniformLocation(m_pgid, name);
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

            const GLint location = glGetUniformLocation(m_pgid, name);
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
            const GLint location = glGetUniformLocation(m_pgid, name);
            glUniformMatrix4fv(location, 1, GL_FALSE, tmatf.ptr);
        }

        void setVertex(const int id, const int unit, const int type, const VertexBufferObject &vbo) {
            glEnableVertexAttribArray(id);
            glBindBuffer(GL_ARRAY_BUFFER, vbo.vbid());
            glVertexAttribPointer(id, unit, type, GL_FALSE, 0, NULL);
        }

    };
}

#endif