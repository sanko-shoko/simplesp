//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_GLEW_H__
#define __SP_GLEW_H__

#define GLEW_STATIC
#include "GL/glew.h"

#include "spcore/spcore.h"
#include "GLFW/glfw3.h"

namespace sp{

    class FrameBufferObject {

    public:
        int m_dsize[2];

        GLuint m_fb;
        GLuint m_tex[2];
        
        // view port backup
        GLint backup[4];

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
                glDeleteFramebuffersEXT(1, &m_fb);
                m_fb = 0;
            }
            for (int i = 0; i < 2; i++) {
                if (m_tex[i]) {
                    glDeleteTextures(1, &m_tex[i]);
                    m_tex[i] = 0;
                }
            }
        }

        void resize(const int dsize[2]) {
            if (cmpSize(2, dsize, m_dsize) == true) return;

            free();

            m_dsize[0] = dsize[0];
            m_dsize[1] = dsize[1];

            glGenFramebuffersEXT(1, &m_fb);
            glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, m_fb);

            {
                glGenTextures(1, &m_tex[0]);
                glBindTexture(GL_TEXTURE_2D, m_tex[0]);

                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, dsize[0], dsize[1], 0, GL_RGB, GL_UNSIGNED_BYTE, 0);

                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

                glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, m_tex[0], 0);
            }
            {
                glGenTextures(1, &m_tex[1]);
                glBindTexture(GL_TEXTURE_2D, m_tex[1]);

                glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, dsize[0], dsize[1], 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);

                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

                //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE);
                //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
                //glTexParameteri(GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE, GL_LUMINANCE);

                glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_TEXTURE_2D, m_tex[1], 0);
            }

            glBindTexture(GL_TEXTURE_2D, 0);
            glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
        }

        void resize(const int dsize0, const int dsize1) {
            int dsize[2] = { dsize0, dsize1 };
            resize(dsize);
        }

        void bind() {
            glGetIntegerv(GL_VIEWPORT, backup);

            glViewport(0, 0, m_dsize[0], m_dsize[1]);

            glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, m_fb);

            const GLenum buffer = GL_COLOR_ATTACHMENT0_EXT;
            glDrawBuffers(1, &buffer);

            glClearColor(0.0, 0.0, 0.0, 1.0);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        }

        void unbind() {
            glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
            glViewport(backup[0], backup[1], backup[2], backup[3]);
        }

        void readImg(Mem2<Col3> &img) {
            img.resize(m_dsize);

            glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, m_fb);
            glFlush();

            Mem2<Col3> tmp(m_dsize);
            glReadPixels(0, 0, tmp.dsize[0], tmp.dsize[1], GL_RGB, GL_UNSIGNED_BYTE, tmp.ptr);

            for (int v = 0; v < tmp.dsize[1]; v++) {
                memcpy(&img(0, v), &tmp(0, tmp.dsize[1] - 1 - v), tmp.dsize[0] * 3);
            }

            glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
        }

        void readDepth(Mem2<double> &depth, const double nearPlane = 1.0, const double farPlane = 10000.0) {
            depth.resize(m_dsize);

            glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, m_fb);
            glFlush();

            Mem2<float> tmp(m_dsize);
            glReadPixels(0, 0, tmp.dsize[0], tmp.dsize[1], GL_DEPTH_COMPONENT, GL_FLOAT, tmp.ptr);

            for (int v = 0; v < tmp.dsize[1]; v++) {
                for (int u = 0; u < tmp.dsize[0]; u++) {
                    const float d = tmp(u, tmp.dsize[1] - 1 - v);
                    depth(u, v) = farPlane * nearPlane / (farPlane - d * (farPlane - nearPlane));
                }
            }

            glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
        }
    };

    class Shader {
    private:
        GLuint m_program;
        Mem1<GLuint> m_buffer;

    public:
        Shader() {
            m_program = 0;
        }

        ~Shader() {
            free();
        }

        bool valid() {
            return m_program != 0 ? true : false;
        }

        bool load(File &vert, File &frag, File &geom = File()) {
            File *flies[3] = { &vert, &frag, &geom };
            string texts[3];

            for (int i = 0; i < 3; i++) {
                File &file = *flies[i];
                string &text = texts[i];
                text = "";
                if (file.valid() == false) continue;

                char str[SP_STRMAX];

                const int pos = file.tell();
                file.seek(0);
                while (file.gets(str)) {
                    text += str;
                }
                file.seek(pos);
            }
            return load(texts[0].c_str(), texts[1].c_str(), texts[2].c_str());
        }
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

            for (int i = 0; i < static_cast<int>(m_buffer.size()); i++) {
                glDisableVertexAttribArray(i);
                glDeleteBuffers(1, &m_buffer[i]);
            }
            m_buffer.clear();
        }

        void setUniform(const char *name, const int &val) {
            const GLint location = glGetUniformLocation(m_program, name);
            glUniform1i(location, val);
        }
        
        void setUniform(const char *name, const double &val) {
            const GLint location = glGetUniformLocation(m_program, name);
            glUniform1f(location, static_cast<float>(val));
        }
        
        void setUniform(const char *name, const Vec2 &vec) {
            const GLint location = glGetUniformLocation(m_program, name);
            glUniform2f(location, static_cast<float>(vec.x), static_cast<float>(vec.y));
        }

        void setUniform(const char *name, const Vec3 &vec) {
            const GLint location = glGetUniformLocation(m_program, name);
            glUniform3f(location, static_cast<float>(vec.x), static_cast<float>(vec.y), static_cast<float>(vec.z));
        }

        void setUniform(const char *name, const Mat &mat) {
            SP_ASSERT(mat.rows() == mat.cols());

            const Mat tmat = trnMat(mat);

            Mem2<float> tmatf(tmat.dsize);
            cnvMem(tmatf, tmat);

            const GLint location = glGetUniformLocation(m_program, name);

            if (mat.rows() == 3) {
                glUniformMatrix3fv(location, 1, GL_FALSE, tmatf.ptr);
            }
            if (mat.rows() == 4) {
                glUniformMatrix4fv(location, 1, GL_FALSE, tmatf.ptr);
            }
        }

        void setVertex(const int id, const Vec3 *vtxs, const int size) {
            GLuint buffer;
            glGenBuffers(1, &buffer);
            glBindBuffer(GL_ARRAY_BUFFER, buffer);
            glBufferData(GL_ARRAY_BUFFER, sizeof(Vec3) * size, vtxs, GL_STATIC_DRAW);

            glEnableVertexAttribArray(id);
            glBindBuffer(GL_ARRAY_BUFFER, buffer);
            glVertexAttribPointer(id, 3, GL_DOUBLE, GL_FALSE, 0, NULL);

            m_buffer.push(buffer);
        }

        void setVertex(const int id, const Vec2 *vtxs, const int size) {
            GLuint buffer;
            glGenBuffers(1, &buffer);
            glBindBuffer(GL_ARRAY_BUFFER, buffer);
            glBufferData(GL_ARRAY_BUFFER, sizeof(Vec2) * size, vtxs, GL_STATIC_DRAW);

            glEnableVertexAttribArray(id);
            glBindBuffer(GL_ARRAY_BUFFER, buffer);
            glVertexAttribPointer(id, 2, GL_DOUBLE, GL_FALSE, 0, NULL);

            m_buffer.push(buffer);
        }


    private:

        void free() {
            if (m_program) {
                glDeleteProgram(m_program);
                m_program = 0;
            }
        }

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
                Mem1<char> info(length + 1);
                glGetProgramInfoLog(program, length, NULL, info.ptr);
                SP_PRINTF("%s\n", info.ptr);
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
                Mem1<char> info(length + 1);
                glGetShaderInfoLog(shader, length, NULL, info.ptr);
                SP_PRINTF("%s\n", info.ptr);
            }

            return shader;
        }

    };
}

#endif