//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_GLEW_H__
#define __SP_GLEW_H__

#ifndef SP_USE_GLEW
#define SP_USE_GLEW 1
#endif

#if defined(_WIN32) && SP_USE_GLEW
#define GLEW_STATIC
#include "GL/glew.h"
#endif

#include "spcore/spcore.h"
#include "GLFW/glfw3.h"

namespace sp{

    class FrameBuffer {

    private:
        int m_dsize[2];

        GLuint m_fbId;
        GLuint m_pixId[2];
        
        GLint backup[4];

    public:
        FrameBuffer() {
            m_fbId = 0;
            m_pixId[0] = 0;
            m_pixId[1] = 0;
        }

        ~FrameBuffer() {
            free();
        }

        void free() {
            if (m_fbId) {
                glDeleteFramebuffersEXT(1, &m_fbId);
                m_fbId = 0;
            }
            for (int i = 0; i < 2; i++) {
                if (m_pixId[i]) {
                    glDeleteTextures(1, &m_pixId[i]);
                    m_pixId[i] = 0;
                }
            }
        }

        void resize(const int dsize0, const int dsize1) {
            free();

            m_dsize[0] = dsize0;
            m_dsize[1] = dsize1;

            glGenFramebuffersEXT(1, &m_fbId);
            glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, m_fbId);

            {
                glGenTextures(1, &m_pixId[0]);
                glBindTexture(GL_TEXTURE_2D, m_pixId[0]);

                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, dsize0, dsize1, 0, GL_RGB, GL_UNSIGNED_BYTE, 0);

                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

                glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, m_pixId[0], 0);

                glBindTexture(GL_TEXTURE_2D, 0);
            }
            {
                glGenTextures(1, &m_pixId[1]);
                glBindTexture(GL_TEXTURE_2D, m_pixId[1]);

                glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, dsize0, dsize1, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);

                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);

                glTexParameteri(GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE, GL_LUMINANCE);

                glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_TEXTURE_2D, m_pixId[1], 0);

                glBindTexture(GL_TEXTURE_2D, 0);
            }

            glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
        }

        void resize(const int dsize[2]) {
            resize(dsize[0], dsize[1]);
        }

        void bind() {
            glGetIntegerv(GL_VIEWPORT, backup);

            glViewport(0, 0, m_dsize[0], m_dsize[1]);

            glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, m_fbId);

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

            glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, m_fbId);
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

            glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, m_fbId);
            glFlush();

            Mem2<float> tmp(m_dsize);
            glReadPixels(0, 0, tmp.dsize[0], tmp.dsize[1], GL_DEPTH_COMPONENT, GL_FLOAT, tmp.ptr);

            for (int v = 0; v < tmp.dsize[1]; v++) {
                for (int u = 0; u < tmp.dsize[0]; u++) {
                    const float d = tmp(u, tmp.dsize[1] - 1 - v);
                    depth(u, v) = -farPlane * nearPlane / (d * (farPlane - nearPlane) - farPlane);
                }
            }

            glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
        }
    };

    class Shader {
    private:
        GLuint m_program;
        GLuint m_vertexArray;
        Mem1<GLuint> m_buffer;

    public:
        Shader() {
            m_program = 0;
            m_vertexArray = 0;
        }

        ~Shader() {
            free();
        }

        void load(const char *vert, const char *frag) {
            free();

            GLuint vertShader = getShader(vert, GL_VERTEX_SHADER);
            GLuint fragShader = getShader(frag, GL_FRAGMENT_SHADER);

            m_program = getProgram(vertShader, fragShader);

            glDeleteShader(vertShader);
            glDeleteShader(fragShader);

            glGenVertexArrays(1, &m_vertexArray);
            glBindVertexArray(m_vertexArray);
        }

        void enable() {

            glUseProgram(m_program);

            Mat proj(4, 4);
            Mat view(4, 4);
            
            glGetDoublev(GL_PROJECTION_MATRIX, proj.ptr);
            glGetDoublev(GL_MODELVIEW_MATRIX, view.ptr);

            proj = trnMat(proj);
            view = trnMat(view);

            const Mat tmat = trnMat(proj * view);

            Mem2<float> tmatf(4, 4);
            cnvMem(tmatf, tmat);

            const GLint location = glGetUniformLocation(m_program, "MVP");
            glUniformMatrix4fv(location, 1, GL_FALSE, tmatf.ptr);
        }

        void setVertex(const Vec3 *pVec, const int size) {
            GLuint buffer;
            glGenBuffers(1, &buffer);
            glBindBuffer(GL_ARRAY_BUFFER, buffer);
            glBufferData(GL_ARRAY_BUFFER, sizeof(Vec3) * size, pVec, GL_STATIC_DRAW);

            const GLuint id = static_cast<int>(m_buffer.size());
            glEnableVertexAttribArray(id);
            glBindBuffer(GL_ARRAY_BUFFER, buffer);
            glVertexAttribPointer(id, 3, GL_DOUBLE, GL_FALSE, 0, NULL);

            m_buffer.push(buffer);
        }

        void disable() {
            glUseProgram(0);

            for (int i = 0; i < static_cast<int>(m_buffer.size()); i++) {
                glDisableVertexAttribArray(i);
                glDeleteBuffers(1, &m_buffer[i]);
            }
            m_buffer.clear();
        }

    private:

        void free() {
            if (m_program) {
                glDeleteProgram(m_program);
                m_program = 0;
            }
            if (m_vertexArray) {
                glDeleteVertexArrays(1, &m_vertexArray);
                m_vertexArray = 0;
            }
        }

        GLuint getProgram(GLuint vertShader, GLuint fragShader) {
            GLuint program = glCreateProgram();

            glAttachShader(program, vertShader);
            glAttachShader(program, fragShader);
            glLinkProgram(program);

            GLint result, length;
            glGetProgramiv(program, GL_LINK_STATUS, &result);
            glGetProgramiv(program, GL_INFO_LOG_LENGTH, &length);
            if (length > 0) {
                char *info = new char[length];
                glGetProgramInfoLog(program, length, NULL, info);
                printf("%s\n", info);
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
                char *info = new char[length];
                glGetShaderInfoLog(shader, length, NULL, info);
                printf("%s\n", info);
                delete[]info;
            }

            return shader;
        }

    };
}
#endif