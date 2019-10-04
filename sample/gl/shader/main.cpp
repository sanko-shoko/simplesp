﻿#define SP_USE_GLEW 1
#define SP_USE_DEBUG 1

#include "simplesp.h"
#include "spex/spgl.h"

using namespace sp;

class RenderGUI : public BaseWindow {

    // camera
    CamParam m_cam;

    // image
    Mem2<Col3> m_img;

    // object mesh model
    Mem1<Mesh3> m_model;

    // object to cam pose
    Pose m_pose;


private:

    void help() {
        printf("\n");
    }

    virtual void init() {

        help();

        m_cam = getCamParam(640, 480);

        m_img.resize(m_cam.dsize);
        m_img.zero();

        m_model = loadBunny(SP_DATA_DIR "/stanford/bun_zipper.ply");
        if (m_model.size() == 0) {
            // if could not find stanford bunny, load dummy model
            m_model = loadGeodesicDorm(100.0, 1);
        }

        m_pose = getPose(getVec3(0.0, 0.0, getModelDistance(m_model, m_cam)));

    }

    void edge() {
        static Shader shader;
        static FrameBufferObject fbo;
        static bool once = true;
        if (once) {
            once = false;

            const char* vert =
                #include "spex/spshader/edge.vert"
                ;

            const char* frag =
                #include "spex/spshader/edge.frag"
                ;

            shader.load(vert, frag, NULL, NULL);
        }

        {
            fbo.resize(m_wcam.dsize);
            fbo.bind();
            glClearColor(0.0, 0.0, 0.0, 1.0);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glLoadView3D(m_wcam, m_viewPos, m_viewScale);

            glLoadMatrix(m_pose);
            glRenderSurface(m_model);

            fbo.unbind();

            {
                if (m_key[GLFW_KEY_S] > 0) {
                    Mem2<Col3> img(m_wcam.dsize);
                    fbo.readi((unsigned char *)img.ptr, 3);

                    saveBMP("test.bmp", img);
                }
            }
        }

        {
            static FrameBufferObject dfbo;
            dfbo.resize(m_wcam.dsize);
            
            shader.enable();
            shader.setUniformTx("depth", 0, dfbo.tx(1));
            shader.setUniform1i("pers", 1);
            shader.setUniform1f("nearPlane", 1.0);
            shader.setUniform1f("farPlane", 10000.0);
            shader.setUniform1f("dx", 1.0 / m_wcam.dsize[0]);
            shader.setUniform1f("dy", 1.0 / m_wcam.dsize[1]);

            glDisable(GL_DEPTH_TEST);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE);

            glBegin(GL_QUADS);
            glVertex2d(-1.0, -1.0);
            glVertex2d(+1.0, -1.0);
            glVertex2d(+1.0, +1.0);
            glVertex2d(-1.0, +1.0);
            glEnd();
            
            glBindTexture(GL_TEXTURE_2D, 0);

            glEnable(GL_DEPTH_TEST);
            glDisable(GL_BLEND);
            shader.disable();
        }
    }

    void edge2() {
        static Shader shader;
        static bool once = true;
        if (once) {
            once = false;

            const char* vert =
#include "spex/spshader/edge.vert"
                ;
            const char* frag =
#include "spex/spshader/edge.frag"
                ;
            char log[1024] = { 0 };
            shader.load(vert, frag, NULL, log);

            if (::strlen(log) > 0) {
                printf("%s\n", log);
            }
        }

        {
            glLoadView3D(m_wcam, m_viewPos, m_viewScale);

            glLoadMatrix(m_pose);
            glRenderSurface(m_model);
        }

        {
            static FrameBufferObject dfbo;
            dfbo.resize(m_wcam.dsize);

            glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
            glBindFramebuffer(GL_DRAW_FRAMEBUFFER, dfbo.id());
            glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, dfbo.tx(0), 0);
            glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, dfbo.tx(1), 0);

            glBlitFramebuffer(0, 0, m_wcam.dsize[0], m_wcam.dsize[1], 0, 0, m_wcam.dsize[0], m_wcam.dsize[1], GL_COLOR_BUFFER_BIT, GL_NEAREST);
            glBlitFramebuffer(0, 0, m_wcam.dsize[0], m_wcam.dsize[1], 0, 0, m_wcam.dsize[0], m_wcam.dsize[1], GL_DEPTH_BUFFER_BIT, GL_NEAREST);
            glBindFramebuffer(GL_FRAMEBUFFER, 0);

            shader.enable();
            shader.setUniformTx("depth", 0, dfbo.tx(1));
            shader.setUniform1i("pers", 1);
            shader.setUniform1f("nearPlane", 1.0);
            shader.setUniform1f("farPlane", 10000.0);
            shader.setUniform1f("dx", 1.0 / m_wcam.dsize[0]);
            shader.setUniform1f("dy", 1.0 / m_wcam.dsize[1]);

            glDisable(GL_DEPTH_TEST);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE);

            glBegin(GL_QUADS);
            glVertex2d(-1.0, -1.0);
            glVertex2d(+1.0, -1.0);
            glVertex2d(+1.0, +1.0);
            glVertex2d(-1.0, +1.0);
            glEnd();

            glBindTexture(GL_TEXTURE_2D, 0);

            glEnable(GL_DEPTH_TEST);
            glDisable(GL_BLEND);
            shader.disable();
        }
    }

    void simple() {
        const char *vert =
            "#version 400 core\n"
            "layout(location = 0) in vec3 vtx;"
            "out vec3 fcol;"
            "uniform mat4 mat;"

            "void main(){"
            "gl_Position = mat * vec4(vtx, 1.0);"
            "fcol = vec3(0.5, 1.0, 1.0);"
            "}"
            ;

        const char *flag =
            "#version 400 core\n"
            "in vec3 fcol;"
            "out vec3 color;"
            "void main()"
            "{"
            "color = fcol;"
            "}"
            ;

        static Shader shader;
        static bool once = true;
        if (once) {
            once = false;
            shader.load(vert, flag, NULL, NULL);
        }

        glLoadView3D(m_cam, m_viewPos, m_viewScale);
        glLoadMatrix(m_pose);

        shader.enable();
        //shader.setUniform("mat", glGetMat(GL_PROJECTION_MATRIX) * glGetMat(GL_MODELVIEW_MATRIX));
        //shader.setVertex(0, (Vec3*)m_model.ptr, m_model.size() * 3);

        glDrawArrays(GL_TRIANGLES, 0, m_model.size() * 3);

        shader.disable();
    }

    virtual void display() {
        glClearColor(0.10f, 0.12f, 0.12f, 0.00f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        edge2();

    }

    virtual void mousePos(double x, double y) {
        controlPose(m_pose, m_mouse, m_wcam, m_viewScale);
    }

    virtual void mouseScroll(double x, double y) {
        controlPose(m_pose, m_mouse, m_wcam, m_viewScale);
    }

};


int main() {

    RenderGUI win;
    win.execute("glew", 800, 600);

    return 0;
}