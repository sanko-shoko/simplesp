﻿#define SP_USE_GLEW 1

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

    FrameBuffer m_fb;


private:

    void help() {
        printf("'a' key : render image\n");
        printf("'s' key : render depth\n");
        printf("'ESC' key : exit\n");
        printf("\n");
    }

    virtual void init() {

        help();

        m_cam = getCamParam(640, 480);

        m_img.resize(m_cam.dsize);
        m_img.zero();

        m_fb.resize(m_cam.dsize);

        if (loadBunny(m_model, SP_DATA_DIR "/stanford/bun_zipper.ply") == false) {

            // if could not find stanford bunny, load dummy model
            loadGeodesicDorm(m_model, 100.0, 1);
        }

        m_pose = getPose(getVec(0.0, 0.0, getModelDistance(m_model, m_cam)));

    }

    virtual void keyFun(int key, int scancode, int action, int mods) {

        const double distance = getModelDistance(m_model, m_cam);
        const double radius = getModelRadius(m_model);

        m_fb.bind();
        display();
        m_fb.unbind();

        if (m_keyAction[GLFW_KEY_A] == 1) {
            m_fb.readImg(m_img);
        }
        if (m_keyAction[GLFW_KEY_S] == 1) {

            Mem2<double> depth;
            m_fb.readDepth(depth);
            cnvDepthToImg(m_img, depth, distance - 2 * radius, distance + 2 * radius);
        }
    }
    void test() {
        //static File vert("shader.vert", "rb");
        //static File frag("shader.frag", "rb");
        //static File geom("shader.geom", "rb");

        //static Shader shader;
        //if (shader.isValid() == false) {
        //    shader.load(vert.getText().c_str(), frag.getText().c_str(), geom.getText().c_str());
        //}
        
        static File vert("showdepth.vert", "rb");
        static File frag("differential.frag", "rb");
        static Shader shader;
        if (shader.isValid() == false) {
            shader.load(getText(vert).c_str(), getText(frag).c_str());
        }

        glLoadView3D(m_cam, m_viewPos, m_viewScale);
        glLoadMatrix(m_pose);


        shader.enable();
        shader.setVertex((Vec3*)m_model.ptr, m_model.size() * 3);

        glDrawArrays(GL_TRIANGLES, 0, m_model.size() * 3);

        shader.disable();

    }

    void shader00() {
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
        if (shader.isValid() == false) {
            shader.load(vert, flag);
        }

        glLoadView3D(m_cam, m_viewPos, m_viewScale);
        glLoadMatrix(m_pose);

        shader.enable();
        shader.setVertex((Vec3*)m_model.ptr, m_model.size() * 3);

        glDrawArrays(GL_TRIANGLES, 0, m_model.size() * 3);

        shader.disable();
    }

    void shader01() {
        const char *vert =
            "#version 400 core\n"
            "layout(location = 0) in vec3 vertexPosition_modelspace;"

            "void main(){"
            "gl_Position.xyz = vertexPosition_modelspace;"
            "gl_Position.w = 1.0;"
            "}"
            ;

        const char *flag =
            "#version 400 core\n"
            "out vec3 color;"
            "void main()"
            "{"
            "color = vec3(1,0,0);"
            "}"
            ;

        static Shader shader;
        if (shader.isValid() == false) {
           shader.load(vert, flag);
        }

        static const double g_vertex_buffer_data[] = {
            -1.0f, -1.0f, 0.0f,
            1.0f, -1.0f, 0.0f,
            0.0f,  1.0f, 0.0f,
        };
        shader.enable();
        shader.setVertex((Vec3*)g_vertex_buffer_data, 3);

        glDrawArrays(GL_TRIANGLES, 0, 3); // 3 indices starting at 0 -> 1 triangle


        shader.disable();
    }
    void shader02() {
        const char *vert =
            "#version 400 core\n"
            "layout(location = 0) in vec3 vtx;"
            "layout(location = 1) in vec3 ccc;"
            "out vec3 fcol;"
            "uniform mat4 mat;"

            "void main(){"
            "gl_Position = mat * vec4(vtx, 1.0);"
            "fcol = ccc;"
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
        if (shader.isValid() == false) {
            shader.load(vert, flag);
        }

        glLoadView3D(m_cam, m_viewPos, m_viewScale);
        glLoadMatrix(m_pose);

        shader.enable();

        Mem1<Vec3> ccc;
        for (int i = 0; i < m_model.size(); i++) {
            ccc.push(getVec(getCol(i + 0)) * (1.0 / 255));
            ccc.push(getVec(getCol(i + 1)) * (1.0 / 255));
            ccc.push(getVec(getCol(i + 2)) * (1.0 / 255));
        }

        shader.setVertex((Vec3*)m_model.ptr, m_model.size() * 3);
        shader.setVertex((Vec3*)ccc.ptr, ccc.size());

        glDrawArrays(GL_TRIANGLES, 0, m_model.size() * 3);

        shader.disable();
    }
    virtual void display() {
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(-1.0, 1.0, -1.0, 0.0, 1.0, -1.0);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        {
            // view 2D
            glLoadView2D(m_cam, m_viewPos + getVec(199, 199), m_viewScale);
            glRenderImg(m_img);
        }
        shader02();
        return;

        {
            // view 3D
            glLoadView3D(m_cam, m_viewPos, m_viewScale);

            renderSurface();
            renderAxis();
        }
    }
    void renderSurface() {
        glLoadMatrix(m_pose);

        const GLfloat diffuse[] = { 0.4f, 0.5f, 0.5f, 1.0f };
        glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse);

        glRenderSurface(m_model);
    }

    void renderAxis() {
        glDisable(GL_DEPTH_TEST);

        glLoadMatrix(m_pose);

        glLineWidth(2.f);
        glBegin(GL_LINES);
        glAxis(100.0);
        glEnd();
        
        glEnable(GL_DEPTH_TEST);
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