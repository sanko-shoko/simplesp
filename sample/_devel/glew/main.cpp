#define SP_USE_GLEW 1

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

    Shader m_shader;

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

    void shader00() {
        static bool onece = true;
        if (onece) {
            onece = false;
            const char *basic_vert =
                "#version 400 core\n"
                "layout(location = 0) in vec3 vertexPosition_modelspace;"
                "out vec3 fragmentColor;"
                "uniform mat4 MVP;"

                "void main(){"
                "gl_Position = MVP * vec4(vertexPosition_modelspace, 1.0);"
                "fragmentColor = vec3(0.5, 1.0, 1.0);"
                "}"
                ;

            const char *basic_flag =
                "#version 400 core\n"
                "in vec3 fragmentColor;"
                "out vec3 color;"
                "void main()"
                "{"
                "color = fragmentColor;"
                "}"
                ;
            m_shader.load(basic_vert, basic_flag);
        }

        glLoadView3D(m_cam, m_viewPos, m_viewScale);
        glLoadMatrix(m_pose);

        m_shader.enable();
        m_shader.setVertex((Vec3*)m_model.ptr, m_model.size() * 3);

        glDrawArrays(GL_TRIANGLES, 0, m_model.size() * 3);

        m_shader.disable();
    }

    void shader01() {

        static bool onece = true;
        if (onece) {
            onece = false;
            const char *basic_vert =
                "#version 400 core\n"
                "layout(location = 0) in vec3 vertexPosition_modelspace;"

                "void main(){"
                "gl_Position.xyz = vertexPosition_modelspace;"
                "gl_Position.w = 1.0;"
                "}"
                ;

            const char *basic_flag =
                "#version 400 core\n"
                "out vec3 color;"
                "void main()"
                "{"
                "color = vec3(1,0,0);"
                "}"
                ;
            m_shader.load(basic_vert, basic_flag);

        }

        static const double g_vertex_buffer_data[] = {
            -1.0f, -1.0f, 0.0f,
            1.0f, -1.0f, 0.0f,
            0.0f,  1.0f, 0.0f,
        };
        m_shader.enable();
        m_shader.setVertex((Vec3*)g_vertex_buffer_data, 3);


        // Draw the triangle !
        glDrawArrays(GL_TRIANGLES, 0, 3); // 3 indices starting at 0 -> 1 triangle


        m_shader.disable();
    }

    virtual void display() {
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(-1.0, 1.0, -1.0, 0.0, 1.0, -1.0);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        shader01();
        return;
        {
            // view 2D
            glLoadView2D(m_cam, m_viewPos, m_viewScale);
            glRenderImg(m_img);
        }

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