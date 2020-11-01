#define SP_USE_GLEW 1
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

    VertexBufferObject m_vbo;


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

        //m_vbo.set(m_model.ptr, sizeof(Mesh3) * m_model.size());

        static float vtx[] = { -0.5f, -0.5f, 0.5f, -0.5f, 0.5f, 0.5f, -0.5f, 0.5f };
        m_vbo.set(vtx, 8 * 4);

        static const GLfloat g_vertex_buffer_data[] = {
            -1.0f, -1.0f, 0.0f,
            1.0f, -1.0f, 0.0f,
            0.0f,  1.0f, 0.0f,
        };
        glGenVertexArrays(1, &VertexArrayID);
        glBindVertexArray(VertexArrayID);

        glGenBuffers(1, &vertexbuffer);
        glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);

    }
    GLuint VertexArrayID;
    GLuint vertexbuffer;
    virtual void terminate() {
        glDeleteBuffers(1, &vertexbuffer);
        glDeleteVertexArrays(1, &VertexArrayID);
        //glDeleteProgram(programID);
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
            "#version 330 core\n"
            "layout(location = 0) in vec3 position;"
            "void main(){"
            "gl_Position = vec4(position, 1.0);"
            "}"
            ;

        const char *flag =
            "#version 330 core\n"
            "out vec4 color;"
            "void main()"
            "{"
            "color = vec4(0.5, 1.0, 1.0, 1.0);"
            "}"
            ;
        glColor3d(1.0, 1.0, 1.0);
        glLineWidth(3.0f);
        {
            glColor3d(1.0, 1.0, 1.0);
            glLineWidth(3.0f);

            glBegin(GL_LINES);
            glVertex2d(0.0, 0.0);
            glVertex2d(1.0, 1.0);
            glEnd();
        }

        static Shader shader;
        if (!shader.pgid()) {
            char log[512] = { 0 };
            shader.load(vert, flag, NULL, log);
            printf("%s\n", log);
        }
        //glEnableClientState(GL_VERTEX_ARRAY);

        //glLoadView3D(m_cam, m_viewPos, m_viewScale);
        //glLoadMatrix(m_pose);
        {
            glUseProgram(shader.pgid());

            glEnableVertexAttribArray(0);
            glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
            glVertexAttribPointer(
                0,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
                3,                  // size
                GL_FLOAT,           // type
                GL_FALSE,           // normalized?
                0,                  // stride
                (void*)0            // array buffer offset
            );

            // Draw the triangle !
            glDrawArrays(GL_TRIANGLES, 0, 3); // 3 indices starting at 0 -> 1 triangle

            glDisableVertexAttribArray(0);
        }

        //shader.enable();
        //shader.setVertex(0, 2, GL_FLOAT, m_vbo);
        ////m_vbo.bind();
        ////glVertexPointer(2, GL_FLOAT, 0, NULL);
        ////m_vbo.unbind();
        //glDrawArrays(GL_LINE_LOOP, 0, 4);
        ////glDisableClientState(GL_VERTEX_ARRAY);
        //glDisableVertexAttribArray(0);

        shader.disable();
    }

    virtual void display() {
        glClearColor(0.10f, 0.12f, 0.12f, 0.00f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        simple();

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