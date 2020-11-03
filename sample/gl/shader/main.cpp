#define SP_REAL float

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
    Mem1<Vec3> m_nrms;
    Mem1<Vec3> m_vtxs;

    // object to cam pose
    Pose m_pose;

    VertexArrayObject m_vao;
    VertexBufferObject m_vbo;


private:

    void help() {
        printf("\n");
    }
    Shader shader;
    float *vtx;

    virtual void init() {
        static float _vtx[] = { -0.5f, -0.5f, 0.5f, -0.5f, 0.5f, 0.5f, -0.5f, 0.5f };
        vtx = _vtx;

        help();

        m_cam = getCamParam(640, 480);

        m_img.resize(m_cam.dsize);
        m_img.zero();

        m_model = loadBunny(SP_DATA_DIR "/stanford/bun_zipper.ply");
        if (m_model.size() == 0) {
            // if could not find stanford bunny, load dummy model
            m_model = loadGeodesicDorm(100.0, 1);
        }
        for (int i = 0; i < m_model.size(); i++) {
            const Vec3 nrm = getMeshNrm(m_model[i]);
            m_nrms.push(nrm);
            m_nrms.push(nrm);
            m_nrms.push(nrm);
        }

        ::srand(0);
        for (int i = 0; i < 100; i++) {
            const int x = ::rand() % 101 - 50;
            const int y = ::rand() % 101 - 50;
            const int z = ::rand() % 101 - 50;
            m_vtxs.push(getVec3(x, y, z));
        }

        m_pose = getPose(getVec3(0.0, 0.0, getModelDistance(m_model, m_cam)));

        //m_vbo.set(m_model.ptr, sizeof(Mesh3) * m_model.size());

        m_vbo.set(vtx, 8 * 4);
        m_vao.bind();
        m_vao.unbind();

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
            
            shader.enable();
            shader.setUniformTx("depth", 0, fbo.tx(1));
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
        {
            static Shader shader;
            static VertexArrayObject vao;

            if (shader.id() == 0) {
                const char *vert =
                    "#version 330 core\n"
                    "layout(location = 0) in vec3 position;"
                    "layout(location = 1) in vec3 normal;"
                    "out vec3 pos;"
                    "out vec3 nrm;"
                    "uniform mat4 transform;"
                    "void main(){"
                    "gl_Position = transform * vec4(position, 1.0);"
                    "pos = position;"
                    "nrm = normal;"
                    "}"
                    ;
                const char *flag =
                    "#version 330 core\n"
                    "in vec3 pos;"
                    "in vec3 nrm;"
                    "out vec4 color;"
                    "uniform vec3 light;"
                    "void main()"
                    "{"
                    "vec3 l = normalize(light - pos);"
                    "float d = max(dot(nrm, l), 0);"
                    "color = vec4(d, d, d, 1.0);"
                    "}"
                    ;
                
                char log[512] = { 0 };
                shader.load(vert, flag, NULL, log);
                printf("%s\n", log);

                vao.init();
                vao.set(0, m_model.ptr, sizeof(Mesh3) * m_model.size());
                vao.set(1, m_nrms.ptr, sizeof(Vec3) * m_nrms.size());

                vao.bind();
                vao.enable(0, 3, GL_FLOAT);
                vao.enable(1, 3, GL_FLOAT);
                vao.unbind();
            }

            const Mat transform = glGetProjMat(m_cam, m_viewPos, m_viewScale) * m_pose;
            const Vec3 light = invPose(m_pose) * getVec3(0, 0, -1000);

            shader.enable();
            shader.setUniform4m("transform", transform.ptr);
            shader.setUniform3f("light", (float*)&light);
            glEnable(GL_DEPTH_TEST);
            vao.bind();

            glDrawArrays(GL_TRIANGLES, 0, m_model.size() * 3);
            
            vao.unbind();
            shader.disable();
        }
    }

    void sample02() {
        {
            static Shader shader;
            static VertexArrayObject vao;

            if (shader.id() == 0) {
                const char *vert =
                    "#version 330 core\n"
                    "layout(location = 0) in vec3 position;"
                    "uniform mat4 transform;"
                    "void main(){"
                    "gl_Position = transform * vec4(position, 1.0);"
                    "}"
                    ;
                const char *geom =
                    "#version 330 core\n"
                    "layout(points) in;"
                    "layout(triangle_strip, max_vertices = 18) out;"
                    "void cube(vec3 n, vec3 a, vec3 b){"
                    "gl_Position = gl_in[0].gl_Position + vec4(n - a - b, 0.0); EmitVertex();"
                    "gl_Position = gl_in[0].gl_Position + vec4(n - a + b, 0.0); EmitVertex();"
                    "gl_Position = gl_in[0].gl_Position + vec4(n + a - b, 0.0); EmitVertex();"
                    "EndPrimitive();"
                    "gl_Position = gl_in[0].gl_Position + vec4(n + a + b, 0.0); EmitVertex();"
                    "gl_Position = gl_in[0].gl_Position + vec4(n + a - b, 0.0); EmitVertex();"
                    "gl_Position = gl_in[0].gl_Position + vec4(n - a + b, 0.0); EmitVertex();"
                    "EndPrimitive();"
                    "}"
                    "void main() {"
                    "cube(vec3(-0.5, 0.0, 0.0), vec3(0.0, 0.5, 0.0), vec3(0.0, 0.0, 0.5));"
                    "cube(vec3(0.0, 0.0, -0.5), vec3(0.5, 0.0, 0.0), vec3(0.0, 0.5, 0.0));"
                    "}"
                    ;

                const char *flag =
                    "#version 330 core\n"
                    "out vec4 color;"
                    "void main()"
                    "{"
                    "color = vec4(1.0, 1.0, 1.0, 1.0);"
                    "}"
                    ;

                char log[512] = { 0 };
                shader.load(vert, flag, geom, log);
                printf("%s\n", log);

                vao.init();
                vao.set(0, m_vtxs.ptr, sizeof(Vec3) * m_vtxs.size());

                vao.bind();
                vao.enable(0, 3, GL_FLOAT);
                vao.unbind();
            }

            const Mat transform = glGetProjMat(m_cam, m_viewPos, m_viewScale) * m_pose;
            shader.enable();
            shader.setUniform4m("transform", transform.ptr);

            glPointSize(3.0f);
            glEnable(GL_DEPTH_TEST);
            vao.bind();

            glDrawArrays(GL_POINTS, 0, m_vtxs.size());

            vao.unbind();
            shader.disable();
        }
    }
    virtual void display() {
        glClearColor(0.10f, 0.12f, 0.12f, 0.00f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        sample02();

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