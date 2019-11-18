//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_GLUTIL_H__
#define __SP_GLUTIL_H__

#include "GLFW/glfw3.h"

#include "spcore/spcore.h"

namespace sp {
    //--------------------------------------------------------------------------------
    // overwrap
    //--------------------------------------------------------------------------------

    SP_CPUFUNC void _glLoadMatrix(const float *mat) {
        glLoadMatrixf(mat);
    }
    SP_CPUFUNC void _glLoadMatrix(const double *mat) {
        glLoadMatrixd(mat);
    }
    SP_CPUFUNC void _glMultMatrix(const float *mat) {
        glMultMatrixf(mat);
    }
    SP_CPUFUNC void _glMultMatrix(const double *mat) {
        glMultMatrixd(mat);
    }

    SP_CPUFUNC void glLoadMatrix(const Mat &mat) {
        const Mat m4x4t = trnMat(extMat(4, 4, mat));
        _glLoadMatrix(m4x4t.ptr);
    }
    SP_CPUFUNC void glMultMatrix(const Mat &mat) {
        const Mat m4x4t = trnMat(extMat(4, 4, mat));
        _glMultMatrix(m4x4t.ptr);
    }
    SP_CPUFUNC void glLoadMatrix(const Pose &pose) {
        glLoadMatrix(getMat(pose));
    }
    SP_CPUFUNC void glMultMatrix(const Pose &pose) {
        glMultMatrix(getMat(pose));
    }

    SP_CPUFUNC void glVertex(const Vec2 &vrt) {
        glVertex2d(vrt.x, vrt.y);
    }
    SP_CPUFUNC void glVertex(const Vec3 &vrt) {
        glVertex3d(vrt.x, vrt.y, vrt.z);
    }
    SP_CPUFUNC void glNormal(const Vec3 &nrm) {
        glNormal3d(nrm.x, nrm.y, nrm.z);
    }

    SP_CPUFUNC void glColor(const Col3 &col) {
        glColor3ub(col.r, col.g, col.b);
    }
    SP_CPUFUNC void glColor(const Col4 &col) {
        glColor4ub(col.r, col.g, col.b, col.a);
    }
    SP_CPUFUNC void glColor(const Col3f &col) {
        glColor3f(col.r, col.g, col.b);
    }
    SP_CPUFUNC void glColor(const Col4f &col) {
        glColor4f(col.r, col.g, col.b, col.a);
    }

    SP_CPUFUNC void glViewport(const Rect2 &rect) {
        GLFWwindow *win = glfwGetCurrentContext();

        int ww, wh;
        glfwGetWindowSize(win, &ww, &wh);

        ::glViewport(rect.dbase[0], wh - (rect.dbase[1] + rect.dsize[1]), rect.dsize[0], rect.dsize[1]);
    }


    //--------------------------------------------------------------------------------
    // get gl param
    //--------------------------------------------------------------------------------

    // ex. GL_MODELVIEW_MATRIX, GL_PROJECTION_MATRIX
    SP_CPUFUNC Mat glGetMat(const int type) {
        Mat mat(4, 4);

        double t[4 * 4];
        glGetDoublev(type, t);
        for (int i = 0; i < 4 * 4; i++) {
            mat[i] = SP_RCAST(t[i]);
        }
        return trnMat(mat);
    }

    SP_CPUFUNC double glGetPixelScale() {
        GLFWwindow *win = glfwGetCurrentContext();

        int fw, fh;
        glfwGetFramebufferSize(win, &fw, &fh);

        int ww, wh;
        glfwGetWindowSize(win, &ww, &wh);
        const double pscale = (static_cast<double>(fw) / ww + static_cast<double>(fh) / wh) / 2.0;

        return pscale;
    }

    SP_CPUFUNC float glGetBasicLineWidth() {
        GLint viewport[4];
        glGetIntegerv(GL_VIEWPORT, viewport);

        return  static_cast<float>(log10(2.0 * minVal(viewport[2], viewport[3]))) * 0.5f;
    }

    //--------------------------------------------------------------------------------
    // load view 2d
    //--------------------------------------------------------------------------------

    SP_CPUFUNC Mat glGetViewMat(const int dsize0, const int dsize1, const Vec2 &viewPos = getVec2(0.0, 0.0), const double viewScale = 1.0) {
        GLint viewport[4];
        glGetIntegerv(GL_VIEWPORT, viewport);

        const Vec2 rectCenter = getVec2(dsize0 - 1, dsize1 - 1) * 0.5;
        const Vec2 viewCenter = getVec2(viewport[2] - 1, viewport[3] - 1) * 0.5;
        const Vec2 shift = (viewPos + viewCenter) - rectCenter * viewScale;

        Mat vmat = eyeMat(4, 4);
        vmat(0, 0) = SP_RCAST(viewScale);
        vmat(1, 1) = SP_RCAST(viewScale);
        vmat(0, 3) = shift.x;
        vmat(1, 3) = shift.y;

        Mat _vmat = vmat;

        {// for retina display
            const double pscale = glGetPixelScale();

            Mat pmat = eyeMat(4, 4);
            pmat(0, 0) = SP_RCAST(pscale);
            pmat(1, 1) = SP_RCAST(pscale);
            pmat(0, 3) = SP_RCAST((1.0 - pscale) * viewCenter.x);
            pmat(1, 3) = SP_RCAST((1.0 - pscale) * viewCenter.y);

            _vmat = pmat * vmat;
        }

        return _vmat;
    }

    SP_CPUFUNC Mat glGetViewMat(const int *dsize, const Vec2 &viewPos = getVec2(0.0, 0.0), const double viewScale = 1.0) {

        return glGetViewMat(dsize[0], dsize[1], viewPos, viewScale);
    }

    SP_CPUFUNC Mat glGetWindowMat(const int dsize0, const int dsize1, const Vec2 &viewPos = getVec2(0.0, 0.0), const double viewScale = 1.0) {
        Mat vmat = glGetViewMat(dsize0, dsize1, viewPos, viewScale);

        Mat _vmat = vmat;

        {// for retina display
            const double pixScale = glGetPixelScale();

            Mat pmat = eyeMat(4, 4);
            pmat(0, 0) = SP_RCAST(1.0 / pixScale);
            pmat(1, 1) = SP_RCAST(1.0 / pixScale);

            _vmat = pmat * vmat;
        }

        return _vmat;
    }

    SP_CPUFUNC Mat glGetWindowMat(const int *dsize, const Vec2 &viewPos = getVec2(0.0, 0.0), const double viewScale = 1.0) {

        return glGetWindowMat(dsize[0], dsize[1], viewPos, viewScale);
    }


    SP_CPUFUNC void glLoadView2D(const int dsize0, const int dsize1, const Mat &vmat) {
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_LIGHTING);
        glDisable(GL_CULL_FACE);

        GLint viewport[4];
        glGetIntegerv(GL_VIEWPORT, viewport);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(-0.5, viewport[2] - 0.5, viewport[3] - 0.5, -0.5, -1.0, 1.0);
        glMultMatrix(vmat);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
    }

    SP_CPUFUNC void glLoadView2D(const int *dsize, const Mat &vmat) {

        glLoadView2D(dsize[0], dsize[1], vmat);
    }

    SP_CPUFUNC void glLoadView2D(const int dsize0, const int dsize1, const Vec2 &viewPos = getVec2(0.0, 0.0), const double viewScale = 1.0) {

        glLoadView2D(dsize0, dsize1, glGetViewMat(dsize0, dsize1, viewPos, viewScale));
    }

    SP_CPUFUNC void glLoadView2D(const int *dsize, const Vec2 &viewPos = getVec2(0.0, 0.0), const double viewScale = 1.0) {

        glLoadView2D(dsize, glGetViewMat(dsize, viewPos, viewScale));
    }

    SP_CPUFUNC void glLoadView2D(const CamParam &cam, const Vec2 &viewPos = getVec2(0.0, 0.0), const double viewScale = 1.0) {

        glLoadView2D(cam.dsize, glGetViewMat(cam.dsize, viewPos, viewScale));
    }


    //--------------------------------------------------------------------------------
    // load view 3d
    //--------------------------------------------------------------------------------

    // default near & far
#define SP_DEFAULT_NEAR SP_RCAST(1.0)
#define SP_DEFAULT_FAR SP_RCAST(10000.0)

    SP_CPUFUNC void glLoadView3D(const CamParam &cam, const Vec2 &viewPos = getVec2(0.0, 0.0), const double viewScale = 1.0, const double nearPlane = SP_DEFAULT_NEAR, const double farPlane = SP_DEFAULT_FAR) {
        glEnable(GL_DEPTH_TEST);

        Mat mat = zeroMat(4, 4);

        GLint viewport[4];
        glGetIntegerv(GL_VIEWPORT, viewport);

        const Vec2 vcent = getVec2(viewport[2] - 1, viewport[3] - 1) * 0.5;
        const Vec2 ccent = getVec2(cam.dsize[0] - 1, cam.dsize[1] - 1) * 0.5 - getVec2(cam.cx, cam.cy);
        const Vec2 cdisp = viewPos + vcent - ccent * viewScale;

        if (cam.type == CamParam_Pers) {

            const double nx = nearPlane / cam.fx;
            const double ny = nearPlane / cam.fy;

            const double sw = (viewport[2] - 1) / viewScale;
            const double sh = (viewport[3] - 1) / viewScale;

            const double l = (-cdisp.x / viewScale) * nx;
            const double r = (-cdisp.x / viewScale + sw) * nx;
            const double t = (-cdisp.y / viewScale) * ny;
            const double b = (-cdisp.y / viewScale + sh) * ny;
            const double n = nearPlane;
            const double f = farPlane;

            mat(0, 0) = SP_RCAST(2 * n / (r - l));
            mat(1, 1) = SP_RCAST(2 * n / (t - b));

            mat(0, 2) = SP_RCAST(-(r + l) / (r - l));
            mat(1, 2) = SP_RCAST(-(t + b) / (t - b));
            mat(2, 2) = SP_RCAST(+(f + n) / (f - n));

            mat(2, 3) = SP_RCAST(-2 * f * n / (f - n));

            mat(3, 2) = SP_RCAST(1.0);
        }
        else {
            const double nx = 1.0 / cam.fx;
            const double ny = 1.0 / cam.fy;

            const double sw = (viewport[2] - 1) / viewScale;
            const double sh = (viewport[3] - 1) / viewScale;

            const double l = (-cdisp.x / viewScale) * nx;
            const double r = (-cdisp.x / viewScale + sw) * nx;
            const double t = (-cdisp.y / viewScale) * ny;
            const double b = (-cdisp.y / viewScale + sh) * ny;
            const double n = nearPlane;
            const double f = farPlane;

            mat(0, 0) = SP_RCAST(2 / (r - l));
            mat(1, 1) = SP_RCAST(2 / (t - b));
            mat(2, 2) = SP_RCAST(2 / (f - n));

            mat(0, 3) = SP_RCAST(-(r + l) / (r - l));
            mat(1, 3) = SP_RCAST(-(t + b) / (t - b));
            mat(2, 3) = SP_RCAST(-(f + n) / (f - n));

            mat(3, 3) = SP_RCAST(1.0);
        }

        glMatrixMode(GL_PROJECTION);
        glLoadMatrix(mat);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
    }


    //--------------------------------------------------------------------------------
    // texture
    //--------------------------------------------------------------------------------

    class Texture {
    private:

        // texture id
        GLuint m_tx;

        // backup
        char *mem;

    private:

        void free() {
            if (m_tx > 0) {
                glDeleteTextures(1, &m_tx);
            }
            if (mem != NULL) {
                delete[]mem;
            }
            reset();
        }

        void reset() {
            memset(this, 0, sizeof(Texture));
        }

        Texture(const Texture &tex) {}
        Texture& operator = (const Texture &tex) {}

    public:

        int ch;
        int dsize[2];

        Texture() {
            reset();
        }

        ~Texture() {
            free();
        }

        template<typename TYPE>
        Texture(const TYPE *img, const int *dsize) {
            reset();
            set(img, dsize);
        }
        template <typename TYPE>
        Texture(const TYPE *img, const int *dsize, const int ch) {
            reset();
            set(img, dsize, ch);
        }

        GLuint id() const {
            return m_tx;
        }

        template<typename TYPE>
        bool set(const TYPE *img, const int *dsize) {
            return set(img, dsize, sizeof(TYPE));
        }

        template<typename TYPE>
        bool set(const TYPE *img, const int *dsize, const int ch) {

            int format;
            switch (ch) {
            case 1: format = GL_LUMINANCE; break;
            case 3: format = GL_RGB; break;
            case 4: format = GL_RGBA; break;
            default: return false;
            }

            if ((this->dsize[0] * this->dsize[1] * this->ch) != (dsize[0] * dsize[1] * ch)) {
                if (mem != NULL) delete[]mem;
                mem = new char[dsize[0] * dsize[1] * ch];
            }

            this->dsize[0] = dsize[0];
            this->dsize[1] = dsize[1];
            this->ch = ch;

            memcpy(mem, img, dsize[0] * dsize[1] * ch);

            if (m_tx == 0) {
                glGenTextures(1, &m_tx);

                glBindTexture(GL_TEXTURE_2D, m_tx);

                glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
            }

            glBindTexture(GL_TEXTURE_2D, m_tx);

            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, dsize[0], dsize[1], 0, format, GL_UNSIGNED_BYTE, mem);

            glBindTexture(GL_TEXTURE_2D, 0);

            return (m_tx > 0) ? true : false;
        }

    };


    //--------------------------------------------------------------------------------
    // util
    //--------------------------------------------------------------------------------

    SP_CPUFUNC void glCircle(const Vec2 &pos, const double radius, const bool fill = false) {
        const int type = (fill == true) ? GL_TRIANGLE_FAN : GL_LINE_LOOP;

        glBegin(type);

        if (fill == true) {
            glVertex(pos);
        }
        for (int i = 0; i <= 36; i++) {
            const double p = i / 36.0 * 2.0 * SP_PI;
            glVertex(getVec2(pos.x + radius * sin(p), pos.y + radius * cos(p)));
        }
        glEnd();
    }

    SP_CPUFUNC void glCircle(const Vec3 &pos, const double radius, const bool fill = false) {
        const int type = (fill == true) ? GL_TRIANGLE_FAN : GL_LINE_LOOP;

        const Mat imat = invMat(glGetMat(GL_MODELVIEW_MATRIX).part(0, 0, 3, 3));
        const Vec3 a = imat * getVec3(1.0, 0.0, 0.0);
        const Vec3 b = imat * getVec3(0.0, 1.0, 0.0);

        glBegin(type);

        if (fill == true) {
            glVertex(pos);
        }
        for (int i = 0; i <= 36; i++) {
            const double p = i / 36.0 * 2.0 * SP_PI;
            glVertex(pos + a * radius * sin(p) + b * radius * cos(p));
        }
        glEnd();
    }

    template<typename VEC>
    SP_CPUFUNC void glLine(const VEC &vtx0, const VEC &vtx1) {
        glBegin(GL_LINES);
        glVertex(vtx0);
        glVertex(vtx1);
        glEnd();
    }

    template<typename VEC>
    SP_CPUFUNC void glLine(const Mem1<VEC> &vtxs, const bool loop = false) {
        for (int i = 0; i < vtxs.size(); i++) {
            if (i == vtxs.size() - 1 && loop == false) break;
            glLine(vtxs.lacs(i + 0), vtxs.lacs(i + 1));
        }
    }

    SP_CPUFUNC void glCross(const Vec2 &vtx, const double size) {
        glBegin(GL_LINES);
        glVertex(vtx + getVec2(0.0, +size)); glVertex(vtx + getVec2(0.0, -size));
        glVertex(vtx + getVec2(+size, 0.0)); glVertex(vtx + getVec2(-size, 0.0));
        glEnd();
    }

    SP_CPUFUNC void glRect(const Rect2 &rect, const double m = 0.5, const bool fill = false) {

        const Vec2 A = getVec2(rect.dbase[0] - m, rect.dbase[1] - m);
        const Vec2 B = getVec2(rect.dbase[0] + rect.dsize[0] - 1.0 + m, rect.dbase[1] + rect.dsize[1] - 1.0 + m);

        glBegin((fill == true) ? GL_QUADS : GL_LINE_LOOP);
        glVertex(getVec2(A.x, A.y)); glVertex(getVec2(B.x, A.y));
        glVertex(getVec2(B.x, B.y)); glVertex(getVec2(A.x, B.y));
        glEnd();
    }
    SP_CPUFUNC void glRect(const Rect3 &rect, const double m = 0.5, const bool fill = false) {

        const Vec3 A = getVec3(rect.dbase[0] - m, rect.dbase[1] - m, rect.dbase[2] - m);
        const Vec3 B = getVec3(rect.dbase[0] + rect.dsize[0] - 1.0 + m, rect.dbase[1] + rect.dsize[1] - 1.0 + m, rect.dbase[2] + rect.dsize[2] - 1.0 + m);

        if (fill == true) {
            glBegin(GL_QUADS);
            glVertex(getVec3(A.x, A.y, A.z)); glVertex(getVec3(B.x, A.y, A.z)); glVertex(getVec3(B.x, B.y, A.z)); glVertex(getVec3(A.x, B.y, A.z));
            glVertex(getVec3(A.x, A.y, B.z)); glVertex(getVec3(B.x, A.y, B.z)); glVertex(getVec3(B.x, B.y, B.z)); glVertex(getVec3(A.x, B.y, B.z));
            glVertex(getVec3(A.x, A.y, A.z)); glVertex(getVec3(A.x, A.y, B.z)); glVertex(getVec3(B.x, A.y, B.z)); glVertex(getVec3(B.x, A.y, A.z));
            glVertex(getVec3(A.x, B.y, A.z)); glVertex(getVec3(A.x, B.y, B.z)); glVertex(getVec3(B.x, B.y, B.z)); glVertex(getVec3(B.x, B.y, A.z));
            glVertex(getVec3(A.x, A.y, A.z)); glVertex(getVec3(A.x, B.y, A.z)); glVertex(getVec3(A.x, B.y, B.z)); glVertex(getVec3(A.x, A.y, B.z));
            glVertex(getVec3(B.x, A.y, A.z)); glVertex(getVec3(B.x, B.y, A.z)); glVertex(getVec3(B.x, B.y, B.z)); glVertex(getVec3(B.x, A.y, B.z));
            glEnd();
        }
        else {
            glBegin(GL_LINE_LOOP);
            glVertex(getVec3(A.x, A.y, A.z)); glVertex(getVec3(B.x, A.y, A.z)); glVertex(getVec3(B.x, B.y, A.z)); glVertex(getVec3(A.x, B.y, A.z));
            glEnd();

            glBegin(GL_LINE_LOOP);
            glVertex(getVec3(A.x, A.y, B.z)); glVertex(getVec3(B.x, A.y, B.z)); glVertex(getVec3(B.x, B.y, B.z)); glVertex(getVec3(A.x, B.y, B.z));
            glEnd();

            glBegin(GL_LINES);
            glVertex(getVec3(A.x, A.y, A.z)); glVertex(getVec3(A.x, A.y, B.z));
            glVertex(getVec3(B.x, A.y, A.z)); glVertex(getVec3(B.x, A.y, B.z));
            glVertex(getVec3(A.x, B.y, A.z)); glVertex(getVec3(A.x, B.y, B.z));
            glVertex(getVec3(B.x, B.y, A.z)); glVertex(getVec3(B.x, B.y, B.z));
            glEnd();
        }
    }

    SP_CPUFUNC void glMesh(const Mesh2 &mesh) {
        glBegin(GL_TRIANGLES);
        glVertex(mesh.pos[0]);
        glVertex(mesh.pos[1]);
        glVertex(mesh.pos[2]);
        glEnd();
    }

    SP_CPUFUNC void glMesh(const Mesh3 &mesh) {
        glBegin(GL_TRIANGLES);
        glNormal(getMeshNrm(mesh));
        glVertex(mesh.pos[0]);
        glVertex(mesh.pos[1]);
        glVertex(mesh.pos[2]);
        glEnd();
    }

    SP_CPUFUNC void glAxis(const double size) {
        glBegin(GL_LINES);
        glColor3ub(255, 0, 0);
        glVertex3d(0.0, 0.0, 0.0); glVertex3d(size, 0.0, 0.0);

        glColor3ub(0, 255, 0);
        glVertex3d(0.0, 0.0, 0.0); glVertex3d(0.0, size, 0.0);

        glColor3ub(0, 0, 255);
        glVertex3d(0.0, 0.0, 0.0); glVertex3d(0.0, 0.0, size);
        glEnd();
    }

    SP_CPUFUNC void glVector(const Vec3 &vtx0, const Vec3 &vtx1, const double radius) {
        const double seg0 = radius * 8.0;
        const double seg1 = normVec(vtx1 - vtx0) - seg0;
        const Vec3 drc = unitVec(vtx1 - vtx0);

        const Mem1<Mesh3> vec = loadCone(drc * seg0, radius * 3.0) + (drc * seg1 + vtx0);
        const Mem1<Mesh3> cyl = loadCylinder(drc * seg1, radius) + vtx0;

        for (int i = 0; i < vec.size(); i++) {
            glMesh(vec[i]);
        }
        for (int i = 0; i < cyl.size(); i++) {
            glMesh(cyl[i]);
        }
    }

    SP_CPUFUNC void glCube(const double size) {
        glBegin(GL_QUADS);

        const double s = size / 2.0;
        for (int i = 0; i < 2; i++) {
            double v = (i == 0) ? +1.0 : -1.0;
            glNormal3d(v, 0.0, 0.0);
            glVertex3d(+s * v, -s, -s);
            glVertex3d(+s * v, -s, +s);
            glVertex3d(+s * v, +s, +s);
            glVertex3d(+s * v, +s, -s);

            glNormal3d(0.0, v, 0.0);
            glVertex3d(-s, +s * v, -s);
            glVertex3d(-s, +s * v, +s);
            glVertex3d(+s, +s * v, +s);
            glVertex3d(+s, +s * v, -s);

            glNormal3d(0.0, 0.0, v);
            glVertex3d(-s, -s, +s * v);
            glVertex3d(-s, +s, +s * v);
            glVertex3d(+s, +s, +s * v);
            glVertex3d(+s, -s, +s * v);
        }
        glEnd();
    }

    SP_CPUFUNC void glGrid(const double size, const int num) {
        glBegin(GL_LINES);
        for (int i = 0; i < num; i++) {
            const double p = i * 2 * size / (num - 1);
            glVertex3d(-size, -size + p, 0.0);
            glVertex3d(+size, -size + p, 0.0);

            glVertex3d(-size + p, -size, 0.0);
            glVertex3d(-size + p, +size, 0.0);
        }
        glEnd();
    }

    SP_CPUFUNC void glCam(const CamParam &cam, const double size) {
        const double f = (cam.fx + cam.fy) / 2.0;
        const double w = (cam.dsize[0] / 2.0) / f * size;
        const double h = (cam.dsize[1] / 2.0) / f * size;

        const Vec2 loop[4] = { getVec2(-w, -h), getVec2(+w, -h), getVec2(+w, +h), getVec2(-w, +h) };
        glBegin(GL_LINES);
        for (int i = 0; i < 4; i++) {
            const Vec2 a = loop[(i + 0) % 4];
            const Vec2 b = loop[(i + 1) % 4];
            glVertex3d(0.0, 0.0, 0.0); glVertex3d(a.x, a.y, size);
            glVertex3d(a.x, a.y, size); glVertex3d(b.x, b.y, size);
        }
        glEnd();
    }

    SP_CPUFUNC void glModel(const Mem1<Mesh3> &model) {
        for (int i = 0; i < model.size(); i++) {
            glMesh(model[i]);
        }
    }


    //--------------------------------------------------------------------------------
    // render
    //--------------------------------------------------------------------------------

    SP_CPUFUNC void glRenderSurface(const Mem1<Mesh3> &model, const Vec3 lpos = getVec3(0.0, 0.0, -1000.0)) {

        glPushAttrib(GL_ALL_ATTRIB_BITS);
        {
            glPushMatrix();
            glLoadIdentity();

            glEnable(GL_LIGHTING);
            glEnable(GL_LIGHT0);

            GLfloat lightPos[4] = { static_cast<float>(lpos.x), static_cast<float>(lpos.y), static_cast<float>(lpos.z), 1.f };
            glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
            glPopMatrix();

            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

            glModel(model);
        }
        glPopAttrib();
    }

    SP_CPUFUNC void glRenderOutline(const Mem1<Mesh3> &model) {

        glPushAttrib(GL_ALL_ATTRIB_BITS);
        {
            glEnable(GL_STENCIL_TEST);

            glClearStencil(0);
            glClear(GL_STENCIL_BUFFER_BIT);

            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE);

            // fill stencil
            {
                glStencilFunc(GL_ALWAYS, 1, 0xFFFF);
                glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                glColor4f(0.0f, 0.0f, 0.0f, 0.0f);

                glModel(model);
            }

            // draw outline
            {
                glStencilFunc(GL_NOTEQUAL, 1, 0xFFFF);
                glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                glLineWidth(2.0f);
                glColor3f(1.0f, 1.0f, 1.0f);

                glModel(model);
            }
        }
        glPopAttrib();
    }


    SP_CPUFUNC void glTexImg(const Texture &tex, const GLint filter = GL_NEAREST, const GLint wrap = GL_CLAMP) {
        if (tex.id() == 0) return;

        glPushAttrib(GL_ENABLE_BIT);
        glEnable(GL_TEXTURE_2D);
        {
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

            glBindTexture(GL_TEXTURE_2D, tex.id());

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrap);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrap);
      
            //glShadeModel(GL_FLAT);
            glColor3d(1.0, 1.0, 1.0);
            glColorMask(1, 1, 1, 1);

            const int w = tex.dsize[0];
            const int h = tex.dsize[1];

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

    template<typename TYPE>
    SP_CPUFUNC void glTexImg(const Mem<TYPE> &src, const GLint filter = GL_NEAREST, const GLint wrap = GL_CLAMP) {
        if (src.size() == 0) return;

        Texture tex;
        if (tex.set(src.ptr, src.dsize) == false) return;

        glTexImg(tex, filter, wrap);
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
