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

    SP_CPUFUNC void glLoadMatrix(const Mat &mat){
        const Mat m4x4t = trnMat(extMat(4, 4, mat));
        _glLoadMatrix(m4x4t.ptr);
    }

    SP_CPUFUNC void glMultMatrix(const Mat &mat){
        const Mat m4x4t = trnMat(extMat(4, 4, mat));
        _glMultMatrix(m4x4t.ptr);
    }

    SP_CPUFUNC void glLoadMatrix(const Pose &pose){
        glLoadMatrix(getMat(pose));
    }

    SP_CPUFUNC void glMultMatrix(const Pose &pose){
        glMultMatrix(getMat(pose));
    }


    SP_CPUFUNC void glVertex(const Vec2 &vec){
        glVertex2d(vec.x, vec.y);
    }

    SP_CPUFUNC void glVertex(const Vec3 &vec){
        glVertex3d(vec.x, vec.y, vec.z);
    }

    SP_CPUFUNC void glNormal(const Vec3 &nrm){
        glNormal3d(nrm.x, nrm.y, nrm.z);
    }

    SP_CPUFUNC void glColor(const Col3 &col) {
        glColor3ub(col.r, col.g, col.b);
    }

    SP_CPUFUNC void glColor(const Col4 &col) {
        glColor4ub(col.r, col.g, col.b, col.a);
    }

    SP_CPUFUNC void glColor(const int label){
        srand(maxVal(label, 0));
        Col3 col;
        cnvHSVToCol(col, getVec((randValUnif() + 1.0) * SP_PI, 1.0, 1.0));
        glColor(col);
    }

    SP_CPUFUNC void glMaterial(GLenum face, GLenum pname, const Col3 col) {
        GLfloat param[4] = { static_cast<float>(col.r) / SP_BYTEMAX, static_cast<float>(col.g) / SP_BYTEMAX, static_cast<float>(col.b) / SP_BYTEMAX, 1.f };
        glMaterialfv(face, pname, param);
    }

    SP_CPUFUNC void glMaterial(GLenum face, GLenum pname, const Col4 col) {
        GLfloat param[4] = { static_cast<float>(col.r) / SP_BYTEMAX, static_cast<float>(col.g) / SP_BYTEMAX, static_cast<float>(col.b) / SP_BYTEMAX, static_cast<float>(col.a) / SP_BYTEMAX };
        glMaterialfv(face, pname, param);
    }

    SP_CPUFUNC void glViewport(const Rect &rect) {
        GLFWwindow *win = glfwGetCurrentContext();

        int ww, wh;
        glfwGetWindowSize(win, &ww, &wh);

        ::glViewport(rect.dbase[0], wh - (rect.dbase[1] + rect.dsize[1]), rect.dsize[0], rect.dsize[1]);
    }

    //--------------------------------------------------------------------------------
    // get
    //--------------------------------------------------------------------------------

    SP_CPUFUNC Mat glGetMat(const int type) {
        Mat mat(4, 4);

        // GL_MODELVIEW_MATRIX, GL_PROJECTION_MATRIX
        double t[4 * 4];
        glGetDoublev(type, t);
        for (int i = 0; i < 4 * 4; i++) {
            mat[i] = SP_CAST(t[i]);
        }
        return trnMat(mat);
    }

	SP_CPUFUNC double glGetPixelScale() {
		GLFWwindow *win = glfwGetCurrentContext();

		int fw, fh;
		glfwGetFramebufferSize(win, &fw, &fh);

		int ww, wh;
		glfwGetWindowSize(win, &ww, &wh);
		const double pixScale = (static_cast<double>(fw) / ww + static_cast<double>(fh) / wh) / 2.0;

		return pixScale;
	}

    //--------------------------------------------------------------------------------
    // load view
    //--------------------------------------------------------------------------------
    SP_CPUFUNC Mat glGetViewMat(const int dsize0, const int dsize1, const Vec2 &viewPos = getVec(0.0, 0.0), const SP_REAL viewScale = 1.0) {
        GLint viewport[4];
        glGetIntegerv(GL_VIEWPORT, viewport);
        
        const Vec2 rectCenter = getVec(dsize0 - 1, dsize1 - 1) * 0.5;
        const Vec2 viewCenter = getVec(viewport[2] - 1, viewport[3] - 1) * 0.5;
        const Vec2 shift = (viewPos + viewCenter) - rectCenter * viewScale;

        Mat vmat = eyeMat(4, 4);
        vmat(0, 0) = viewScale;
        vmat(1, 1) = viewScale;
        vmat(0, 3) = shift.x;
        vmat(1, 3) = shift.y;

        Mat _vmat = vmat;

        {// for retina display
            const SP_REAL pixScale = glGetPixelScale();

            Mat pmat = eyeMat(4, 4);
            pmat(0, 0) = pixScale;
            pmat(1, 1) = pixScale;
            pmat(0, 3) = (1.0 - pixScale) * viewCenter.x;
            pmat(1, 3) = (1.0 - pixScale) * viewCenter.y;

            _vmat = pmat * vmat;
        }

        return _vmat;
    }

    SP_CPUFUNC Mat glGetViewMat(const int *dsize, const Vec2 &viewPos = getVec(0.0, 0.0), const SP_REAL viewScale = 1.0) {

        return glGetViewMat(dsize[0], dsize[1], viewPos, viewScale);
    }

    SP_CPUFUNC Mat glGetWindowMat(const int dsize0, const int dsize1, const Vec2 &viewPos = getVec(0.0, 0.0), const SP_REAL viewScale = 1.0) {
        Mat vmat = glGetViewMat(dsize0, dsize1, viewPos, viewScale);

        Mat _vmat = vmat;

        {// for retina display
			const SP_REAL pixScale = glGetPixelScale();

            Mat pmat = eyeMat(4, 4);
            pmat(0, 0) = SP_CAST(1.0 / pixScale);
            pmat(1, 1) = SP_CAST(1.0 / pixScale);

            _vmat = pmat * vmat;
        }

        return _vmat;
    }

    SP_CPUFUNC Mat glGetWindowMat(const int *dsize, const Vec2 &viewPos = getVec(0.0, 0.0), const SP_REAL viewScale = 1.0) {

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

    SP_CPUFUNC void glLoadView2D(const int dsize0, const int dsize1, const Vec2 &viewPos = getVec(0.0, 0.0), const SP_REAL viewScale = 1.0){

        glLoadView2D(dsize0, dsize1, glGetViewMat(dsize0, dsize1, viewPos, viewScale));
    }

    SP_CPUFUNC void glLoadView2D(const int *dsize, const Vec2 &viewPos = getVec(0.0, 0.0), const SP_REAL viewScale = 1.0) {

        glLoadView2D(dsize, glGetViewMat(dsize, viewPos, viewScale));
    }

    SP_CPUFUNC void glLoadView2D(const CamParam &cam, const Vec2 &viewPos = getVec(0.0, 0.0), const SP_REAL viewScale = 1.0){

        glLoadView2D(cam.dsize, glGetViewMat(cam.dsize, viewPos, viewScale));
    }

    SP_CPUFUNC void glLoadView3D(const CamParam &cam, const Vec2 &viewPos = getVec(0.0, 0.0), const SP_REAL viewScale = 1.0, const SP_REAL nearPlane = 1.0, const SP_REAL farPlane = 10000.0){
        GLint viewport[4];
        glGetIntegerv(GL_VIEWPORT, viewport);

        glEnable(GL_DEPTH_TEST);

        Mat mat = zeroMat(4, 4);
        {
            Vec2 cDispPos;
            cDispPos.x = viewPos.x + (viewport[2] - 1) * 0.5 - ((cam.dsize[0] - 1) * 0.5 - cam.cx) * viewScale;
            cDispPos.y = viewPos.y + (viewport[3] - 1) * 0.5 - ((cam.dsize[1] - 1) * 0.5 - cam.cy) * viewScale;

            const SP_REAL nx = nearPlane / cam.fx;
            const SP_REAL ny = nearPlane / cam.fy;

            const SP_REAL sw = (viewport[2] - 1) / viewScale;
            const SP_REAL sh = (viewport[3] - 1) / viewScale;

            const SP_REAL l = (-cDispPos.x / viewScale) * nx;
            const SP_REAL r = (-cDispPos.x / viewScale + sw) * nx;
            const SP_REAL t = (-cDispPos.y / viewScale) * ny;
            const SP_REAL b = (-cDispPos.y / viewScale + sh) * ny;
            const SP_REAL n = nearPlane;
            const SP_REAL f = farPlane;

            mat(0, 0) = 2 * n / (r - l);
            mat(1, 1) = 2 * n / (t - b);

            mat(0, 2) = -(r + l) / (r - l);
            mat(1, 2) = -(t + b) / (t - b);
            mat(2, 2) = +(f + n) / (f - n);

            mat(2, 3) = -2 * f * n / (f - n);

            mat(3, 2) = 1.0;
        }

        glMatrixMode(GL_PROJECTION);
        glLoadMatrix(mat);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
    }

    SP_CPUFUNC void glLoadView3DOrth(const CamParam &cam, const SP_REAL z, const Vec2 &viewPos = getVec(0.0, 0.0), const SP_REAL viewScale = 1.0, const SP_REAL nearPlane = 1.0, const SP_REAL farPlane = 10000.0) {
        GLint viewport[4];
        glGetIntegerv(GL_VIEWPORT, viewport);

        glEnable(GL_DEPTH_TEST);

        Mat mat = zeroMat(4, 4);
        {
            Vec2 cDispPos;
            cDispPos.x = viewPos.x + (viewport[2] - 1) * 0.5 - ((cam.dsize[0] - 1) * 0.5 - cam.cx) * viewScale;
            cDispPos.y = viewPos.y + (viewport[3] - 1) * 0.5 - ((cam.dsize[1] - 1) * 0.5 - cam.cy) * viewScale;

            const SP_REAL nx = z / cam.fx;
            const SP_REAL ny = z / cam.fy;

            const SP_REAL sw = (viewport[2] - 1) / viewScale;
            const SP_REAL sh = (viewport[3] - 1) / viewScale;

            const SP_REAL l = (-cDispPos.x / viewScale) * nx;
            const SP_REAL r = (-cDispPos.x / viewScale + sw) * nx;
            const SP_REAL t = (-cDispPos.y / viewScale) * ny;
            const SP_REAL b = (-cDispPos.y / viewScale + sh) * ny;
            const SP_REAL n = nearPlane;
            const SP_REAL f = farPlane;

            mat(0, 0) = 2 / (r - l);
            mat(1, 1) = 2 / (t - b);
            mat(2, 2) = 2 / (f - n);

            mat(0, 3) = -(r + l) / (r - l);
            mat(1, 3) = -(t + b) / (t - b);
            mat(2, 3) = -(f + n) / (f - n);

            mat(3, 3) = 1.0;
        }

        glMatrixMode(GL_PROJECTION);
        glLoadMatrix(mat);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
    }

    //--------------------------------------------------------------------------------
    // util
    //--------------------------------------------------------------------------------

    SP_CPUFUNC void glCircle(const Vec2 &pos, const SP_REAL radius) {
        glBegin(GL_LINE_LOOP);
        for (int i = 0; i < 36; i++) {
            const SP_REAL p = i / 36.0 * 2.0 * SP_PI;
            glVertex(getVec(pos.x + radius * sin(p), pos.y + radius * cos(p)));
        }
        glEnd();
    }

    SP_CPUFUNC void glRect(const Rect &rect, const bool fill = false) {
        if (rect.dim != 2) return;

        const Vec2 a = getVec(rect.dbase[0], rect.dbase[1]);
        const Vec2 b = getVec(rect.dbase[0] + rect.dsize[0] - 1, rect.dbase[1] + rect.dsize[1] - 1);

        glBegin((fill == true) ? GL_TRIANGLE_FAN : GL_LINE_LOOP);
        glVertex(getVec(a.x - 0.5, a.y - 0.5));
        glVertex(getVec(b.x + 0.5, a.y - 0.5));
        glVertex(getVec(b.x + 0.5, b.y + 0.5));
        glVertex(getVec(a.x - 0.5, b.y + 0.5));
        glEnd();
    }

    SP_CPUFUNC void glMesh(const Mesh2 &mesh) {
        glBegin(GL_TRIANGLES);
        glVertex(mesh.pos[0]);
        glVertex(mesh.pos[1]);
        glVertex(mesh.pos[2]);
        glEnd();
    }

    SP_CPUFUNC void glMesh(const Mesh3 &mesh){
        glBegin(GL_TRIANGLES);
        glNormal(getMeshNrm(mesh));
        glVertex(mesh.pos[0]);
        glVertex(mesh.pos[1]);
        glVertex(mesh.pos[2]);
        glEnd();
    }

    SP_CPUFUNC void glMesh(const Mesh3 &mesh, const Vec3 *nrms) {
        glBegin(GL_TRIANGLES);
        glNormal(nrms[0]);
        glVertex(mesh.pos[0]);
        glNormal(nrms[1]);
        glVertex(mesh.pos[1]);
        glNormal(nrms[2]);
        glVertex(mesh.pos[2]);
        glEnd();
    }

    SP_CPUFUNC void glAxis(const SP_REAL size){
        glBegin(GL_LINES);
        glColor3ub(255, 0, 0);
        glVertex3d(0.0, 0.0, 0.0); glVertex3d(size, 0.0, 0.0);

        glColor3ub(0, 255, 0);
        glVertex3d(0.0, 0.0, 0.0); glVertex3d(0.0, size, 0.0);

        glColor3ub(0, 0, 255);
        glVertex3d(0.0, 0.0, 0.0); glVertex3d(0.0, 0.0, size);
        glEnd();
    }

    SP_CPUFUNC void glCube(const SP_REAL size){
        glBegin(GL_QUADS);

        const SP_REAL s = size / 2.0;
        for(int i = 0; i < 2; i++){
            SP_REAL v = (i == 0) ? +1.0 : -1.0;
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

    SP_CPUFUNC void glGrid(const SP_REAL size, const int num){
        glBegin(GL_LINES);
        for (int i = 0; i < num; i++){
            const SP_REAL p = i * 2 * size / (num - 1);
            glVertex3d(-size, -size + p, 0.0);
            glVertex3d(+size, -size + p, 0.0);

            glVertex3d(-size + p, -size, 0.0);
            glVertex3d(-size + p, +size, 0.0);
        }
        glEnd();
    }

    SP_CPUFUNC void glCam(const CamParam &cam, const SP_REAL size) {
        const SP_REAL f = (cam.fx + cam.fy) / 2.0;
        const SP_REAL w = (cam.dsize[0] / 2.0) / f * size;
        const SP_REAL h = (cam.dsize[1] / 2.0) / f * size;

        const Vec2 loop[4] = { getVec(-w, -h), getVec(+w, -h), getVec(+w, +h), getVec(-w, +h) };
        glBegin(GL_LINES);
        for (int i = 0; i < 4; i++) {
            const Vec2 a = loop[(i + 0) % 4];
            const Vec2 b = loop[(i + 1) % 4];
            glVertex3d(0.0, 0.0, 0.0); glVertex3d(a.x, a.y, size);
            glVertex3d(a.x, a.y, size); glVertex3d(b.x, b.y, size);
        }
        glEnd();
    }

    SP_CPUFUNC void glLine(const Mem1<Vec2> &vtxs, const bool loop = false) {
        glBegin(GL_LINES);
        for (int i = 0; i < vtxs.size(); i++) {
            if (i == vtxs.size() - 1 && loop == false) break;
            glVertex(vtxs(i + 0, true));
            glVertex(vtxs(i + 1, true));
        }
        glEnd();
    }

    SP_CPUFUNC void glModel(const Mem1<Mesh3> &model, const Mem1<Vec3> nrms = Mem1<Vec3>()) {
        if (nrms.size() == 0) {
            for (int i = 0; i < model.size(); i++) {
                glMesh(model[i]);
            }
        }
        else {
            for (int i = 0; i < model.size(); i++) {
                glMesh(model[i], &nrms[i * 3]);
            }
        }
    }


    //--------------------------------------------------------------------------------
    // render
    //--------------------------------------------------------------------------------

    SP_CPUFUNC void glRenderSurface(const Mem1<Mesh3> &model, const Mem1<Vec3> nrms = Mem1<Vec3>()) {

        glPushAttrib(GL_ALL_ATTRIB_BITS);
        {
            glPushMatrix();
            glLoadIdentity();

            glEnable(GL_LIGHTING);
            glEnable(GL_LIGHT0);

            GLfloat lightPos[4] = { 0.f, 0.f, -1000.f, 1.f };
            glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
            glPopMatrix();

            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

            glModel(model, nrms);
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

    SP_CPUFUNC void glRenderVoxel(const Voxel &voxel) {

        glPushAttrib(GL_ALL_ATTRIB_BITS);
        {
            glPushMatrix();
            glLoadIdentity();

            glEnable(GL_LIGHTING);
            glEnable(GL_LIGHT0);

            GLfloat lightPos[4] = { 0.f, 0.f, -1000.f, 1.f };
            glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
            glPopMatrix();

            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

            const Vec3 cent = voxel.center();

            for (int z = 0; z < voxel.vmap.dsize[2]; z++) {
                for (int y = 0; y < voxel.vmap.dsize[1]; y++) {
                    for (int x = 0; x < voxel.vmap.dsize[0]; x++) {
                        const char &val = voxel.vmap(x, y, z);
                        if (val < 0) continue;

                        const Vec3 mpos = getVec(x, y, z);
                        const Vec3 cpos = ((mpos - cent) * voxel.unit);

                        glPushMatrix();
                        glMultMatrix(getPose(cpos));

                        glCube(voxel.unit);

                        glPopMatrix();
                    }
                }
            }
        }
        glPopAttrib();
    }

}

#endif
