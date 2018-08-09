//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_GLUTIL_H__
#define __SP_GLUTIL_H__

#include "spcore/spcore.h"
#include "GLFW/glfw3.h"

namespace sp{

    //--------------------------------------------------------------------------------
    // overwrap
    //--------------------------------------------------------------------------------

    SP_CPUFUNC void glLoadMatrix(const Mat &mat){
        const Mat m4x4t = trnMat(extMat(4, 4, mat));
        glLoadMatrixd(m4x4t.ptr);
    }

    SP_CPUFUNC void glMultMatrix(const Mat &mat){
        const Mat m4x4t = trnMat(extMat(4, 4, mat));
        glMultMatrixd(m4x4t.ptr);
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

    //--------------------------------------------------------------------------------
    // get
    //--------------------------------------------------------------------------------

    SP_CPUFUNC Mat glGetMat(const int type) {
        Mat mat(4, 4);

        // GL_MODELVIEW_MATRIX, GL_PROJECTION_MATRIX
        glGetDoublev(type, mat.ptr);
        return trnMat(mat);
    }

    //--------------------------------------------------------------------------------
    // load view
    //--------------------------------------------------------------------------------
    
    SP_CPUFUNC Mat glGetViewMat(const int dsize0, const int dsize1, const Vec2 &viewPos = getVec(0.0, 0.0), const double viewScale = 1.0) {
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

        return vmat;
    }

    SP_CPUFUNC Mat glGetViewMat(const int *dsize, const Vec2 &viewPos = getVec(0.0, 0.0), const double viewScale = 1.0) {

        return glGetViewMat(dsize[0], dsize[1], viewPos, viewScale);
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

        glMatrixMode(GL_MODELVIEW);
        glLoadMatrix(vmat);
    }

    SP_CPUFUNC void glLoadView2D(const int *dsize, const Mat &vmat) {

        glLoadView2D(dsize[0], dsize[1], vmat);
    }

    SP_CPUFUNC void glLoadView2D(const int dsize0, const int dsize1, const Vec2 &viewPos = getVec(0.0, 0.0), const double viewScale = 1.0){

        glLoadView2D(dsize0, dsize1, glGetViewMat(dsize0, dsize1, viewPos, viewScale));
    }

    SP_CPUFUNC void glLoadView2D(const int *dsize, const Vec2 &viewPos = getVec(0.0, 0.0), const double viewScale = 1.0) {

        glLoadView2D(dsize, glGetViewMat(dsize, viewPos, viewScale));
    }

    SP_CPUFUNC void glLoadView2D(const CamParam &cam, const Vec2 &viewPos = getVec(0.0, 0.0), const double viewScale = 1.0){

        glLoadView2D(cam.dsize, glGetViewMat(cam.dsize, viewPos, viewScale));
    }

    SP_CPUFUNC void glLoadView3D(const CamParam &cam, const Vec2 &viewPos = getVec(0.0, 0.0), const double viewScale = 1.0, const double nearPlane = 1.0, const double farPlane = 10000.0){
        GLint viewport[4];
        glGetIntegerv(GL_VIEWPORT, viewport);

        glEnable(GL_DEPTH_TEST);

        Mat mat = zeroMat(4, 4);
        {
            Vec2 cDispPos;
            cDispPos.x = viewPos.x + (viewport[2] - 1) * 0.5 - ((cam.dsize[0] - 1) * 0.5 - cam.cx) * viewScale;
            cDispPos.y = viewPos.y + (viewport[3] - 1) * 0.5 - ((cam.dsize[1] - 1) * 0.5 - cam.cy) * viewScale;

            const double nx = nearPlane / cam.fx;
            const double ny = nearPlane / cam.fy;

            const double sw = (viewport[2] - 1) / viewScale;
            const double sh = (viewport[3] - 1) / viewScale;

            const double l = (-cDispPos.x / viewScale) * nx;
            const double r = (-cDispPos.x / viewScale + sw) * nx;
            const double t = (-cDispPos.y / viewScale) * ny;
            const double b = (-cDispPos.y / viewScale + sh) * ny;
            const double n = nearPlane;
            const double f = farPlane;

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


    //--------------------------------------------------------------------------------
    // util
    //--------------------------------------------------------------------------------

    SP_CPUFUNC void glCircle(const Vec2 &pos, const double radius) {
        for (int i = 0; i < 36; i++) {
            const double p0 = (i + 0) / 36.0 * 2.0 * SP_PI;
            const double p1 = (i + 1) / 36.0 * 2.0 * SP_PI;
            const Vec2 a = getVec(pos.x + radius * sin(p0), pos.y + radius * cos(p0));
            const Vec2 b = getVec(pos.x + radius * sin(p1), pos.y + radius * cos(p1));
            glVertex(a); glVertex(b);
        }
    }

    SP_CPUFUNC void glRect(const Rect &rect) {
        if (rect.dim == 2) {
            Vec2 pixA = getVec(rect.dbase[0], rect.dbase[1]);
            Vec2 pixB = getVec(rect.dbase[0] + rect.dsize[0] - 1, rect.dbase[1] + rect.dsize[1] - 1);

            glVertex(getVec(pixA.x - 0.5, pixA.y - 0.5));
            glVertex(getVec(pixB.x + 0.5, pixA.y - 0.5));

            glVertex(getVec(pixB.x + 0.5, pixA.y - 0.5));
            glVertex(getVec(pixB.x + 0.5, pixB.y + 0.5));

            glVertex(getVec(pixB.x + 0.5, pixB.y + 0.5));
            glVertex(getVec(pixA.x - 0.5, pixB.y + 0.5));

            glVertex(getVec(pixA.x - 0.5, pixB.y + 0.5));
            glVertex(getVec(pixA.x - 0.5, pixA.y - 0.5));
        }
    }

    SP_CPUFUNC void glMesh(const Mesh2 &mesh) {
        glVertex(mesh.pos[0]);
        glVertex(mesh.pos[1]);
        glVertex(mesh.pos[2]);
    }

    SP_CPUFUNC void glMesh(const Mesh3 &mesh){
        glNormal(getMeshNrm(mesh));
        glVertex(mesh.pos[0]);
        glVertex(mesh.pos[1]);
        glVertex(mesh.pos[2]);
    }

    SP_CPUFUNC void glAxis(const double size){
        glColor3ub(255, 0, 0);
        glVertex3d(0.0, 0.0, 0.0); glVertex3d(size, 0.0, 0.0);

        glColor3ub(0, 255, 0);
        glVertex3d(0.0, 0.0, 0.0); glVertex3d(0.0, size, 0.0);

        glColor3ub(0, 0, 255);
        glVertex3d(0.0, 0.0, 0.0); glVertex3d(0.0, 0.0, size);
    }

    SP_CPUFUNC void glCube(const double size){

        const double s = size / 2.0;
        for(int i = 0; i < 2; i++){
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
    }

    SP_CPUFUNC void glGrid(const double size, const int num){
        for (int i = 0; i < num; i++){
            const double p = i * 2 * size / (num - 1);
            glVertex3d(-size, -size + p, 0.0);
            glVertex3d(+size, -size + p, 0.0);

            glVertex3d(-size + p, -size, 0.0);
            glVertex3d(-size + p, +size, 0.0);
        }
    }

    SP_CPUFUNC void glCam(const CamParam &cam, const double size) {
        const double f = (cam.fx + cam.fy) / 2.0;
        const double w = (cam.dsize[0] / 2.0) / f * size;
        const double h = (cam.dsize[1] / 2.0) / f * size;

        const Vec2 loop[4] = { getVec(-w, -h), getVec(+w, -h), getVec(+w, +h), getVec(-w, +h) };
        for (int i = 0; i < 4; i++) {
            const Vec2 a = loop[(i + 0) % 4];
            const Vec2 b = loop[(i + 1) % 4];
            glVertex3d(0.0, 0.0, 0.0); glVertex3d(a.x, a.y, size);
            glVertex3d(a.x, a.y, size); glVertex3d(b.x, b.y, size);
        }
    }

    SP_CPUFUNC void glLine(const Mem1<Vec2> &vtxs, const bool loop = false) {
        for (int i = 0; i < vtxs.size(); i++) {
            if (i == vtxs.size() - 1 && loop == false) break;
            glVertex(vtxs(i + 0, true));
            glVertex(vtxs(i + 1, true));
        }
    }

    SP_CPUFUNC void glModel(const Mem1<Mesh3> &model) {
        for (int i = 0; i < model.size(); i++) {
            glMesh(model[i]);
        }
    }


    //--------------------------------------------------------------------------------
    // texture
    //--------------------------------------------------------------------------------

    template<typename TYPE>
    SP_CPUFUNC unsigned int getTextureId(const Mem<TYPE> &src) {
        int format;
        switch (sizeof(TYPE)) {
        case 1: format = GL_LUMINANCE; break;
        case 3: format = GL_RGB; break;
        default: return -1;
        }

        unsigned int texId;
        glGenTextures(1, &texId);

        glBindTexture(GL_TEXTURE_2D, texId);

        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, src.dsize[0], src.dsize[1], 0, format, GL_UNSIGNED_BYTE, src.ptr);

        return texId;
    }

    template<typename TYPE>
    SP_CPUFUNC void glRenderImg(const Mem<TYPE> &src) {
        if (src.size() == 0) return;

        const GLuint texId = getTextureId(src);
        if (texId < 0) return;

        glPushAttrib(GL_ENABLE_BIT);
        glEnable(GL_TEXTURE_2D);

        {
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

            glBindTexture(GL_TEXTURE_2D, texId);

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

            //glShadeModel(GL_FLAT);
            glColor3d(1.0, 1.0, 1.0);
            glColorMask(1, 1, 1, 1);

            const int w = src.dsize[0];
            const int h = src.dsize[1];

            glBegin(GL_QUADS);
            glTexCoord2i(0, 0); glVertex2d(0 - 0.5, 0 - 0.5);
            glTexCoord2i(0, 1); glVertex2d(0 - 0.5, h - 0.5);
            glTexCoord2i(1, 1); glVertex2d(w - 0.5, h - 0.5);
            glTexCoord2i(1, 0);    glVertex2d(w - 0.5, 0 - 0.5);
            glEnd();

            glBindTexture(GL_TEXTURE_2D, 0);
        }
        glPopAttrib();

        glDeleteTextures(1, &texId);
    }


    //--------------------------------------------------------------------------------
    // render
    //--------------------------------------------------------------------------------

    SP_CPUFUNC void glRenderSurface(const Mem1<Mesh3> &model) {

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

            glBegin(GL_TRIANGLES);
            glModel(model);
            glEnd();
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

                glBegin(GL_TRIANGLES);
                glModel(model);
                glEnd();
            }

            // draw outline
            {
                glStencilFunc(GL_NOTEQUAL, 1, 0xFFFF);
                glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                glLineWidth(2.0f);
                glColor3f(1.0f, 1.0f, 1.0f);

                glBegin(GL_TRIANGLES);
                glModel(model);
                glEnd();
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
                        if (val > 0) continue;

                        const Vec3 mpos = getVec(x, y, z);
                        const Vec3 cpos = ((mpos - cent) * voxel.unit);

                        glPushMatrix();
                        glMultMatrix(getPose(cpos));

                        glBegin(GL_QUADS);
                        glCube(voxel.unit);
                        glEnd();

                        glPopMatrix();
                    }
                }
            }
        }
        glPopAttrib();
    }

}

#endif
