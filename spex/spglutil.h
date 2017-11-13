//--------------------------------------------------------------------------------
// Copyright (c) 2017, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SPGL_UTIL_H__
#define __SPGL_UTIL_H__

#if defined(_WIN32) && SP_USE_GLEW
#include "GL/glew.h"
#endif

#if SP_USE_IMGUI
#include "imgui.h"
#include "imgui_impl_glfw.h"
#endif

#include "simplesp.h"
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

	SP_CPUFUNC void glColor(const Col3 &col){
		glColor3ub(col.r, col.g, col.b);
	}

	SP_CPUFUNC void glColor(const int label){
		srand(maxVal(label, 0));
		Col3 col;
		cnvHSVToCol(col, getVec((randValUnif() + 1.0) * SP_PI, 1.0, 1.0));
		glColor(col);
	}


	//--------------------------------------------------------------------------------
	// load view
	//--------------------------------------------------------------------------------

	SP_CPUFUNC void glLoadView2D(const int dsize0, const int dsize1, const Vec2 &viewPos = getVec(0.0, 0.0), const double viewScale = 1.0){
		GLint viewport[4];
		glGetIntegerv(GL_VIEWPORT, viewport);

		glDisable(GL_DEPTH_TEST);
		glDisable(GL_LIGHTING);
		glDisable(GL_CULL_FACE);

		Mat mat = eyeMat(4, 4);
		{
			const Vec2 rectCenter = getVec(dsize0 - 1, dsize1 - 1) * 0.5;
			const Vec2 viewCenter = getVec(viewport[2] - 1, viewport[3] - 1) * 0.5;
			const Vec2 shift = (viewPos + viewCenter) - rectCenter * viewScale;

			mat(0, 0) = viewScale;
			mat(1, 1) = viewScale;
			mat(2, 2) = viewScale;

			mat(0, 3) = shift.x;
			mat(1, 3) = shift.y;
		}

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(-0.5, viewport[2] - 0.5, viewport[3] - 0.5, -0.5, -1.0, 1.0);

		glMatrixMode(GL_MODELVIEW);
		glLoadMatrix(mat);
	}

	SP_CPUFUNC void glLoadView2D(const CamParam &cam, const Vec2 &viewPos = getVec(0.0, 0.0), const double viewScale = 1.0){

		glLoadView2D(cam.dsize[0], cam.dsize[1], viewPos, viewScale);
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
	// get view pos
	//--------------------------------------------------------------------------------

	SP_CPUFUNC Vec2 glGetViewPos(const double x, const double y, const int dsize0, const int dsize1, const Vec2 &viewPos = getVec(0.0, 0.0), const double viewScale = 1.0) {
		GLint viewport[4];
		glGetIntegerv(GL_VIEWPORT, viewport);

		Vec2 dst = getVec(x, y);
		{
			const Vec2 rectCenter = getVec(dsize0 - 1, dsize1 - 1) * 0.5;
			const Vec2 viewCenter = getVec(viewport[2] - 1, viewport[3] - 1) * 0.5;
			const Vec2 shift = (viewPos + viewCenter) - rectCenter * viewScale;

			dst -= shift;
			dst /= viewScale;
		}
		return dst;
	}

	SP_CPUFUNC Vec2 glGetViewPos(const double x, const double y, const CamParam &cam, const Vec2 &viewPos = getVec(0.0, 0.0), const double viewScale = 1.0) {

		return glGetViewPos(x, y, cam.dsize[0], cam.dsize[1], viewPos, viewScale);
	}


	//--------------------------------------------------------------------------------
	// texture
	//--------------------------------------------------------------------------------

	template<typename TYPE>
	SP_CPUFUNC unsigned int glLoadTexture(const Mem<TYPE> &src) {
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
	SP_CPUFUNC void glRenderImage(const Mem<TYPE> &src){
		if (src.size() == 0) return;

		const unsigned int texId = glLoadTexture(src);
		if (texId < 0) return;

		glPushAttrib(GL_ENABLE_BIT);
		glEnable(GL_TEXTURE_2D);
		{
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
			glTexCoord2i(1, 0);	glVertex2d(w - 0.5, 0 - 0.5);
			glEnd();

			glBindTexture(GL_TEXTURE_2D, 0);
		}
		glPopAttrib();

		glDeleteTextures(1, &texId);
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

	SP_CPUFUNC void glMesh(const Mesh &mesh){
		glVertex(mesh.vtx[0]);
		glVertex(mesh.vtx[1]);
		glVertex(mesh.vtx[2]);
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
		const Vec2 loop[4] = { getVec(-s, -s), getVec(+s, -s), getVec(+s, +s), getVec(-s, +s) };
		for (int i = 0; i < 4; i++){
			const Vec2 a = loop[(i + 0) % 4];
			const Vec2 b = loop[(i + 1) % 4];

			glVertex3d(a.x, a.y, -s); glVertex3d(b.x, b.y, -s);
			glVertex3d(a.x, a.y, +s); glVertex3d(b.x, b.y, +s);
			glVertex3d(a.x, a.y, -s); glVertex3d(a.x, a.y, +s);
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

}

#endif