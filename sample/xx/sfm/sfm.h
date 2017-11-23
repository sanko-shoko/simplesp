#include "simplesp.h"
#include "spex/spgl.h"
#include "spex/spcv.h"

#include <cstdlib>

using namespace sp;

class SfMGUI : public BaseWindow {

	SfM m_sfm;

	// cam pose (object to cam pose)
	Pose m_pose;

	// axis pose (cam to axis pose)
	Pose m_axis;

	bool m_denoise;

private:

	void help() {
		printf("drag & drop images\n");
		printf("'u' key : update\n");
		printf("'c' key : clear\n");
		printf("'d' key : denoise\n");
		printf("'ESC' key : exit\n");
		printf("\n");
	}

	virtual void init() {
		help();
		
		const double distance = 5.0;
		m_pose = getPose(getVec(0.0, 0.0, +distance));
		m_axis = getPose(getVec(0.0, 0.0, -distance));

		m_denoise = false;
	}

	virtual void keyFun(int key, int scancode, int action, int mods) {

		if (m_keyAction[GLFW_KEY_U] == 1) {
			update();
		}
		if (m_keyAction[GLFW_KEY_C] == 1) {
			m_sfm.clear();
			std::system("cls");
			help();
		}
		if (m_keyAction[GLFW_KEY_D] == 1) {
			m_denoise ^= true;
		}
	}

	virtual void drop(int num, const char **paths) {

		static CamParam cam = getCamParam(0, 0);
		
		// set cam parameter
		for (int i = 0; i < num; i++) {
			if (checkFileExt(paths[i], "txt") == true) {
				SP_ASSERT(loadText(cam, paths[i]));
				break;
			}
		}

		// add img and cam
		for (int i = 0; i < num; i++) {
			printf("\raddData [%s]", progressBar(i, num));

			if (checkFileExt(paths[i], "txt") == false) {
				Mem2<Col3> img;
				SP_ASSERT(cvLoadImg(img, paths[i]));

				Mem2<Col3> tmp;
				const double scale = (600 * 2.0) / (img.dsize[0] + img.dsize[1]);
				if (scale < 1.0) {
					rescale<Col3, Byte>(tmp, img, scale, scale);
				}
				else {
					tmp = img;
				}
				m_sfm.addData(tmp, cam);
			}
		}
		printf("\n");

		update();
	}

	void update() {
		const int maxit = 20;
		for (int i = 0; i < maxit; i++) {
			printf("\r update [%s]", progressBar(i, maxit));
			m_sfm.update();
		}
		printf("\n");

		//// save image
		//for (int i = 0; i < m_sfm.m_views.size(); i++) {
		//	const SfM::ViewData &view = m_sfm.m_views[i];

		//	Mem2<Col3> img = view.img;
		//	for (int m = 0; m < view.fts.size(); m++) {
		//		if (view.index[m] < 0) continue;

		//		const Vec2 pix = view.fts[m].pix;
		//		renderPoint(img, pix, getCol(0, 255, 0), 4);
		//	}

		//	char str[SP_STRMAX];
		//	sprintf(str, "image%03d.bmp", i);
		//	saveBMP(img, str);
		//}
		//m_sfm.savePly("pnts.ply");
	}

	virtual void display() {

		// view 3D
		glLoadView3D(m_wcam, m_viewPos, m_viewScale);

		{
			glLoadMatrix(m_pose);

			glPointSize(4.f);
			glBegin(GL_POINTS);

			const int thresh = minVal(3, m_sfm.m_views.size() / 2);
			for (int i = 0; i < m_sfm.m_gpnts.size(); i++) {
				if (m_denoise == true && m_sfm.m_gpnts[i].index.size() < thresh) continue;

				glColor(m_sfm.m_gpnts[i].col);
				glVertex(m_sfm.m_gpnts[i].vtx);
			}
			glEnd();
		}

		// render cam
		{
			glLineWidth(2.f);
			glColor3d(0.5, 0.5,0.8);

			for (int i = 0; i < m_sfm.m_views.size(); i++) {
				if (m_sfm.m_views[i].valid == false) continue;
				glLoadMatrix(m_pose * invPose(m_sfm.m_views[i].pose));

				glBegin(GL_LINES);
				glCam(m_sfm.m_views[i].cam, 0.2);
				glEnd();
			}
		}

		// render axis
		{
			glLoadMatrix(invPose(m_axis) * m_pose.rot);

			glLineWidth(2.f);
			glBegin(GL_LINES);
			glAxis(1.0);
			glEnd();
		}
	}

	virtual void mousePos(double x, double y) {
		Pose delta = zeroPose();
		controlPose(delta, m_mouse, m_wcam, m_viewScale, m_axis);
		m_pose = delta * m_pose;
	}

	virtual void mouseScroll(double x, double y) {
		Pose delta = zeroPose();
		controlPose(delta, m_mouse, m_wcam, m_viewScale, m_axis);
		m_pose.trn.z += delta.trn.z;
		m_axis.trn.z -= delta.trn.z;
	}

};
