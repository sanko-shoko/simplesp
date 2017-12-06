#include "simplesp.h"
#include "spex/spgl.h"

using namespace sp;

class GrayCodeGUI : public BaseWindow{

	// camera
	CamParam m_cam;

	// image
	Mem2<Col3> m_img;

	// object mesh model
	Mem1<Mesh> m_model;

	// object to cam pose
	Pose m_pose;

	// projector
	CamParam m_prj;

	// cam to prj pose
	Pose m_cam2prj;

	// pattern
	Mem2<Byte> m_ptn;

	// GrayCode
	GrayCode m_graycode;

	int m_capture;


private:

	void help() {
		printf("'ESC' key : exit\n");
		printf("\n");
	}

	virtual void init() {

		help();

		{
			m_cam = getCamParam(640, 480);

			m_img.resize(m_cam.dsize);
			m_img.zero();
		}

		{
			m_prj = getCamParam(320, 240);

			m_ptn.resize(m_prj.dsize);
			m_ptn.zero();

			for (int v = 0; v < m_ptn.dsize[1]; v++) {
				for (int u = 0; u < m_ptn.dsize[0]; u++) {
					m_ptn(u, v) = ((v / 5) % 2 == 0) ? 255 : 0;
				}
			}

			m_cam2prj = getPose(getRotAngleY(- 10.0 / 180.0 * SP_PI), getVec(100.0, 0.0, 0.0));
		}

		{
			if (loadBunny(m_model, SP_DATA_DIR "/stanford/bun_zipper.ply") == false) {

				// if could not find stanford bunny, load dummy model
				loadGeodesicDorm(m_model, 100.0, 1);
			}

			m_pose = getPose(getVec(0.0, 0.0, getModelDistance(m_model, m_cam)));

		}
		{
			m_graycode.setSize(m_prj.dsize[0], m_prj.dsize[1]);
			m_capture = -1;
		}
	}

	void capture() {
		const int axis = 0;

		static Mem1<Mem2<Byte> > ptns;
		static Mem2<Byte> wimg, bimg;
		static Mem1<Mem2<Byte> > imgs;

		if (m_capture == 0){
			renderPattern(wimg, m_cam, m_pose, m_model, m_prj, m_cam2prj, m_graycode.getPlain(255));
			cnvImg(m_img, wimg);
			m_capture++;
			return;
		}
		if (m_capture == 1) {
			renderPattern(bimg, m_cam, m_pose, m_model, m_prj, m_cam2prj, m_graycode.getPlain(0));
			cnvImg(m_img, bimg);
			m_capture++;
			return;
		}

		if (m_capture >= 2) {
			const int c = m_capture - 2;
			if (c == 0) {
				ptns = m_graycode.encode(axis);
				imgs.resize(ptns.size());
			}

			if (c < imgs.size()){
				renderPattern(imgs[c], m_cam, m_pose, m_model, m_prj, m_cam2prj, ptns[c]);
				cnvImg(m_img, imgs[c]);
				m_capture++;
			}
			else {
				const Mem2<int> map = m_graycode.decode(axis, imgs, wimg, bimg);
				m_img.zero();

				for (int i = 0; i < m_img.size(); i++) {
					if (map[i] < 0) continue;
					cnvPhaseToCol(m_img[i], static_cast<double>(map[i]) / m_prj.dsize[0]);
				}

				m_capture = -1;
			}
			return;
		}

	}

	virtual void keyFun(int key, int scancode, int action, int mods) {
		if (m_keyAction[GLFW_KEY_A] == 1) {
			//renderImage(m_img, m_cam, m_pose, m_model);
			m_img /= 2;
		}
		if (m_keyAction[GLFW_KEY_S] == 1) {
			//renderPattern(m_img, m_cam, m_pose, m_model, m_prj, m_cam2prj, m_ptn);
			//m_img /= 2;
		}
		if (m_keyAction[GLFW_KEY_C] == 1) {
			m_capture = 0;
		}
	}

	virtual void display() {
		if (m_capture >= 0) {
			static double c = 0.0;
			if (static_cast<double>(clock() - c) / CLOCKS_PER_SEC > 0.1) {
				capture();
				c = clock();
			}
		}

		{
			// view 2D
			glLoadView2D(m_cam, m_viewPos, m_viewScale);
			glRenderImage(m_img);
		}

		{
			// view 3D
			glLoadView3D(m_cam, m_viewPos, m_viewScale);
			renderOutline();
		}
	}

	void renderOutline() {

		glPushAttrib(GL_ALL_ATTRIB_BITS);
		{
			glLoadMatrix(m_pose);

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
				for (int i = 0; i < m_model.size(); i++) {
					glMesh(m_model[i]);
				}
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
				for (int i = 0; i < m_model.size(); i++) {
					glMesh(m_model[i]);
				}
				glEnd();
			}
		}
		glPopAttrib();

		glClear(GL_DEPTH_BUFFER_BIT);
	}

	virtual void mousePos(double x, double y) {
		if (m_capture >= 0) return;

		controlPose(m_pose, m_mouse, m_wcam, m_viewScale);
	}

	virtual void mouseScroll(double x, double y) {
		if (m_capture >= 0) return;

		controlPose(m_pose, m_mouse, m_wcam, m_viewScale);
	}

};
