#include "simplesp.h"
#include "spex/spgl.h"

using namespace sp;

class GrayCodeGUI : public BaseWindow{

	// camera
	CamParam m_cam;

	// projector
	CamParam m_prj;

	// cam to prj pose
	Pose m_cam2prj;

	// image
	Mem2<Col3> m_img;

	// object mesh model
	Mem1<Mesh> m_model;

	// object to cam pose
	Pose m_pose;

	// view pose
	Pose m_view;

	// pnts
	Mem1<Vec3> m_pnts;

	// pattern
	Mem2<Byte> m_ptn;

	// GrayCode
	GrayCode m_graycode;

	int m_capture;

	bool m_view3d;

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

			m_prj.k1 = 0.2;
			m_prj.k2 = 0.1;
			m_prj.k3 = 0.1;
			m_prj.p1 = 0.1;
			m_prj.p2 = 0.1;

			m_cam2prj = getPose(getRotAngleY(- 10.0 / 180.0 * SP_PI), getVec(0.0, 100.0, 0.0));
		
			m_graycode.setSize(m_prj.dsize[0], m_prj.dsize[1]);
			m_capture = -1;
		}

		{
			if (loadBunny(m_model, SP_DATA_DIR "/stanford/bun_zipper.ply") == false) {

				// if could not find stanford bunny, load dummy model
				loadGeodesicDorm(m_model, 100.0, 1);
			}

			m_pose = getPose(getVec(0.0, 0.0, getModelDistance(m_model, m_cam)));

			m_view3d = false;
			m_view = zeroPose();
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
				const Mem2<double> map = m_graycode.decode(axis, imgs, wimg, bimg);
				m_img.zero();
				Mem2<Vec2> vmap;
				renderCrsp(vmap, m_cam, m_pose, m_model, m_prj, m_cam2prj);

				for (int i = 0; i < m_img.size(); i++) {
					if (map[i] < 0.0) continue;
					cnvPhaseToCol(m_img[i], map[i] / m_prj.dsize[0]);
				}

				Mem2<VecVN3> vnmap;
				renderVecVN(vnmap, m_cam, m_pose, m_model);

				m_pnts.clear();
				Mem1<double> diff;
				for (int v = 0; v < map.dsize[1]; v++) {
					for (int u = 0; u < map.dsize[0]; u++) {
						if (vmap(u, v).y < 0) continue;
						Vec3 pnt;
						if (calcPnt3dY(pnt, zeroPose(), m_cam, getVec(u, v), m_cam2prj, m_prj, vmap(u, v).y) == true) {
							m_pnts.push(pnt);
						}
						diff.push(::fabs(pnt.z - vnmap(u, v).vtx.z));
					}
				}

				printf("mean diff%lf\n", meanVal(diff));
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
		if (m_keyAction[GLFW_KEY_D] == 1) {
			m_view3d ^= true;
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


		if (m_view3d == false) {
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
		else {
			glLoadView3D(m_cam, m_viewPos, m_viewScale);

			// render points
			glLoadMatrix(m_view);

			glPointSize(1.f);
			glColor3d(1.0, 1.0, 1.0);
			glBegin(GL_POINTS);
			for(int i= 0 ; i < m_pnts.size(); i++){
				glVertex(m_pnts[i]);
			}
			glEnd();
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
				glColor4d(0.0, 0.0, 0.0, 0.0);

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
				glColor3d(1.0, 1.0, 1.0);

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

		if (m_view3d == false) {
			controlPose(m_pose, m_mouse, m_wcam, m_viewScale);
		}
		else {
			controlPose(m_view, m_mouse, m_wcam, m_viewScale, invPose(m_pose));
		}
	}

	virtual void mouseScroll(double x, double y) {
		if (m_capture >= 0) return;

		if (m_view3d == false) {
			controlPose(m_pose, m_mouse, m_wcam, m_viewScale);
		}
		else {
			controlPose(m_view, m_mouse, m_wcam, m_viewScale, invPose(m_pose));
		}

	}

};
