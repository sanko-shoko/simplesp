#include "simplesp.h"
#include "spex/spgl.h"

using namespace sp;

#define AXIS 0

class ActiveStereoGUI : public BaseWindow{

	// camera & projector param
	CamParam m_cam, m_prj;

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

	// GrayCode
	GrayCode m_graycode;
	PhaseShift m_phaseshift;

	int m_capture;

	bool m_view3d;

	// pnts
	Mem1<Vec3> m_pnts;

	struct {
		Mem2<Byte> wimg, bimg;
		Mem1<Mem2<Byte> > gcimgs;
		Mem1<Mem2<Byte> > psimgs;
	}m_imgSet;

private:

	void help() {
		printf("'a' key : capture simulation image\n");
		printf("'s' key : decode gray code\n");
		printf("'d' key : decode gray code + phase shift\n");
		printf("'f' key : switch view mode (2d <-> 3d)\n");
		printf("'ESC' key : exit\n");
		printf("\n");
	}

	virtual void init() {

		help();

		{
			m_cam = getCamParam(640, 480);
			
			m_img.resize(m_cam.dsize);
			m_img.zero();

			m_prj = getCamParam(320, 240);

			if (AXIS == 0) {
				m_cam2prj = getPose(getRotAngleY(-10.0 / 180.0 * SP_PI), getVec(100.0, 0.0, 0.0));
			}
			else {
				m_cam2prj = getPose(getRotAngleX(+10.0 / 180.0 * SP_PI), getVec(0.0, 100.0, 0.0));
			}
		
			m_graycode.init(m_prj.dsize[0], m_prj.dsize[1]);
			m_phaseshift.init(m_prj.dsize[0], m_prj.dsize[1]);
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

	virtual void keyFun(int key, int scancode, int action, int mods) {
		if (m_capture >= 0) return;

		if (m_keyAction[GLFW_KEY_A] == 1) {
			m_capture = 0;
		}
		if (m_keyAction[GLFW_KEY_S] == 1) {
			decodeGC();
		}
		if (m_keyAction[GLFW_KEY_D] == 1) {
			decodeGCPS();
		}
		if (m_keyAction[GLFW_KEY_F] == 1) {
			m_view3d ^= true;
		}
	}

	void calcPnt3d(const Mem2<double> &map) {
		m_img.zero();
		for (int i = 0; i < m_img.size(); i++) {
			if (map[i] < 0.0) continue;
			cnvPhaseToCol(m_img[i], map[i] / m_prj.dsize[AXIS]);
		}

		m_pnts.clear();
		for (int v = 0; v < map.dsize[1]; v++) {
			for (int u = 0; u < map.dsize[0]; u++) {
				if (map(u, v) < 0.0) continue;
				Vec3 pnt;
				if (AXIS == 0) {
					if (calcPnt3dX(pnt, zeroPose(), m_cam, getVec(u, v), m_cam2prj, m_prj, map(u, v)) == true) {
						m_pnts.push(pnt);
					}
				}
				else {
					if (calcPnt3dY(pnt, zeroPose(), m_cam, getVec(u, v), m_cam2prj, m_prj, map(u, v)) == true) {
						m_pnts.push(pnt);
					}
				}
			}
		}

	}

	void decodeGC() {
		if (m_imgSet.gcimgs.size() == 0) return;

		const Mem2<double> map = m_graycode.decode(AXIS, m_imgSet.gcimgs, m_imgSet.wimg, m_imgSet.bimg);
		
		calcPnt3d(map);
	}

	void decodeGCPS() {
		if (m_imgSet.gcimgs.size() == 0) return;

		const Mem2<double> gcmap = m_graycode.decode(AXIS, m_imgSet.gcimgs, m_imgSet.wimg, m_imgSet.bimg);
		const Mem2<double> psmap = m_phaseshift.decode(AXIS, m_imgSet.psimgs, m_imgSet.wimg, m_imgSet.bimg);
		const Mem2<double> map = m_phaseshift.refineGrayCode(AXIS, psmap, gcmap);

		calcPnt3d(map);
	}

	void capture() {
		static double pre = 0.0;
		if (static_cast<double>(clock() - pre) / CLOCKS_PER_SEC < 0.1) return;
		pre = clock();

		static Mem1<Mem2<Byte> > gcptns = m_graycode.encode(AXIS);
		static Mem1<Mem2<Byte> > psptns = m_phaseshift.encode(AXIS);
		m_imgSet.gcimgs.resize(gcptns.size());
		m_imgSet.psimgs.resize(psptns.size());

		int cnt = m_capture;

		if (cnt == 0){
			renderPattern(m_imgSet.wimg, m_cam, m_pose, m_model, m_cam2prj, m_prj, m_graycode.getPlain(255));
			cnvImg(m_img, m_imgSet.wimg);
			m_capture++;
			return;
		}
		cnt--;

		if (cnt == 0) {
			renderPattern(m_imgSet.bimg, m_cam, m_pose, m_model, m_cam2prj, m_prj, m_graycode.getPlain(0));
			cnvImg(m_img, m_imgSet.bimg);
			m_capture++;
			return;
		}
		cnt--;

		if (cnt >= 0 && cnt < gcptns.size()) {
			renderPattern(m_imgSet.gcimgs[cnt], m_cam, m_pose, m_model, m_cam2prj, m_prj, gcptns[cnt]);
			cnvImg(m_img, m_imgSet.gcimgs[cnt]);
			m_capture++;
			return;
		}
		cnt -= gcptns.size();

		if (cnt >= 0 && cnt < psptns.size()) {
			renderPattern(m_imgSet.psimgs[cnt], m_cam, m_pose, m_model, m_cam2prj, m_prj, psptns[cnt]);
			cnvImg(m_img, m_imgSet.psimgs[cnt]);
			m_capture++;
			return;
		}
		cnt -= psptns.size();

		m_capture = -1;
	}

	virtual void display() {
		if (m_capture >= 0) {
			capture();
		}

		if (m_view3d == false) {
			// view 2D
			glLoadView2D(m_cam, m_viewPos, m_viewScale);
			glRenderImage(m_img);

			// view 3D
			glLoadView3D(m_cam, m_viewPos, m_viewScale);
			renderOutline();
		}
		else {
			glLoadView3D(m_cam, m_viewPos, m_viewScale);

			// render points
			glLoadMatrix(m_view);

			glPointSize(1.f);
			glColor3d(0.5, 0.5, 1.0);
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
