#include "simplesp.h"
#include "spex/spgl.h"

using namespace sp;

class ICPGUI : public BaseWindow{

	// camera
	CamParam m_cam;

	// image
	Mem2<Col3> m_img;

	// model
	Mem1<Mesh> m_model;

	// data A
	Pose m_poseA;
	Mem<VecVN3> m_dataA;

	// datd B
	Mem<VecVN3> m_dataB;

private:

	void help() {
		printf("dataA (point cloud) controlled by mouse\n");
		printf("'p' key : render dataB (point cloud)\n");
		printf("'d' key : render dataB (depth map)\n");
		printf("'c' key : calc ICP (dataA <-> dataB)\n");
		printf("'ESC' key : exit\n");
		printf("\n");
	}

	virtual void init(){
		help();

		m_cam = getCamParam(640, 480);

		if (loadBunny(m_model, SP_DATA_DIR "/stanford/bun_zipper.ply") == false){

			// if could not find stanford bunny, load dummy model
			loadGeodesicDorm(m_model, 100.0, 1);
		}

		m_poseA = getPose(getVec(0.0, 0.0, getModelDistance(m_model, m_cam)));
		m_dataA = getModelPoint(m_model);
	}

	virtual void keyFun(int key, int scancode, int action, int mods) {

		if (m_keyAction[GLFW_KEY_D] == 1) {
			m_dataB.resize(2, m_cam.dsize);

			m_dataB.zero();
			renderVecVN(m_dataB, m_cam, m_poseA, m_model);

			const double distance = getModelDistance(m_model, m_cam);
			const double radius = getModelRadius(m_model);
			cnvDepthToImg(m_img, m_dataB, distance - 2 * radius, distance + 2 * radius);
		}

		if (m_keyAction[GLFW_KEY_P] == 1) {
			const int dsize[1] = { m_dataA.size() };
			m_dataB.resize(1, dsize);

			for (int i = 0; i < m_dataA.size(); i++) {
				m_dataB[i] = m_poseA * m_dataA[i];
			}
		}

		if (m_keyAction[GLFW_KEY_C] > 0) {

			// point to point
			if (m_dataB.dim == 1) {
				calcICP(m_poseA, m_dataA, m_dataB, 1);
			}

			// point to 2d map
			else if (m_dataB.dim == 2) {
				calcICP(m_poseA, m_cam, m_dataA, m_dataB, 1);
			}
		}
	}

	virtual void display(){

		// render dataB
		{
			if (m_dataB.dim == 1) {
				glLoadView3D(m_cam, m_viewPos, m_viewScale);
				glPointSize(5.f);
				glBegin(GL_POINTS);
				glColor3f(0.2f, 0.2f, 0.7f);
				for (int i = 0; i < m_dataB.size(); i++) {
					glVertex(m_dataB[i].vtx);
				}
				glEnd();
			}

			if (m_dataB.dim == 2) {
				glLoadView2D(m_cam, m_viewPos, m_viewScale);
				glRenderImage(m_img);
			}
		}

		// render dataA
		{
			glLoadView3D(m_cam, m_viewPos, m_viewScale);

			glClear(GL_DEPTH_BUFFER_BIT);
			{
				glLoadMatrix(m_poseA);

				// render points
				glPointSize(3.f);
				glBegin(GL_POINTS);
				glColor3f(0.2f, 0.7f, 0.2f);
				for (int i = 0; i < m_dataA.size(); i++){
					glVertex(m_dataA[i].vtx);
				}
				glEnd();
				
				// render axis
				glLineWidth(2.f);
				glBegin(GL_LINES);
				glAxis(100.0);
				glEnd();
			}
		}

	}

	virtual void mousePos(double x, double y) {
		controlPose(m_poseA, m_mouse, m_wcam, m_viewScale);
	}

	virtual void mouseScroll(double x, double y) {
		controlPose(m_poseA, m_mouse, m_wcam, m_viewScale);
	}

};
