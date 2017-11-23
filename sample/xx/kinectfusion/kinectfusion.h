#include "simplesp.h"
#include "spex/spgl.h"
#include "spex/sprs.h"

using namespace sp;

class KinectFusionGUI : public BaseWindow{

	// realsense 
	RealSense m_rs;

	// Kinect Fusion
	KinectFusion m_kfusion;

	// start flag
	bool m_start;

private:

	void help() {
		printf("'s' key : start kinect fusion\n");
		printf("'r' key : reset kinect fusion\n");
		printf("'ESC' key : exit\n");
		printf("\n");
	}

	virtual void init(){
		m_start = false;

		m_rs.init(0);
		m_rs.enableDepth();

		help();
	}

	virtual void action() {
		if (m_keyAction[GLFW_KEY_S] == 1) {
			m_start = true;
		}
		if (m_keyAction[GLFW_KEY_R] == 1) {
			m_kfusion.reset();
			m_start = false;
		}
	}

	virtual void display() {

		const double nearPlane = 100;
		const double farPlane = 2000;

		m_rs.capture();
		//{
		//	static int id = 0;
		//	if (m_rs.testLoad("data", id++) == false){
		//		id = 0;
		//	}
		//}
		const Mem2<double> *depth = m_rs.getDepth();
		if (depth == NULL) return;

		// make min data for computational cost
		CamParam minCam;
		Mem2<double> minDepth;
		{
			pyrdown(minCam, *m_rs.getDepthCam());
			pyrdownDepth(minDepth, *depth);
		}

		// kinect fusion
		{
			m_kfusion.setCam(minCam);

			if (m_start == false) {
				Mem1<double> list;
				list.reserve(minDepth.size());
				for (int i = 0; i < minDepth.size(); i++) {
					if (minDepth[i] > 0.0) list.push(minDepth[i]);
				}

				const double mean = meanVal(list);

				m_kfusion.setMap(120, mean / 100.0, getPose(getVec(0.0, 0.0, mean)));
			}
			else {
				m_kfusion.execute(minDepth);
			}
		}

		// render
		{
			Mem2<Col3> view;
			if (m_start == false) {
				cnvDepthToImg(view, minDepth, nearPlane, farPlane);
			}
			else {
				cnvNormalToImg(view, m_kfusion.getCast(), nearPlane, farPlane);
			}

			glLoadView2D(m_kfusion.getCam(), m_viewPos, m_viewScale * 2.0);
			glRenderImage(view);

			glLoadView3D(m_kfusion.getCam(), m_viewPos, m_viewScale * 2.0);
			glLoadMatrix(*m_kfusion.getPose());

			glBegin(GL_LINES);
			glCube(m_kfusion.getCubeLength());
			glEnd();
		}
	}


};
