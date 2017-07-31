#include "simplesp.h"
#include "spex/spgl.h"

using namespace sp;

class RenderGUI : public BaseWindow{

	// camera
	CamParam m_cam;

	// image
	Mem2<Col3> m_img;

	// object mesh model
	Mem1<Mesh> m_model;

	// object surface points
	Mem1<VecVN3> m_pnts;

	// object to cam pose
	Pose m_pose;

private:

	void help() {
		printf("'d' key : render depth\n");
		printf("'n' key : render normal\n");
		printf("'ESC' key : exit\n");
		printf("\n");
	}

	virtual void init(){

		m_cam = getCamParam(640, 480);

		m_img.resize(m_cam.dsize);
		m_img.zero();

		help();
	
		if (loadBunny(m_model, SP_DATA_DIR "/stanford/bun_zipper.ply") == false){

			// if could not find stanford bunny, load dummy model
			loadGeodesicDorm(m_model, 100.0, 1);
		}
		
		m_pnts = getModelPoint(m_model);
		m_pose = getPose(getVec(0.0, 0.0, getModelDistance(m_model, m_cam)));

	}

	virtual void action() {

		if (m_keyAction == GLFW_KEY_D || m_keyAction == GLFW_KEY_N) {
			const double distance = getModelDistance(m_model, m_cam);
			const double radius = getModelRadius(m_model);

			Mem2<VecVN3> map;
			renderVecVN(map, m_cam, m_pose, m_model);

			if (m_keyAction == GLFW_KEY_D) {
				cnvDepthToImg(m_img, map, distance - 2 * radius, distance + 2 * radius);
			}
			if (m_keyAction == GLFW_KEY_N) {
				cnvNormalToImg(m_img, map, distance - 2 * radius, distance + 2 * radius);
			}
		}
	}

	virtual void display(){

		{
			// view 2D
			glLoadView2D(m_cam, m_viewPos, m_viewScale);
			glRenderImage(m_img);
		}

		{
			// view 3D
			glLoadView3D(m_cam, m_viewPos, m_viewScale);

			{
				glLoadIdentity();
				// glLoadMatrix(m_pose);

				glPointSize(3.f);
				glBegin(GL_POINTS);
				glColor3f(0.2f, 0.7f, 0.2f);
				for (int i = 0; i < m_pnts.size(); i++){
					// X_C = R * X_M + T
					glVertex(m_pose.rot * m_pnts[i].vtx + m_pose.trn);
					//glVertex(m_pnts[i].vtx);
				}
				glEnd();
			}

			// render axis
			{
				glLoadMatrix(m_pose);
				
				glLineWidth(2.f);
				glBegin(GL_LINES);
				glAxis(100.0);
				glEnd();
			}
			
			// render mesh model
			//{
			//	glLoadIdentity();

			//	GLfloat lightPos[4] = { 0.f, 0.f, -1500.f, 1.f };
			//	glEnable(GL_LIGHTING);
			//	glEnable(GL_LIGHT0);
			//	glLightfv(GL_LIGHT0, GL_POSITION, lightPos);

			//	const GLfloat diffuse[] = { 0.4f, 0.5f, 0.5f, 1.0f };
			//	glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse);

			//	glLoadMatrix(m_pose);

			//	glLineWidth(1.f);
			//	for (int i = 0; i < m_model.size(); i++){
			//		glBegin(GL_TRIANGLES);
			//		glNormal(getMeshNrm(m_model[i]));
			//		glMesh(m_model[i]);
			//		glEnd();
			//	}
			//}
		}

	}

	virtual void mousePos(double x, double y) {
		controlPose(m_pose, m_mouse, m_wcam, m_viewScale);
	}

	virtual void mouseScroll(double x, double y) {
		controlPose(m_pose, m_mouse, m_wcam, m_viewScale);
	}

};

