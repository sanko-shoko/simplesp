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

	int m_mode;

private:

	void help() {
		printf("'d' key : render depth\n");
		printf("'n' key : render normal\n");
		printf("'m' key : switch render mode (->points ->meshes ->outline)\n");
		printf("'ESC' key : exit\n");
		printf("\n");
	}

	virtual void init(){

		help();

		m_mode = 0;

		m_cam = getCamParam(640, 480);

		m_img.resize(m_cam.dsize);
		m_img.zero();
	
		if (loadBunny(m_model, SP_DATA_DIR "/stanford/bun_zipper.ply") == false){

			// if could not find stanford bunny, load dummy model
			loadGeodesicDorm(m_model, 100.0, 1);
		}
		
		m_pnts = getModelPoint(m_model);
		m_pose = getPose(getVec(0.0, 0.0, getModelDistance(m_model, m_cam)));
	}

	virtual void keyFun(int key, int scancode, int action, int mods) {

		if (m_keyAction[GLFW_KEY_D] == 1|| m_keyAction[GLFW_KEY_N] == 1) {
			const double distance = getModelDistance(m_model, m_cam);
			const double radius = getModelRadius(m_model);

			Mem2<VecVN3> map;
			renderVecVN(map, m_cam, m_pose, m_model);

			if (m_keyAction[GLFW_KEY_D] == 1) {
				cnvDepthToImg(m_img, map, distance - 2 * radius, distance + 2 * radius);
			}
			if (m_keyAction[GLFW_KEY_N] == 1) {
				cnvNormalToImg(m_img, map, distance - 2 * radius, distance + 2 * radius);
			}
		}

		if (m_keyAction[GLFW_KEY_M] == 1) {
			if (++m_mode >= 3) m_mode = 0;
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

			switch (m_mode) {
			case 0:	renderPoints(); break;
			case 1: renderMeshes(); break;
			case 2: renderOutline(); break;
			default: break;
			}
			renderAxis();
		}
	}

	void renderPoints() {

		{
			glLoadIdentity();

			glPointSize(3.f);
			glBegin(GL_POINTS);
			glColor3f(0.2f, 0.7f, 0.2f);

			for (int i = 0; i < m_pnts.size(); i++) {
				// X_C = R * X_M + T
				glVertex(m_pose.rot * m_pnts[i].vtx + m_pose.trn);
			}
			glEnd();
		}

		{
			//glLoadMatrix(m_pose);

			//glPointSize(3.f);
			//glBegin(GL_POINTS);
			//glColor3f(0.2f, 0.7f, 0.2f);

			//for (int i = 0; i < m_pnts.size(); i++) {
			//	glVertex(m_pnts[i].vtx);
			//}
			//glEnd();
		}
	}

	void renderMeshes() {

		glPushAttrib(GL_ALL_ATTRIB_BITS);
		{
			glLoadIdentity();

			glEnable(GL_LIGHTING);
			glEnable(GL_LIGHT0);

			GLfloat lightPos[4] = { 0.f, 0.f, -1500.f, 1.f };
			glLightfv(GL_LIGHT0, GL_POSITION, lightPos);

			glLoadMatrix(m_pose);

			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

			const GLfloat diffuse[] = { 0.4f, 0.5f, 0.5f, 1.0f };
			glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse);

			glBegin(GL_TRIANGLES);
			for (int i = 0; i < m_model.size(); i++) {
				glNormal(getMeshNrm(m_model[i]));
				glMesh(m_model[i]);
			}
			glEnd();
		}
		glPopAttrib();

		glClear(GL_DEPTH_BUFFER_BIT);
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

	void renderAxis() {
		{
			glLoadMatrix(m_pose);

			glLineWidth(2.f);
			glBegin(GL_LINES);
			glAxis(100.0);
			glEnd();
		}
	}

	virtual void mousePos(double x, double y) {
		controlPose(m_pose, m_mouse, m_wcam, m_viewScale);
	}

	virtual void mouseScroll(double x, double y) {
		controlPose(m_pose, m_mouse, m_wcam, m_viewScale);
	}

};

