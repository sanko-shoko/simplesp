﻿#include "simplesp.h"
#include "spex/spgl.h"

using namespace sp;

class ModelTrackGUI : public BaseWindow{

	// camera
	CamParam m_cam;

	// image
	Mem2<Col3> m_img;

	// model
	Mem1<Mesh> m_model;

	// pose
	Pose m_pose;

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

		m_pose = getPose(getVec(0.0, 0.0, getModelDistance(m_model, m_cam)));
	}

	virtual void keyFun(int key, int scancode, int action, int mods) {

		if (m_keyAction[GLFW_KEY_D] == 1) {
			const double distance = getModelDistance(m_model, m_cam);
			const double radius = getModelRadius(m_model);

			Mem2<VecPN3> map;
			renderVecPN(map, m_cam, m_pose, m_model);

			cnvNormalToImg(m_img, map, distance - 2 * radius, distance + 2 * radius);
		}

		if (m_keyAction[GLFW_KEY_P] == 1) {
		}

		if (m_keyAction[GLFW_KEY_C] > 0) {
		}
	}

	virtual void display(){

		// render dataB
		{

			glLoadView2D(m_cam, m_viewPos, m_viewScale);
			glRenderImage(m_img);
		}

		// render dataA
		{
			glLoadView3D(m_cam, m_viewPos, m_viewScale);

			glClear(GL_DEPTH_BUFFER_BIT);
			{
				glLoadMatrix(m_pose);

				// render points
				glPointSize(3.f);
				glBegin(GL_POINTS);
				glColor3f(0.2f, 0.7f, 0.2f);
				//for (int i = 0; i < m_dataA.size(); i++){
				//	glVertex(m_dataA[i].pos);
				//}
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
		controlPose(m_pose, m_mouse, m_wcam, m_viewScale);
	}

	virtual void mouseScroll(double x, double y) {
		controlPose(m_pose, m_mouse, m_wcam, m_viewScale);
	}

};
