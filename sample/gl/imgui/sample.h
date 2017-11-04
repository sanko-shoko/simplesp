#include "simplesp.h"
#include "spex/spgl.h"

using namespace sp;

class SampleGUI : public BaseWindow {

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

	virtual void initialize() {

		m_cam = getCamParam(640, 480);

		m_img.resize(m_cam.dsize);
		m_img.zero();

		help();

		if (loadBunny(m_model, SP_DATA_DIR "/stanford/bun_zipper.ply") == false) {

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

	virtual void display() {
		{
			// view 2D
			glLoadView2D(m_cam, m_viewPos, m_viewScale);
			glRenderImage(m_img);
		}

		bool show_test_window = true;
		bool show_another_window = false;
		ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
		// 1. Show a simple window.
		// Tip: if we don't call ImGui::Begin()/ImGui::End() the widgets appears in a window automatically called "Debug".
		{
			static float f = 0.0f;
			ImGui::Text("Hello, world!");
			ImGui::SliderFloat("float", &f, 0.0f, 1.0f);
			ImGui::ColorEdit3("clear color", (float*)&clear_color);
			if (ImGui::Button("Test Window")) show_test_window ^= 1;
			if (ImGui::Button("Another Window")) show_another_window ^= 1;
			ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		}

	}

	virtual void mousePos(double x, double y) {
		if (ImGui::GetIO().WantCaptureMouse == false) {
			controlPose(m_pose, m_mouse, m_wcam, m_viewScale);
		}
	}

	virtual void mouseScroll(double x, double y) {
		if (ImGui::GetIO().WantCaptureMouse == false) {
			controlPose(m_pose, m_mouse, m_wcam, m_viewScale);
		}
	}

};

