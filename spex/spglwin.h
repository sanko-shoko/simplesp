//--------------------------------------------------------------------------------
// Copyright (c) 2017, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SPGL_WIN_H__
#define __SPGL_WIN_H__

#if defined(_WIN32) && SP_USE_GLEW
#include "GL/glew.h"
#endif

#if SP_USE_IMGUI
#include "imgui.h"
#include "imgui_impl_glfw.h"
#endif

#include "simplesp.h"
#include "GLFW/glfw3.h"

namespace sp {

	//--------------------------------------------------------------------------------
	// mouse
	//--------------------------------------------------------------------------------

	struct Mouse {

		// cursor position and move
		Vec2 position, move;

		// scroll value
		double scroll;

		// button
		bool bDownL, bDownR, bDownM;

		Mouse() {
			memset(this, 0, sizeof(Mouse));
		}

		void setButton(const int button, const int action, const int mods) {
			bool state = (action) ? true : false;

			if (button == GLFW_MOUSE_BUTTON_LEFT) {
				bDownL = state;
			}
			if (button == GLFW_MOUSE_BUTTON_RIGHT) {
				bDownR = state;
			}
			if (button == GLFW_MOUSE_BUTTON_MIDDLE) {
				bDownM = state;
			}
		}

		void setPosition(const double x, const double y) {

			if (bDownL || bDownR || bDownM) {
				move = getVec(x, y) - position;
			}
			position = getVec(x, y);
		}

		void setScroll(const double x, const double y) {
			scroll = y;
		}
	};


	//--------------------------------------------------------------------------------
	// control
	//--------------------------------------------------------------------------------

	SP_CPUFUNC void controlView(Vec2 &viewpPos, double &viewScale, const Mouse &mouse) {
		if (mouse.bDownL) {
			viewpPos += mouse.move;
		}

		if (mouse.scroll != 0) {
			viewpPos *= (1.0 + mouse.scroll * 0.1);
			viewScale *= (1.0 + mouse.scroll * 0.1);
		}
	}

	SP_CPUFUNC void controlPose(Pose &pose, const Mouse &mouse, const CamParam &cam, const double viewScale, const Pose base = zeroPose()) {

		Pose cpose = pose * invPose(base);
		if (cpose.trn.z < 0.0) return;

		if (mouse.bDownM) {
			cpose.trn.x += mouse.move.x * cpose.trn.z / (cam.fx * viewScale);
			cpose.trn.y += mouse.move.y * cpose.trn.z / (cam.fy * viewScale);
		}

		if (mouse.bDownL) {
			cpose.rot = getRotAngle(getVec(+mouse.move.y, -mouse.move.x, 0.0), 0.01 * normVec(mouse.move)) * cpose.rot;
		}

		if (mouse.scroll != 0) {
			cpose.trn -= unitVec(cpose.trn) * (cpose.trn.z * mouse.scroll * 0.02);
		}

		pose = cpose * base;
	}


	//--------------------------------------------------------------------------------
	// base window
	//--------------------------------------------------------------------------------

	class BaseWindow{
	public:

		virtual void windowSize(int width, int height){
		}

		virtual void mouseButton(int button, int action, int mods){
		}

		virtual void mousePos(double x, double y){
		}

		virtual void mouseScroll(double x, double y){
		}

		virtual void keyFun(int key, int scancode, int action, int mods){
		}

		virtual void charFun(unsigned int charInfo){
		}

		virtual void init(){
		}

		virtual void display(){
		}

		virtual void action() {
		}

		virtual void drop(int num, const char **paths) {
		}

	protected:
		
		// view position
		Vec2 m_viewPos;

		// view scale
		double m_viewScale;

		// keybord state
		int m_keyState;

		// keybord action
		int m_keyAction;

		// mouse event class
		Mouse m_mouse;

		// window cam
		CamParam m_wcam;

	public:
		
		BaseWindow(){
			m_viewPos = getVec(0.0, 0.0);
			m_viewScale = 1.0;
			m_keyState = 0;
			m_keyAction = 0;
		}

		//--------------------------------------------------------------------------------
		// execute
		//--------------------------------------------------------------------------------

		void execute(const char *name, const int width, const int height){
			
			m_wcam = getCamParam(width, height);

			// glfw init
			SP_ASSERT(glfwInit());

#if defined(_WIN32) && SP_USE_GLEW
			// glew init
			SP_ASSERT(glewInit() != GLEW_OK);
#endif

			glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);

			// glfw create window
			GLFWwindow *window = glfwCreateWindow(width, height, name, NULL, NULL);
			if (!window){
				SP_PRINTF(" Can't create GLFW window.\n");
				glfwTerminate();
				return;
			}

			// glfw make context
			glfwMakeContextCurrent(window);

#if SP_USE_IMGUI
			// imgui init
			ImGui_ImplGlfwGL2_Init(window, true);
#endif

			// glfw set event callbacks
			setCallback(window);


			init();

			while (!glfwWindowShouldClose(window) && !glfwGetKey(window, GLFW_KEY_ESCAPE)){
				glfwPollEvents();

#if SP_USE_IMGUI
				ImGui_ImplGlfwGL2_NewFrame();
#endif

				glClearColor(0.10f, 0.15f, 0.15f, 1.0f);
				glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
				
				display();
				action();

				m_keyAction = 0;

#if SP_USE_IMGUI
				ImGui::Render();
#endif

				glfwSwapBuffers(window);
			}

			// glfw terminate
			glfwTerminate();

#if SP_USE_IMGUI
			ImGui_ImplGlfwGL2_Shutdown();
#endif
		}


	protected:

		//--------------------------------------------------------------------------------
		// base event process
		//--------------------------------------------------------------------------------
		
		bool _onGUI() {
#if SP_USE_IMGUI
			return ImGui::GetIO().WantCaptureMouse;
#endif
			return false;
		}

		void _windowSize(int width, int height){
			glViewport(0, 0, width, height);

			windowSize(width, height);

		}

		void _mouseButton(int button, int action, int mods){
			if (_onGUI() == true) return;

			m_mouse.setButton(button, action, mods);

			mouseButton(button, action, mods);
		}

		void _mousePos(double x, double y){
			if (_onGUI() == true) return;

			m_mouse.setPosition(x, y);

			// control view
			if (m_keyState == GLFW_KEY_SPACE){
  				controlView(m_viewPos, m_viewScale, m_mouse);
				return;
			}

			mousePos(x, y);
		}

		void _mouseScroll(double x, double y){
			if (_onGUI() == true) return;

			m_mouse.setScroll(x, y);

			// control view
			if (m_keyState == GLFW_KEY_SPACE){
				controlView(m_viewPos, m_viewScale, m_mouse);
				m_mouse.setScroll(0.0, 0.0);
				return;
			}

			mouseScroll(x, y);
			m_mouse.setScroll(0.0, 0.0);
		}

		void _keyFun(int key, int scancode, int action, int mods){
			m_keyState = (action >= 1) ? key : 0;
			m_keyAction = (action == 1) ? key : 0;


			keyFun(key, scancode, action, mods);
		}

		void _charFun(unsigned int charInfo){

			charFun(charInfo);
		}

		void _dropCB(int num, const char **paths) {
			drop(num, paths);
		}

	protected:

		//--------------------------------------------------------------------------------
		// callback function
		//--------------------------------------------------------------------------------

		void setCallback(GLFWwindow *window){
			// set my ptr
			glfwSetWindowUserPointer(window, this);

			glfwSetWindowSizeCallback(window, windowSizeCB);
			
			glfwSetMouseButtonCallback(window, mouseButtonCB);
			glfwSetCursorPosCallback(window, mousePosCB);
			glfwSetScrollCallback(window, mouseScrollCB);

			glfwSetKeyCallback(window, keyFunCB);
			glfwSetCharCallback(window, charFunCB);

			glfwSetDropCallback(window, dropCB);
		}

		static BaseWindow* getThisPtr(GLFWwindow *window){
			return static_cast<BaseWindow*>(glfwGetWindowUserPointer(window));
		}

		static void windowSizeCB(GLFWwindow *window, int width, int height){
			getThisPtr(window)->_windowSize(width, height);
		}

		static void mouseButtonCB(GLFWwindow *window, int button, int action, int mods){
			getThisPtr(window)->_mouseButton(button, action, mods);
		}

		static void mousePosCB(GLFWwindow *window, double x, double y){
			getThisPtr(window)->_mousePos(x, y);

		}
		static void mouseScrollCB(GLFWwindow *window, double x, double y){
			getThisPtr(window)->_mouseScroll(x, y);
		}

		static void keyFunCB(GLFWwindow* window, int key, int scancode, int action, int mods){
			getThisPtr(window)->_keyFun(key, scancode, action, mods);
		}
		static void charFunCB(GLFWwindow* window, unsigned int charInfo){
			getThisPtr(window)->_charFun(charInfo);
		}

		static void dropCB(GLFWwindow *window, int num, const char **paths) {
			getThisPtr(window)->_dropCB(num, paths);
		}
	};

}

#endif