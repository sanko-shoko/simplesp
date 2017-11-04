//--------------------------------------------------------------------------------
// Copyright (c) 2017, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SPTW_WIN_H__
#define __SPTW_WIN_H__

#include "spex/spgl.h"
#include "AntTweakBar.h"

namespace sp{

	//--------------------------------------------------------------------------------
	// Tw functions
	//--------------------------------------------------------------------------------

	template<class T, void (T::*FUNC)()>
	void TW_CALL __TwButtonCB(void *ptr){
		(static_cast<T*>(ptr)->*FUNC)();
	}
#define TwButtonCB(CLASS, FUNC) __TwButtonCB<CLASS, &CLASS::FUNC>


	static const TwStructMember __TW_TYPE_VEC2[] = {
		{ "x", TW_TYPE_DOUBLE, offsetof(Vec2, x), "" },
		{ "y", TW_TYPE_DOUBLE, offsetof(Vec2, y), "" }
	};
#define TW_TYPE_VEC2 TwDefineStruct(NULL, __TW_TYPE_VEC2, 2, sizeof(Vec2), NULL, NULL)


	static const TwStructMember __TW_TYPE_VEC3[] = {
		{ "x", TW_TYPE_DOUBLE, offsetof(Vec3, x), "" },
		{ "y", TW_TYPE_DOUBLE, offsetof(Vec3, y), "" },
		{ "z", TW_TYPE_DOUBLE, offsetof(Vec3, z), "" }
	};
#define TW_TYPE_VEC3 TwDefineStruct(NULL, __TW_TYPE_VEC3, 3, sizeof(Vec3), NULL, NULL)


	//--------------------------------------------------------------------------------
	// tw window
	//--------------------------------------------------------------------------------

	class TwWindow : public BaseWindow{
	public:

		virtual void windowSize(int width, int height){
			if (TwWindowSize(width, height)){
				return;
			}
		}

		virtual void mouseButton(int button, int action, int mods){
			if (TwEventMouseButtonGLFW(button, action)){
				return;
			}
		}

		virtual void mousePos(double x, double y){
			if (TwEventMousePosGLFW((int)x, (int)y)){
				return;
			}
		}

		virtual void mouseScroll(double x, double y){
			if (TwEventMouseWheelGLFW((int)y)){
				return;
			}
		}

		virtual void keyFun(int key, int scancode, int action, int mods){
			if (TwEventKeyGLFW(key, action)){
				return;
			}

		}

		virtual void charFun(unsigned int charInfo){
			if (TwEventCharGLFW(charInfo, 1)){
				return;
			}
		}

		virtual void init(){

			TwBar *bar = TwNewBar("display");
			TwDefine("display iconified = true");

			TwAddVarRW(bar, "viewPos", TW_TYPE_VEC2, &m_viewPos, "");
			TwAddVarRW(bar, "viewScale", TW_TYPE_DOUBLE, &m_viewScale, "");
		}

		virtual void display(){
		}

	public:

		//--------------------------------------------------------------------------------
		// main process
		//--------------------------------------------------------------------------------

		void execute(const char *name, const int width, const int height){
			
			m_wcam = getCamParam(width, height);

			// glfw init
			SP_ASSERT(glfwInit());

#if defined(_WIN32) && SP_USE_GLEW
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

			// set TW
			TwInit(TW_OPENGL, NULL);
			TwWindowSize(width, height);

			// set GLFW event callbacks
			setCallback(window);

			// initialize gui
			initialize();

			while (!glfwWindowShouldClose(window) && !glfwGetKey(window, GLFW_KEY_ESCAPE)){
				glClearColor(0.10f, 0.15f, 0.15f, 1.0f);
				glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
				
				display();

				TwDraw();

				glfwSwapBuffers(window);

				m_keyAction = 0;
				glfwPollEvents();
			}

			// terminate GLFW & TW
			TwTerminate();
			glfwTerminate();
		}

	};

}

#endif