#ifndef __GTUTIL__
#define __GTUTIL__

#include "simplesp.h"
#include "spex/spgl.h"
#include "spex/spcv.h"

#include "tinyfiledialogs.h"

using namespace sp;

#define ImGuiWindowFlags_Block (ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove)

class BaseMode {

public:
	static const BaseWindow *m_parent;
	
	static string m_imDir;
	static Mem1<string> m_imNames;

	static string m_gtDir;
	static Mem1<string> m_gtNames;

	static int m_selectid;

	static Mat m_vmat;
	static Mem2<Col3> m_img;

	static void init(const BaseWindow *parent) {
		m_parent = parent;
		m_selectid = -1;
	}

	static bool open() {

		const char *path = tinyfd_selectFolderDialog("select image folder", getCrntDir().c_str());
		if (path == NULL) return false;

		m_imNames = getFileList(path, "bmp, BMP, png, PNG, jpeg, JPEG, jpg, JPG");
		if (m_imNames.size() == 0) {
			printf("no image in the directory");
			return false;
		}
		else {
			for (int i = 0; i < m_imNames.size(); i++) {
				printf("%06d %s\n", i, m_imNames[i].c_str());
			}

			m_imDir = path;
			m_gtDir = getTimeStamp();

			m_selectid = -1;

			return true;
		}
	}

public:

	virtual void reset() {
	}

	virtual bool select(const int id) {
		return false;
	}

	virtual void save() {
	}

	virtual void load() {
	}

	virtual void menu(const char *name) {
	}

	virtual void display() {
	}

	virtual void windowSize(int width, int height) {
	}

	virtual void mouseButton(int button, int action, int mods) {
	}

	virtual void mousePos(double x, double y) {
	}

	virtual void mouseScroll(double x, double y) {
	}

	virtual void keyFun(int key, int scancode, int action, int mods) {
	}

	virtual void charFun(unsigned int charInfo) {
	}

};

const BaseWindow *BaseMode::m_parent;

string BaseMode::m_imDir;
string BaseMode::m_gtDir;
Mem1<string> BaseMode::m_imNames;
Mem1<string> BaseMode::m_gtNames;

int BaseMode::m_selectid;

Mat BaseMode::m_vmat;

Mem2<Col3> BaseMode::m_img;

#endif