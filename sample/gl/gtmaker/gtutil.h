#ifndef __GTUTIL__
#define __GTUTIL__

#include "simplesp.h"
#include "spex/spgl.h"
#include "spex/spcv.h"

#include "tinyfiledialogs.h"

using namespace sp;

#define ImGuiWindowFlags_Block (ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove)


class LabelInfo {

public:
	char name[SP_STRMAX];
	Mem<LabelInfo> child;

	LabelInfo() {
	}

	LabelInfo(const char *name) {
		sprintf(this->name, name);
	}

	LabelInfo(const LabelInfo &labelinfo) {
		copy(labelinfo);
	}

	LabelInfo& operator = (const LabelInfo &labelinfo) {
		copy(labelinfo);
		return *this;
	}

	void copy(const LabelInfo &labelinfo) {
		sprintf(this->name, labelinfo.name);
		child = labelinfo.child;
	}
};


class BaseMode {

public:
	static string m_imdir;
	static string m_gtdir;
	static Mem1<string> m_names;

	static const BaseWindow *m_parent;

	static void init(const BaseWindow *parent) {
		m_parent = parent;
	}

	static bool open() {
		const char *path = tinyfd_selectFolderDialog("select image folder", getCrntDir().c_str());
		if (path == NULL) return false;

		m_imdir = path;
		m_gtdir = getTimeStamp();
		mkdir(m_gtdir.c_str());

		m_names = getFileList(path, "bmp, BMP, png, PNG, jpeg, JPEG, jpg, JPG");

		for (int i = 0; i < m_names.size(); i++) {
			printf("%s\n", m_names[i].c_str());
		}
		return true;
	}

	static Mat m_vmat;
	static Mem2<Col3> m_img;

public:

	virtual void reset() {
	}

	virtual void select(const int id) {
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

string BaseMode::m_imdir;
string BaseMode::m_gtdir;
Mem1<string> BaseMode::m_names;

const BaseWindow *BaseMode::m_parent;

Mat BaseMode::m_vmat;

Mem2<Col3> BaseMode::m_img;

#endif