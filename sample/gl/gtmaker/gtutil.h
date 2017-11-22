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
	static const BaseWindow *m_parent;

	static void init(const BaseWindow *parent) {
		m_parent = parent;
	}

	static string m_folder;
	static Mem1<string> m_names;

	static void open() {
		const char *path = tinyfd_selectFolderDialog("select image folder", NULL);
		if (path == NULL) return;

		m_folder = path;
		m_names = getFileList(path, "bmp, BMP, png, PNG, jpeg, JPEG, jpg, JPG");
	}

	static Mat m_vmat;
	static Mem2<Col3> m_img;

public:
	virtual void select(const int id) {
	}
	virtual void save() {
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

string BaseMode::m_folder;
Mem1<string> BaseMode::m_names;

Mat BaseMode::m_vmat;

Mem2<Col3> BaseMode::m_img;

#endif