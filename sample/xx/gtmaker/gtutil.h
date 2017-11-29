#ifndef __GTUTIL__
#define __GTUTIL__

#include "simplesp.h"
#include "spex/spgl.h"
#include "spex/spcv.h"

#include "tinyfiledialogs.h"

using namespace sp;

struct RectGT {
	Rect rect;
	int label;
};

class DataBase {

public:
	// work directory
	string wkDir;

	// image directory
	string imDir;

	// image names
	Mem1<string> imNames;

	// gt names;
	Mem1<string> gtNames;

	// gt rects list
	Mem1<MemP<RectGT>> gtsList;

public:

	DataBase() {
		wkDir = getTimeStamp();
	}

	bool open_imDir() {

		const char *path = tinyfd_selectFolderDialog("open image dir", getCrntDir().c_str());
		if (path == NULL) return false;

		imDir = path;
		imNames = getFileList(path, "bmp, BMP, png, PNG, jpeg, JPEG, jpg, JPG");
		
		gtsList.resize(imNames.size());

		if (imNames.size() == 0) {
			printf("no image in the directory");
			return false;
		}
		else {
			for (int i = 0; i < imNames.size(); i++) {
				printf("%06d %s\n", i, imNames[i].c_str());
			}
			return true;
		}
	}

	bool open_wkDir() {

		const char *path = tinyfd_selectFolderDialog("open work dir", getCrntDir().c_str());
		if (path == NULL) return false;

		wkDir = path;

		gtNames.clear();

		{
			File file;
			if (file.open((wkDir + "\\label_names.csv").c_str(), "r") == false) return false;

			file.scanf("index, name, \n");

			char str[SP_STRMAX];
			while (file.gets(str) == true) {
				const Mem1<string> split = strSplit(str, ",");
				if (split.size() < 2) break;

				gtNames.push(strTrim(split[1].c_str(), " "));
			}
		}

		{
			for (int i = 0; i < imNames.size(); i++) {
				MemP<RectGT> &gts = gtsList[i];
				gts.clear();

				File file;
				if (file.open((wkDir + "\\rect\\" + imNames[i] + ".csv").c_str(), "r") == false) continue;

				file.scanf("index, label, x, y, width, height, \n");

				char str[SP_STRMAX];
				while (file.gets(str) == true) {
					const Mem1<string> split = strSplit(str, ",");
					if (split.size() < 6) break;

					int buf;
					RectGT &gt = *gts.malloc();
					gt.rect.dim = 2;
					sscanf(str, "%d, %d, %d, %d, %d, %d, \n", &buf, &gt.label, &gt.rect.dbase[0], &gt.rect.dbase[1], &gt.rect.dsize[0], &gt.rect.dsize[1]);
				}
			}
		}
		return true;
	}

	void updateLabel(const int id, const int val) {
		for (int i = 0; i < gtsList.size(); i++) {
			MemP<RectGT> &gts = gtsList[i];

			for (int j = 0; j < gts.size(); j++) {
				RectGT &gt = gts[j];
				if (gt.label == id && val < 0) {
					gt.label = -1;
					continue;
				}
				if (gt.label >= id) {
					gt.label += val;
				}
			}
		}

	}

	void save() {

		{
			mkdir(wkDir.c_str());

			File file;
			SP_ASSERT(file.open((wkDir + "\\label_names.csv").c_str(), "w"));

			file.printf("index, name, \n");

			for (int i = 0; i < gtNames.size(); i++) {
				file.printf("%d, %s, \n", i, gtNames[i].c_str());
			}
		}


		{
			mkdir((wkDir + "\\rect").c_str());
			for (int i = 0; i < gtsList.size(); i++) {

				MemP<RectGT> &gts = gtsList[i];
				if (gts.size() == 0) continue;

				File file;
				SP_ASSERT(file.open((wkDir + "\\rect\\" + imNames[i] + ".csv").c_str(), "w"));

				file.printf("index, label, x, y, width, height, \n");

				for (int j = 0; j < gts.size(); j++) {
					const RectGT &gt = gts[j];
					file.printf("%d, %d, %d, %d, %d, %d, \n", j, gt.label, gt.rect.dbase[0], gt.rect.dbase[1], gt.rect.dsize[0], gt.rect.dsize[1]);
				}
			}
		}
	}



};


#endif