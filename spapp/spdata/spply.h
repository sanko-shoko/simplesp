//--------------------------------------------------------------------------------
// Copyright (c) 2017, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_PLY_H__
#define __SP_PLY_H__

#include "spapp/spdata/spfile.h"
#include "spapp/spdata/spstr.h"

namespace sp{

	//--------------------------------------------------------------------------------
	// local method
	//--------------------------------------------------------------------------------
	
	using namespace std;

	namespace _ply{
		struct Element{
			int num;
			string name;
			Mem1<string> prop;

			Element(){}
			Element(const Element &elem){
				*this = elem;
			}

			void operator = (const Element &elem){
				num = elem.num;
				name = elem.name;
				prop = elem.prop;
			}
		};

		SP_CPUFUNC bool loadHeader(File &file, Mem1<Element> &elemList){
			elemList.clear();

			Element *pElem = NULL;

			char line[SP_STRMAX];
			while (file.gets(line)){
				Mem1<string> words = strSplit(line);
				
				if (words[0] == "end_header") break;

				if (words[0] == "element"){
					Element elem;

					elem.name = words[1];
					elem.num = atoi(words[2].c_str());

					elemList.push(elem);
					pElem = &elemList[elemList.size() - 1];
				}

				if (words[0] == "property" && pElem){
					pElem->prop.push(words[words.size() - 1]);
				}
			}

			for (int i = 0; i < elemList.size(); i++){
				const Element &elem = elemList[i];
				if (elem.num == 0 || elem.prop.size() == 0) return false;

				string prop;
				for (int p = 0; p < elem.prop.size(); p++){
					prop += elem.prop[p] + "";
				}

				SP_PRINTD("elem %d %s\n", elem.num, elem.name.c_str());
				SP_PRINTD("prop %s\n", prop.c_str());
			}

			return true;
		}

		SP_CPUFUNC Vec3 getVertex(const char *line, const Element &elem){
			Vec3 vec;

			const Mem1<string> words = strSplit(line);

			for (int p = 0; p < elem.prop.size(); p++){
				const double val = atof(words[p].c_str());

				if (elem.prop[p] == "x") vec.x = val;
				if (elem.prop[p] == "y") vec.y = val;
				if (elem.prop[p] == "z") vec.z = val;
			}

			return vec;
		}

		SP_CPUFUNC Mem1<int> getIndex(const char *line, const Element &elem){
			Mem1<int> index;

			const Mem1<string> words = strSplit(line);

			for (int i = 0; i < atoi(words[0].c_str()); i++){
				index.push(atoi(words[i + 1].c_str()));
			}

			return index;
		}
	}
	using namespace _ply;

	SP_CPUFUNC bool loadPLY(Mem1<Mesh> &meshes, const char *path){
		File file;
		if (file.open(path, "r") == false) return false;

		Mem1<Element> elemList;
		if (loadHeader(file, elemList) == false) return false;

		Mem1<Vec3> vList;
		Mem1<Mem1<int>> iList;

		for (int i = 0; i < elemList.size(); i++){
			const Element &elem = elemList[i];

			char line[SP_STRMAX];
			if (elem.name == "vertex"){
				for (int j = 0; j < elem.num; j++){
					file.gets(line);
					vList.push(getVertex(line, elem));
				}
			}
			if (elem.name == "face"){
				for (int j = 0; j < elem.num; j++){
					file.gets(line);
					iList.push(getIndex(line, elem));
				}
			}
		}

		const int num = static_cast<int>(iList.size());

		meshes.resize(num);
		for (int i = 0; i < num; i++){
			const Mem1<int> &index = iList[i];
			meshes[i] = getMesh(vList[index[0]], vList[index[1]], vList[index[2]]);
		}

		return true;
	}

	SP_CPUFUNC bool savePLY(const Mem1<Vec3> &pnts, const char *path){
		File file;
		if (file.open(path, "w") == false) return false;

		file.printf("ply\n");
		file.printf("format ascii 1.0\n");
		file.printf("element vertex %d\n", pnts.size());
		file.printf("property float x\n");
		file.printf("property float y\n");
		file.printf("property float z\n");
		file.printf("end_header\n");

		for (int i = 0; i < pnts.size(); i++){
			file.printf("%lf %lf %lf ", pnts[i].x, pnts[i].y, pnts[i].z);
			file.printf("\n");
		}

		return true;
	}

	SP_CPUFUNC bool savePLY(const Mem1<Vec3> &pnts, const Mem1<Col3> &cols, const char *path){
		File file;
		if (file.open(path, "w") == false) return false;
		if (cmpSize(pnts, cols) == false) return false;

		file.printf("ply\n");
		file.printf("format ascii 1.0\n");
		file.printf("element vertex %d\n", pnts.size());
		file.printf("property float x\n");
		file.printf("property float y\n");
		file.printf("property float z\n");
		file.printf("property uchar red\n");
		file.printf("property uchar green\n");
		file.printf("property uchar blue\n");
		file.printf("end_header\n");

		for (int i = 0; i < pnts.size(); i++){
			file.printf("%lf %lf %lf ", pnts[i].x, pnts[i].y, pnts[i].z);
			file.printf("%d %d %d ", cols[i].r, cols[i].g, cols[i].b);
			file.printf("\n");
		}

		return true;
	}
}

#endif