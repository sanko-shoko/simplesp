//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_MODEL_H__
#define __SP_MODEL_H__

#include "spcore/spcore.h"
#include "spapp/spimg/sprender.h"

namespace sp{
	
	//--------------------------------------------------------------------------------
	// model util
	//--------------------------------------------------------------------------------

	SP_CPUFUNC Vec3 getModelCenter(const Mem1<Mesh> &model){
		Vec3 sum = zero<Vec3>();
		for (int i = 0; i < model.size(); i++){
			sum += getMeshPos(model[i]);
		}

		return sum / model.size();
	}

	SP_CPUFUNC double getModelRadius(const Mem1<Mesh> &model){
		Mem1<double> mem(model.size());
		for (int i = 0; i < mem.size(); i++){
			mem[i] = normVec(getMeshPos(model[i]));
		}

		return maxVal(mem);
	}

	SP_CPUFUNC double getModelDistance(const Mem1<Mesh> &model, const CamParam &cam){

		const double radius = getModelRadius(model);
		const double distance = 1.5 * maxVal(cam.fx, cam.fy) * radius / (0.5 * minVal(cam.dsize[0], cam.dsize[1]));
	
		return distance;
	}

	SP_CPUFUNC Mem1<VecVN3> getModelPoint(const Mem1<Mesh> &model, const int density = 60){
		const CamParam cam = getCamParam(density, density);
		const double distance = getModelDistance(model, cam);
		
		Mem1<VecVN3> tmp;
		const int num = getGeodesicMeshNum(0);
		for (int i = 0; i < num; i++){
			const Vec3 v = getMeshPos(getGeodesicMesh(0, i)) * (-1.0);
			const Pose pose = getPose(getRotDirection(v), getVec(0.0, 0.0, distance));
			Mem2<VecVN3> map;
			renderVecVN(map, cam, pose, model);

			const Mat mat = getMat(invPose(pose));
			for (int j = 0; j < map.size(); j++){
				if (map[j].vtx.z > 0 && dotVec(map[j].vtx, map[j].nrm) < 0){
					tmp.push(mat * map[j]);
				}
			}
		}

		tmp = shuffle(tmp);

		Mem1<VecVN3> dst;
		const double unit = 2 * distance / (cam.fx + cam.fy);

		for (int i = 0; i < tmp.size(); i++){
			bool check = true;
			for (int j = 0; j < dst.size(); j++){
				if (dotVec(dst[j].nrm, tmp[i].nrm) > 0.5 && normVec(dst[j].vtx - tmp[i].vtx) < unit){
					check = false;
					break;
				}
			}
			if (check == true){
				dst.push(tmp[i]);
			}
		}

		return dst;
	}


	//--------------------------------------------------------------------------------
	// sample model
	//--------------------------------------------------------------------------------

	SP_CPUFUNC bool loadBunny(Mem1<Mesh> &model, const char *path) {
		if (loadPLY(model, path) == false) return false;

		Vec3 center = getModelCenter(model);
		for (int i = 0; i < model.size(); i++) {
			model[i] -= center;

			// m -> mm
			model[i] *= 1000.0;
		}
		return true;
	}

	SP_CPUFUNC void loadGeodesicDorm(Mem1<Mesh> &model, const double size, const int div) {
		model.clear();

		const int num = getGeodesicMeshNum(div);
		for (int i = 0; i < num; i++) {
			model.push(getGeodesicMesh(div, i) * size);
		}

	}


}

#endif