//--------------------------------------------------------------------------------
// Copyright (c) 2017, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_KINECTFUSION_H__
#define __SP_KINECTFUSION_H__

#include "spcore/spcore.h"
#include "spapp/spgeom/spdepth.h"
#include "spapp/spgeomex/sptsdf.h"

namespace sp{

	class KinectFusion{

	private:
		// camera parameter
		CamParam m_cam;

		// tsdf map
		Mem3<TSDF> m_tsdf;

		// casted vn
		Mem2<VecVN3> m_cast;

		// map pose
		Pose m_pose;

		// map unit length
		double m_unit;
	
		// flag for tracking
		bool m_track;

	public:
		SP_LOGGER_INSTANCE;

		KinectFusion(){
			m_track = false;
			m_cam = getCamParam(0, 0);
			setMap(100, 2.0, zeroPose());
		}

		const void reset(){
			m_track = false;
		}

		void setMap(const int size, const double unit, const Pose &base){
			m_tsdf.resize(size, size, size);

			m_unit = unit;
			m_pose = base;
		}

		void setCam(const CamParam &cam) {
			m_cam = cam;
		}


		//--------------------------------------------------------------------------------
		// get status 
		//--------------------------------------------------------------------------------

		const CamParam& getCam() const{
			return m_cam;
		}
	
		const Pose* getPose() const{
			return &m_pose;
		}

		const Mem2<VecVN3>& getCast() const{
			return m_cast;
		}

		const double getCubeLength() const{
			return m_tsdf.dsize[0] * m_unit;
		}


		//--------------------------------------------------------------------------------
		// execute 
		//--------------------------------------------------------------------------------
	
		bool execute(const Mem2<double> &depth){

			return _execute(depth);
		}

	private:

		bool _execute(const Mem2<double> &depth){
			SP_LOGGER_SET("-execute");

			if (cmpSize(2, m_cam.dsize, depth.dsize) == false) {
				return false;
			}

			// clear data
			if (m_track == false){
				m_tsdf.zero();
			}

			try{
				Mem2<VecVN3> vnmap;

				if (m_track == true){
					SP_LOGGER_SET("measurement");
					Mem2<double> bilateral;
					bilateralFilterDepth(bilateral, depth, 0.8, 10.0);
						
					cnvDepthToVecVN(vnmap, m_cam, bilateral);
				}

				if (m_track == true){
					SP_LOGGER_SET("update pose");
					updatePose(m_pose, m_cam, vnmap, m_cast);
				}

				{
					SP_LOGGER_SET("update map");
					updateTSDF(m_tsdf, m_unit, m_cam, m_pose, depth);
				}

				{
					SP_LOGGER_SET("rayCasting");
					rayCasting(m_cast, m_cam, m_pose, m_tsdf, m_unit);
				}
				m_track = true;
			}
			catch (const char *str){
				SP_PRINTD("KinectFusion.execute [%s]\n", str);
				m_track = false;
				return false;
			}

			return true;
		}

		bool updatePose(Pose &pose, const CamParam &cam, const Mem2<VecVN3> &vnmap, const Mem2<VecVN3> &cast){

			Mem1<VecVN3> sample;
			sample.reserve(vnmap.size());

			const int block = 4;
			for (int v = block; v < vnmap.dsize[1] - block; v += block){
				for (int u = block; u < vnmap.dsize[0] - block; u += block){
					const VecVN3 &vn = vnmap(u, v);
					if (vn.vtx.z == 0.0) continue;

					sample.push(vn);
				}
			}

			Pose delta = zeroPose();
			if (calcICP(delta, cam, sample, cast, 20) == false) return false;
			pose = invPose(delta) * pose;

			return true;
		}
	};

}
#endif