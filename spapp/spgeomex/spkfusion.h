//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_KINECTFUSION_H__
#define __SP_KINECTFUSION_H__

#include "spcore/spcore.h"
#include "spapp/spgeom/spdepth.h"
#include "spapp/spdata/spvoxel.h"

namespace sp{

    class KinectFusion{

    private:
        // tsdf map
        Voxel<> m_tsdf;

        // casted pn
        Mem2<VecPN3> m_cast;

        // camera parameter
        CamParam m_cam;

        // camera <- map pose
        Pose m_pose;

        // flag for tracking
        bool m_track;

    public:

        KinectFusion(){
            init(100, 2.0, getCamParam(0, 0), zeroPose());
        }

        void init(const int size, const double unit, const CamParam &cam, const Pose &base) {
            m_tsdf.init(size, unit);

            m_cast.resize(cam.dsize);
            m_cast.zero();

            m_cam = cam;
            m_pose = base;
            m_track = false;
        }

        const CamParam& getCam() const {
            return m_cam;
        }

        void reset() {
            m_track = false;
        }

        bool valid() {
            return m_track;
        }


        //--------------------------------------------------------------------------------
        // data
        //--------------------------------------------------------------------------------

        const Pose* getPose() const{
            return (m_track == true) ? &m_pose : NULL;
        }

        const Mem2<VecPN3>* getCast() const{
            return (m_track == true) ? &m_cast : NULL; ;
        }

        const Voxel<>* getMap() const {
            return (m_track == true) ? &m_tsdf : NULL; ;
        }


        //--------------------------------------------------------------------------------
        // execute 
        //--------------------------------------------------------------------------------
    
        bool execute(const Mem2<SP_REAL> &depth){

            return _execute(depth);
        }

    private:

        bool _execute(const Mem2<SP_REAL> &depth){
            SP_LOGGER_SET("KinectFusion::execute");

            if (cmpSize(2, m_cam.dsize, depth.dsize) == false) {
                return false;
            }

            // clear data
            if (m_track == false){
                m_tsdf.zero();
                m_cast.zero();
            }

            try{
                Mem2<VecPN3> pnmap;

                if (m_track == true){
                    SP_LOGGER_SET("KinectFusion::bilateralFilter");
                    Mem2<SP_REAL> bilateral;
                    bilateralFilterDepth(bilateral, depth, 0.8, 10.0);
                        
                    cnvDepthToVecPN(pnmap, m_cam, bilateral);
                }

                if (m_track == true){
                    SP_LOGGER_SET("KinectFusion::updatePose");
                    updatePose(m_pose, m_cam, pnmap, m_cast);
                }

                {
                    SP_LOGGER_SET("KinectFusion::updateTSDF");
                    updateTSDF(m_tsdf, m_cam, m_pose, depth);
                }

                {
                    SP_LOGGER_SET("KinectFusion::rayCasting");
                    rayCasting(m_cast, m_cam, m_pose, m_tsdf);
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

        bool updatePose(Pose &pose, const CamParam &cam, const Mem2<VecPN3> &pnmap, const Mem2<VecPN3> &cast){

            Mem1<VecPN3> sample;
            sample.reserve(pnmap.size());

            const int step = 4;
            for (int v = step; v < pnmap.dsize[1] - step; v += step){
                for (int u = step; u < pnmap.dsize[0] - step; u += step){
                    const VecPN3 &pn = pnmap(u, v);
                    if (pn.pos.z == 0.0) continue;

                    sample.push(pn);
                }
            }

            Pose delta = zeroPose();
            if (calcICP(delta, cam, cast, sample, 20) == false) return false;
            pose = invPose(delta) * pose;

            return true;
        }
    };

}
#endif