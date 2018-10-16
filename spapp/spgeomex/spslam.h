//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_SLAM_H__
#define __SP_SLAM_H__

#include "spcore/spcore.h"
#include "spapp/spgeomex/spsfm.h"

namespace sp {

    class SLAM {

    private:
        SfM m_sfm;

        CamParam m_cam;
        
        // flag for tracking
        bool m_track;

        Pose m_pose;

    public:
        SP_LOGGER_INSTANCE;

        SLAM() {
        }

        void init(const CamParam &cam, const int maxview = SP_SFM_MAXVIEW) {
            clear();
            m_cam = cam;
            m_sfm.init(maxview);
        }

        void clear() {
            m_track = false;
            m_cam = getCamParam(0, 0);
            m_pose = zeroPose();
        }


        //--------------------------------------------------------------------------------
        // input parameter
        //--------------------------------------------------------------------------------

        const CamParam& getCam() const {
            return m_cam;
        }


        //--------------------------------------------------------------------------------
        // output parameter
        //--------------------------------------------------------------------------------
        
        const Pose* getPose() const {
            return (m_track == true) ? &m_pose : NULL;
        }

        const Mem1<SfM::ViewData>* getViews() const {
            return m_sfm.getViews();
        }

        const Mem1<SfM::MapData>* getMPnts() const {
            return m_sfm.getMPnts();
        }


        //--------------------------------------------------------------------------------
        // execute
        //--------------------------------------------------------------------------------

        bool updatePose(const Mem1<Col3> &img) {
            return _updatePose(img);
        }

        bool updateMap(const Mem1<Col3> &img, const bool force = false) {
            return _updateMap(img, force);
        }

    private:

        //--------------------------------------------------------------------------------
        // execute main flow
        //--------------------------------------------------------------------------------
      
        bool _updatePose(const Mem1<Col3> &img) {
            SP_LOGGER_SET("-updatePose");

            try {
                if (img.size() == 0 || cmpSize(2, m_cam.dsize, img.dsize) == false) throw "input size";

                m_track = true;
            }
            catch (const char *str) {
                SP_PRINTD("SLAM::updatePose [%s]\n", str);

                m_track = false;
                return false;
            }
        }

        bool _updateMap(const Mem1<Col3> &img, const bool force = false) {
            SP_LOGGER_SET("-updateMap");

            try {
                if (img.size() == 0 || cmpSize(2, m_cam.dsize, img.dsize) == false) throw "input size";

                if (force == true) {
                    m_sfm.addView(img, &m_cam);
                }

                m_sfm.update();
            }
            catch (const char *str) {
                SP_PRINTD("SLAM::updateMap [%s]\n", str);

                return false;
            }
            return true;
        }


        //--------------------------------------------------------------------------------
        // modules
        //--------------------------------------------------------------------------------


    };

}
#endif