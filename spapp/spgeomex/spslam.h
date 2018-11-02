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

        SLAM() {
            clear();
        }

        void init(const CamParam &cam) {
            m_cam = cam;
            clear();
        }

        void clear() {
            m_track = false;
            m_cam = getCamParam(0, 0);
            m_pose = zeroPose();
            m_sfm.clear();
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

        const View* getViews(const int i) const {
            return m_sfm.getView(i);
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

            try {
                if (img.size() == 0 || cmpSize(2, m_cam.dsize, img.dsize) == false) throw "input size";

                if (force == true) {
                    m_sfm.addView(m_cam, img);
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