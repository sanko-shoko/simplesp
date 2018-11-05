//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_SLAM_H__
#define __SP_SLAM_H__

#include "spcore/spcore.h"
#include "spapp/spgeomex/spsfm.h"
#include "spapp/spgeomex/spviewtrack.h"

namespace sp {

    class SLAM {

    private:
        
        SfM m_sfm;

        ViewTrack m_vtrack;

    public:

        SLAM() {
            clear();
        }

        void clear() {
            m_sfm.clear();
            m_vtrack.clear();
        }

        //--------------------------------------------------------------------------------
        // input parameter
        //--------------------------------------------------------------------------------

        void setCam(const CamParam &cam) {
            m_vtrack.setCam(cam);
        }
        const CamParam& getCam() const {
            return m_vtrack.getCam();
        }
        
        void setBase(const Mem2<Col3> &img, const Pose &pose, const Mem1<Feature> *fts = NULL) {
            m_vtrack.setBase(img, pose, fts);
        }

        void setBase(const View &view) {
            m_vtrack.setBase(view);
        }


        //--------------------------------------------------------------------------------
        // output parameter
        //--------------------------------------------------------------------------------
        
        const Pose* getPose() const {
            return m_vtrack.getPose();
        }

        const Mem1<Vec2>* getBases() const {
            return m_vtrack.getBases();
        }

        const Mem1<Vec2>* getCrsps() const {
            return m_vtrack.getCrsps();
        }

        const Mem1<bool>* getMask() const {
            return m_vtrack.getMask();
        }

        int vsize() const {
            return m_sfm.vsize();
        }

        const View* getView(const int i) const {
            return m_sfm.getView(i);
        }

        int msize() const {
            return m_sfm.msize();
        }

        const MapPnt* getMPnt(const int i) const {
            return m_sfm.getMPnt(i);
        }


        //--------------------------------------------------------------------------------
        // tracking
        //--------------------------------------------------------------------------------

        bool updatePose(const Mem2<Col3> &img) {
            return m_vtrack.execute(img);
        }

        //--------------------------------------------------------------------------------
        // mapping
        //--------------------------------------------------------------------------------

        void addView(const Mem2<Col3> &img, const Pose *hint = NULL) {
            m_sfm.addView(m_vtrack.getCam(), img, hint);
        }
        
        bool updateMap() {
            return m_sfm.update();
        }

    };

}
#endif