//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
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
            m_sfm.setMode(SfM::MODE_SERIAL);
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
        
        void setBase(const Mem2<Col3> &img, const Pose &pose, const Mem1<Ftr> *ftrs = NULL) {
            m_vtrack.setBase(img, pose, ftrs);
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

            const Pose *pose = m_vtrack.getPose();
            if (pose != NULL){
                const View *view = m_sfm.searchNearView(*pose);
                if (view != NULL) {
                    m_vtrack.setBase(*view);
                }
            }

            return m_vtrack.execute(img);
        }

        //--------------------------------------------------------------------------------
        // mapping
        //--------------------------------------------------------------------------------

        void addView(const Mem2<Col3> &img, const Pose *hint = NULL) {
            if (m_vtrack.getView() == NULL) return;

            m_sfm.addView(m_vtrack.getCam(), img, hint);
        }
        
        bool updateMap(const Mem2<Col3> &img) {
            if (m_vtrack.getView() == NULL) return false;

            if (m_sfm.vsize() == 0) {
                m_sfm.addView(m_vtrack.getCam(), m_vtrack.getView()->img, m_vtrack.getPose());
            }
            else if (m_vtrack.getPose() != NULL) {
                const int i = m_sfm.vsize() - 1;
                const Pose prePose = m_sfm.getView(i)->pose;

                SP_REAL move = SP_INFINITY;
                SP_REAL norm = SP_INFINITY;
                SP_REAL angle = SP_INFINITY;

                if (m_sfm.msize() == 0) {
                    const Pose nearPose = m_sfm.getView(i)->pose;
                    const Pose dif = *m_vtrack.getPose() * invPose(nearPose);

                    norm = minval(norm, normVec(dif.trn));
                    angle = minval(angle, getAngle(dif.rot, 2));
                }

                const View *view = m_sfm.searchNearView(*m_vtrack.getPose());
                if(view != NULL){
                    const Pose nearPose = view->pose;
                    const Pose dif = *m_vtrack.getPose() * invPose(nearPose);

                    norm = minval(norm, normVec(dif.trn));
                    angle = minval(angle, getAngle(dif.rot, 2));

                }

                const SP_REAL nThresh = 0.5;
                const SP_REAL aThresh = 5.0 * SP_PI / 180.0;

                const SP_REAL t = norm / nThresh + angle / aThresh;

                if (t > 1.0) {
                    m_sfm.addView(m_vtrack.getCam(), img, m_vtrack.getPose());
                }
            }
            return m_sfm.update();
        }

    };

}
#endif