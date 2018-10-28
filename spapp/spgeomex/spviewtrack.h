//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_VIEWTRACK_H__
#define __SP_VIEWTRACK_H__

#include "spcore/spcore.h"
#include "spapp/spdata/spscene.h"
#include "spapp/spimgex/splucaskanade.h"


namespace sp{

    class ViewTrack {
    public:

        SP_LOGGER_INSTANCE;
        SP_HOLDER_INSTANCE;

        const View *m_view;
        View m_inst;

        bool m_track;

        Pose m_pose;

        Mem1<Vec2> m_crsps;
        Mem1<bool> m_mask;


        //--------------------------------------------------------------------------------
        // memory pool
        //--------------------------------------------------------------------------------

        MemP<MapPnt> _mpnts;

    public:

        ViewTrack() {
            clear();
        }

        void clear() {
            m_track = false;
            m_pose = zeroPose();
            m_view = NULL;

            m_crsps.clear();
            m_mask.clear();
        }

        //--------------------------------------------------------------------------------
        // input parameter
        //--------------------------------------------------------------------------------
        
        void setView(const CamParam &cam, const Mem2<Col3> &img, const Pose &pose, const Mem1<Feature> *fts = NULL) {
            _setView(cam, img, pose, fts);
        }

        void setView(const View &view) {
            _setView(view);
        }

        const View* getView() const {
            return m_view;
        }


        //--------------------------------------------------------------------------------
        // output parameter
        //--------------------------------------------------------------------------------

        const Pose* getPose() const {
            return (m_track == true) ? &m_pose : NULL;
        }


        //--------------------------------------------------------------------------------
        // execute
        //--------------------------------------------------------------------------------

        bool execute(const CamParam &cam, const Mem2<Col3> &img, const Pose &pose) {
            SP_LOGGER_SET("-execute");

            return _execute(cam, img);
        }


    private:

        void _setView(const CamParam &cam, const Mem2<Col3> &img, const Pose &pose, const Mem1<Feature> *fts = NULL) {
            m_inst.cam = cam;
            m_inst.img = img;
            m_inst.pose = pose;

            m_inst.fts = (fts != NULL) ? *fts : SIFT::getFeatures(img);

            m_inst.valid = true;

            m_view = &m_inst;
        }

        void _setView(const View &view) {
            m_view = &view;
        }

        bool _execute(const CamParam &cam, const Mem2<Col3> &img) {

 
            try {
                if (m_view == NULL) throw "view invalid";
                if (img.size() == 0) throw "input size";

                if (m_view == &m_inst && check(*m_view) == false) {
                    if (initMap(m_pose, m_crsps, m_mask, m_inst, cam, img) == false) throw "initMap";
                }
                else {
                    if (track(m_pose, m_crsps, m_mask, *m_view, cam, img, m_track) == false) throw "track";
                    m_track = true;
                }
            }
            catch (const char *str) {
                SP_PRINTD("ViewTrack::execute [%s]\n", str);
                m_track = false;
                return false;
            }
            return true;
        }

        //--------------------------------------------------------------------------------
        // modules
        //--------------------------------------------------------------------------------

        bool check(const View &view) {
            int cnt = 0;
            for (int i = 0; i < view.fts.size(); i++) {
                if (view.fts[i].mpnt != NULL) cnt++;
            }
            const bool ret = (cnt > 0) ? true : false;
            return ret;
        }

        void calcFlow(Mem1<Vec2> &flows, Mem1<bool> &mask, const View &view, const CamParam &cam, const Mem2<Col3> &img) {
            Mem1<Vec2> pixs;
            Mem1<double> scls;

            const Mem1<Feature> &fts = view.fts;
            for (int i = 0; i < fts.size(); i++) {
                pixs.push(fts[i].pix);
                scls.push(fts[i].scl);
            }
            opticalFlowLK(flows, mask, img, view.img, pixs, scls);

        }

        bool initMap(Pose &pose, Mem1<Vec2> &crsps, Mem1<bool> &mask, View &view, const CamParam &cam, const Mem2<Col3> &img) {

            static Mem1<Vec2> flows;
            calcFlow(flows, mask, view, cam, img);

            crsps.resize(flows.size());
            crsps.zero();

            Mem1<Vec2> pixs0;
            Mem1<Vec2> pixs1;
            for (int i = 0; i < flows.size(); i++) {
                if (mask[i] == false) continue;

                const Vec2 pix0 = view.fts[i].pix;
                const Vec2 pix1 = view.fts[i].pix + flows[i];

                crsps[i] = pix1;
              
                pixs0.push(pix0);
                pixs1.push(pix1);
            }

            {
                const double eval = evalStereo(view.cam, pixs0, cam, pixs1);
                printf("eval %lf\n", eval);
                if (eval < 0.4) return false;
            }

            {
                Pose delta = zeroPose();
                calcPoseRANSAC(delta, view.cam, pixs0, cam, pixs1);
                pose = delta * view.pose;
            }

            _mpnts.clear();
            for (int i = 0; i < mask.size(); i++) {
                if (mask[i] == false) continue;

                const Vec2 pix0 = view.fts[i].pix;
                const Vec2 pix1 = view.fts[i].pix + flows[i];

                Vec3 pos;
                if (calcPnt3d(pos, view.pose, view.cam, pix0, pose, cam, pix1) == false) continue;

                MapPnt *m = _mpnts.malloc();
                m->pos = pos;
                view.fts[i].mpnt = m;
            }
            return true;
        }

        bool track(Pose &pose, Mem1<Vec2> &crsps, Mem1<bool> &mask, const View &view, const CamParam &cam, const Mem2<Col3> &img, const bool track = false) {

            Mem1<Vec2> flows(view.fts.size());
            flows.zero();

            if (track == true) {
                const Pose stereo = pose * invPose(view.pose);

                for (int i = 0; i < view.fts.size(); i++) {
                    if (view.fts[i].mpnt == NULL) continue;

                    const Vec3 obj = stereo * view.fts[i].mpnt->pos;

                    const Vec2 pix0 = view.fts[i].pix;
                    const Vec2 pix1 = mulCam(cam, prjVec(obj));

                    flows[i] = pix1 - pix0;
                }
            }

            calcFlow(flows, mask, view, cam, img);

            crsps.resize(flows.size());
            crsps.zero();
            {
                Mem1<Vec2> pixs;
                Mem1<Vec3> objs;
                for (int i = 0; i < view.fts.size(); i++) {
                    if (mask[i] == false) continue;

                    const Vec2 pix = view.fts[i].pix + flows[i];
                    crsps[i] = pix;

                    if (view.fts[i].mpnt == NULL) continue;
                    const Vec3 obj = view.fts[i].mpnt->pos;

                    pixs.push(pix);
                    objs.push(obj);
                }

                if (calcPoseRANSAC(pose, cam, pixs, objs) == false) return false;
            }

            return true;
        }
    };

}
#endif