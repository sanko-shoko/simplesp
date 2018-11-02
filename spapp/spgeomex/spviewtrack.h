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
    private:

        //--------------------------------------------------------------------------------
        // input parameter
        //--------------------------------------------------------------------------------

        const View *m_view;
        View m_inst;

        //--------------------------------------------------------------------------------
        // output parameter
        //--------------------------------------------------------------------------------

        // success flag
        bool m_track;

        // estimated parameter
        Pose m_pose;

        double m_meanZ;

        // image points
        Mem1<Vec2> m_bases;
        Mem1<Vec2> m_crsps;
        Mem1<bool> m_mask;

        //--------------------------------------------------------------------------------
        // memory pool
        //--------------------------------------------------------------------------------

        MemP<MapPnt> _mpnts;

    public:
        SP_LOGGER_INSTANCE;
        SP_HOLDER_INSTANCE;

        ViewTrack() {
            clear();
        }

        void clear() {
            m_track = false;
            m_pose = zeroPose();
            m_meanZ = 0.0;
            m_view = NULL;

            m_bases.clear();
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
        Mem2<Col3> m_test;

        //--------------------------------------------------------------------------------
        // output parameter
        //--------------------------------------------------------------------------------

        const Pose* getPose() const {
            return (m_track == true) ? &m_pose : NULL;
        }

        const Mem1<Vec2>* getBases() const {
            return (m_bases.size() > 0) ? &m_bases : NULL;
        }

        const Mem1<Vec2>* getCrsps() const {
            return (m_crsps.size() > 0) ? &m_crsps : NULL;
        }

        const Mem1<bool>* getMask() const {
            return (m_mask.size() > 0) ? &m_mask : NULL;
        }


        //--------------------------------------------------------------------------------
        // execute
        //--------------------------------------------------------------------------------

        bool execute( const Mem2<Col3> &img) {
            SP_LOGGER_SET("-execute");

            return _execute(img);
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

        bool _execute(const Mem2<Col3> &img) {

            try {
                if (m_view == NULL || img.size() == 0) throw "input invalid";

                if (cntMPnt(*m_view) == 0) {
                    if (m_view == &m_inst) {
                        if (initMap(m_pose, m_crsps, m_bases, m_mask, m_inst, img) == false) throw "initMap";
                    }
                    else {
                        throw "map invalid";
                    }
                }

                if (m_meanZ == 0.0) {
                    m_meanZ = calcMeanZ(*m_view);
                }

                if (track(m_pose, m_crsps, m_bases, m_mask, *m_view, img, m_meanZ) == false) throw "track";

                m_track = true;
            }
            catch (const char *str) {
                
                SP_PRINTD("ViewTrack::execute [%s]\n", str);

                m_pose = m_view->pose;
                m_meanZ = 0.0;
                m_track = false;
            }

            return m_track;
        }


        //--------------------------------------------------------------------------------
        // modules
        //--------------------------------------------------------------------------------

        int cntMPnt(const View &view) {
            int cnt = 0;
            for (int i = 0; i < view.fts.size(); i++) {
                if (view.fts[i].mpnt != NULL) cnt++;
            }
            return cnt;
        }

        double calcMeanZ(const View &view) {
            Mem1<double> zlist;

            const Mem1<Feature> &fts = view.fts;
            for (int i = 0; i < fts.size(); i++) {
                if (fts[i].mpnt == NULL) continue;

                zlist.push(fts[i].mpnt->pos.z);
            }
            
            const double meanZ = meanVal(zlist);

            return meanZ;
        }

        bool calcFlow(Mem1<Vec2> &flows, Mem1<bool> &mask, const View &view, const Mem2<Col3> &img) {
            Mem1<Vec2> pixs;
            Mem1<double> scls;

            const Mem1<Feature> &fts = view.fts;
            for (int i = 0; i < fts.size(); i++) {
                pixs.push(fts[i].pix);
                scls.push(fts[i].scl);
            }
            opticalFlowLK(flows, mask, img, view.img, pixs, scls);

            return true;
        }

        bool calcFlow(Mem1<Vec2> &flows, Mem1<bool> &mask, const View &view, const Mem2<Col3> &img, const Pose &pose, const double meanZ) {

            const Pose stereo = pose * invPose(view.pose);
            
            Mat hom;
            {
                const Mat cmat = extMat(4, 4, getMat(view.cam));
                const Mat imat = invMat(cmat);

                const Mat pmat = cmat * getMat(stereo, 4, 4) * imat;

                hom.resize(3, 3);
                for (int r = 0; r < 3; r++) {
                    for (int c = 0; c < 3; c++) {
                        hom(r, c) = (c < 2) ? pmat(r, c) * meanZ : pmat(r, c) * meanZ + pmat(r, c + 1);
                    }
                }
                hom = hom / hom(2, 2);
            }

            Mem2<Col3> wimg;
            {
                wimg.resize(view.img.dsize);
                wimg.zero();
                warp<Col3, Byte>(wimg, view.img, hom);
            }

            flows.resize(view.fts.size());
            flows.zero();

            Mem1<Vec2> pixs;
            Mem1<Vec2> tmps;
            Mem1<double> scls;

            {
                const Vec3 h2 = getVec(hom(2, 0), hom(2, 1), hom(2, 2));
                const Mem1<Feature> &fts = view.fts;
                for (int i = 0; i < fts.size(); i++) {
                    const Vec2 &p = fts[i].pix;
                    const double s = fts[i].scl;

                    const Vec2 pix0 = hom * p;
                    const double t = h2.x * p.x + h2.y * p.y + h2.z;
                    if (fabs(t - 1.0) > 0.4) return false;

                    pixs.push(pix0);
                    tmps.push(p);
                    scls.push(s * t);

                    if (view.fts[i].mpnt == NULL) continue;

                    const Vec3 obj = stereo * fts[i].mpnt->pos;

                    const Vec2 pix1 = mulCam(view.cam, prjVec(obj));

                    flows[i] = pix1 - pix0;
                }
            }

            opticalFlowLK(flows, mask, img, wimg, pixs, scls);
        
            for (int i = 0; i < mask.size(); i++) {
                if (mask[i] == false) continue;
                flows[i] += pixs[i] - tmps[i];
            }

            return true;
        }

        bool initMap(Pose &pose, Mem1<Vec2> &crsps, Mem1<Vec2> &bases, Mem1<bool> &mask, View &view, const Mem2<Col3> &img) {

            static Mem1<Vec2> flows;
            calcFlow(flows, mask, view, img);

            crsps.resize(flows.size());
            bases.resize(flows.size());
            crsps.zero();
            bases.zero();

            Mem1<Vec2> pixs0;
            Mem1<Vec2> pixs1;
            for (int i = 0; i < flows.size(); i++) {
                if (mask[i] == false) continue;

                const Vec2 pix0 = view.fts[i].pix;
                const Vec2 pix1 = view.fts[i].pix + flows[i];

                bases[i] = pix0;
                crsps[i] = pix1;
              
                pixs0.push(pix0);
                pixs1.push(pix1);
            }

            {
                const double eval = evalStereo(view.cam, pixs0, view.cam, pixs1);
                SP_PRINTD("eval %lf\n", eval);
                if (eval < 0.4) return false;
            }

            {
                Pose delta = zeroPose();
                calcPoseRANSAC(delta, view.cam, pixs1, view.cam, pixs0);
                pose = delta * view.pose;
            }

            _mpnts.clear();
            for (int i = 0; i < mask.size(); i++) {
                if (mask[i] == false) continue;

                const Vec2 pix0 = view.fts[i].pix;
                const Vec2 pix1 = view.fts[i].pix + flows[i];

                Vec3 pnt;
                if (calcPnt3d(pnt, view.pose, view.cam, pix0, pose, view.cam, pix1) == false) continue;

                MapPnt *m = _mpnts.malloc();
                m->pos = pnt;
                view.fts[i].mpnt = m;
            }

            return true;
        }

        bool track(Pose &pose, Mem1<Vec2> &crsps, Mem1<Vec2> &bases, Mem1<bool> &mask, const View &view, const Mem2<Col3> &img, const double meanZ) {

            Mem1<Vec2> flows;
            calcFlow(flows, mask, view, img, pose, meanZ);

            crsps.resize(flows.size());
            bases.resize(flows.size());
            crsps.zero();
            bases.zero();

            Mem1<Vec2> pixs;
            Mem1<Vec3> objs;
            for (int i = 0; i < view.fts.size(); i++) {
                if (mask[i] == false) continue;

                const Vec2 pix0 = view.fts[i].pix;
                const Vec2 pix1 = view.fts[i].pix + flows[i];
                bases[i] = pix0;
                crsps[i] = pix1;

                if (view.fts[i].mpnt == NULL) continue;

                const Vec3 obj = view.fts[i].mpnt->pos;
                pixs.push(pix1);
                objs.push(obj);
            }

            if (calcPoseRANSAC(pose, view.cam, pixs, objs) == false) return false;

            return true;
        }
    };

}
#endif