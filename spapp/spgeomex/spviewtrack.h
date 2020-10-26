//--------------------------------------------------------------------------------
// Copyright (c) 2017-2020, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_VIEWTRACK_H__
#define __SP_VIEWTRACK_H__

#include "spcore/spcore.h"
#include "spapp/spimgex/spfeature.h"
#include "spapp/spimgex/spoptflow.h"

namespace sp{

    class ViewTrack {

    private:

        //--------------------------------------------------------------------------------
        // input parameter
        //--------------------------------------------------------------------------------

        // camera parameter
        CamParam m_cam;
        
        const View *m_view;

        View m_inst;

        //--------------------------------------------------------------------------------
        // output parameter
        //--------------------------------------------------------------------------------

        // success flag
        bool m_track;

        // estimated parameter
        Pose m_pose;

        SP_REAL m_meanZ;

        // image points
        Mem1<Vec2> m_bases;
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
            m_view = NULL;

            m_cam = getCamParam(0, 0);
            m_pose = zeroPose();

            m_meanZ = 0.0;

            m_bases.clear();
            m_crsps.clear();
            m_mask.clear();
        }

        //--------------------------------------------------------------------------------
        // input parameter
        //--------------------------------------------------------------------------------
        
        void setCam(const CamParam &cam) {
            m_cam = cam;
        }

        const CamParam& getCam() const {
            return m_cam;
        }
        
        void setBase(const Mem2<Col3> &img, const Pose &pose, const Mem1<Ftr> *ftrs = NULL) {
            _setBase(img, pose, ftrs);
        }

        void setBase(const View &view) {
            _setBase(view);
        }

  
        //--------------------------------------------------------------------------------
        // output parameter
        //--------------------------------------------------------------------------------

        const View* getView() const {
            return m_view;
        }

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

            return _execute(img);
        }


    private:

        void _setBase(const Mem2<Col3> &img, const Pose &pose, const Mem1<Ftr> *ftrs = NULL) {
          
            // set default camera parameter
            if (cmp(m_cam.dsize, img.dsize, 2) == false) {
                m_cam = getCamParam(img.dsize);
            }

            m_inst.cam = m_cam;
            m_inst.img = img;
            m_inst.pose = pose;

            m_inst.ftrs = (ftrs != NULL) ? *ftrs : SIFT::getFtrs(img);

            m_inst.valid = true;

            m_view = &m_inst;
        }

        void _setBase(const View &view) {
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

                m_meanZ = calcMeanZ(*m_view);

                if (track(m_pose, m_crsps, m_bases, m_mask, *m_view, img, m_meanZ) == false) throw "track";

                m_track = true;
            }
            catch (const char *str) {
                
                SP_PRINTD("ViewTrack::execute [%s]\n", str);

                if (m_view != NULL) {
                    m_pose = m_view->pose;
                }

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
            for (int i = 0; i < view.ftrs.size(); i++) {
                if (view.ftrs[i].mpnt != NULL) cnt++;
            }
            return cnt;
        }

        SP_REAL calcMeanZ(const View &view) {
            Mem1<SP_REAL> zlist;

            const Mem1<Ftr> &ftrs = view.ftrs;
            for (int i = 0; i < ftrs.size(); i++) {
                if (ftrs[i].mpnt == NULL) continue;

                zlist.push(ftrs[i].mpnt->pos.z);
            }
            
            sort(zlist);
            zlist = zlist.part(round(0.1 * zlist.size()), round(0.8 * zlist.size()));

            const SP_REAL meanZ = meanVal(zlist);

            return meanZ;
        }

        bool calcFlow(Mem1<Vec2> &flows, Mem1<bool> &mask, const View &view, const Mem2<Col3> &img) {
            Mem1<Vec2> pixs;
            Mem1<SP_REAL> scls;

            const Mem1<Ftr> &ftrs = view.ftrs;
            for (int i = 0; i < ftrs.size(); i++) {
                pixs.push(ftrs[i].pix);
                scls.push(ftrs[i].scl);
            }
            opticalFlowLK(flows, mask, img, view.img, pixs, scls);

            return true;
        }

        bool calcFlow(Mem1<Vec2> &flows, Mem1<bool> &mask, const View &view, const Mem2<Col3> &img, const Pose &pose, const SP_REAL meanZ) {

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

            const int num = view.ftrs.size();
            flows.resize(num);
            flows.zero();

            Mem1<Vec2> pixs(num);
            Mem1<SP_REAL> scls(num);
            pixs.zero();
            scls.zero();

            {
                const Vec3 h2 = getVec3(hom(2, 0), hom(2, 1), hom(2, 2));
                const Mem1<Ftr> &ftrs = view.ftrs;
                for (int i = 0; i < num; i++) {
                    const Vec2 pix1 = ftrs[i].pix;
                    const SP_REAL s = ftrs[i].scl;

                    const Vec2 pix0 = hom * pix1;
                    const SP_REAL t = h2.x * pix1.x + h2.y * pix1.y + h2.z;
                    if (fabs(t - 1.0) > 0.5) return false;

                    pixs[i] = pix1;
                    scls[i] = s * t;
                    flows[i] = pix0 - pix1;
                }
            }
 
            opticalFlowLK(flows, mask, img, view.img, pixs, scls);

            for (int i = 0; i < num; i++) {
                const MapPnt *mpnt = view.ftrs[i].mpnt;
                if (mpnt == NULL || mpnt->valid == false) mask[i] = false;
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

                const Vec2 pix0 = view.ftrs[i].pix;
                const Vec2 pix1 = view.ftrs[i].pix + flows[i];

                bases[i] = pix0;
                crsps[i] = pix1;
              
                pixs0.push(pix0);
                pixs1.push(pix1);
            }

            {
                const SP_REAL eval = evalStereo(view.cam, pixs0, view.cam, pixs1);
                //SP_PRINTD("eval %lf\n", eval);
                if (eval < 0.4) return false;
            }

            {
                Pose delta = zeroPose();
                calcPoseRANSAC(delta, view.cam, pixs0, view.cam, pixs1);
                pose = delta * view.pose;
            }

            _mpnts.clear();
            for (int i = 0; i < mask.size(); i++) {
                if (mask[i] == false) continue;

                const Vec2 pix0 = view.ftrs[i].pix;
                const Vec2 pix1 = view.ftrs[i].pix + flows[i];

                Vec3 pnt;
                if (calcPnt3d(pnt, view.pose, view.cam, pix0, pose, view.cam, pix1) == false) continue;

                MapPnt *m = _mpnts.malloc();
                m->pos = pnt;
                m->valid = true;
                view.ftrs[i].mpnt = m;
            }

            return true;
        }

        bool track(Pose &pose, Mem1<Vec2> &crsps, Mem1<Vec2> &bases, Mem1<bool> &mask, const View &view, const Mem2<Col3> &img, const SP_REAL meanZ) {

            Mem1<Vec2> flows;
            calcFlow(flows, mask, view, img, pose, meanZ);

            crsps.resize(flows.size());
            bases.resize(flows.size());
            crsps.zero();
            bases.zero();

            Mem1<Vec2> pixs;
            Mem1<Vec3> objs;
            for (int i = 0; i < view.ftrs.size(); i++) {
                if (mask[i] == false) continue;

                const Vec2 pix0 = view.ftrs[i].pix;
                const Vec2 pix1 = view.ftrs[i].pix + flows[i];
                bases[i] = pix0;
                crsps[i] = pix1;

                if (view.ftrs[i].mpnt == NULL) continue;

                const Vec3 obj = view.ftrs[i].mpnt->pos;
                pixs.push(pix1);
                objs.push(obj);
            }

            if (calcPoseRANSAC(pose, view.cam, pixs, objs) == false) return false;

            return true;
        }
    };

}
#endif