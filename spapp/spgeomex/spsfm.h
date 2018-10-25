//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_SMF_H__
#define __SP_SMF_H__

#include "spcore/spcore.h"

#include "spapp/spdata/spmap.h"
#include "spapp/spgeom/spgeometry.h"

namespace sp {

 
#define SP_SFM_MAXVIEW 1000

    class SfM {

    public:

        typedef MemA<int, 2> Int2;

        struct ViewEx : public View {

            // init pair count
            int icnt;

            // map points count
            int mcnt;

            ViewEx() : View(){
                icnt = 0;
                mcnt = 0;
            }

            ViewEx(const ViewEx &view) {
                *this = view;
            }

            ViewEx& operator = (const ViewEx &view) {
                static_cast<View>(*this) = static_cast<View>(view);

                icnt = view.icnt;
                mcnt = view.mcnt;
                return *this;
            }
        };


    public:
        SP_LOGGER_INSTANCE;
        SP_HOLDER_INSTANCE;

        // update counter
        int m_update;

        // initial pair
        Int2 m_ipair;

        int m_maxview;
        
        // key views
        Mem1<ViewEx> m_views;

        // pair data matrix
        Mem2<PairData> m_pairs;

        // map points
        Mem1<MapPoint> m_mpnts;

    public:

        SfM() {
            init();
        }

        void init(const int maxview = SP_SFM_MAXVIEW) {
            clear();

            m_maxview = maxview;
            m_views.reserve(maxview);
            m_pairs.resize(maxview, maxview);
            m_mpnts.reserve(100 * maxview);
        }

        void clear() {
            m_update = 0;
            m_ipair.set(-1, -1);

            m_views.clear();
            m_pairs.clear();
            m_mpnts.clear();
        }


        //--------------------------------------------------------------------------------
        // output parameter
        //--------------------------------------------------------------------------------

        int size() const {
            return m_views.size();
        }

        const View* getView(const int i) const {
            return (i < m_views.size()) ? &m_views[i] : NULL;
        }

        const Mem1<MapPoint>* getMPnts() const {
            return (m_mpnts.size() > 0) ? &m_mpnts : NULL;
        }

 
        //--------------------------------------------------------------------------------
        // execute
        //--------------------------------------------------------------------------------

        bool addView(const Mem2<Col3> &img, const CamParam *cam = NULL) {
            SP_LOGGER_SET("-addView");

            return _addView(img, cam);
        }

        bool update(const int itmax = 1) {
            SP_LOGGER_SET("-update");

            return _update(itmax);
        }


        //--------------------------------------------------------------------------------
        // util
        //--------------------------------------------------------------------------------

        bool save(const char *path) {
            Mem1<Vec3> pnts;
            Mem1<Col3> cols;
            for (int i = 0; i < m_mpnts.size(); i++) {
                pnts.push(m_mpnts[i].pos);
                cols.push(m_mpnts[i].col);
            }

            return savePLY(path, pnts, cols);
        }

    private:

        double MIN_MATCHEVAL = 0.2;
        double MIN_STEREOEVAL = 0.3;
        
        int MIN_POSEPNT = 10;
        
        double MPNT_PRJERR = 5.0;
        double MPNT_ANGLE = 3.0 * SP_PI / 180.0;

        int MAX_POSEUPDATE = 20;
        int MAX_MPNTUPDATE = 1000;

    private:

        //--------------------------------------------------------------------------------
        // execute main flow
        //--------------------------------------------------------------------------------
        
        bool _addView(const Mem2<Col3> &img, const CamParam *cam = NULL) {

            if (m_views.size() == m_maxview) return false;

            const int v = m_views.size();

            CamParam tmp = (cam != NULL && cmpSize(2, cam->dsize, img.dsize) == true) ? *cam : getCamParam(img.dsize);

            setView(m_views, v, img, tmp);

            return true;
        }

        bool _update(const int itmax = 1) {

            for (int it = 0; it < itmax; it++) {

                try {
                    if (m_views.size() < 2) throw "input size < 2";

                    srand(m_update);

                    if (m_update == 0) {
                        updatePair(m_views, m_pairs, m_views.size());

                        if (initStereo(m_views, m_pairs, m_mpnts) == false) throw "initStereo";
                    }
                    else {
                        updatePair(m_views, m_pairs, 1);

                        // update view [invalid -> valid]
                        updateView(m_views, m_pairs, m_mpnts, m_update);
                    }

                    {
                        // update map pnt [add new]
                        updateMPnt1(m_views, m_pairs, m_mpnts, m_update);

                        // update map pnt [refine]
                        updateMPnt2(m_views, m_pairs, m_mpnts, m_update);

                        // update view pose
                        updatePose(m_views, m_pairs, m_mpnts, m_update);
                    }
                    m_update++;
                }
                catch (const char *str) {
                    SP_PRINTD("SfM.update [%s]\n", str);

                    return false;
                }
            }

            return true;
        }


        //--------------------------------------------------------------------------------
        // view (v: view id)
        //--------------------------------------------------------------------------------

        void setView(Mem1<ViewEx> &views, const int v, const Mem2<Col3> &img, const CamParam &cam) {
            if (v + 1 > views.size()) views.extend(v + 1 - views.size());

            views[v].img = img;
            views[v].cam = cam;

            views[v].fts = SIFT::getFeatures(img);
        }

        void setView(Mem1<ViewEx> &views, const int v, const Pose &pose) {
            views[v].state = View::POSE_VALID;
            views[v].pose = pose;
        }


        //--------------------------------------------------------------------------------
        // pair (a, b: view id)
        //--------------------------------------------------------------------------------
        
        void initPair(Mem1<ViewEx> &views, Mem2<PairData> &pairs, const int a, const int b) {

            PairData &pair = pairs(a, b);

            pair.a = a;
            pair.b = b;

            pair.matches = findMatch(views[a].fts, views[b].fts);
            pair.eval = getMatchEval(pair.matches);

            if(pair.eval > MIN_MATCHEVAL){
                views[a].links.push(b);
            }

            views[a].icnt++;

            //printf("[%d %d]: size %d, cnt %d, eval %.2lf\n", pair.a, pair.b, pair.matches.size(), getMatchCnt(pair.matches), pair.eval);
        }

        Mem1<PairData*> getPairs(Mem1<ViewEx> &views, Mem2<PairData> &pairs, const View::State &stateA, const View::State &stateB) {
            Mem1<PairData*> ptrs;

            for (int a = 0; a < views.size(); a++) {
                if (views[a].state != stateA) continue;

                for (int i = 0; i < views[a].links.size(); i++) {
                    const int b = views[a].links[i];
                    if (views[b].state != stateB) continue;

                    if (stateA == stateB && b < a) continue;

                    ptrs.push(&pairs(a, b));
                }
            }
            return ptrs;
        }

        //--------------------------------------------------------------------------------
        // map point (m: mpnts id, v: views id, f: features id)
        //--------------------------------------------------------------------------------

        double errMPnt(Mem1<ViewEx> &views, Mem1<MapPoint> &mpnts, const int m) {

            Mem1<double> errs;

            const Mem1<Int2> &index = mpnts[m].index;
            for (int i = 0; i < index.size(); i++) {
                const int v = index[i][0];
                const int f = index[i][1];

                const Pose &pose = views[v].pose;
                const CamParam &cam = views[v].cam;
                const Vec2 &pix = views[v].fts[f].pix;
                const Vec3 &pos = mpnts[m].pos;

                const double err = errPrj(pose, cam, pix, pos);
                errs.push(err);
            }

            return (errs.size() > 0) ? medianVal(errs) : SP_INFINITY;
        }

        Col3 colMPnt(Mem1<ViewEx> &views, Mem1<MapPoint> &mpnts, const int m) {

            Vec3 vec = getVec(0.0, 0.0, 0.0);

            const Mem1<Int2> &index = mpnts[m].index;
            for (int i = 0; i < index.size(); i++) {
                const int v = index[i][0];
                const int f = index[i][1];

                const Vec2 pix = views[v].fts[f].pix;
                vec += getVec(acsc(views[v].img, pix.x, pix.y)) / index.size();
            }

            return getCol(vec);
        }

        void setMPnt(Mem1<ViewEx> &views, Mem1<MapPoint> &mpnts, const int m, const Vec3 &pos) {
            if (m + 1 > mpnts.size()) mpnts.extend(m + 1 - mpnts.size());
           
            mpnts[m].pos = pos;
            mpnts[m].col = colMPnt(views, mpnts, m);
            mpnts[m].err = errMPnt(views, mpnts, m);
        }

        void setMPnt(Mem1<ViewEx> &views, Mem1<MapPoint> &mpnts, const int m, const int v, const int f) {
            if (views[v].fts[f].mid >= 0) return;

            mpnts[m].index.push(Int2(v, f));
            mpnts[m].col = colMPnt(views, mpnts, m);
            mpnts[m].err = errMPnt(views, mpnts, m);
            views[v].fts[f].mid = m;
            views[v].mcnt++;
        }


        //--------------------------------------------------------------------------------
        // initialize
        //--------------------------------------------------------------------------------

        bool initStereo(Mem1<ViewEx> &views, Mem2<PairData> &pairs, Mem1<MapPoint> &mpnts) {

            // select pair
            PairData *pair = NULL;
            {
                const Mem1<PairData*> list = shuffle(getPairs(views, pairs, View::POSE_NULL, View::POSE_NULL));

                double maxv = 0.0;
                for (int i = 0; i < list.size(); i++) {
                    const int a = list[i]->a;
                    const int b = list[i]->b;

                    const Mem1<Vec2> pixs0 = getMatchPixs(views[a].fts, pairs(a, b).matches, true);
                    const Mem1<Vec2> pixs1 = getMatchPixs(views[b].fts, pairs(a, b).matches, false);
                    const double eval = evalStereo(views[a].cam, pixs0, views[b].cam, pixs1);

                    if (eval < maxv) continue;

                    maxv = eval;
                    pair = list[i];
                }
                if (maxv < MIN_STEREOEVAL) return false;
            }

            // initialize pair
            {
                const int a = pair->a;
                const int b = pair->b;

                const Mem1<Vec2> pixs0 = getMatchPixs(views[a].fts, pairs(a, b).matches, true);
                const Mem1<Vec2> pixs1 = getMatchPixs(views[b].fts, pairs(a, b).matches, false);

                Pose pose = zeroPose();
                if (calcPoseRANSAC(pose, views[a].cam, pixs0, views[b].cam, pixs1) == false) return false;

                setView(views, a, zeroPose());
                setView(views, b, pose);

                m_ipair.set(a, b);
            }

            return true;
        }

        void addNewMPnt(Mem1<ViewEx> &views, Mem2<PairData> &pairs, Mem1<MapPoint> &mpnts, const int a, const int b) {

            const bool init = (mpnts.size() == 0) ? true : false;

            const Mem1<Feature> &fts0 = views[a].fts;
            const Mem1<Feature> &fts1 = views[b].fts;
            const Mem1<int> &matches = pairs(a, b).matches;

            for (int f = 0; f < matches.size(); f++) {
                if (views[a].fts[f].mid >= 0) continue;

                int find = -1;
                for (int i = 0; i < views[a].links.size(); i++) {
                    const int v = views[a].links[i];
                    if (v == b) continue;

                    const int g = pairs(a, v).matches[f];
                    if (g < 0) continue;

                    const int m = views[v].fts[g].mid;
                    if (m < 0) continue;

                    //const double err = errPrj(views[a].pose, views[a].cam, fts0[f].pix, mpnts[m].pos);
                    //if (evalErr(err) < 1.0) continue;

                    find = m;
                    break;
                }
                if (find >= 0) {
                    setMPnt(views, mpnts, find, a, f);
                    continue;
                }

                const int g = matches[f];
                if (g < 0) continue;

                Vec3 pos;
                if (calcPnt3d(pos, views[a].pose, views[a].cam, fts0[f].pix, views[b].pose, views[b].cam, fts1[g].pix) == false) continue;

                const double err0 = errPrj(views[a].pose, views[a].cam, fts0[f].pix, pos);
                const double err1 = errPrj(views[b].pose, views[b].cam, fts1[g].pix, pos);
                if ((err0 + err1) / 2.0 > MPNT_PRJERR) continue;

                if (init == false) {
                    const Vec3 vec0 = unitVec(views[a].pose.trn - pos);
                    const Vec3 vec1 = unitVec(views[b].pose.trn - pos);
                    const double angle = acos(dotVec(vec0, vec1));
                    if (angle < MPNT_ANGLE) continue;
                }

                const int m = mpnts.size();
                setMPnt(views, mpnts, m, pos);
                setMPnt(views, mpnts, m, a, f);
                setMPnt(views, mpnts, m, b, g);
            }
        }

        //--------------------------------------------------------------------------------
        // update
        //--------------------------------------------------------------------------------
    
        bool updatePair(Mem1<ViewEx> &views, Mem2<PairData> &pairs, const int itmax) {
            SP_LOGGER_SET("updatePair");

            for (int it = 0; it < itmax; it++) {
                int a = -1;
                {
                    int minv = views.size() - 1;
                    for (int v = 0; v < views.size(); v++) {
                        if (views[v].icnt < minv) {
                            a = v;
                            minv = views[v].icnt;
                        }
                    }
                }
                if (a < 0) return false;

                Mem1<int> list;
                for (int b = 0; b < views.size(); b++) {
                    if (b != a && (pairs(a, b).a < 0 || pairs(a, b).b < 0)) {
                        list.push(b);
                    }
                }
                if (list.size() == 0) return false;

                list = shuffle(list);

                for (int i = 0; i < minVal(10, list.size()); i++) {
                    const int b = list[i];
                    initPair(views, pairs, a, b);
                    initPair(views, pairs, b, a);
                }
            }
            return true;
        }

        bool updateView(Mem1<ViewEx> &views, Mem2<PairData> &pairs, Mem1<MapPoint> &mpnts, const int update) {
            SP_LOGGER_SET("updateView");

            // select candidate pairs
            Mem1<PairData*> cand;
            {
                // [invalid, valid] pair
                Mem1<PairData*> list = shuffle(getPairs(views, pairs, View::POSE_NULL, View::POSE_VALID));
                if (list.size() == 0) return false;

                //pair = list[update % list.size()];

                auto compare_max = [](const void *a, const void *b) -> int{
                    return ((*static_cast<const PairData* const*>(a))->eval > (*static_cast<const PairData* const*>(b))->eval) ? -1 : +1;
                };

                sort(list, compare_max);
                for (int i = 0; i < minVal(3, list.size()); i++) {
                    cand.push(list[i]);
                }
            }

            // calc pose & add pnts
            {
                const int a = cand[0]->a;
                const int b = cand[0]->b;

                const Mem1<int> &matches = pairs(b, a).matches;

                Mem1<Vec2> pixs;
                Mem1<Vec3> objs;
                Mem1<Int2> index;
                for (int g = 0; g < matches.size(); g++) {
                    const int f = matches[g];
                    const int m = views[b].fts[g].mid;
                    if (f < 0 || m < 0) continue;

                    pixs.push(views[a].fts[f].pix);
                    objs.push(mpnts[m].pos);
                    index.push(Int2(f, m));
                }
                if (index.size() < MIN_POSEPNT) return false;

                Pose pose = zeroPose();
                if (calcPoseRANSAC(pose, views[a].cam, pixs, objs) == false) return false;

                const Mem1<double> errs = errPrj(pose, views[a].cam, pixs, objs);
                const double eval = evalErr(errs, MPNT_PRJERR);
                if (eval < 0.5) return false;

                // add index
                for (int i = 0; i < index.size(); i++) {
                    const int f = index[i][0];
                    const int m = index[i][1];

                    const double err = errPrj(pose, views[a].cam, pixs[i], objs[i]);
                    if (evalErr(err) == 0.0) continue;

                    setMPnt(views, mpnts, m, a, f);
                }

                setView(views, a, pose);

                addNewMPnt(views, pairs, mpnts, a, b);
            }
            return true;
        }

        bool updateMPnt1(Mem1<ViewEx> &views, Mem2<PairData> &pairs, Mem1<MapPoint> &mpnts, const int update) {
            SP_LOGGER_SET("updateMPnt1");

            // select pair
            PairData *pair = NULL;
            {
                // [valid, valid] pair
                const Mem1<PairData*> list = shuffle(getPairs(views, pairs, View::POSE_VALID, View::POSE_VALID));
                if (list.size() == 0) return false;

                pair = list[update % list.size()];
            }

            addNewMPnt(views, pairs, mpnts, pair->a, pair->b);

            return true;
        }

        bool updateMPnt2(Mem1<ViewEx> &views, Mem2<PairData> &pairs, Mem1<MapPoint> &mpnts, const int update) {
            SP_LOGGER_SET("updateMPnt2");

            srand(update);

            for (int i = 0; i < minVal(MAX_MPNTUPDATE, mpnts.size()); i++) {
                const int m = rand() % mpnts.size();

                const Mem1<Int2> &index = mpnts[m].index;
                if (index.size() < 2) continue;

                Mem1<Pose> poses(index.size());
                Mem1<CamParam> cams(index.size());
                Mem1<Vec2> pixs(index.size());
                for (int i = 0; i < index.size(); i++) {
                    const int v = index[i][0];
                    const int f = index[i][1];
                    poses[i] = views[v].pose;
                    cams[i] = views[v].cam;
                    pixs[i] = views[v].fts[f].pix;
                }

                Vec3 pos = getVec(0.0, 0.0, 0.0);
                if (calcPnt3dRANSAC(pos, poses, cams, pixs) == false) continue;
                    
                Mem1<double> errs;
                for (int v = 0; v < index.size(); v++) {
                    const double err = errPrj(poses[v], cams[v], pixs[v], pos);
                    errs.push(err);
                }
                if (medianVal(errs) > MPNT_PRJERR) continue;

                Mem1<double> angles;
                for (int a = 0; a < index.size(); a++) {
                    for (int b = a + 1; b < index.size(); b++) {
                        if (errs[a] > MPNT_PRJERR || errs[b] > MPNT_PRJERR) continue;

                        const Vec3 vec0 = unitVec(views[a].pose.trn - pos);
                        const Vec3 vec1 = unitVec(views[b].pose.trn - pos);
                        const double angle = acos(dotVec(vec0, vec1));
                        angles.push(angle);
                    }
                }
                                
                if (maxVal(angles) < MPNT_ANGLE) continue;


                setMPnt(views, mpnts, m, pos);
            }

            return true;
        }

        bool updatePose(Mem1<ViewEx> &views, Mem2<PairData> &pairs, Mem1<MapPoint> &mpnts, const int update) {
            SP_LOGGER_SET("updatePose");

            Mem1<int> list;
            for (int v = 0; v < views.size(); v++) {
                if (v == m_ipair[0] || v == m_ipair[1]) continue;
                if (views[v].state != View::POSE_VALID) continue;
                list.push(v);
            }
            list = shuffle(list, update);

            for (int i = 0; i < minVal(MAX_POSEUPDATE, list.size()); i++) {
                const int v = list[i];

                Mem1<Vec2> pixs;
                Mem1<Vec3> objs;
                Mem1<double> errs;
                for (int f = 0; f < views[v].fts.size(); f++) {
                    const int m = views[v].fts[f].mid;
                    if (m < 0) continue;

                    pixs.push(views[v].fts[f].pix);
                    objs.push(mpnts[m].pos);
                    errs.push(mpnts[m].err);
                }

                Pose pose = views[v].pose;
                if (calcPoseRANSAC(pose, views[v].cam, pixs, objs) == false) return false;
                //if (refinePoseEx(pose, views[v].cam, pixs, objs, errs) == false) return false;

                setView(views, v, pose);
            }
            return true;
        }
       
        
        //--------------------------------------------------------------------------------
        // modules
        //--------------------------------------------------------------------------------
      
        bool refinePoseEx(Pose &pose, const CamParam &cam, const Mem1<Vec2> &pixs, const Mem1<Vec3> &objs, const Mem1<double> &weights, const int maxit = 10) {
            SP_ASSERT(pixs.size() == objs.size());

            const int num = pixs.size();

            const int unit = 3;
            if (num < unit) return false;

            Mat J(2 * num, 6);
            Mat E(2 * num, 1);
            Mem1<double> errs(num);

            for (int it = 0; it < maxit; it++) {
                for (int i = 0; i < num; i++) {
                    jacobPoseToPix(&J(i * 2, 0), cam, pose, objs[i]);

                    const Vec2 err = pixs[i] - mulCamD(cam, prjVec(pose * objs[i]));
                    E(i * 2 + 0, 0) = err.x;
                    E(i * 2 + 1, 0) = err.y;
                    errs[i] = normVec(err) * weights[i];
                }
                Mat delta;
                if (solveEq(delta, J, E, errs) == false) return false;

                pose = sp::updatePose(pose, delta.ptr);
            }

            return true;
        }
    };

}
#endif