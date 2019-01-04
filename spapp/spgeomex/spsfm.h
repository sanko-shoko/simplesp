//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_SFM_H__
#define __SP_SFM_H__

#include "spcore/spcore.h"
#include "spapp/spimgex/spfeature.h"
#include "spapp/spgeom/spgeom.h"

namespace sp {

    class SfM {

    private:
        class MatchPair;

        class ViewEx : public View {
        public:

            // pose state
            enum PoseState {
                POSE_NULL = 0,
                POSE_HINT = 1, 
                POSE_VALID = 2
            };
            PoseState state;

            // links
            Mem1<ViewEx*> views;
            Mem1<MatchPair*> pairs;

            // init pair count
            int icnt;

            // update counter
            int upcnt;

            // fix flag
            bool fix;

            ViewEx() : View() {
                state = POSE_NULL;

                icnt = 0;

                upcnt = 0;
                fix = false;
            }

            ViewEx(const ViewEx &view) {
                *this = view;
            }

            ViewEx& operator = (const ViewEx &view) {
                static_cast<View>(*this) = static_cast<View>(view);
                state = view.state;

                views = view.views;
                pairs = view.pairs;

                icnt = view.icnt;

                upcnt = view.upcnt;

                fix = view.fix;

                return *this;
            }
        };

        class MatchPair {
        public:

            // pair id
            int a, b;

            // match data
            Mem1<int> matches;

            // match features eval
            double eval;

            MatchPair() {
                a = -1;
                b = -1;
                eval = -1.0;
            }

            MatchPair(const MatchPair &pair) {
                *this = pair;
            }

            MatchPair& operator = (const MatchPair &pair) {
                a = pair.a;
                b = pair.b;
                matches = pair.matches;
                eval = pair.eval;
                return *this;
            }
        };


    public:

        enum MODE {
            MODE_NULL = 0,
            MODE_SERIAL = 1
        };


    private:

        //--------------------------------------------------------------------------------
        // sfm data
        //--------------------------------------------------------------------------------

        // update counter
        int m_update;

        MODE m_mode;

        // key views
        Mem1<ViewEx*> m_views;

        // match pair matrix
        Mem2<MatchPair*> m_pairs;

        // map points
        Mem1<MapPnt*> m_mpnts;

        Mem1<ViewEx*> m_queue;

    private:

        //--------------------------------------------------------------------------------
        // memory pool
        //--------------------------------------------------------------------------------

        MemP<ViewEx> _viewsPool;

        MemP<MatchPair> _pairsPool;

        MemP<MapPnt> _mpntsPool;

    public:


        SfM() {
            clear();
        }

        void clear() {
            m_mode = MODE_NULL;

            m_update = 0;

            m_views.clear();
            m_pairs.clear();
            m_mpnts.clear();

            _viewsPool.clear();
            _pairsPool.clear();
            _mpntsPool.clear();

            m_queue.clear();
        }


        //--------------------------------------------------------------------------------
        // input parameter
        //--------------------------------------------------------------------------------

        void setMode(const MODE &mode) {
            m_mode = mode;
        }


        //--------------------------------------------------------------------------------
        // output parameter
        //--------------------------------------------------------------------------------

        int vsize() const {
            return m_views.size();
        }

        const View* getView(const int i) const {
            return (i >= 0 && i < m_views.size()) ? m_views[i] : NULL;
        }

        int msize() const {
            return m_mpnts.size();
        }

        const MapPnt* getMPnt(const int i) const {
            return (i >= 0 && i < m_mpnts.size()) ? m_mpnts[i] : NULL;
        }


        //--------------------------------------------------------------------------------
        // execute
        //--------------------------------------------------------------------------------

        void addView(const CamParam &cam, const Mem2<Col3> &img, const Pose *hint = NULL) {

            addView(m_views, cam, img, hint);
        }

        bool update(const int itmax = 1) {

            for (int it = 0; it < itmax; it++) {
                if (_update() == false) return false;
            }
            return true;
        }


    private:

        double MIN_MATCHEVAL = 0.2;
        double MIN_STEREOEVAL = 0.4;

        int MIN_POSEPNT = 10;

        double MPNT_PRJERR = 5.0;
        double MPNT_ANGLE = 2.0 * SP_PI / 180.0;

        int MAX_UPDATE = 3;

        double MAX_NEARPOSE = 30.0 * SP_PI / 180.0;


    private:

        //--------------------------------------------------------------------------------
        // execute main flow
        //--------------------------------------------------------------------------------

        bool _update() {

            try {
                initMem();

                const int seed = m_update;

                if (m_views.size() < 2) throw "input size < 2";
                
                if (m_update == 0) {
                    // update match pair
                    updatePair(m_views, m_pairs, m_views.size());

                    // calc first stere pose
                    if (firstPose(m_views) == false) throw "firstPose";
                }
                else {
                    updatePair(m_views, m_pairs, 1);

                    // update view [invalid -> valid]
                    updateView(m_views);
                }

                Mem1<ViewEx*> uplist;
                {
                    for (int v = 0; v < m_views.size(); v++) {
                        if (m_views[v]->state == ViewEx::POSE_VALID) uplist.push(m_views[v]);
                    }

                    auto compare_min = [](const void *a, const void *b) -> int {
                        return ((*static_cast<const ViewEx* const*>(a))->upcnt >(*static_cast<const ViewEx* const*>(b))->upcnt) ? +1 : -1;
                    };

                    sort(uplist, compare_min);

                    if (uplist[0]->upcnt == m_views.size()) uplist = shuffle(uplist, seed);
                    uplist = uplist.part(0, MAX_UPDATE);
                }

                for(int i = 0; i < uplist.size(); i++){
                    ViewEx &view = *uplist[i];

                    // update map points
                    updateMPnt(m_views, m_mpnts, view);

                    // update view pose
                    updatePose(m_views, m_mpnts, view);

                    view.upcnt = minVal(m_views.size(), view.upcnt + 1);
                }

                m_update++;
            }
            catch (const char *str) {
                SP_PRINTD("SfM.update [%s]\n", str);

                return false;
            }

            return true;
        }

        void initMem() {
            m_views.push(m_queue);
            m_queue.clear();

            if (m_views.size() > m_pairs.dsize[0]) {
                const Mem2<MatchPair*> tmp = m_pairs;

                m_pairs.resize(2 * m_views.size(), 2 * m_views.size());
                m_pairs.zero();

                for (int y = 0; y < tmp.dsize[1]; y++) {
                    for (int x = 0; x < tmp.dsize[0]; x++) {
                        m_pairs(x, y) = tmp(x, y);
                    }
                }
            }
        }

        //--------------------------------------------------------------------------------
        // view (v: view id)
        //--------------------------------------------------------------------------------

        void addView(Mem1<ViewEx*> &views, const CamParam &cam, const Mem2<Col3> &img, const Pose *hint) {
            if (cmpSize(2, cam.dsize, img.dsize) == false) return;

            ViewEx &view = *_viewsPool.malloc();
            view.img = img;
            view.cam = cam;
            view.ftrs = SIFT::getFtrs(img);

            if (hint != NULL) {
                view.state = ViewEx::POSE_HINT;
                view.pose = *hint;
            }

            m_queue.push(&view);
        }

        void setView(ViewEx &view, const Pose &pose) {
            view.state = ViewEx::POSE_VALID;
            view.pose = pose;
            view.valid = true;
        }

        //--------------------------------------------------------------------------------
        // pair (a, b: view id)
        //--------------------------------------------------------------------------------

        void initPair(Mem1<ViewEx*> &views, Mem2<MatchPair*> &pairs, const int a, const int b) {
            MatchPair &pair = *_pairsPool.malloc();

            pair.a = a;
            pair.b = b;

            pair.matches = findMatch(views[a]->ftrs, views[b]->ftrs);
            pair.eval = getMatchEval(pair.matches);

            if (pair.eval > MIN_MATCHEVAL) {
                views[a]->views.push(views[b]);
                views[a]->pairs.push(&pair);
            }

            views[a]->icnt++;
            pairs(a, b) = &pair;

            //printf("[%d %d]: size %d, cnt %d, eval %.2lf\n", pair.a, pair.b, pair.matches.size(), getMatchCnt(pair.matches), pair.eval);
        }

        Mem1<MatchPair*> getPairs(Mem1<ViewEx*> &views, const ViewEx::PoseState &stateA, const ViewEx::PoseState &stateB) {
            Mem1<MatchPair*> ptrs;

            for (int a = 0; a < views.size(); a++) {
                if (views[a]->state != stateA) continue;

                for (int i = 0; i < views[a]->pairs.size(); i++) {
                    const int b = views[a]->pairs[i]->b;
                    if (views[b]->state != stateB) continue;

                    if (stateA == stateB && b < a) continue;

                    ptrs.push(views[a]->pairs[i]);
                }
            }
            return ptrs;
        }

        bool updatePair(Mem1<ViewEx*> &views, Mem2<MatchPair*> &pairs, const int itmax) {

            for (int it = 0; it < itmax; it++) {
                int a = -1;
                {
                    int minv = views.size() - 1;
                    for (int v = 0; v < views.size(); v++) {
                        if (views[v]->icnt < minv) {
                            a = v;
                            minv = views[v]->icnt;
                        }
                    }
                }
                if (a < 0) return false;

                Mem1<int> list;
                for (int b = 0; b < views.size(); b++) {
                    if (b != a && pairs(a, b) == NULL) {
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

                if (msize() > 0 && views[a]->state == ViewEx::POSE_HINT) {
                    const int v = searchNearViewId(views[a]->pose);
                    if (pairs(a, v) != NULL) continue;

                    initPair(views, pairs, a, v);
                    initPair(views, pairs, v, a);
                }

            }
            return true;
        }

        //--------------------------------------------------------------------------------
        // map point
        //--------------------------------------------------------------------------------

        void addMPnt(Mem1<MapPnt*> &mpnts) {
            MapPnt &mpnt = *_mpntsPool.malloc();
            mpnts.push(&mpnt);
        }
        
        void addMPnt(Mem1<MapPnt*> &mpnts, const Vec3 &pos) {
            MapPnt &mpnt = *_mpntsPool.malloc();

            setMPnt(mpnt, pos);
            mpnts.push(&mpnt);
        }

        void setMPnt(MapPnt &mpnt, const Vec3 &pos) {
            mpnt.valid = true;
            mpnt.pos = pos;
            mpnt.updateCol();
            mpnt.updateErr();
        }

        void setMPnt(MapPnt &mpnt, ViewEx& view, const int f) {
            if (view.ftrs[f].mpnt != NULL) return;

            for (int i = 0; i < mpnt.views.size(); i++) {
                if (mpnt.views[i] == &view) return;
            }

            mpnt.views.push(&view);
            mpnt.ftrs.push(&view.ftrs[f]);
            mpnt.updateCol();
            mpnt.updateErr();

            view.ftrs[f].mpnt = &mpnt;
        }

        //--------------------------------------------------------------------------------
        // initialize
        //--------------------------------------------------------------------------------

        bool firstPose(Mem1<ViewEx*> &views) {

            // select pair
            MatchPair *pair = NULL;
            {
                // [invalid, invalid] pair
                Mem1<MatchPair*> list;
                list.push(getPairs(views, ViewEx::POSE_NULL, ViewEx::POSE_NULL));
                list.push(getPairs(views, ViewEx::POSE_HINT, ViewEx::POSE_NULL));
                list.push(getPairs(views, ViewEx::POSE_NULL, ViewEx::POSE_HINT));
            
                double maxv = 0.0;
                for (int i = 0; i < list.size(); i++) {
                    const int a = list[i]->a;
                    const int b = list[i]->b;

                    if ((m_mode & MODE_SERIAL)) {
                        if (a != 0) continue;
                    }

                    const Mem1<Vec2> pixs0 = getMatchPixs(views[a]->ftrs, list[i]->matches, true);
                    const Mem1<Vec2> pixs1 = getMatchPixs(views[b]->ftrs, list[i]->matches, false);
                    const double eval = evalStereo(views[b]->cam, pixs1, views[a]->cam, pixs0, MPNT_ANGLE * 1.2);

                    if (eval > maxv) {
                        maxv = eval;
                        pair = list[i];
                    }
                }

                if (maxv < MIN_STEREOEVAL) return false;

            }

            // initialize pair
            {
                const int a = pair->a;
                const int b = pair->b;

                const Mem1<Vec2> pixs0 = getMatchPixs(views[a]->ftrs, pair->matches, true);
                const Mem1<Vec2> pixs1 = getMatchPixs(views[b]->ftrs, pair->matches, false);

                Pose pose = zeroPose();
                if (calcPoseRANSAC(pose, views[b]->cam, pixs1, views[a]->cam, pixs0) == false) return false;
               
                setView(*views[a], zeroPose());
                setView(*views[b], pose);

                //views[a]->fix = true;
            }

            return true;
        }


        //--------------------------------------------------------------------------------
        // update
        //--------------------------------------------------------------------------------
     
        bool updateView(Mem1<ViewEx*> &views) {

            Mem1<MatchPair*> hypos;
            {
                // [invalid, valid] pair
                Mem1<MatchPair*> list = getPairs(views, ViewEx::POSE_NULL, ViewEx::POSE_VALID);

                //pair = list[seed % list.size()];

                auto compare_max = [](const void *a, const void *b) -> int {
                    return ((*static_cast<const MatchPair* const*>(a))->eval > (*static_cast<const MatchPair* const*>(b))->eval) ? -1 : +1;
                };
                sort(list, compare_max);

                const int select = 3;
                {
                    hypos.push(list.part(0, select));
                }

                if (list.size() > select) {
                    list = shuffle(list.part(select, list.size() - select));
                    hypos.push(list.part(0, select));
                }
            }

            {
                // [invalid, valid] pair
                Mem1<MatchPair*> list = getPairs(views, ViewEx::POSE_HINT, ViewEx::POSE_VALID);
                hypos.push(list);
            }
            if (hypos.size() == 0) return false;

            // calc pose & add pnts
            {
                Pose pose;

                int bestid = -1;
                double maxe = 0.0;
                for (int i = 0; i < hypos.size(); i++) {

                    const int a = hypos[i]->a;
                    const int b = hypos[i]->b;
                    const Mem1<int> &matches = hypos[i]->matches;

                    Mem1<Vec2> pixs;
                    Mem1<Vec3> objs;
                    for (int f = 0; f < matches.size(); f++) {
                        const int g = matches[f];
                        if (g < 0) continue;

                        MapPnt *mpnt = views[b]->ftrs[g].mpnt;
                        if (mpnt == NULL || mpnt->valid == false) continue;

                        pixs.push(views[a]->ftrs[f].pix);
                        objs.push(mpnt->pos);
                    }
                    //printf("%d %d %d\n", a, b, pixs.size());
                    if (pixs.size() < MIN_POSEPNT) continue;

                    Pose tmp = zeroPose();
                    if (calcPoseRANSAC(tmp, views[a]->cam, pixs, objs) == false) continue;

                    const Mem1<double> errs = calcPrjErr(tmp, views[a]->cam, pixs, objs);
                    const double eval = evalErr(errs, MPNT_PRJERR);

                    if (eval > maxe) {
                        maxe = eval;
                        bestid = i;

                        pose = tmp;
                    }
                }
                if (bestid < 0) return false;

                {
                    const int a = hypos[bestid]->a;
                    const int b = hypos[bestid]->b;
                    const Mem1<int> &matches = hypos[bestid]->matches;

                    for (int f = 0; f < matches.size(); f++) {
                        const int g = matches[f];
                        if (g < 0) continue;

                        MapPnt *mpnt = views[b]->ftrs[g].mpnt;
                        if (mpnt == NULL || mpnt->valid == false) continue;

                        const double err = calcPrjErr(pose, views[a]->cam, views[a]->ftrs[f].pix, mpnt->pos);
                        if (err > MPNT_PRJERR) continue;

                        setMPnt(*mpnt, *views[a], f);
                    }

                    setView(*views[a], pose);
                }
            }
            return true;
        }

        bool updateMPnt(Mem1<ViewEx*> &views, Mem1<MapPnt*> &mpnts, ViewEx &view) {

            for (int f = 0; f < view.ftrs.size(); f++) {

                // add new
                if (view.ftrs[f].mpnt == NULL) {

                    MapPnt *find = NULL;

                    Mem1<ViewEx*> vrefs;
                    Mem1<int> frefs;

                    for (int i = 0; i < view.views.size(); i++) {

                        ViewEx* vref = view.views[i];
                        if (vref->valid == false) continue;

                        const int g = view.pairs[i]->matches[f];
                        if (g < 0) continue;

                        MapPnt *mpnt = vref->ftrs[g].mpnt;
                        if (mpnt != NULL) {
                            find = mpnt;
                            break;
                        }
                        else {
                            vrefs.push(vref);
                            frefs.push(g);
                        }
                    }

                    if (find != NULL) {
                        setMPnt(*find, view, f);
                    }
                    else if (vrefs.size() > 0) {

                        const int m = mpnts.size();
                        addMPnt(mpnts);
                        setMPnt(*mpnts[m], view, f);

                        for (int i = 0; i < vrefs.size(); i++) {
                            setMPnt(*mpnts[m], *vrefs[i], frefs[i]);
                        }
                    }
                }

                // refine 
                if (view.ftrs[f].mpnt != NULL) {

                    MapPnt *mpnt = view.ftrs[f].mpnt;

                    const Mem1<View*> &mviews = mpnt->views;
                    const Mem1<Ftr*> &mfts = mpnt->ftrs;

                    const int num = mviews.size();
                    if (num < 2) continue;

                    Mem1<Pose> poses(num);
                    Mem1<CamParam> cams(num);
                    Mem1<Vec2> pixs(num);
                    for (int i = 0; i < num; i++) {
                        poses[i] = mviews[i]->pose;
                        cams[i] = mviews[i]->cam;
                        pixs[i] = mfts[i]->pix;
                    }
                    Vec3 pos = getVec(0.0, 0.0, 0.0);
                    if (calcPnt3dRANSAC(pos, poses, cams, pixs) == false) continue;

                    const Mem1<double> errs = calcPrjErr(poses, cams, pixs, pos);
                    if (medianVal(errs) > MPNT_PRJERR) continue;

                    Mem1<double> angles;
                    for (int a = 0; a < num; a++) {
                        for (int b = a + 1; b < num; b++) {
                            if (errs[a] > MPNT_PRJERR || errs[b] > MPNT_PRJERR) continue;

                            const Vec3 vec0 = unitVec(poses[a].trn - pos);
                            const Vec3 vec1 = unitVec(poses[b].trn - pos);
                            const double angle = acos(dotVec(vec0, vec1));

                            angles.push(angle);
                        }
                    }

                    if (maxVal(angles) < MPNT_ANGLE) continue;

                    setMPnt(*mpnt, pos);
                }
            }

            return true;
        }

        bool updatePose(Mem1<ViewEx*> &views, Mem1<MapPnt*> &mpnts, ViewEx &view) {
            if (view.fix == true) return false;

            Mem1<Vec2> pixs;
            Mem1<Vec3> objs;
            Mem1<double> errs;
            for (int f = 0; f < view.ftrs.size(); f++) {
                MapPnt *mpnt = view.ftrs[f].mpnt;
                if (mpnt == NULL || mpnt->valid == false) continue;

                pixs.push(view.ftrs[f].pix);
                objs.push(mpnt->pos);
                errs.push(mpnt->err);
            }

            Pose pose = view.pose;
            if (calcPoseRANSAC(pose, view.cam, pixs, objs) == false) return false;

            setView(view, pose);

            return true;
        }

    public:

        //--------------------------------------------------------------------------------
        // util
        //--------------------------------------------------------------------------------

        int searchNearViewId(const Pose &pose) {
            int id = -1;
            double maxd = SP_INFINITY;
            for (int i = 0; i < m_views.size(); i++) {
                if ((m_views[i]->state & ViewEx::POSE_VALID) == false) continue;
                const Pose hypo = m_views[i]->pose;

                const double angle = difRot(pose.rot, hypo.rot, 2);
                if (angle > MAX_NEARPOSE) continue;

                const double d = normVec(pose.trn - hypo.trn);
                if (d < maxd) {
                    maxd = d;
                    id = i;
                }
            }
            return id;
        }

        const View* searchNearView(const Pose &pose) {
            ViewEx *view = NULL;
            const int id = searchNearViewId(pose);
            if (id >= 0) {
                view = m_views[id];
            }
            return view;
        }

        const Pose* getlatestPose() {
            Pose *pose = NULL;
            for (int i = m_views.size() - 1; i >= 0; i--) {
                if (m_views[i]->valid == true) {
                    pose = &m_views[i]->pose;
                    break;
                }
            }
            return pose;
        }

    };

}
#endif