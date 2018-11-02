//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_SMF_H__
#define __SP_SMF_H__

#include "spcore/spcore.h"

#include "spapp/spdata/spscene.h"
#include "spapp/spgeom/spgeometry.h"

namespace sp {
 
    class SfM {

    private:

        class ViewEx : public View {
        public:

            // pose state
            enum PoseState {
                POSE_NULL = 0, POSE_HINT = 1, POSE_VALID = 2
            };
            PoseState state;

            // view links
            Mem1<int> links;
            
            // init pair count
            int icnt;

            // map points count
            int mcnt;

            // fix flag
            bool fix;

            ViewEx() : View(){
                state = POSE_NULL;
             
                icnt = 0;
                mcnt = 0;
                fix = false;
            }

            ViewEx(const ViewEx &view) {
                *this = view;
            }

            ViewEx& operator = (const ViewEx &view) {
                static_cast<View>(*this) = static_cast<View>(view);
                state = view.state;

                links = view.links;
                icnt = view.icnt;
                mcnt = view.mcnt;

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

    private:
        //--------------------------------------------------------------------------------
        // sfm data
        //--------------------------------------------------------------------------------

        // update counter
        int m_update;

        // key views
        Mem1<ViewEx*> m_views;

        // match pair matrix
        Mem2<MatchPair*> m_pairs;

        // map points
        Mem1<MapPnt*> m_mpnts;
        

    private:
        //--------------------------------------------------------------------------------
        // memory pool & stack
        //--------------------------------------------------------------------------------
      
        MemP<ViewEx> _viewsPool;

        MemP<MatchPair> _pairsPool;

        MemP<MapPnt> _mpntsPool;

        Mem1<ViewEx*> _viewsStack;


    public:
        SP_HOLDER_INSTANCE;

        SfM() {
            clear();
        }

        void clear() {
            m_update = 0;

            m_views.clear();
            m_pairs.clear();
            m_mpnts.clear();

            _viewsPool.clear();
            _pairsPool.clear();
            _mpntsPool.clear();

            _viewsStack.clear();
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

        void addView(const CamParam &cam, const Mem2<Col3> &img) {

            addView(m_views, cam, img);
        }

        bool update(const int itmax = 1) {

            for (int it = 0; it < itmax; it++) {
                if (_update() == false) return false;
            }
            return true;
        }


        //--------------------------------------------------------------------------------
        // util
        //--------------------------------------------------------------------------------

        bool save(const char *path) {
            Mem1<Vec3> pnts;
            Mem1<Col3> cols;
            for (int i = 0; i < m_mpnts.size(); i++) {
                pnts.push(m_mpnts[i]->pos);
                cols.push(m_mpnts[i]->col);
            }

            return savePLY(path, pnts, cols);
        }

    private:

        double MIN_MATCHEVAL = 0.2;
        double MIN_STEREOEVAL = 0.4;
        
        int MIN_POSEPNT = 10;
        
        double MPNT_PRJERR = 5.0;
        double MPNT_ANGLE = 3.0 * SP_PI / 180.0;

        int MAX_POSEUPDATE = 20;
        int MAX_MPNTUPDATE = 1000;

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
                    if (firstPose(m_views, m_pairs, m_mpnts) == false) throw "firstPose";
                }
                else {
                    updatePair(m_views, m_pairs, 1);

                    // update view [invalid -> valid]
                    updateView(m_views, m_pairs, m_mpnts, seed);
                }

                {
                    // add new map points
                    addNewMPnt(m_views, m_pairs, m_mpnts, seed);

                    // update map points [refine]
                    updateMPnt(m_views, m_pairs, m_mpnts, seed);

                    // update view pose
                    updatePose(m_views, m_pairs, m_mpnts, seed);
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
            m_views.push(_viewsStack);
            _viewsStack.clear();

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

        void addView(Mem1<ViewEx*> &views, const CamParam &cam, const Mem2<Col3> &img) {
            ViewEx &view = *_viewsPool.malloc();
            view.img = img;
            view.cam = cam;
            view.fts = SIFT::getFeatures(img);

            _viewsStack.push(&view);
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

            pair.matches = findMatch(views[a]->fts, views[b]->fts);
            pair.eval = getMatchEval(pair.matches);

            if(pair.eval > MIN_MATCHEVAL){
                views[a]->links.push(b);
            }

            views[a]->icnt++;
            pairs(a, b) = &pair;

            //printf("[%d %d]: size %d, cnt %d, eval %.2lf\n", pair.a, pair.b, pair.matches.size(), getMatchCnt(pair.matches), pair.eval);
        }

        Mem1<MatchPair*> getPairs(Mem1<ViewEx*> &views, Mem2<MatchPair*> &pairs, const ViewEx::PoseState &stateA, const ViewEx::PoseState &stateB) {
            Mem1<MatchPair*> ptrs;

            for (int a = 0; a < views.size(); a++) {
                if (views[a]->state != stateA) continue;

                for (int i = 0; i < views[a]->links.size(); i++) {
                    const int b = views[a]->links[i];
                    if (views[b]->state != stateB) continue;

                    if (stateA == stateB && b < a) continue;
                    if (pairs(a, b) == NULL) continue;

                    ptrs.push(pairs(a, b));
                }
            }
            return ptrs;
        }


        //--------------------------------------------------------------------------------
        // map point
        //--------------------------------------------------------------------------------

        void addMPnt(Mem1<MapPnt*> &mpnts, const Vec3 &pos) {
            MapPnt &mpnt = *_mpntsPool.malloc();
           
            setMPnt(mpnt, pos);
            mpnts.push(&mpnt);
        }

        void setMPnt(MapPnt &mpnt, const Vec3 &pos) {
            mpnt.pos = pos;
            mpnt.updateCol();
            mpnt.updatePrjErr();
        }

        void setMPnt(MapPnt &mpnt, ViewEx& view, const int f) {
            if (view.fts[f].mpnt != NULL) return;

            for (int i = 0; i < mpnt.views.size(); i++) {
                if (mpnt.views[i] == &view) return;
            }

            mpnt.views.push(&view);
            mpnt.fts.push(&view.fts[f]);
            mpnt.updateCol();
            mpnt.updatePrjErr();

            view.mcnt++;
            view.fts[f].mpnt = &mpnt;
        }


        //--------------------------------------------------------------------------------
        // initialize
        //--------------------------------------------------------------------------------

        bool firstPose(Mem1<ViewEx*> &views, Mem2<MatchPair*> &pairs, Mem1<MapPnt*> &mpnts) {

            // select pair
            MatchPair *pair = NULL;
            {
                // [invalid, invalid] pair
                Mem1<MatchPair*> list = getPairs(views, pairs, ViewEx::POSE_NULL, ViewEx::POSE_NULL);

                double maxv = 0.0;
                for (int i = 0; i < list.size(); i++) {
                    const int a = list[i]->a;
                    const int b = list[i]->b;

                    const Mem1<Vec2> pixs0 = getMatchPixs(views[a]->fts, pairs(a, b)->matches, true);
                    const Mem1<Vec2> pixs1 = getMatchPixs(views[b]->fts, pairs(a, b)->matches, false);
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
                const Mem1<Vec2> pixs0 = getMatchPixs(views[a]->fts, pairs(a, b)->matches, true);
                const Mem1<Vec2> pixs1 = getMatchPixs(views[b]->fts, pairs(a, b)->matches, false);

                Pose pose = zeroPose();
                if (calcPoseRANSAC(pose, views[b]->cam, pixs1, views[a]->cam, pixs0) == false) return false;

                setView(*views[a], zeroPose());
                setView(*views[b], pose);

                views[a]->fix = true;
            }

            return true;
        }

        //--------------------------------------------------------------------------------
        // update
        //--------------------------------------------------------------------------------
    
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
            }
            return true;
        }

        bool updateView(Mem1<ViewEx*> &views, Mem2<MatchPair*> &pairs, Mem1<MapPnt*> &mpnts, const int seed) {

            Mem1<MatchPair*> hypo;
            {
                // [invalid, valid] pair
                Mem1<MatchPair*> list = getPairs(views, pairs, ViewEx::POSE_NULL, ViewEx::POSE_VALID);
                if (list.size() == 0) return false;

                //pair = list[seed % list.size()];

                auto compare_max = [](const void *a, const void *b) -> int{
                    return ((*static_cast<const MatchPair* const*>(a))->eval > (*static_cast<const MatchPair* const*>(b))->eval) ? -1 : +1;
                };
                sort(list, compare_max);

                const int select = 3;
                {
                    hypo.push(list.slice(0, 0, select));
                }

                if (list.size() > select) {
                    list = shuffle(list.slice(0, 3, list.size()));
                    hypo.push(list.slice(0, 0, select));
                }
            }

            // calc pose & add pnts
            {
                Pose pose;

                int bestid = -1;
                double maxe = 0.0;
                for (int i = 0; i < hypo.size(); i++) {
                    const int a = hypo[i]->a;
                    const int b = hypo[i]->b;

                    const Mem1<int> &matches = pairs(b, a)->matches;

                    Mem1<Vec2> pixs;
                    Mem1<Vec3> objs;
                    for (int g = 0; g < matches.size(); g++) {
                        const int f = matches[g];
                        MapPnt *mpnt = views[b]->fts[g].mpnt;
                        if (f < 0 || mpnt == NULL) continue;

                        pixs.push(views[a]->fts[f].pix);
                        objs.push(mpnt->pos);
                    }
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
                if(bestid < 0) return false;

                {
                    const int a = hypo[bestid]->a;
                    const int b = hypo[bestid]->b;

                    const Mem1<int> &matches = pairs(b, a)->matches;

                    for (int g = 0; g < matches.size(); g++) {
                        const int f = matches[g];
                        MapPnt *mpnt = views[b]->fts[g].mpnt;
                        if (f < 0 || mpnt == NULL) continue;

                        const double err = calcPrjErr(pose, views[a]->cam, views[a]->fts[f].pix, mpnt->pos);
                        if (err > MPNT_PRJERR) continue;

                        setMPnt(*mpnt, *views[a], f);
                    }

                    setView(*views[a], pose);

                    addNewMPnt(views, pairs, mpnts, a, b);
                }
            }
            return true;
        }

        bool updateMPnt(Mem1<ViewEx*> &views, Mem2<MatchPair*> &pairs, Mem1<MapPnt*> &mpnts, const int seed) {

            Mem1<MapPnt*> list;
            {
                list = shuffle(mpnts, seed).slice(0, 0, MAX_MPNTUPDATE);
            }

            for (int i = 0; i < list.size(); i++) {
                MapPnt *mpnt = list[i];

                const Mem1<View*> &mviews = mpnt->views;
                const Mem1<Feature*> &mfts= mpnt->fts;

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

                Mem1<double> errs;
                for (int v = 0; v < num; v++) {
                    const double err = calcPrjErr(poses[v], cams[v], pixs[v], pos);
                    errs.push(err);
                }
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

            return true;
        }

        bool updatePose(Mem1<ViewEx*> &views, Mem2<MatchPair*> &pairs, Mem1<MapPnt*> &mpnts, const int seed) {

            Mem1<ViewEx*> list;
            {
                for (int v = 0; v < views.size(); v++) {
                    ViewEx *view = views[v];
                    if (view->fix == true) continue;

                    if (view->state == ViewEx::POSE_VALID) {
                        list.push(view);
                    }
                }
                list = shuffle(list, seed).slice(0, 0, MAX_POSEUPDATE);
            }

            for (int i = 0; i < list.size(); i++) {
                ViewEx *view = list[i];

                Mem1<Vec2> pixs;
                Mem1<Vec3> objs;
                Mem1<double> errs;
                for (int f = 0; f < view->fts.size(); f++) {
                    MapPnt *mpnt = view->fts[f].mpnt;
                    if (mpnt == NULL) continue;

                    pixs.push(view->fts[f].pix);
                    objs.push(mpnt->pos);
                    errs.push(mpnt->err);
                }

                Pose pose = view->pose;
                if (calcPoseRANSAC(pose, view->cam, pixs, objs) == false) return false;

                setView(*view, pose);
            }
            return true;
        }

       
        //--------------------------------------------------------------------------------
        // modules
        //--------------------------------------------------------------------------------
     
        void addNewMPnt(Mem1<ViewEx*> &views, Mem2<MatchPair*> &pairs, Mem1<MapPnt*> &mpnts, const int a, const int b) {

            const Mem1<Feature> &fts0 = views[a]->fts;
            const Mem1<Feature> &fts1 = views[b]->fts;
            const Mem1<int> &matches = pairs(a, b)->matches;

            for (int f = 0; f < matches.size(); f++) {
                if (views[a]->fts[f].mpnt != NULL) continue;

                MapPnt *find = NULL;
                for (int i = 0; i < views[a]->links.size(); i++) {
                    const int v = views[a]->links[i];
                    if (v == b) continue;

                    const int g = pairs(a, v)->matches[f];
                    if (g < 0) continue;

                    MapPnt *mpnt = views[v]->fts[g].mpnt;
                    if (mpnt == NULL) continue;

                    //const double err = calcPrjErr(views[a]->pose, views[a]->cam, fts0[f].pix, mpnt->pos);
                    //if (evalErr(err) < 1.0) continue;

                    find = mpnt;
                    break;
                }
                if (find != NULL) {
                    setMPnt(*find, *views[a], f);
                    continue;
                }

                const int g = matches[f];
                if (g < 0) continue;

                Vec3 pos;
                if (calcPnt3d(pos, views[a]->pose, views[a]->cam, fts0[f].pix, views[b]->pose, views[b]->cam, fts1[g].pix) == false) continue;

                const double err0 = calcPrjErr(views[a]->pose, views[a]->cam, fts0[f].pix, pos);
                const double err1 = calcPrjErr(views[b]->pose, views[b]->cam, fts1[g].pix, pos);
                if ((err0 + err1) / 2.0 > MPNT_PRJERR) continue;

                const Vec3 vec0 = unitVec(views[a]->pose.trn - pos);
                const Vec3 vec1 = unitVec(views[b]->pose.trn - pos);
                const double angle = acos(dotVec(vec0, vec1));
                if (angle < MPNT_ANGLE) continue;

                const int m = mpnts.size();
                addMPnt(mpnts, pos);
                setMPnt(*mpnts[m], *views[a], f);
                setMPnt(*mpnts[m], *views[b], g);
            }
        }

        bool addNewMPnt(Mem1<ViewEx*> &views, Mem2<MatchPair*> &pairs, Mem1<MapPnt*> &mpnts, const int seed) {

            // select pair
            MatchPair *pair = NULL;
            {
                // [valid, valid] pair
                Mem1<MatchPair*> list = getPairs(views, pairs, ViewEx::POSE_VALID, ViewEx::POSE_VALID);
                if (list.size() == 0) return false;

                srand(seed);
                pair = list[seed % list.size()];
            }

            addNewMPnt(views, pairs, mpnts, pair->a, pair->b);

            return true;
        }

    };

}
#endif