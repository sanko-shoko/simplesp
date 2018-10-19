//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_SMF_H__
#define __SP_SMF_H__

#include "spcore/spcore.h"

namespace sp {

#define SP_SFM_MAXVIEW 1000

    class SfM {

    public:

        typedef MemA<int, 2> Int2;

        struct ViewData {

            // camera pose is valid
            bool valid;

            // camera parameter
            CamParam cam;

            // captured image
            Mem2<Col3> img;

            // features
            Mem1<Feature> fts;

            // camera pose
            Pose pose;

            // pair links
            Mem1<int> links;

            // init pair count
            int icnt;

            // map points count
            int mcnt;

            ViewData() {
                cam = getCamParam(0, 0);

                valid = false;
                pose = zeroPose();

                icnt = 0;
                mcnt = 0;
            }

            ViewData(const ViewData &view) {
                *this = view;
            }

            ViewData& operator = (const ViewData &view) {
                cam = view.cam;
                img = view.img;
                fts = view.fts;

                links = view.links;

                valid = view.valid;
                pose = view.pose;

                icnt = view.icnt;
                mcnt = view.mcnt;
                return *this;
            }
        };
        
        struct PairData {
            bool valid;

            // pair view id
            int a, b;

            Mem1<int> matches;

            // match features rate
            double rate;

            // stereo eval
            double eval;

            PairData() {
                valid = false;
                a = -1; 
                b = -1;
                rate = -1.0;
                eval = -1.0;
            }

            PairData(const PairData &pd) {
                *this = pd;
            }

            PairData& operator = (const PairData &pd) {
                valid = pd.valid;
                matches = pd.matches;
                a = pd.a;
                b = pd.b;
                rate = pd.rate;
                eval = pd.eval;
                return *this;
            }
        };

        struct MapData {

            Vec3 pos;

            Col3 col;

            double err;

            // index -> [view, feature]
            Mem1<Int2> index;

            MapData() {
                pos = getVec(0.0, 0.0, 0.0);
                col = getCol(0, 0, 0);
                err = SP_INFINITY;
            }

            MapData(const MapData &md) {
                *this = md;
            }

            MapData& operator = (const MapData &md) {
                pos = md.pos;
                col = md.col;
                err = md.err;
                index = md.index;
                return *this;
            }
        };

    public:

        SP_LOGGER_INSTANCE;
        SP_HOLDER_INSTANCE;

        int m_update;

        int m_maxview;

        Int2 m_bpair;

        Mem1<ViewData> m_views;

        // pair data matrix
        Mem2<PairData> m_pairs;

        // map points
        Mem1<MapData> m_mpnts;

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
            m_bpair.set(-1, -1);

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

        const Mem1<ViewData>* getViews() const {
            return (m_views.size() > 0) ? &m_views : NULL;
        }

        const Mem1<MapData>* getMPnts() const {
            return (m_mpnts.size() > 0) ? &m_mpnts : NULL;
        }

 
        //--------------------------------------------------------------------------------
        // execute
        //--------------------------------------------------------------------------------

        bool addView(const Mem2<Col3> &img, const CamParam *cam = NULL) {
            SP_LOGGER_SET("-addView");

            if (m_views.size() == m_maxview) return false;

            CamParam tmp = (cam != NULL && cmpSize(2, cam->dsize, img.dsize) == true) ? *cam : getCamParam(img.dsize);
            addView(m_views, m_pairs, img, tmp);

            return true;
        }

        bool update(const int itmax = 1) {
            SP_LOGGER_SET("-update");

            return _update(itmax);
        }

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

        double MIN_MATCHRATE = 0.2;
        double MIN_MATCHEVAL = 0.1;
        double MAX_PIXERR = 5.0;

        int MAX_POSEUPDATE = 20;
        int MAX_MPNTUPDATE = 1000;

    private:

        //--------------------------------------------------------------------------------
        // execute main flow
        //--------------------------------------------------------------------------------
        
        bool _update(const int itmax = 1) {

            for (int it = 0; it < itmax; it++) {

                try {
                    if (m_views.size() < 2) throw "input size < 2";

                    if (m_update == 0) {
                        updateMatch(m_views, m_pairs, m_views.size());

                        if (findGoodPair(m_views, m_pairs, m_mpnts) == false) throw "findGoodPair";
                    }
                    else {
                        updateMatch(m_views, m_pairs, 1);

                        // update invalid -> valid and calc pose
                        updateValid(m_views, m_pairs, m_mpnts, m_update);

                        // update pnt 3d
                        updatePnt(m_views, m_pairs, m_mpnts, m_update);

                        // update valid view pose
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
        // view
        //--------------------------------------------------------------------------------

        void addView(Mem1<ViewData> &views, Mem2<PairData> &pairs, const Mem2<Col3> &img, const CamParam &cam) {

            ViewData view;

            // set img and cam
            view.img = img;
            view.cam = cam;

            view.fts = SIFT::getFeatures(img);
            
            views.push(view);
        }

        void setView(Mem1<ViewData> &views, const int v, const Pose &pose) {
            views[v].valid = true;
            views[v].pose = pose;
        }

        //--------------------------------------------------------------------------------
        // pair
        //--------------------------------------------------------------------------------
        
        // a, b: view id, 
        void initPair(Mem1<ViewData> &views, Mem2<PairData> &pairs, const int a, const int b) {

            PairData &pair = pairs(a, b);

            pair.valid = true;
            pair.a = a;
            pair.b = b;

            pair.matches = findMatch(views[a].fts, views[b].fts);
            pair.rate = getMatchRate(pair.matches);
            pair.eval = evalPair(views, pairs, a, b);

            if (pair.rate > MIN_MATCHRATE) {
                views[a].links.push(b);
            }
            views[a].icnt++;
        }

        Mem1<PairData*> getPairs(const Mem1<ViewData> &views, const Mem2<PairData> &pairs, const bool aValid, const bool bValid) {
            Mem1<PairData*> ptrs;

            for (int a = 0; a < views.size(); a++) {
                for (int i = 0; i < views[a].links.size(); i++) {
                    const int b = views[a].links[i];

                    if (views[a].valid != aValid) continue;
                    if (views[b].valid != bValid) continue;
                    if (aValid == bValid && b < a) continue;

                    ptrs.push(const_cast<PairData*>(&pairs(a, b)));
                }
            }
            return ptrs;
        }

        double evalPair(Mem1<ViewData> &views, Mem2<PairData> &pairs, const int a, const int b, const Pose *stereo = NULL) {
            PairData &pd = pairs(a, b);

            if (pd.rate <= MIN_MATCHRATE) return -1.0;

            const Mem1<Feature> &fts0 = views[a].fts;
            const Mem1<Feature> &fts1 = views[b].fts;
            const Mem1<int> &matches = pd.matches;

            Mem1<Vec2> pixs0, pixs1;
            for (int i = 0; i < matches.size(); i++) {
                const int j = matches[i];
                if (j < 0) continue;

                pixs0.push(fts0[i].pix);
                pixs1.push(fts1[j].pix);
            }
            Pose pose;
            if (stereo == NULL) {
                if (calcPose(pose, views[a].cam, pixs0, views[b].cam, pixs1) == false) return 0.0;
            }
            else {
                pose = *stereo;
            }
            pose.trn /= normVec(pose.trn);

            Mem1<double> zlist;
            for (int i = 0; i < pixs0.size(); i++) {

                Vec3 pnt;
                if (calcPnt3d(pnt, zeroPose(), views[a].cam, pixs0[i], pose, views[b].cam, pixs1[i]) == false) continue;

                zlist.push(pnt.z);
            }
            if (zlist.size() == 0) return -1.0;

            const double pnum = 0.1 * minVal(100, zlist.size());
            const double zval = maxVal(10.0, medianVal(zlist));

            const double eval = pnum / zval;
            return eval;
        }

        //--------------------------------------------------------------------------------
        // map point
        //--------------------------------------------------------------------------------

        double errMPnt(Mem1<ViewData> &views, Mem1<MapData> &mpnts, const int m) {

            Mem1<double> errs;

            const Mem1<Int2> &index = mpnts[m].index;
            for (int i = 0; i < index.size(); i++) {
                const int v = index[i][0];
                const int f = index[i][1];

                const Pose &pose = views[v].pose;
                const CamParam &cam = views[v].cam;
                const Vec2 &pix = views[v].fts[f].pix;
                const Vec3 &pos = mpnts[m].pos;

                const double err = errPose(pose, cam, pix, pos);
                errs.push(err);
            }

            return (errs.size() > 0) ? medianVal(errs) : SP_INFINITY;
        }

        Col3 colMPnt(Mem1<ViewData> &views, Mem1<MapData> &mpnts, const int m) {

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

        void setMPnt(Mem1<ViewData> &views, Mem1<MapData> &mpnts, const int m, const Vec3 &pos) {
            if (m + 1 > mpnts.size()) mpnts.extend(m + 1 - mpnts.size());
           
            mpnts[m].pos = pos;
            mpnts[m].col = colMPnt(views, mpnts, m);
            mpnts[m].err = errMPnt(views, mpnts, m);
        }

        // m: mpnts id, v: views id, f: features id
        void setMPnt(Mem1<ViewData> &views, Mem1<MapData> &mpnts, const int m, const int v, const int f) {
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

        // find good pair
        bool findGoodPair(Mem1<ViewData> &views, Mem2<PairData> &pairs, Mem1<MapData> &mpnts) {

            // select pair
            PairData *md = NULL;
            {
                const Mem1<PairData*> mds = shuffle(getPairs(views, pairs, false, false));

                double maxv = 0.0;
                for (int i = 0; i < minVal(10, mds.size()); i++) {

                    const double eval = mds[i]->eval;
                    if (eval < maxv) continue;

                    maxv = eval;
                    md = mds[i];
                }
                if (maxv < MIN_MATCHEVAL) return false;
            }

            // initialize pair
            {
                const int a = md->a;
                const int b = md->b;
                const Mem1<Feature> &fts0 = views[a].fts;
                const Mem1<Feature> &fts1 = views[b].fts;
                const Mem1<int> &matches = pairs(a, b).matches;

                Mem1<Vec2> pixs0, pixs1;
                for (int f = 0; f < matches.size(); f++) {
                    const int g = matches[f];
                    if (g < 0) continue;

                    pixs0.push(fts0[f].pix);
                    pixs1.push(fts1[g].pix);
                }

                Pose pose;
                if (calcPose(pose, views[a].cam, pixs0, views[b].cam, pixs1) == false) return false;

                setView(views, a, zeroPose());
                setView(views, b, pose);

                m_bpair.set(a, b);

                for (int f = 0; f < matches.size(); f++) {
                    const int g = matches[f];
                    if (g < 0) continue;

                    Vec3 pos;
                    if (calcPnt3d(pos, zeroPose(), views[a].cam, fts0[f].pix, pose, views[b].cam, fts1[g].pix) == false) continue;

                    const double err0 = errPose(zeroPose(), views[a].cam, fts0[f].pix, pos);
                    const double err1 = errPose(pose, views[b].cam, fts1[g].pix, pos);
                    if ((err0 + err1) / 2.0 > MAX_PIXERR) continue;

                    const int m = mpnts.size();
                    setMPnt(views, mpnts, m, pos);
                    setMPnt(views, mpnts, m, a, f);
                    setMPnt(views, mpnts, m, b, g);
                }

            }
            return true;
        }


        //--------------------------------------------------------------------------------
        // update
        //--------------------------------------------------------------------------------
    
        void updateMatch(Mem1<ViewData> &views, Mem2<PairData> &pairs, const int itmax) {
            SP_LOGGER_SET("updateMatch");

            for (int it = 0; it < itmax; it++) {
                int a = -1;
                {
                    int minv = views.size() - 1;
                    for (int i = 0; i < views.size(); i++) {
                        if (views[i].icnt < minv) {
                            a = i;
                            minv = views[i].icnt;
                        }
                    }
                }
                if (a < 0) return;

                Mem1<int> list;
                for (int v = 0; v < views.size(); v++) {
                    if (v != a && pairs(a, v).valid == false) {
                        list.push(v);
                    }
                }
                if (list.size() == 0) return;

                list = shuffle(list);

                for (int i = 0; i < minVal(10, list.size()); i++) {
                    const int b = list[i];
                    initPair(views, pairs, a, b);
                    initPair(views, pairs, b, a);
                }
            }
        }

        bool updateValid(Mem1<ViewData> &views, Mem2<PairData> &pairs, Mem1<MapData> &mpnts, const int update) {
            SP_LOGGER_SET("updateValid");

            // select pair
            PairData *md = NULL;
            {
                // [invalid, valid] pair
                const Mem1<PairData*> mds = shuffle(getPairs(views, pairs, false, true), update);
                if (mds.size() == 0) return false;

                md = mds[0];
            }
            
            // calc pose & add pnts
            {
                const int a = md->a;
                const int b = md->b;

                const Mem1<Feature> &fts0 = views[a].fts;
                const Mem1<Feature> &fts1 = views[b].fts;
                const Mem1<int> &matches = pairs(b, a).matches;

                Mem1<Int2> index;
                for (int i = 0; i < matches.size(); i++) {
                    const int j = matches[i];
                    const int m = views[b].fts[i].mid;
                    if (j < 0 || m < 0) continue;

                    index.push(Int2(j, m));
                }
                if (index.size() < 6) return false;

                Mem1<Vec2> pixs;
                Mem1<Vec3> objs;
                for (int i = 0; i < index.size(); i++) {
                    const int j = index[i][0];
                    const int m = index[i][1];
                    pixs.push(fts0[j].pix);
                    objs.push(mpnts[m].pos);
                }

                // calc pose
                Pose pose = views[b].pose;
                if (refinePose(pose, views[a].cam, pixs, objs) == false) return false;

                int cnt = 0;
                for (int i = 0; i < index.size(); i++) {
                    const int j = index[i][0];
                    const int m = index[i][1];

                    const double err = errPose(pose, views[a].cam, fts0[j].pix, mpnts[m].pos);
                    if (evalErr(err) == 0.0) continue;
                    cnt++;
                }
                if (cnt < 0.5 * index.size()) return false;

                // add index
                for (int i = 0; i < index.size(); i++) {
                    const int j = index[i][0];
                    const int m = index[i][1];

                    const double err = errPose(pose, views[a].cam, fts0[j].pix, mpnts[m].pos);
                    if (evalErr(err) == 0.0) continue;

                    setMPnt(views, mpnts, m, a, j);
                }

                setView(views, a, pose);
            }

            // add new pnt
            addNewPnt(views, pairs, mpnts, md->a, md->b);

            return true;
        }


        bool updatePnt(Mem1<ViewData> &views, Mem2<PairData> &pairs, Mem1<MapData> &mpnts, const int update) {
            SP_LOGGER_SET("updatePnt");

            // select pair
            PairData *md = NULL;
            {
                // [valid, valid] pair
                const Mem1<PairData*> mds = getPairs(views, pairs, true, true);
                if (mds.size() == 0) return false;

                struct Tmp {
                    int id, cnt;
                    bool operator > (const Tmp t) const { return this->cnt > t.cnt; }
                    bool operator < (const Tmp t) const { return this->cnt < t.cnt; }
                };

                Mem1<Tmp> tmps;
                for (int i = 0; i < mds.size(); i++) {
                    Tmp tmp;
                    tmp.id = i;
                    tmp.cnt = minVal(views[mds[i]->a].mcnt, views[mds[i]->b].mcnt);
                    tmps.push(tmp);
                }
                sort(tmps);

                const double x = (randValUnif() + 1.0) / 2.0;
                const int i = tmps[floor(pow(x, 2.0) * tmps.size())].id;

                md = mds[tmps[i].id];
            }

            // add new pnt
            addNewPnt(views, pairs, mpnts, md->a, md->b);

            // refine pnt
            {
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

                    Vec3 pos = mpnts[m].pos;
                    if (refinePnt3d(pos, poses, cams, pixs) == false) continue;
                    
                    Mem1<double> errs;
                    for (int i = 0; i < index.size(); i++) {
                        const double err = errPose(poses[i], cams[i], pixs[i], pos);
                        errs.push(err);
                    }

                    setMPnt(views, mpnts, m, pos);
                }
            }
            return false;
        }

        bool updatePose(Mem1<ViewData> &views, Mem2<PairData> &pairs, Mem1<MapData> &mpnts, const int update) {
            SP_LOGGER_SET("updatePose");

            Mem1<int> list;
            for (int v = 0; v < views.size(); v++) {
                if (v == m_bpair[0] || v == m_bpair[1]) continue;
                if (views[v].valid == false) continue;
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
                if (refinePose(pose, views[v].cam, pixs, objs) == false) {
                    return false;
                }

                setView(views, v, pose);
            }
            return true;
        }

        void addNewPnt(Mem1<ViewData> &views, Mem2<PairData> &pairs, Mem1<MapData> &mpnts, const int a, const int b) {

            const Mem1<Feature> &fts0 = views[a].fts;
            const Mem1<Feature> &fts1 = views[b].fts;
            const Mem1<int> &matches = pairs(a, b).matches;

            const Pose pose = views[b].pose * invPose(views[a].pose);
            for (int f = 0; f < matches.size(); f++) {
                if (views[a].fts[f].mid >= 0) continue;

                int find = -1;
                for (int i = 0; i < views[a].links.size(); i++) {
                    const int v = views[a].links[i];
                    if (v == b) continue;

                    const int j = pairs(a, v).matches[f];
                    if (j < 0) continue;

                    const int m = views[v].fts[j].mid;
                    if (m < 0) continue;

                    //const double err = errPose(views[a].pose, views[a].cam, fts0[f].pix, mpnts[m].pos);
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

                const double err0 = errPose(views[a].pose, views[a].cam, fts0[f].pix, pos);
                const double err1 = errPose(views[b].pose, views[b].cam, fts1[g].pix, pos);
                if ((err0 + err1) / 2.0 > MAX_PIXERR) continue;

                const int m = mpnts.size();
                setMPnt(views, mpnts, m, pos);
                setMPnt(views, mpnts, m, a, f);
                setMPnt(views, mpnts, m, b, g);
            }
        }
    };

}
#endif