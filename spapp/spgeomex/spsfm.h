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
            bool valid;

            CamParam cam;

            Mem2<Col3> img;

            Mem1<Feature> fts;

            Pose pose;

            // matching view count
            int pcnt;

            // points count;
            int mcnt;

            ViewData() {
                cam = getCamParam(0, 0);

                valid = false;
                pose = zeroPose();

                pcnt = 0;
                mcnt = 0;
            }

            ViewData(const ViewData &view) {
                *this = view;
            }

            ViewData& operator = (const ViewData &view) {
                cam = view.cam;
                img = view.img;
                fts = view.fts;

                valid = view.valid;
                pose = view.pose;

                pcnt = view.pcnt;
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

            // index -> [view, feature]
            Mem1<Int2> index;

            MapData() {
                pos = getVec(0.0, 0.0, 0.0);
                col = getCol(0, 0, 0);
            }

            MapData(const MapData &md) {
                *this = md;
            }

            MapData& operator = (const MapData &md) {
                pos = md.pos;
                col = md.col;
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

    private:

        //--------------------------------------------------------------------------------
        // execute main flow
        //--------------------------------------------------------------------------------
        
        bool _update(const int itmax = 1) {

            for (int it = 0; it < itmax; it++) {

                try {
                    if (m_views.size() < 2) throw "input size < 2";

                    if (m_update == 0) {
                        updateMatch(m_views, m_pairs, m_views.size() / 2);

                        if (initPair(m_views, m_pairs, m_mpnts) == false) throw "initPair";
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
        // modules
        //--------------------------------------------------------------------------------

        void addView(Mem1<ViewData> &views, Mem2<PairData> &pairs, const Mem2<Col3> &img, const CamParam &cam) {

            ViewData view;

            // set img and cam
            view.img = img;
            view.cam = cam;

            view.fts = SIFT::getFeatures(img);
            
            views.push(view);
        }

        // m: mpnts id, v: views id, f: features id
        void setPoint(Mem1<ViewData> &views, Mem1<MapData> &mpnts, const int m, const int v, const int f) {
            if (views[v].fts[f].mid >= 0) return;

            mpnts[m].index.push(Int2(v, f));
            views[v].fts[f].mid = m;
            views[v].mcnt++;
        }

        // a, b: view id, 
        void setPair(Mem1<ViewData> &views, Mem2<PairData> &pairs, const int a, const int b) {

            PairData &pd = pairs(a, b);

            pd.valid = true;
            pd.a = a;
            pd.b = b;

            pd.matches = findMatch(views[a].fts, views[b].fts);
            pd.rate = getMatchRate(pd.matches);
            pd.eval = evalPair(views, pairs, a, b);

            views[a].pcnt++;
        }

        Mem1<PairData*> getPairs(const Mem1<ViewData> &views, const Mem2<PairData> &pairs, const bool aValid, const bool bValid) {
            Mem1<PairData*> mds;

            for (int a = 0; a < views.size(); a++) {
                if (views[a].valid != aValid) continue;

                for (int b = 0; b < views.size(); b++) {
                    if (a == b) continue;

                    if (views[b].valid != bValid) continue;
                    if (aValid == bValid && b < a) continue;

                    if (pairs(a, b).rate < MIN_MATCHRATE) continue;
                    mds.push(const_cast<PairData*>(&pairs(a, b)));
                }
            }
            return mds;
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

            const double eval = (zlist.size() == 0) ? 0.0 : zlist.size() / maxVal(5.0, medianVal(zlist));
            return eval;
        }

        // initialize pair
        bool initPair(Mem1<ViewData> &views, Mem2<PairData> &pairs, Mem1<MapData> &mpnts) {

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
                if (maxv == 0.0) return false;
            }

            // initialize pair
            {
                const int a = md->a;
                const int b = md->b;
                const Mem1<Feature> &fts0 = views[a].fts;
                const Mem1<Feature> &fts1 = views[b].fts;
                const Mem1<int> &matches = pairs(a, b).matches;

                Mem1<Vec2> pixs0, pixs1;
                for (int i = 0; i < matches.size(); i++) {
                    const int j = matches[i];
                    if (j < 0) continue;

                    pixs0.push(fts0[i].pix);
                    pixs1.push(fts1[j].pix);
                }

                Pose pose;
                if (calcPose(pose, views[a].cam, pixs0, views[b].cam, pixs1) == false) return false;

                mpnts.clear();
                for (int i = 0; i < matches.size(); i++) {
                    const int j = matches[i];
                    if (j < 0) continue;

                    Vec3 pnt;
                    if (calcPnt3d(pnt, zeroPose(), views[a].cam, fts0[i].pix, pose, views[b].cam, fts1[j].pix) == false) continue;

                    const double err = errPose(zeroPose(), views[a].cam, fts0[i].pix, pnt);
                    if (evalErr(err) == 0.0) continue;

                    MapData *gp = mpnts.extend();

                    gp->pos = pnt;
                    setPoint(views, mpnts, mpnts.size() - 1, a, i);
                    setPoint(views, mpnts, mpnts.size() - 1, b, j);

                    updateColor(views, mpnts, mpnts.size() - 1);
                }

                views[a].valid = true;
                views[b].valid = true;

                views[a].pose = zeroPose();
                views[b].pose = pose;

                m_bpair.set(a, b);
            }
            return true;
        }


        //--------------------------------------------------------------------------------
        // update
        //--------------------------------------------------------------------------------
    
        void updateMatch(Mem1<ViewData> &views, Mem2<PairData> &pairs, const int itmax) {
            SP_LOGGER_SET("updateMatch");

            for (int it = 0; it < itmax; it++) {
                int a = 0;
                {
                    int minv = views.size();
                    for (int i = 0; i < views.size(); i++) {
                        if (views[i].pcnt < minv) {
                            a = i;
                            minv = views[i].pcnt;
                        }
                    }
                }

                Mem1<int> list;
                for (int i = 0; i < views.size(); i++) {
                    if (pairs(a, i).valid == false) {
                        list.push(i);
                    }
                }
                if (list.size() == 0) return;

                list = shuffle(list);

                for (int i = 0; i < minVal(10, list.size()); i++) {
                    const int b = list[i];

                    setPair(views, pairs, a, b);
                    setPair(views, pairs, b, a);
                }
            }
        }

        void updateColor(Mem1<ViewData> &views, Mem1<MapData> &mpnts, const int m) {

            const Mem1<Int2> &index = mpnts[m].index;

            Vec3 vec = getVec(0.0, 0.0, 0.0);

            for (int i = 0; i < index.size(); i++) {
                const int v = index[i][0];
                const int m = index[i][1];
                const Vec2 pix = views[v].fts[m].pix;
                vec += getVec(acsc(views[v].img, pix.x, pix.y));
            }
            if (index.size() > 0) {
                vec /= index.size();
            }
            mpnts[m].col = getCol(vec);
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
                views[a].pose = views[b].pose;
                if (refinePose(views[a].pose, views[a].cam, pixs, objs) == false) return false;

                int cnt = 0;
                for (int i = 0; i < index.size(); i++) {
                    const int j = index[i][0];
                    const int m = index[i][1];

                    const double err = errPose(views[a].pose, views[a].cam, fts0[j].pix, mpnts[m].pos);
                    if (evalErr(err) == 0.0)continue;
                    cnt++;
                }
                if (cnt < 0.5 * index.size()) return false;

                // add index
                for (int i = 0; i < index.size(); i++) {
                    const int j = index[i][0];
                    const int m = index[i][1];

                    const double err = errPose(views[a].pose, views[a].cam, fts0[j].pix, mpnts[m].pos);
                    if (evalErr(err) == 0.0)continue;

                    setPoint(views, mpnts, m, a, j);
                }

                views[a].valid = true;
            }

            // add new pnt
            addNewPnt(views, pairs, mpnts, md->a, md->b);

            return true;
        }

        void addNewPnt(Mem1<ViewData> &views, Mem2<PairData> &pairs, Mem1<MapData> &mpnts, const int a, const int b) {

            const Mem1<Feature> &fts0 = views[a].fts;
            const Mem1<Feature> &fts1 = views[b].fts;
            const Mem1<int> &matches = pairs(a, b).matches;

            const Pose pose = views[b].pose * invPose(views[a].pose);
            for (int i = 0; i < matches.size(); i++) {
                if (views[a].fts[i].mid >= 0) continue;

                int find = -1;
                for (int v = 0; v < views.size(); v++) {
                    if (a == v || pairs(a, v).rate < MIN_MATCHRATE) continue;

                    const int j = pairs(a, v).matches[i];
                    if (j < 0) continue;

                    const int m = views[v].fts[j].mid;
                    if (m < 0) continue;

                    //const double err = errPose(views[a].pose, views[a].cam, fts0[i].pix, mpnts[m].pos);
                    //if (evalErr(err) < 1.0) continue;

                    find = m;
                    break;
                }
                if (find >= 0) {
                    setPoint(views, mpnts, find, a, i);
                    continue;
                }

                const int j = matches[i];
                if (j < 0) continue;

                Vec3 pnt;
                if (calcPnt3d(pnt, views[a].pose, views[a].cam, fts0[i].pix, views[b].pose, views[b].cam, fts1[j].pix) == false) continue;

                const double err = errPose(views[a].pose, views[a].cam, fts0[i].pix, pnt);
                if (evalErr(err) < 1.0) continue;

                MapData *gp = mpnts.extend();
                gp->pos = pnt;

                setPoint(views, mpnts, mpnts.size() - 1, a, i);
                setPoint(views, mpnts, mpnts.size() - 1, b, j);

                updateColor(views, mpnts, mpnts.size() - 1);
            }
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

                const int MAX_PNTUPDATE = 1000;
                for (int i = 0; i < minVal(MAX_PNTUPDATE, mpnts.size()); i++) {
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

                    Vec3 pnt = mpnts[m].pos;
                    if (refinePnt3d(pnt, poses, cams, pixs) == false) continue;
                    
                    mpnts[m].pos = pnt;

                    updateColor(views, mpnts, m);
                }
            }
            return false;
        }

        bool updatePose(Mem1<ViewData> &views, Mem2<PairData> &pairs, Mem1<MapData> &mpnts, const int update) {
            SP_LOGGER_SET("updatePose");

            Mem1<int> list;
            for (int a = 0; a < views.size(); a++) {
                //if (a == m_bpair[0] || a == m_bpair[1]) continue;
                if (a == m_bpair[0]) continue;
                if (views[a].valid == false) continue;
                list.push(a);
            }
            list = shuffle(list, update);

            const int MAX_POSEUPDATE = 20;
            for (int i = 0; i < minVal(MAX_POSEUPDATE, list.size()); i++) {
                const int a = list[i];

                Mem1<Vec2> pixs;
                Mem1<Vec3> objs;
                for (int f = 0; f < views[a].fts.size(); f++) {
                    const int m = views[a].fts[f].mid;
                    if (m < 0) continue;

                    pixs.push(views[a].fts[f].pix);
                    objs.push(mpnts[m].pos);
                }

                const Pose backup = views[a].pose;

                // calc pose
                if (refinePose(views[a].pose, views[a].cam, pixs, objs) == false) {
                    return false;
                }

            }
            return true;
        }

    };

}
#endif