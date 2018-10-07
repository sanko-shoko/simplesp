//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_SMF_H__
#define __SP_SMF_H__

#include "spcore/spcore.h"

namespace sp {

    class SfM {

#define SP_SFM_MAXVIEW 1000

    public:

        typedef MemA<int, 2> Int2;

        struct MatchData {

            Mem1<int> matches;

            // pair view id
            int a, b;

            // match features rate
            double rate;

            // stereo eval
            double eval;

            MatchData() {
                a = -1;
                b = -1;
                rate = -1.0;
                eval = -1.0;
            }

            MatchData(const MatchData &md) {
                *this = md;
            }

            MatchData& operator = (const MatchData &md) {
                matches = md.matches;
                a = md.a;
                b = md.b;
                rate = md.rate;
                eval = md.eval;
                return *this;
            }
        };

        struct ViewData {
            CamParam cam;

            Mem2<Col3> img;

            Mem1<Feature> fts;

            // pose valid
            bool valid;
            
            Pose pose;

            // matching view count
            int mcnt;

            // points count;
            int pcnt;

            ViewData() {
                cam = getCamParam(0, 0);
                
                valid = false;
                pose = zeroPose();

                mcnt = 0;
                pcnt = 0;
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

                mcnt = view.mcnt;
                pcnt = view.pcnt;
                return *this;
            }
        };

        struct PointData {

            Vec3 pos;

            Col3 col;

            // index -> [view, match]
            Mem1<Int2> index;

            PointData() {
                pos = getVec(0.0, 0.0, 0.0);
                col = getCol(0, 0, 0);
            }

            PointData(const PointData &pd) {
                *this = pd;
            }

            PointData& operator = (const PointData &pd) {
                pos = pd.pos;
                col = pd.col;
                index = pd.index;
                return *this;
            }
        };

    public:

        SP_LOGGER_INSTANCE;
        SP_HOLDER_INSTANCE;

        int m_update;

        int m_maxview;

        Mem1<ViewData> m_views;

        // match data matrix
        Mem2<MatchData> m_mdmat;

        // global points
        Mem1<PointData> m_gpnts;

    public:

        SfM() {
            init();
        }

        void init(const int maxview = SP_SFM_MAXVIEW) {
            clear();

            m_maxview = maxview;
            m_views.reserve(maxview);
            m_mdmat.resize(maxview, maxview);
            m_gpnts.reserve(100 * maxview);
        }

        void clear() {
            m_update = 0;

            m_views.clear();
            m_mdmat.clear();
            m_gpnts.clear();
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

        const Mem1<PointData>* getPnts() const {
            return (m_gpnts.size() > 0) ? &m_gpnts : NULL;
        }

 
        //--------------------------------------------------------------------------------
        // execute
        //--------------------------------------------------------------------------------

        bool addView(const Mem2<Col3> &img, const CamParam *cam = NULL) {
            SP_LOGGER_SET("-addView");

            if (m_views.size() == m_maxview) return false;

            CamParam tmp = (cam != NULL && cmpSize(2, cam->dsize, img.dsize) == true) ? *cam : getCamParam(img.dsize);
            addView(m_views, m_mdmat, img, tmp);

            return true;
        }

        bool update(const int itmax = 1) {
            SP_LOGGER_SET("-update");

            return _update(itmax);
        }

        bool save(const char *path) {
            Mem1<Vec3> pnts;
            Mem1<Col3> cols;
            for (int i = 0; i < m_gpnts.size(); i++) {
                pnts.push(m_gpnts[i].pos);
                cols.push(m_gpnts[i].col);
            }

            return savePLY(path, pnts, cols);
        }

    private:

        double MIN_MATCHRATE = 0.1;

    private:

        //--------------------------------------------------------------------------------
        // execute main flow
        //--------------------------------------------------------------------------------
        
        bool _update(const int itmax = 1) {

            for (int it = 0; it < itmax; it++) {

                try {
                    if (m_views.size() < 2) throw "input size < 2";

                    if (m_update == 0) {
                        updateMatch(m_views, m_mdmat, m_views.size() / 2);

                        if (initPair(m_views, m_mdmat, m_gpnts) == false) throw "initPair";
                    }
                    else {
                        updateMatch(m_views, m_mdmat, 1);

                        // update invalid -> valid and calc pose
                        updateValid(m_views, m_mdmat, m_gpnts, m_update);

                        // update pnt 3d
                        updatePnt(m_views, m_mdmat, m_gpnts, m_update);

                        // update valid view pose
                        updatePose(m_views, m_mdmat, m_gpnts, m_update);
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

        void addView(Mem1<ViewData> &views, Mem2<MatchData> &mdmat, const Mem2<Col3> &img, const CamParam &cam) {

            ViewData view;

            // set img and cam
            view.img = img;
            view.cam = cam;

            // set features
            SIFT sift;
            if (sift.execute(img) == true) {
                view.fts = *sift.getFeatrue();
            }
            
            views.push(view);
        }

        // g: gpnts id, v: views id, f: features id
        void addPoint(Mem1<ViewData> &views, Mem1<PointData> &gpnts, const int g, const int v, const int f) {
            if (views[v].fts[f].mid >= 0) return;

            gpnts[g].index.push(Int2(v, f));
            views[v].fts[f].mid = g;
            views[v].pcnt++;
        }

        // a, b: view id, 
        void addMatch(Mem1<ViewData> &views, Mem2<MatchData> &mdmat, const int a, const int b) {

            MatchData &md = mdmat(a, b);
            md.matches = findMatch(views[a].fts, views[b].fts);
            md.a = a;
            md.b = b;
            md.rate = getMatchRate(md.matches);

            if (md.rate > MIN_MATCHRATE) {
                Mem1<Vec2> mpixs0, mpixs1;
                getMatchPixs(mpixs0, mpixs1, views[a].fts, views[b].fts, md.matches);

                md.eval = evalStereo(views[a].cam, mpixs0, views[b].cam, mpixs1);
            }

            views[a].mcnt++;
        }

        Mem1<MatchData*> getPairs(const Mem1<ViewData> &views, const Mem2<MatchData> &mdmat, const bool aValid, const bool bValid) {
            Mem1<MatchData*> mds;
            mds.reserve((views.size() - 1) * views.size());
            for (int a = 0; a < views.size(); a++) {
                if (views[a].valid != aValid) continue;

                for (int b = 0; b < views.size(); b++) {
                    if (a == b) continue;

                    if (views[b].valid != bValid) continue;
                    if (aValid == bValid && b < a) continue;

                    if (mdmat(a, b).rate < MIN_MATCHRATE) continue;
                    mds.push(const_cast<MatchData*>(&mdmat(a, b)));
                }
            }
            return mds;
        }

        // initialize pair
        bool initPair(Mem1<ViewData> &views, Mem2<MatchData> &mdmat, Mem1<PointData> &gpnts) {

            // select pair
            MatchData *md = NULL;
            {
                const Mem1<MatchData*> mds = shuffle(getPairs(views, mdmat, false, false));

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
                const Mem1<int> &matches = mdmat(a, b).matches;

                Mem1<Vec2> pixs0, pixs1;
                for (int i = 0; i < matches.size(); i++) {
                    const int j = matches[i];
                    if (j < 0) continue;

                    pixs0.push(fts0[i].pix);
                    pixs1.push(fts1[j].pix);
                }

                Pose pose;
                if (calcPose(pose, views[a].cam, pixs0, views[b].cam, pixs1) == false) return false;

                gpnts.clear();
                for (int i = 0; i < matches.size(); i++) {
                    const int j = matches[i];
                    if (j < 0) continue;

                    Vec3 pnt;
                    if (calcPnt3d(pnt, zeroPose(), views[a].cam, fts0[i].pix, pose, views[b].cam, fts1[j].pix) == false) continue;

                    const double err = errPose(zeroPose(), views[a].cam, fts0[i].pix, pnt);
                    if (evalErr(err) == 0.0) continue;

                    PointData *gp = gpnts.extend();

                    gp->pos = pnt;
                    addPoint(views, gpnts, gpnts.size() - 1, a, i);
                    addPoint(views, gpnts, gpnts.size() - 1, b, j);

                    updateColor(views, gpnts, gpnts.size() - 1);
                }

                views[a].valid = true;
                views[b].valid = true;

                views[a].pose = zeroPose();
                views[b].pose = pose;
            }
            return true;
        }

        double evalStereo(const CamParam &cam0, const Mem1<Vec2> &pixs0, const CamParam &cam1, const  Mem1<Vec2> &pixs1, const Pose *stereo = NULL) {

            Pose pose;
            if (stereo == NULL) {
                if (calcPose(pose, cam0, pixs0, cam1, pixs1) == false) return 0.0;
            }
            else {
                pose = *stereo;
            }
            pose.trn /= normVec(pose.trn);

            Mem1<double> zlist;
            for (int i = 0; i < pixs0.size(); i++) {

                Vec3 pnt;
                if (calcPnt3d(pnt, zeroPose(), cam0, pixs0[i], pose, cam1, pixs1[i]) == false) continue;

                zlist.push(pnt.z);
            }

            const double eval = (zlist.size() == 0) ? 0.0 : zlist.size() / maxVal(1.0, medianVal(zlist));
            return eval;
        }

        double evalPair(Mem1<ViewData> &views, Mem2<MatchData> &mdmat, const int a, const int b) {
            MatchData &mdata = mdmat(a, b);

            if (mdata.eval >= 0.0 || mdata.rate < MIN_MATCHRATE) return mdata.eval;
            mdata.eval = 0.0;

            const Mem1<Feature> &fts0 = views[a].fts;
            const Mem1<Feature> &fts1 = views[b].fts;
            const Mem1<int> &matches = mdata.matches;

            Mem1<Vec2> pixs0, pixs1;
            for (int i = 0; i < matches.size(); i++) {
                const int j = matches[i];
                if (j < 0) continue;

                pixs0.push(fts0[i].pix);
                pixs1.push(fts1[j].pix);
            }

            Pose pose;
            if ((views[a].valid & views[b].valid) == false) {
                if (calcPose(pose, views[a].cam, pixs0, views[b].cam, pixs1) == false) return 0.0;
            }
            else {
                pose = views[b].pose * invPose(views[a].pose);
            }
            pose.trn /= normVec(pose.trn);

            Mem1<double> zlist;
            for (int i = 0; i < matches.size(); i++) {
                const int j = matches[i];
                if (j < 0) continue;

                Vec3 pnt;
                if (calcPnt3d(pnt, zeroPose(), views[a].cam, fts0[i].pix, pose, views[b].cam, fts1[j].pix) == false) continue;

                const double err = errPose(zeroPose(), views[a].cam, fts0[i].pix, pnt);
                if (evalErr(err) == 0.0) continue;

                zlist.push(pnt.z);
            }

            mdmat(a, b).eval = (zlist.size() == 0) ? 0.0 : zlist.size() / maxVal(1.0, medianVal(zlist));
            return mdmat(a, b).eval;
        }


        //--------------------------------------------------------------------------------
        // update
        //--------------------------------------------------------------------------------
    
        void updateMatch(Mem1<ViewData> &views, Mem2<MatchData> &mdmat, const int itmax) {
            SP_LOGGER_SET("updateMatch");

            for (int it = 0; it < itmax; it++) {
                int a = 0;
                {
                    int minv = views.size();
                    for (int i = 0; i < views.size(); i++) {
                        if (views[i].mcnt < minv) {
                            a = i;
                            minv = views[i].mcnt;
                        }
                    }
                }

                Mem1<int> list;
                for (int i = 0; i < views.size(); i++) {
                    if (mdmat(a, i).rate < 0.0) {
                        list.push(i);
                    }
                }

                if (list.size() == 0) return;

                list = shuffle(list);

                for (int i = 0; i < minVal(10, list.size()); i++) {
                    const int b = list[i];

                    addMatch(views, mdmat, a, b);
                    addMatch(views, mdmat, b, a);
                }
            }
        }

        void updateColor(Mem1<ViewData> &views, Mem1<PointData> &gpnts, const int g) {

            const Mem1<Int2> &index = gpnts[g].index;

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
            gpnts[g].col = getCol(vec);
        }

        bool updateValid(Mem1<ViewData> &views, Mem2<MatchData> &mdmat, Mem1<PointData> &gpnts, const int update) {
            SP_LOGGER_SET("updateValid");

            // select pair
            MatchData *md = NULL;
            {
                // [invalid, valid] pair
                const Mem1<MatchData*> mds = shuffle(getPairs(views, mdmat, false, true), update);
                if (mds.size() == 0) return false;

                md = mds[0];
            }
            
            // calc pose & add pnts
            {
                const int a = md->a;
                const int b = md->b;

                const Mem1<Feature> &fts0 = views[a].fts;
                const Mem1<Feature> &fts1 = views[b].fts;
                const Mem1<int> &matches = mdmat(b, a).matches;

                Mem1<Int2> index;
                for (int i = 0; i < matches.size(); i++) {
                    const int j = matches[i];
                    const int g = views[b].fts[i].mid;
                    if (j < 0 || g < 0) continue;

                    index.push(Int2(j, g));
                }
                if (index.size() < 6) return false;

                Mem1<Vec2> pixs;
                Mem1<Vec3> objs;
                for (int i = 0; i < index.size(); i++) {
                    const int j = index[i][0];
                    const int g = index[i][1];
                    pixs.push(fts0[j].pix);
                    objs.push(gpnts[g].pos);
                }

                // calc pose
                views[a].pose = views[b].pose;
                if (refinePose(views[a].pose, views[a].cam, pixs, objs) == false) return false;

                int cnt = 0;
                for (int i = 0; i < index.size(); i++) {
                    const int j = index[i][0];
                    const int g = index[i][1];

                    const double err = errPose(views[a].pose, views[a].cam, fts0[j].pix, gpnts[g].pos);
                    if (evalErr(err) == 0.0)continue;
                    cnt++;
                }
                if (cnt < 0.5 * index.size()) return false;

                // add index
                for (int i = 0; i < index.size(); i++) {
                    const int j = index[i][0];
                    const int g = index[i][1];

                    const double err = errPose(views[a].pose, views[a].cam, fts0[j].pix, gpnts[g].pos);
                    if (evalErr(err) == 0.0)continue;

                    addPoint(views, gpnts, g, a, j);
                }

                views[a].valid = true;
            }
            return true;
        }

        bool updatePnt(Mem1<ViewData> &views, Mem2<MatchData> &mdmat, Mem1<PointData> &gpnts, const int update) {
            SP_LOGGER_SET("updatePnt");

            // select pair
            MatchData *md = NULL;
            {
                // [valid, valid] pair
                const Mem1<MatchData*> mds = getPairs(views, mdmat, true, true);
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
                    tmp.cnt = minVal(views[mds[i]->a].pcnt, views[mds[i]->b].pcnt);
                    tmps.push(tmp);
                }
                sort(tmps);

                const double x = (randValUnif() + 1.0) / 2.0;
                const int i = tmps[floor(pow(x, 2.0) * tmps.size())].id;

                md = mds[tmps[i].id];
            }

            // add new pnt
            {
                const int a = md->a;
                const int b = md->b;

                const Mem1<Feature> &fts0 = views[a].fts;
                const Mem1<Feature> &fts1 = views[b].fts;
                const Mem1<int> &matches = mdmat(a, b).matches;

                const Pose pose = views[b].pose * invPose(views[a].pose);
                for (int i = 0; i < matches.size(); i++) {
                    if (views[a].fts[i].mid >= 0) continue;

                    int find = -1;
                    for (int v = 0; v < views.size(); v++) {
                        if (a == v || mdmat(a, v).rate < MIN_MATCHRATE) continue;

                        const int j = mdmat(a, v).matches[i];
                        if (j < 0) continue;

                        const int g = views[v].fts[j].mid;
                        if (g < 0) continue;

                        const double err = errPose(views[a].pose, views[a].cam, fts0[i].pix, gpnts[g].pos);
                        if (evalErr(err) < 1.0) continue;

                        find = g;
                        break;
                    }
                    if (find >= 0) {
                        addPoint(views, gpnts, find, a, i);
                        continue;
                    }

                    const int j = matches[i];
                    if (j < 0) continue;

                    Vec3 pnt;
                    if(calcPnt3d(pnt, views[a].pose, views[a].cam, fts0[i].pix, views[b].pose, views[b].cam, fts1[j].pix) == false) continue;

                    const double err = errPose(views[a].pose, views[a].cam, fts0[i].pix, pnt);
                    if (evalErr(err) < 1.0) continue;

                    PointData *gp = gpnts.extend();
                    gp->pos = pnt;

                    addPoint(views, gpnts, gpnts.size() - 1, a, i);
                    addPoint(views, gpnts, gpnts.size() - 1, b, j);

                    updateColor(views, gpnts, gpnts.size() - 1);
                }
            }

            // refine pnt
            {
                srand(update);

                const int MAX_PNTUPDATE = 1000;
                for (int i = 0; i < minVal(MAX_PNTUPDATE, gpnts.size()); i++) {
                    const int g = rand() % gpnts.size();

                    const Mem1<Int2> &index = gpnts[g].index;
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

                    Vec3 pnt = gpnts[g].pos;
                    if (refinePnt3d(pnt, poses, cams, pixs) == false) continue;
                    
                    gpnts[g].pos = pnt;

                    updateColor(views, gpnts,g);
                }
            }
            return false;
        }

        bool updatePose(Mem1<ViewData> &views, Mem2<MatchData> &mdmat, Mem1<PointData> &gpnts, const int update) {
            SP_LOGGER_SET("updatePose");

            Mem1<int> list;
            for (int a = 0; a < views.size(); a++) {
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
                    const int g = views[a].fts[f].mid;
                    if (g < 0) continue;

                    pixs.push(views[a].fts[f].pix);
                    objs.push(gpnts[g].pos);
                }

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