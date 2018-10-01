//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_SMF_H__
#define __SP_SMF_H__

#include "spcore/spcore.h"

namespace sp {

    class SfM {

    public:

        typedef MemA<int, 2> Int2;

        struct MatchData {

            Mem1<int> matches;

            // match features rate
            double rate;

            // stereo eval
            double eval;

            MatchData() {
                rate = -1.0;
                eval = -1.0;
            }
        };

        struct ViewData {
            CamParam cam;

            Mem2<Col3> img;

            Mem1<Feature> fts;

            Pose pose;

            // pose valid
            bool valid;

            // index -> [point]
            Mem1<int> index;

            // 
            int mcnt;
        };

        struct PointData {

            Vec3 pos;

            Col3 col;

            // index -> [view, match]
            Mem1<Int2> index;

            PointData() {
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

        Mem1<ViewData> m_views;

        // match data matrix
        Mem2<MatchData> m_mdmat;

        // global points
        Mem1<PointData> m_gpnts;

    public:

        SfM() {
            m_update = 0;
            reserve();
        }

        void reserve(const int maxview = 1000) {
            m_views.reserve(maxview);
            m_mdmat.resize(maxview, maxview);
            m_gpnts.reserve(1000 * maxview);
        }

        void clear() {
            m_update = 0;

            m_views.clear();
            m_mdmat.clear();
            m_gpnts.clear();
            reserve();
        }


        //--------------------------------------------------------------------------------
        // data
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
        // execute sfm
        //--------------------------------------------------------------------------------

        void addView(const Mem2<Col3> &img, const CamParam &cam = getCamParam(0, 0)) {
            SP_LOGGER_SET("-addView");

            CamParam tmp = (cmpSize(2, cam.dsize, img.dsize) == true) ? cam : getCamParam(img.dsize);
            addView(m_views, m_mdmat, img, tmp);
        }

        bool update(const int itmax = 1) {
            for (int it = 0; it < itmax; it++) {

                SP_LOGGER_SET("-update");

                try {
                    if (m_views.size() < 2) throw "data size < 2";

                    if (m_update == 0) {
                        updateMatch(m_views, m_mdmat, m_views.size() / 2);

                        if (initPair(m_gpnts, m_views, m_mdmat) == false) throw "initPair";
                    }
                    else {
                        updateMatch(m_views, m_mdmat, 1);

                        // update invalid -> valid and calc pose
                        updateValid(m_gpnts, m_views, m_mdmat, m_update);

                        // update pnt 3d
                        updatePnt(m_gpnts, m_views, m_mdmat, m_update);

                        // update valid view pose
                        updatePose(m_gpnts, m_views, m_mdmat, m_update);
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

        bool savePly(const char *path) {
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

        void addView(Mem1<ViewData> &views, Mem2<MatchData> &mdmat, const Mem2<Col3> &img, const CamParam &cam) {

            ViewData &view = *views.extend();

            // set pose
            view.valid = false;
            view.pose = zeroPose();

            // set img and cam
            view.img = img;
            view.cam = cam;

            // set features
            SIFT sift;
            sift.execute(img);
            view.fts = sift.getFeatrue();

            // init index
            view.index.resize(view.fts.size());
            setElm(view.index, -1);

            view.mcnt = 0;
        }

        // g: gpnts id, v: views id, m: matches id
        void addIndex(Mem1<PointData> &gpnts, Mem1<ViewData> &views, const int g, const int v, const int m) {
            if (views[v].index[m] >= 0) return;

            gpnts[g].index.push(Int2(v, m));
            views[v].index[m] = g;
        }

        // initialize pair
        bool initPair(Mem1<PointData> &gpnts, Mem1<ViewData> &views, Mem2<MatchData> &mdmat) {

            // select pair
            Int2 pair;
            {
                Mem1<Int2> pairs;
                for (int a = 0; a < views.size(); a++) {
                    for (int b = a + 1; b < views.size(); b++) {
                        if (mdmat(a, b).rate < MIN_MATCHRATE) continue;
                        pairs.push(Int2(a, b));
                    }
                }
                pairs = shuffle(pairs);

                double maxv = 0.0;
                for (int i = 0; i < minVal(10, pairs.size()); i++) {
                    const double eval = evalPair(views, mdmat, pairs[i][0], pairs[i][1]);
                    if (eval < maxv) continue;

                    maxv = eval;
                    pair = pairs[i];
                }
                if (maxv == 0.0) return false;
            }

            // initialize pair
            {
                const int a = pair[0];
                const int b = pair[1];

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
                    addIndex(gpnts, views, gpnts.size() - 1, a, i);
                    addIndex(gpnts, views, gpnts.size() - 1, b, j);

                    updateColor(gpnts, views, gpnts.size() - 1);
                }

                views[a].valid = true;
                views[b].valid = true;

                views[a].pose = zeroPose();
                views[b].pose = pose;
            }
            return true;
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
                for (int i = 0; i < minVal(10, views.size()); i++) {
                    if (mdmat(a, i).eval < 0.0) {
                        list.push(i);
                    }
                }

                if (list.size() == 0) return;

                list = shuffle(list);

                for (int i = 0; i < list.size(); i++) {
                    const int b = list[i];

                    mdmat(a, b).matches = findMatch(views[a].fts, views[b].fts);
                    mdmat(a, b).rate = getMatchRate(mdmat(a, b).matches);

                    mdmat(b, a).matches = findMatch(views[b].fts, views[a].fts);
                    mdmat(b, a).rate = getMatchRate(mdmat(b, a).matches);

                    views[a].mcnt++;
                    views[b].mcnt++;
                }
            }
        }

        void updateColor(Mem1<PointData> &gpnts, Mem1<ViewData> &views, const int g) {

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

        bool updateValid(Mem1<PointData> &gpnts, Mem1<ViewData> &views, Mem2<MatchData> &mdmat, const int update) {
            SP_LOGGER_SET("updateValid");

            // select pair
            Int2 pair;
            {
                Mem1<double> rates;
                Mem1<Int2> tmps;

                for (int a = 0; a < views.size(); a++) {
                    if (views[a].valid == true) continue;

                    for (int b = 0; b < views.size(); b++) {
                        if (views[b].valid == false) continue;

                        if (mdmat(a, b).rate < MIN_MATCHRATE) continue;
                        tmps.push(Int2(a, b));
                        rates.push(mdmat(a, b).rate);
                    }
                }
                if (tmps.size() == 0) return false;

                const double thresh = medianVal(rates);

                Mem1<Int2> pairs;
                for (int i = 0; i < tmps.size(); i++) {
                    if (rates[i] >= thresh) {
                        pairs.push(tmps[i]);
                    }
                }

                srand(update);
                pair = pairs[rand() % pairs.size()];
            }

            // calc pose
            {
                const int a = pair[0];
                const int b = pair[1];

                const Mem1<Feature> &fts0 = views[a].fts;
                const Mem1<Feature> &fts1 = views[b].fts;
                const Mem1<int> &matches = mdmat(b, a).matches;

                Mem1<Int2> index;
                for (int i = 0; i < matches.size(); i++) {
                    const int j = matches[i];
                    const int g = views[b].index[i];
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

                views[a].valid = true;

                // add index
                for (int i = 0; i < index.size(); i++) {
                    const int j = index[i][0];
                    const int g = index[i][1];

                    const double err = errPose(views[a].pose, views[a].cam, fts0[j].pix, gpnts[g].pos);
                    if (evalErr(err) == 0.0)continue;

                    addIndex(gpnts, views, g, a, j);
                }
            }
            return true;
        }

        bool updatePnt(Mem1<PointData> &gpnts, Mem1<ViewData> &views, Mem2<MatchData> &mdmat, const int update) {
            SP_LOGGER_SET("updatePnt");

            // select pair
            Int2 pair;
            {
                Mem1<Int2> pairs;
                for (int a = 0; a < views.size(); a++) {
                    if (views[a].valid == false) continue;

                    Int2 select;
                    double maxv = 0.0;
                    for (int b = a + 1; b < views.size(); b++) {
                        if (views[b].valid == false) continue;

                        if (mdmat(a, b).rate < MIN_MATCHRATE) continue;

                        const double eval = evalPair(views, mdmat, a, b);
                        if (eval < maxv) continue;

                        maxv = eval;
                        select.set(a, b);
                    }
                    if (maxv > 0.0) {
                        pairs.push(select);
                    }
                }
                if (pairs.size() == 0) return false;

                srand(update);
                pair = pairs[rand() % pairs.size()];
            }

            // add new pnt
            {
                const int a = pair[0];
                const int b = pair[1];

                const Mem1<Feature> &fts0 = views[a].fts;
                const Mem1<Feature> &fts1 = views[b].fts;
                const Mem1<int> &matches = mdmat(a, b).matches;

                const Pose pose = views[b].pose * invPose(views[a].pose);
                for (int i = 0; i < matches.size(); i++) {
                    if (views[a].index[i] >= 0) continue;

                    int find = -1;
                    for (int v = 0; v < views.size(); v++) {
                        if (a == v || mdmat(a, v).rate < MIN_MATCHRATE) continue;

                        const int j = mdmat(a, v).matches[i];
                        if (j < 0) continue;

                        const int g = views[v].index[j];
                        if (g < 0) continue;

                        const double err = errPose(views[a].pose, views[a].cam, fts0[i].pix, gpnts[g].pos);
                        if (evalErr(err) < 1.0) continue;

                        find = g;
                        break;
                    }
                    if (find >= 0) {
                        addIndex(gpnts, views, find, a, i);
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

                    addIndex(gpnts, views, gpnts.size() - 1, a, i);
                    addIndex(gpnts, views, gpnts.size() - 1, b, j);

                    updateColor(gpnts, views, gpnts.size() - 1);
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
                        const int m = index[i][1];
                        poses[i] = views[v].pose;
                        cams[i] = views[v].cam;
                        pixs[i] = views[v].fts[m].pix;
                    }

                    Vec3 pnt = gpnts[g].pos;
                    if (refinePnt3d(pnt, poses, cams, pixs) == false) continue;
                    
                    gpnts[g].pos = pnt;

                    updateColor(gpnts, views, g);
                }
            }
            return false;
        }

        bool updatePose(Mem1<PointData> &gpnts, Mem1<ViewData> &views, Mem2<MatchData> &mdmat, const int update) {
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
                for (int m = 0; m < views[a].index.size(); m++) {
                    const int g = views[a].index[m];
                    if (g < 0) continue;

                    pixs.push(views[a].fts[m].pix);
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