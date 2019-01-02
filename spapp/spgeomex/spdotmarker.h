//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_DOTMARKER_H__
#define __SP_DOTMARKER_H__

#include "spcore/spcore.h"
#include "spapp/spimg/spimg.h"
#include "spapp/spimg/spbin.h"
#include "spapp/spimg/splabel.h"
#include "spapp/spimg/spfilter.h"
#include "spapp/spgeom/spgeom.h"
#include "spapp/spalgo/spkdtree.h"

namespace sp{

    //--------------------------------------------------------------------------------
    // dot marker design parameter
    //--------------------------------------------------------------------------------

    class DotMarkerParam{
    public:
        Mem2<Vec2> map;

        double distance;

        DotMarkerParam(const int dsize0 = 5, const int dsize1 = 5, const double distance = 30.0){
            setParam(dsize0, dsize1, distance);
        }

        DotMarkerParam(const DotMarkerParam &mrk){
            *this = mrk;
        }

        DotMarkerParam& operator = (const DotMarkerParam &mrk){
            setParam(mrk.map.dsize[0], mrk.map.dsize[1], mrk.distance);
            return *this;
        }

        void setParam(const int dsize0, const int dsize1, const double distance){
            map = grid(dsize0, dsize1) - getVec(dsize0 - 1, dsize1 - 1) * 0.5;

            this->distance = distance;
        }

    };


    //--------------------------------------------------------------------------------
    // dot marker pose estimator
    //--------------------------------------------------------------------------------

    class DotMarker {
    private:

        // input parameter
        CamParam m_cam;

        DotMarkerParam m_mrk;


        // output parameter

        // flag for tracking
        bool m_track;

        // marker to camera pose
        Pose m_pose;

        // marker to image homography
        Mat m_hom;

        // crsp pnts
        Mem1<Vec2> m_cpixs, m_cobjs;

    public:

        DotMarker(){
            m_cam = getCamParam(0, 0);
            m_mrk = DotMarkerParam();

            m_pose = zeroPose();
            m_track = false;
        }


        //--------------------------------------------------------------------------------
        // input parameter
        //--------------------------------------------------------------------------------

        void setMrk(const DotMarkerParam &mrk){
            m_mrk = mrk;
        }

        void setCam(const CamParam &cam) {
            m_cam = cam;
        }

        const DotMarkerParam& getMrk() const{
            return m_mrk;
        }

        const CamParam& getCam() const{
            return m_cam;
        }


        //--------------------------------------------------------------------------------
        // output parameter
        //--------------------------------------------------------------------------------

        const Mat* getHom() const {
            return (m_track == true) ? &m_hom : NULL;
        }

        const Pose* getPose() const{
            return (m_track == true) ? &m_pose : NULL;
        }

        const Mem1<Vec2>* getCrspPixs() const{
            return (m_track == true) ? &m_cpixs : NULL;
        }

        const Mem1<Vec2>* getCrspObjs() const{
            return (m_track == true) ? &m_cobjs : NULL;
        }

        //--------------------------------------------------------------------------------
        // execute pose estimation
        //--------------------------------------------------------------------------------

        bool execute(const void *img, const int dsize0, const int dsize1, const int ch){
            Mem2<Byte> gry;
            cnvPtrToImg(gry, img, dsize0, dsize1, ch);
            return _execute(gry);
        }

        bool execute(const Mem2<Col3> &img){
            Mem2<Byte> gry;
            cnvImg(gry, img);
            return _execute(gry);
        }

        bool execute(const Mem2<Byte> &img){
            return _execute(img);
        }

    private:

        bool _execute(const Mem2<Byte> &img){

            // set default camera parameter
            if (cmpSize(2, m_cam.dsize, img.dsize) == false) {
                m_cam = getCamParam(img.dsize);
            }

            // clear data
            {
                m_pose = zeroPose();
                m_cpixs.clear();
                m_cobjs.clear();
            }

            try{
                if (img.size() == 0) throw "input size";

                // detect blob
                Mem1<Vec2> pixs;
                detect(pixs, img);

                // estimate pose
                estimate(img, pixs);

                m_track = true;
            }
            catch (const char *str){
                SP_PRINTD("DotMarker::execute [%s]\n", str);

                m_track = false;
                return false;
            }

            return true;
        }

    public:
        //--------------------------------------------------------------------------------
        // const parameter
        //--------------------------------------------------------------------------------

        int MIN_IMGSIZE = 200;

        double BIN_BLOCKSIZE = 0.05;

        double DOT_MAXSIZE = 0.2;
        double DOT_CONTRAST = 0.05;

    private:

        double getSearchRange(){
            return maxVal(m_cam.dsize[0], m_cam.dsize[1]) / 2.0;
        }

        double getMinScale(){
            return static_cast<double>(MIN_IMGSIZE) / maxVal(m_cam.dsize[0], m_cam.dsize[1]);
        }

        //--------------------------------------------------------------------------------
        // execute main flow
        //--------------------------------------------------------------------------------

        void detect(Mem1<Vec2> &pixs, const Mem2<Byte> &img){
            Mem2<Byte> minImg;
            {
                rescale(minImg, img, getMinScale(), getMinScale());
                gaussianFilter3x3(minImg, minImg);

                SP_HOLDER_SET("minImg", minImg);
            }

            Mem2<int> labelMap;
            {

                Mem2<Byte> minBin;
                binalizeBlock(minBin, minImg, round(BIN_BLOCKSIZE * MIN_IMGSIZE));
                invert(minBin, minBin);

                labeling(labelMap, minBin);
                 
                SP_HOLDER_SET("labelMap", labelMap);
            }

            // get label center (and refine center)
            {

                getBlob(pixs, img, minImg, labelMap, getMinScale());

                SP_HOLDER_SET("pixs", pixs);
            }
        }

        // marker direction (backup)
        Vec2 m_direct = getVec(1.0, 0.0);

        void estimate(const Mem2<Byte> &img, const Mem1<Vec2> &pixs){

            // make kd tree
            KdTree<double> kdtree(2);
            {

                for (int i = 0; i < pixs.size(); i++){
                    kdtree.addData(&pixs[i]);
                }
                kdtree.makeTree();
            }

            Mat trkHom;
            double trkEval = 0.0;
            if (m_track == true){

                trkEval = track(trkHom, m_hom, m_mrk, pixs, kdtree);
            }

            Mat rcgHom;
            double rcgEval = 0.0;
            {
                
                Mem1<Mem1<Vec2> > links;
                getLink(links, pixs, kdtree, m_direct);
                
                SP_HOLDER_SET("links", links);
                
                if (links.size() == 0){
                    throw "links";
                }

                rcgEval = recog(rcgHom, m_mrk, links, pixs, kdtree);
            }

            {

                if (maxVal(rcgEval, trkEval) < 0.6 * m_mrk.map.dsize[0] * m_mrk.map.dsize[1]){
                    throw "eval";
                }

                m_hom = (trkEval > 0.95 * rcgEval) ? trkHom : rcgHom;

                m_direct = unitVec(m_hom * getVec(1.0, 0.0) - m_hom * getVec(0.0, 0.0));
            }

            {

                getFineCrsp(m_cpixs, m_cobjs, m_hom, m_mrk.map, pixs, kdtree);
                m_cobjs *= m_mrk.distance;

                if (calcPose(m_pose, m_cam, m_cpixs, m_cobjs) == false){
                    throw "calcPose";
                }
            }
        }


        //--------------------------------------------------------------------------------
        // modules
        //--------------------------------------------------------------------------------

        void getBlob(Mem1<Vec2> &pixs, const Mem2<Byte> &orgImg, const Mem2<Byte> &minImg, const Mem2<int> &labelMap, const double minScale){

            const Mem1<Rect> rects = getLabelRect(labelMap);

            for (int i = 0; i < rects.size(); i++) {
                const Rect minRect = adjustRect(rects[i], 1);

                // check outside area
                if (isInRect(getRect2(minImg.dsize), minRect) == false) continue;

                // check size
                const int maxSize = round(minVal(minImg.dsize[0], minImg.dsize[1]) * DOT_MAXSIZE);
                if (maxVal(minRect.dsize[0], minRect.dsize[1]) > maxSize) continue;

                Byte maxv = 0;
                Byte minv = SP_BYTEMAX;
                for (int v = minRect.dbase[1]; v < minRect.dbase[1] + minRect.dsize[1]; v++) {
                    for (int u = minRect.dbase[0]; u < minRect.dbase[0] + minRect.dsize[0]; u++){
                        const Byte val = minImg(u, v);
                        maxv = maxVal(maxv, val);
                        minv = minVal(minv, val);
                    }
                }

                // check contrast
                if (maxv - minv < SP_BYTEMAX * DOT_CONTRAST) continue;

                // set thresh
                const int thresh = (maxv + minv) / 2;

                // fine binalize
                Rect orgRect;
                orgRect.dim = 2;
                for (int d = 0; d < 2; d++){
                    orgRect.dbase[d] = round(minRect.dbase[d] / minScale);
                    orgRect.dsize[d] = round(minRect.dsize[d] / minScale);
                }

                int cnt = 0;
                Vec2 sum = getVec(0.0, 0.0);
                for (int v = orgRect.dbase[1]; v < orgRect.dbase[1] + orgRect.dsize[1]; v++) {
                    for (int u = orgRect.dbase[0]; u < orgRect.dbase[0] + orgRect.dsize[0]; u++) {
                        const Byte val = orgImg(u, v);
                        if (val >= thresh) continue;
                        
                        sum += getVec(u, v);
                        cnt++;
                    }
                }
                if (cnt > 0){
                    pixs.push(sum / cnt);
                }
            }
        }

        double track(Mat &hom, const Mat &pre, const DotMarkerParam &mrk, const Mem1<Vec2> &pixs, const KdTree<double> &kdtree){

            Mem2<int> crspMap(mrk.map.dsize);
            for (int v = 0; v < mrk.map.dsize[1]; v++) {
                for (int u = 0; u < mrk.map.dsize[0]; u++) {
                    crspMap(u, v) = -1;

                    const Vec2 obj = mrk.map(u, v);
                    const Vec2 pix = pre * obj;

                    const int id = kdtree.search(&pix);
                    if (id < 0) continue;
                    crspMap(u, v) = id;
                }
            }

            double maxv = 0.0;
            for (int v = 0; v < mrk.map.dsize[1] - 1; v++) {
                for (int u = 0; u < mrk.map.dsize[0] - 1; u++) {

                    Mem1<Vec2> unit, crsp;
                    for (int y = 0; y < 2; y++){
                        for (int x = 0; x < 2; x++){
                            const int id = crspMap(u + x, v + y);
                            if (id < 0) continue;
                            
                            unit.push(mrk.map(u + x, v + y));
                            crsp.push(pixs[id]);
                        }
                    }
                    if (unit.size() < 4) continue;

                    Mat h;
                    if (calcHMat(h, crsp, unit) == false) continue;
                    if (refineHMat(h, mrk.map, pixs, kdtree) == false) continue;

                    Mem2<double> evalMap;
                    getEvalMap(evalMap, h, mrk.map, pixs, kdtree);
                    const double eval = sumVal(evalMap);

                    if (eval > maxv){
                        maxv = eval;
                        hom = h;
                    }
                }
            }

            return maxv;
        }

        double recog(Mat &hom, const DotMarkerParam &mrk, const Mem1<Mem1<Vec2> > &links, const Mem1<Vec2> &pixs, const KdTree<double> &kdtree){

            const Mem2<Vec2> ext = grid(2 * (mrk.map.dsize[0] - 1), 2 * (mrk.map.dsize[1] - 1));
            const Mem2<Vec2> unit = grid(2, 2) + meanVec(ext) - getVec(0.5, 0.5);

            const int maxn = 10;
            const double step = maxVal(static_cast<double>(links.size()) / maxn, 1.0);

            Mat H = eyeMat(3, 3);
            Vec2 V = getVec(0.0, 0.0);

            double maxv = 0.0;
            for (int n = 0; n < maxn; n++){
                const int i = round(n * step);
                if (i >= links.size()) break;

                Mat h;
                if (calcHMat(h, links[i], unit) == false) continue;
                if (refineHMat(h, ext, pixs, kdtree) == false) continue;

                Mem2<double> evalMap;
                getEvalMap(evalMap, h, ext, pixs, kdtree);

                Vec2 v;
                const double eval = searchPeak(v, evalMap, mrk.map.dsize[0], mrk.map.dsize[1]);
                if (eval > maxv){
                    H = h;
                    V = v;
                    maxv = eval;
                }
            }
            Mat offset = eyeMat(3, 3);
            offset(0, 2) = V.x - mrk.map[0].x;
            offset(1, 2) = V.y - mrk.map[0].y;

            hom = H * offset;

            return maxv;
        }

        void getLink(Mem1<Mem1<Vec2> > &links, const Mem1<Vec2> &pixs, const KdTree<double> &kdtree, const Vec2 &direct){
            const double MIN_ASPECT = 0.3;
            const double MIN_COS = cos(SP_PI * (90.0 + 30.0) / 180.0);

            for (int p0 = 0; p0 < pixs.size(); p0++){
                Mem1<int> index = kdtree.search(&pixs[p0], getSearchRange());

                for (int n1 = 0; n1 < index.size(); n1++){
                    const int p1 = index[n1];
                    if (p1 == p0 || pixs[p1].y < pixs[p0].y) continue;

                    for (int n2 = n1 + 1; n2 < index.size(); n2++) {
                        const int p2 = index[n2];
                        if (p2 == p0 || pixs[p2].y < pixs[p0].y) continue;

                        // select 3 pixs
                        // p0, p1, p2 ( pixs[p1].y, pixs[p2].y >= pixs[p0].y )
                        // s0, s1, s2 ( |pixs[s2] -  pixs[s1]| >= others )

                        int s0, s1, s2;
                        {
                            const double lng01 = normVec(pixs[p1] - pixs[p0]);
                            const double lng12 = normVec(pixs[p2] - pixs[p1]);
                            const double lng20 = normVec(pixs[p0] - pixs[p2]);
                            
                            if (lng12 >= maxVal(lng01, lng20)) {
                                s0 = p0, s1 = p1, s2 = p2;
                            }
                            if (lng20 >= maxVal(lng12, lng01)) {
                                s0 = p1, s1 = p2, s2 = p0;
                            }
                            if (lng01 >= maxVal(lng20, lng12)) {
                                s0 = p2, s1 = p0, s2 = p1;
                            }
                        }

                        // check 3 pixs relation
                        const Vec2 A = pixs[s1] - pixs[s0];
                        const Vec2 B = pixs[s2] - pixs[s0];
                        {
                            const double lngA = normVec(A);
                            const double lngB = normVec(B);
                            const double cosAB = dotVec(A, B) / (lngA * lngB);

                            if (minVal(lngA, lngB) / maxVal(lngA, lngB) < MIN_ASPECT) continue;
                            if (cosAB > 0.0 || cosAB < MIN_COS) continue;
                        }

                        // check outlier in triangle and search s3
                        int s3 = -1;
                        {
                            const double margin = 0.2;

                            double inv[2 * 2];
                            double mat[2 * 2] = { A.x, B.x, A.y, B.y };
                            if (invMat22(inv, mat) == false) continue;

                            bool check = true;
                            for (int k = 0; k < index.size(); k++) {
                                const int pk = index[k];
                                if (pk == p0 || pk == p1 || pk == p2) continue;

                                const Vec2 e = mulMat(inv, 2, 2, pixs[pk] - pixs[s0]);
                                if (minVal(e.x, e.y) < -margin || maxVal(e.x, e.y) > 1.0 + margin) continue;

                                if (s3 < 0 && minVal(e.x, e.y) > 1.0 - margin) {
                                    s3 = pk;
                                }
                                else{
                                    check = false;
                                    break;
                                }
                            }

                            if (s3 < 0 || check == false) continue;
                        }

                        // align pixs
                        Vec2 v[4] = { pixs[s0], pixs[s1], pixs[s2], pixs[s3] };
                        {
                            const Vec2 xdirect = direct;
                            const Vec2 ydirect = getVec(-direct.y, direct.x);

                            for (int i = 0; i < 4; i++){
                                for (int j = i + 1; j < 4; j++){
                                    if (dotVec(ydirect, v[i]) > dotVec(ydirect, v[j])){
                                        swap(v[i], v[j]);
                                    }
                                }
                            }

                            if (dotVec(xdirect, v[0]) > dotVec(xdirect, v[1])) swap(v[0], v[1]);
                            if (dotVec(xdirect, v[2]) > dotVec(xdirect, v[3])) swap(v[2], v[3]);
                        }

                        links.push(Mem1<Vec2>(4, &v));
                    }
                }
            }

        }

        void getEvalMap(Mem2<double> &evalMap, const Mat &hom, const Mem2<Vec2> &map, const Mem1<Vec2> &pixs, const KdTree<double> &kdtree){
            evalMap.resize(map.dsize);
            evalMap.zero();

            const Mat ihom = invMat(hom);
            const Rect rect = getRect2(m_cam.dsize) - 10;

            for (int y = 0; y < evalMap.dsize[1]; y++) {
                for (int x = 0; x < evalMap.dsize[0]; x++) {
                    const Vec2 obj = map(x, y);
                    const Vec2 pix = hom * obj;

                    if (isInRect2(rect, floor(pix.x), floor(pix.y)) == false){
                        evalMap(x, y) = 0.3;
                    }

                    const int id = kdtree.search(&pix);
                    if (id < 0) continue;

                    const double err = normVec(obj - ihom * pixs[id]);

                    if (err < 0.1){
                        evalMap(x, y) = 1.0;
                    }
                    if (err >= 0.2 && err < 0.8){
                        evalMap(x, y) -= 0.5;
                    }
                }
            }
        }

        double searchPeak(Vec2 &peak, const Mem2<double> &evalMap, const int dsize0, const int dsize1){

            double maxEval = 0.0;
            for (int y = 0; y <= evalMap.dsize[1] - dsize1; y++){
                for (int x = 0; x <= evalMap.dsize[0] - dsize0; x++){
                    double eval = 0.0;
                    for (int b = 0; b < dsize1; b++){
                        for (int a = 0; a < dsize0; a++){
                            eval += evalMap(x + a, y + b);
                        }
                    }
                    if (eval > maxEval){
                        maxEval = eval;
                        peak = getVec(x, y);
                    }
                }
            }
            return maxEval;
        }

        void getFineCrsp(Mem1<Vec2> &cpixs, Mem1<Vec2> &cobjs, const Mat &hom, const Mem2<Vec2> &map, const Mem1<Vec2> &pixs, const KdTree<double> &kdtree){
            cpixs.clear();
            cobjs.clear();
            const Mat ihom = invMat(hom);

            for (int y = 0; y < map.dsize[1]; y++) {
                for (int x = 0; x < map.dsize[0]; x++) {
                    const Vec2 obj = map(x, y);
                    const Vec2 pix = hom * obj;

                    const int id = kdtree.search(&pix);
                    if (id < 0) continue;

                    const double err = normVec(obj - ihom * pixs[id]);

                    if (err < 0.1){
                        cpixs.push(pixs[id]);
                        cobjs.push(obj);
                    }
                }
            }
        }

        bool refineHMat(Mat &hom, const  Mem2<Vec2> &map, const Mem1<Vec2> &pixs, const KdTree<double> &kdtree){
            Mem1<Vec2> cpixs, cobjs;
            getFineCrsp(cpixs, cobjs, hom, map, pixs, kdtree);

            return calcHMat(hom, cpixs, cobjs);
        }

    };


    //--------------------------------------------------------------------------------
    // diminish dot marker
    //--------------------------------------------------------------------------------

    SP_CPUFUNC void diminishDotMarker(Mem2<Col3> &img, const DotMarkerParam &mrk, const Mat &hom) {

        Mem2<Col3> cmap(mrk.map.dsize[0] + 1, mrk.map.dsize[1] + 1);
        {
            const Vec2 offset = getVec(mrk.map.dsize[0], mrk.map.dsize[1]) * 0.5;

            Mem2<bool> valid(cmap.dsize);
            valid.zero();

            double sum[3] = { 0 };
            for (int y = 0; y < cmap.dsize[1]; y++) {
                for (int x = 0; x < cmap.dsize[0]; x++) {
                    const Vec2 pix = hom * (getVec(x, y) - offset);
                    if (isInRect2(img.dsize, pix.x, pix.y) == false) continue;

                    acsc(cmap, x, y) = acsc(img, pix.x, pix.y);
                    valid(x, y) = true;
                }
            }

            for (int y = 0; y < cmap.dsize[1]; y++) {
                for (int x = 0; x < cmap.dsize[0]; x++) {
                    if (valid(x, y) == true) continue;

                    int cnt = 0;
                    double sum[3] = { 0 };

                    const int win = 2;
                    for (int b = -win; b <= win; b++) {
                        for (int a = -win; a <= win; a++) {
                            if (valid(x + a, y + b) == false) continue;
                            for (int c = 0; c < 3; c++) {
                                sum[c] += acs2<Col3, Byte>(cmap, x + a, y + b, c);
                            }
                            cnt++;
                        }
                    }
                    if (cnt > 0) {
                        for (int c = 0; c < 3; c++) {
                            cnvVal(acs2<Col3, Byte>(cmap, x, y, c), sum[c] / cnt);
                        }
                    }
                }
            }
        }
        gaussianFilter<Col3, Byte>(cmap, cmap);

        {
            const Mat ihom = invMat(hom);

            for (int v = 0; v < img.dsize[1]; v++) {
                for (int u = 0; u < img.dsize[0]; u++) {
                    const Vec2 vec = mulMat(ihom.ptr, 3, 3, getVec(u, v));

                    const double dw = mrk.map.dsize[0] / 2.0;
                    const double dh = mrk.map.dsize[1] / 2.0;
                    if (fabs(vec.x) > dw + 0.2 || fabs(vec.y) > dh + 0.2) continue;

                    const double rw = maxVal(0.0, fabs(vec.x) - dw);
                    const double rh = maxVal(0.0, fabs(vec.y) - dh);
                    const double rate = 2 * maxVal(rw, rh);

                    const Vec2 pos = vec + getVec(dw, dh);

                    img(u, v) = blendCol(img(u, v), acsc(cmap, pos.x, pos.y), rate);
                }
            }
        }
    }
}
#endif