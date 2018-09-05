//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_BITMARKER_H__
#define __SP_BITMARKER_H__

#include "spcore/spcore.h"
#include "spapp/spimg/spimage.h"
#include "spapp/spimg/spbinalize.h"
#include "spapp/spimg/splabeling.h"
#include "spapp/spimg/spfilter.h"
#include "spapp/spgeom/spgeometry.h"
#include "spapp/spgeomex/sptrack.h"

namespace sp{

    //--------------------------------------------------------------------------------
    // bit marker design parameter
    //--------------------------------------------------------------------------------
    
    class BitMarkerParam{
    private:
        static const int size = 50;

    public:
        Mem2<Byte> img;
        double length;
        Pose offset;

        BitMarkerParam(const int id = 0, const double length = 50.0){
            setImg(id);
            setLength(length);
            this->offset = zeroPose();
        }

        BitMarkerParam(const Mem2<Col3> &img, const double length = 50.0){
            setImg(img);
            setLength(length);
            this->offset = zeroPose();
        }

        BitMarkerParam(const BitMarkerParam &mrk){
            *this = mrk;
        }

        BitMarkerParam& operator = (const BitMarkerParam &mrk){
            img = mrk.img;
            length = mrk.length;
            offset = mrk.offset;
            return *this;
        }

        void setImg(const int id, const int block = 3){
            setImg(makeImg(id, block));
        }

        void setImg(const Mem2<Col3> &img){
            Mem2<Col3> tmp(size, size);
            rescale<Col3, Byte>(tmp, img);

            Mem2<Byte> gry;
            cnvImg(gry, tmp);

            const Byte minv = minVal(gry);
            const Byte maxv = maxVal(gry);
            const double rate = SP_BYTEMAX / maxVal(static_cast<double>(maxv - minv), 1.0);

            cnvMem(this->img, gry, rate, minv);
        }

        void setLength(const double length){
            this->length = length;
        }

        void setOffset(const Pose &offset){
            this->offset = offset;
        }

        void setOffset(const int id, const int dsize0, const int dsize1, const double distance){
            const int x = id % dsize0;
            const int y = id / dsize0;

            this->offset.rot = zeroRot();
            this->offset.trn = getVec((dsize0 - 1) / 2.0 - x, (dsize1 - 1) / 2.0 - y, 0.0) * distance;
        }
    private:

        Mem2<Col3> makeImg(const int id, const int block){
            Mem2<Col3> dst(size, size);
            dst.zero();

            Mem1<bool> bit(block * block);
            for (int i = 0; i < bit.size(); i++){
                bit[i] = ((id >> i) & 0x01) ? true : false;
            }

            const int step = size / (block + 2);
            const Rect rect = getRect2(0, 0, block, block);
            for (int v = 0; v < dst.dsize[1]; v++){
                for (int u = 0; u < dst.dsize[0]; u++){
                    const int x = (u / step - 1);
                    const int y = (v / step - 1);
                    if (isInRect2(rect, x, y) == false) continue;

                    const Byte val = bit[y * block + x] ? 0 : 255;
                    dst(u, v) = getCol(val, val, val);
                }
            }
            return dst;
        }

    };

    //--------------------------------------------------------------------------------
    // bit marker pose estimater
    //--------------------------------------------------------------------------------

    class BitMarker {
    private:
        // input parameter
        CamParam m_cam;

        Mem1<BitMarkerParam> m_mrks;

        // marker to camera poses
        Mem1<Pose> m_poses;

        // marker corners
        Mem1<Mem1<Vec2> > m_corners;

    public:
        SP_LOGGER_INSTANCE;
        SP_HOLDER_INSTANCE;

        BitMarker(){
            m_cam = sp::getCamParam(0, 0);
            m_mrks.push(BitMarkerParam());
        }

        //--------------------------------------------------------------------------------
        // input parameter
        //--------------------------------------------------------------------------------

        void setMrks(const Mem1<BitMarkerParam> &mrks){
            m_mrks = mrks;
        }

        void setMrks(const BitMarkerParam &mrk){
            m_mrks.clear();
            m_mrks.push(mrk);
        }

        void setCam(const CamParam &cam) {
            m_cam = cam;
        }


        const Mem1<BitMarkerParam>& getMrks() const{
            return m_mrks;
        }

        const CamParam& getCam() const{
            return m_cam;
        }


        //--------------------------------------------------------------------------------
        // output parameter
        //--------------------------------------------------------------------------------

        const Pose* getPose(const int i) const {
            if (i < 0 || i >= m_poses.size()) return NULL;
            return (cmpPose(m_poses[i], zeroPose()) == false) ? &m_poses[i] : NULL;
        }

        const Mem1<Vec2>* getCorners(const int i) const {
            if (i < 0 || i >= m_corners.size()) return NULL;
            return (m_corners[i].size() > 0) ? &m_corners[i] : NULL;
        }


        //--------------------------------------------------------------------------------
        // execute pose estimation
        //--------------------------------------------------------------------------------

        bool execute(const void *src, const int dsize0, const int dsize1, const int ch) {
            Mem2<Byte> gry;
            cnvPtrToImg(gry, src, dsize0, dsize1, ch);
            return _execute(gry);
        }

        bool execute(const Mem2<Col3> &src) {
            Mem2<Byte> gry;
            cnvImg(gry, src);
            return _execute(gry);
        }

        bool execute(const Mem2<Byte> &src) {
            return _execute(src);
        }

    private:
        bool _execute(const Mem2<Byte> &src){
            SP_LOGGER_SET("-execute");

            // set default camera parameter
            if (cmpSize(2, m_cam.dsize, src.dsize) == false) {
                m_cam = sp::getCamParam(src.dsize);
            }

            // clear output
            m_poses.clear();

            try{
                if (src.size() == 0) throw "image size";
                if (m_mrks.size() == 0) throw "mrks size";

                // detect corners
                const Mem1<Mem1<Vec2> > corners = detect(src);

                // estimate marker pose
                estimate(src, corners);

            }
            catch (const char *str){
                SP_PRINTD("BitMarker.execute [%s]\n", str);
                return false;
            }

            return true;
        }

    public:
        //--------------------------------------------------------------------------------
        // const parameter
        //--------------------------------------------------------------------------------
        int MIN_IMGSIZE = 320;

        double BIN_BLOCKSIZE = 0.05;

        double MRK_MINSIZE = 0.01;
        double MRK_CONTRAST = 0.05;

    private:
        double getMinScale(){
            return static_cast<double>(MIN_IMGSIZE) / maxVal(m_cam.dsize[0], m_cam.dsize[1]);
        }

        //--------------------------------------------------------------------------------
        // main flow
        //--------------------------------------------------------------------------------

        Mem1<Mem1<Vec2> > detect(const Mem2<Byte> &img){
            Mem2<Byte> minImg;

            {
                SP_LOGGER_SET("rescale + filter");
                rescale(minImg, img, getMinScale(), getMinScale());
                gaussianFilter3x3(minImg, minImg);

                SP_HOLDER_SET("minImg", minImg);
            }

            Mem2<int> labelMap;
            {
                SP_LOGGER_SET("labeling");
                Mem2<Byte> minBin;
                binalizeBlock(minBin, minImg, round(BIN_BLOCKSIZE * MIN_IMGSIZE));
                invert(minBin, minBin);

                labeling(labelMap, minBin);
                SP_HOLDER_SET("labelMap", labelMap);
            }

            Mem1<Mem1<Vec2> > contours;
            {
                SP_LOGGER_SET("contour");
                contours = getContour(minImg, labelMap);
                contours *= (1.0 / getMinScale());

                SP_HOLDER_SET("contours", contours);
                if (contours.size() == 0) throw "contours";
            }

            Mem1<Mem1<Vec2> > corners;
            {
                SP_LOGGER_SET("corner");
                corners = getCorner(img, contours);

                SP_HOLDER_SET("corners", corners);
                if (corners.size() == 0) throw "corners";
            }

            return corners;
        }

        void estimate(const Mem2<Byte> &img, const Mem1<Mem1<Vec2> > corners) {

            Mem1<Pose> poses;
            Mem1<Mem1<Vec2> > corners2;
            {
                SP_LOGGER_SET("calcMrk");

                if (calcMrk(poses, corners2, img, corners) == false) throw "calcMrk";
            }

            Mem1<int> crsps;
            {
                SP_LOGGER_SET("crspMrk");

                crsps = crspMrk(poses, img);
                if (maxVal(crsps) < 0) throw "crspMrk";

                m_poses.resize(crsps.size());
                m_corners.resize(crsps.size());

                for (int i = 0; i < m_poses.size(); i++){
                    m_poses[i] = zeroPose();
                    if (crsps[i] < 0) continue;
                    m_poses[i] = poses[crsps[i]];
                    m_poses[i].trn *= m_mrks[i].length;
                    m_corners[i] = corners2[crsps[i]];
                }
            }

        }


        //--------------------------------------------------------------------------------
        // modules
        //--------------------------------------------------------------------------------

        Mem1<Mem1<Vec2> > getContour(const Mem2<Byte> &img, Mem2<int> &labelMap) {
            Mem1<Mem1<Vec2> > dst;

            const Mem1<Mem1<Vec2> > contours = getLabelContour(labelMap, true, false);

            for (int i = 0; i < contours.size(); i++) {
                const Mem1<Vec2> &contour = contours[i];
                if (contour.size() == 0) continue;

                // center position
                const Vec2 cent = meanVec(contour);

                // tracing contour and count good contrast
                int cnt = 0;
                double minLng = SP_INFINITY;
                for (int j = 0; j < contour.size(); j++) {
                    const Vec2 crnt = contour[j];
                    const Vec2 side = crnt + unitVec(crnt - cent) * 2.0;
                    const double contrast = img(round(side.x), round(side.y)) - img(round(crnt.x), round(crnt.y));
                    if (contrast > SP_BYTEMAX * MRK_CONTRAST) {
                        cnt++;
                    }

                    minLng = minVal(minLng, normVec(crnt - cent));
                }

                // check contrast
                if (static_cast<double>(cnt) / contour.size() < 0.8) continue;

                // check size
                if (minLng < minVal(img.dsize[0], img.dsize[1]) * MRK_MINSIZE) continue;

                dst.push(contour);
            }
            return dst;
        }

        Mem1<Mem1<Vec2> > getCorner(const Mem2<Byte> &img, const Mem1<Mem1<Vec2> > &contours) {
            Mem1<Mem1<Vec2> > dst;

            const int CORNER_SIDE = 5;

            // detect corner
            for (int i = 0; i < contours.size(); i++) {
                const Mem1<Vec2> &contour = contours[i];
                const Vec2 cent = meanVec(contour);

                Mem1<Vec2> corner(4);
                for (int c = 0; c < 4; c++){

                    double maxv = 0.0;
                    for (int p = 0; p < contour.size(); p++) {
                        const Vec2 crnt = contour[p];
                        const Vec2 prev = contour[(p + contour.size() - CORNER_SIDE) % contour.size()];
                        const Vec2 next = contour[(p + contour.size() + CORNER_SIDE) % contour.size()];

                        if (c > 0 && normVec(crnt - corner[c - 1]) < CORNER_SIDE) continue;

                        const double crs = (c > 0) ? crsVec(crnt - corner[c - 1], cent - corner[c - 1]).z : 1.0;

                        const double val = crs * dotVec(unitVec(prev - crnt) + unitVec(next - crnt), unitVec(cent - crnt));
                        if (val > maxv) {
                            maxv = val;
                            corner[c] = crnt;
                        }
                    }
                }
                dst.push(corner);
            }

            Mem2<Byte> part, bin;
            const int WIN_SIZE = round(10.0 / getMinScale());

            // refine
            for (int i = 0; i < dst.size(); i++) {
                Mem1<Vec2> &corner = dst[i];
                const Vec2 cent = meanVec(corner);

                for (int c = 0; c < 4; c++){
                    const int u = round(corner[c].x);
                    const int v = round(corner[c].y);

                    // crop
                    crop(part, img, getRect2(u, v, 1, 1) + WIN_SIZE);

                    // binalize
                    binalize(bin, part, (maxVal(part) + minVal(part)) / 2);

                    // update position
                    const int step = 3;
                    const int maxit = WIN_SIZE / step;

                    const Vec2 a = unitVec(corner[(c + 4 - 1) % 4] - corner[c]);
                    const Vec2 b = unitVec(corner[(c + 4 + 1) % 4] - corner[c]);

                    Vec2 pos = getVec(0.0, 0.0);
                    for (int it = 0; it < maxit; it++){
                        Vec2 delta = unitVec(cent - corner[c]);

                        double maxv = SP_INFINITY;
                        for (int y = -step; y <= step; y++){
                            for (int x = -step; x <= step; x++){
                                const int xx = round(pos.x) + x + WIN_SIZE;
                                const int yy = round(pos.y) + y + WIN_SIZE;
                                if (bin(xx, yy) > 0) continue;

                                const Vec2 crnt = getVec(x, y);
                                const double val = dotVec(crnt, a) + dotVec(crnt, b);
                                if (val < maxv){
                                    maxv = val;
                                    delta = crnt;
                                }
                            }
                        }
                        if (normVec(delta) < SP_SMALL) break;
                        pos += delta;
                    }

                    corner[c] = getVec(u, v) + pos;
                }
            }

            return dst;
        }

        bool calcMrk(Mem1<Pose> &poses, Mem1<Mem1<Vec2> > &corners2, const Mem2<Byte> &img, const Mem1<Mem1<Vec2> > corners) {

            // calc pose
            {
                const Vec2 _unit[4] = { getVec(-0.5, -0.5), getVec(+0.5, -0.5), getVec(+0.5, +0.5), getVec(-0.5, +0.5) };
                const Mem1<Vec2> unit(4, _unit);

                Mem1<Vec2> objs, drcs;
                const double step = 0.1;
                for (int c = 0; c < 4; c++){
                    const Vec2 pos = unit[c];
                    const Vec2 drc = unit[(c + 1) % 4] - unit[c];
                    for (int i = 1; i < 10 - 1; i++){
                        objs.push(pos + drc * (i * step));
                        drcs.push(drc);
                    }
                }

                poses.clear();
                for (int i = 0; i < corners.size(); i++) {
                    Pose pose;
                    if (calcPose(pose, m_cam, corners[i], unit) == false) continue;

                    if (track2D(pose, img, m_cam, objs, drcs) == false) continue;

                    bool check = true;
                    for (int c = 0; c < unit.size(); c++){
                        const Vec2 pix = mulCamD(m_cam, prjVec(pose * unit[c]));
                        if (normVec(pix - corners[i][c]) > 5.0){
                            check = false;
                        }
                    }
                    if (check == true){
                        poses.push(pose);
                        corners2.push(corners[i]);
                    }
                }
                if (poses.size() > 0) false;
            }

            // calc z rotation
            {
                const int CHECK_SIZE = 5;

                Mem1<Vec2> objs, drcs;
                objs.push(getVec(-0.5, 0.0));
                objs.push(getVec(0.0, -0.5));
                objs.push(getVec(+0.5, 0.0));
                objs.push(getVec(0.0, +0.5));

                drcs.push(getVec(0.0, -1.0));
                drcs.push(getVec(+1.0, 0.0));
                drcs.push(getVec(0.0, +1.0));
                drcs.push(getVec(-1.0, 0.0));

                for (int i = 0; i < poses.size(); i++) {
                    const Pose pose = poses[i];

                    int id = 0;
                    double maxv = 0.0;
                    for (int j = 0; j < 4; j++){
                        const Vec3 pos = poses[i] * getVec(objs[j].x, objs[j].y, 0.0);
                        const Vec3 drc = poses[i].rot * getVec(drcs[j].x, drcs[j].y, 0.0);

                        double jNpxToDist[2 * 2];
                        jacobNpxToDist(jNpxToDist, m_cam, prjVec(pos));

                        const Vec2 drc2 = mulMat(jNpxToDist, 2, 2, getVec(drc.x, drc.y));
                        const Vec2 nrm = unitVec(getVec(-drc2.y, drc2.x));

                        const Vec2 pixA = mulCamD(m_cam, prjVec(pos + drc * 0.1));
                        const Vec2 pixB = mulCamD(m_cam, prjVec(pos - drc * 0.1));
                        const Vec2 pixC = mulCamD(m_cam, prjVec(pos));

                        double sum = 0.0;
                        for (int k = -CHECK_SIZE; k <= CHECK_SIZE; k++){
                            const Vec2 a = pixA + nrm * k;
                            const Vec2 b = pixB + nrm * k;
                            const Vec2 c = pixC + nrm * k;
                            sum += 2 * acs2(img, c.x, c.y) - (acs2(img, a.x, a.y) + acs2(img, b.x, b.y));
                        }

                        if (sum > maxv){
                            maxv = sum;
                            id = j;
                        }
                    }

                    if (id > 0) {
                        poses[i] = pose * getRotAngleZ(id * SP_PI / 2.0);

                        const Mem1<Vec2> tmp = corners2[i];
                        for (int j = 0; j < 4; j++) {
                            corners2[i][j] = tmp[(j + id) % 4];
                        }
                    }
                }
            }
            return true;
        }

        Mem1<int> crspMrk(const Mem1<Pose> &poses, const Mem2<Byte> &img){

            const int *dsize = m_mrks[0].img.dsize;
            const int margin = static_cast<int>(0.1 * dsize[0]);

            Mem1<Mem2<Byte> > pimgs(poses.size());
            for (int p = 0; p < pimgs.size(); p++){
                Mem2<Byte> &pimg = pimgs[p];
                pimg.resize(dsize);

                const CamParam &cam = m_cam;
                const Pose &pose = poses[p];

                const Vec2 offset = getVec(dsize[0] - 1, dsize[1] - 1) * 0.5;
                for (int v = 0; v < dsize[1]; v++){
                    for (int u = 0; u < dsize[0]; u++){
                        const Vec2 vec = getVec(u, v) - offset;
                        const Vec3 obj = getVec(vec.x / dsize[0], vec.y / dsize[1], 0.0);

                        const Vec2 pix = mulCamD(cam, prjVec(pose * obj));

                        pimg(u, v) = img(round(pix.x), round(pix.y));
                    }
                }
                Mem2<Byte> timg;
                gaussianFilter3x3(timg, pimg);

                Byte maxv = 0;
                Byte minv = SP_BYTEMAX;
                for (int v = margin; v < dsize[1] - margin; v++){
                    for (int u = margin; u < dsize[0] - margin; u++){
                        maxv = maxVal(maxv, timg(u, v));
                        minv = minVal(minv, timg(u, v));
                    }
                }

                for (int s = 0; s < pimg.size(); s++){
                    pimg[s] = minVal(pimg[s], maxv);
                    pimg[s] = maxVal(pimg[s], minv);
                }

                const double rate = SP_BYTEMAX / maxVal(static_cast<double>(maxv - minv), 1.0);
                cnvMem(pimg, pimg, rate, minv);
            }

            Mem1<Mem1<double> > evals(m_mrks.size());
            for (int m = 0; m < m_mrks.size(); m++){
                const Mem2<Byte> &mimg = m_mrks[m].img;

                for (int p = 0; p < pimgs.size(); p++){
                    const Mem2<Byte> &pimg = pimgs[p];

                    int cnt = 0;
                    double sum = 0.0;
                    for (int v = margin; v < mimg.dsize[1] - margin; v++){
                        for (int u = margin; u < mimg.dsize[0] - margin; u++){
                            sum += abs(mimg(u, v) - pimg(u, v));
                            cnt++;
                        }
                    }
                    const double eval = 1.0 - 2.0 * sum / (cnt * SP_BYTEMAX);
                    evals[m].push(eval);
                }
            }

            Mem1<int> dst(m_mrks.size());
            for (int i = 0; i < dst.size(); i++){
                dst[i] = -1;
            }

            for (int i = 0; i < pimgs.size(); i++){
                int id = -1;
                int crsp = -1;

                double maxEval = -1.0;
                for (int m = 0; m < m_mrks.size(); m++){
                    for (int p = 0; p < pimgs.size(); p++){
                        const Mem2<Byte> &mimg = m_mrks[i].img;
                        const double eval = evals[m][p];

                        if (eval > maxEval){
                            maxEval = eval;
                            id = m;
                            crsp = p;
                        }
                    }
                }
                if (id < 0 || maxEval < 0.3) break;

                dst[id] = crsp;
                for (int m = 0; m < m_mrks.size(); m++){
                    for (int p = 0; p < pimgs.size(); p++){
                        if (m == id || p == crsp){
                            evals[m][p] = -1.0;
                        }
                    }
                }
            }

            return dst;
        }


    };


    //--------------------------------------------------------------------------------
    // Bit Marker Array
    //--------------------------------------------------------------------------------
  
    Mem1<BitMarkerParam> getBitMarkerArray(const int dsize0, const int dsize1, const double length, const double distance) {
        Mem1<BitMarkerParam> mrks;

        for (int y = 0; y < dsize1; y++) {
            for (int x = 0; x < dsize0; x++) {
                const int id = mrks.size();

                BitMarkerParam mrk(id, length);
                mrk.setOffset(id, dsize0, dsize1, distance);

                mrks.push(mrk);
            }
        }

        return mrks;
    }

    bool calcBitMarkerArrayPose(Pose &dst, const Mem2<Byte> &img, const CamParam &cam, const Mem1<BitMarkerParam> &mrks) {
        dst = zeroPose();

        BitMarker bitMarker;

        bitMarker.setCam(cam);
        bitMarker.setMrks(mrks);
        
        bitMarker.execute(img);

        const Vec2 _unit[4] = { getVec(-0.5, -0.5), getVec(+0.5, -0.5), getVec(+0.5, +0.5), getVec(-0.5, +0.5) };
        const Mem1<Vec2> unit(4, _unit);

        Mem1<Pose> vposes;
        Mem1<Vec2> pixs;
        Mem1<Vec3> objs;
        for (int i = 0; i < mrks.size(); i++) {
            if (bitMarker.getPose(i) == NULL || bitMarker.getCorners(i) == NULL) continue;
            
            Mem1<Vec3> pnts;
            for (int j = 0; j < unit.size(); j++) {
                pnts.push(invPose(mrks[i].offset) * (getVec(unit[j], 0.0) * mrks[i].length));
            }

            objs.push(pnts);
            pixs.push(*bitMarker.getCorners(i));
            vposes.push(*bitMarker.getPose(i) * mrks[i].offset);

        }
        if (vposes.size() == 0) return false;

        double maxEval = 0.0;
        for (int i = 0; i < vposes.size(); i++) {
            Pose &pose = vposes[i];
            if (refinePose(pose, cam, pixs, objs) == false) continue;

            const double eval = evalErr(errPose(pose, cam, pixs, objs));

            if (eval > maxEval) {
                maxEval = eval;
                dst = pose;
            }
        }
        return true;
    }

    bool calcBitMarkerArrayPose(Pose &dst, const Mem2<Col3> &img, const CamParam &cam, const Mem1<BitMarkerParam> &mrks) {
        Mem2<Byte> gry;
        cnvImg(gry, img);
        return calcBitMarkerArrayPose(dst, gry, cam, mrks);
    }
}
#endif