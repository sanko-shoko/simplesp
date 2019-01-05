//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_BITMARKER_H__
#define __SP_BITMARKER_H__

#include "spcore/spcore.h"
#include "spapp/spimg/spimg.h"
#include "spapp/spimg/spbin.h"
#include "spapp/spimg/splabel.h"
#include "spapp/spimg/spfilter.h"
#include "spapp/spgeom/spgeom.h"
#include "spapp/spgeomex/spfit.h"
#include "spapp/spdata/spsvg.h"

namespace sp{

    //--------------------------------------------------------------------------------
    // bit marker design parameter
    //--------------------------------------------------------------------------------

#define _SP_BITMARKERSIZE 80

    class BitMarkerParam{

    public:
        // marker size [mm]
        double length;

        // marker image
        Mem2<Byte> img;

        Pose offset;

        BitMarkerParam() {
        }

        BitMarkerParam(const Mem2<Col3> &img, const double length, const Pose &offset = zeroPose()){
            setImg(img);
            this->length = length;
            this->offset = offset;
        }

        BitMarkerParam(const BitMarkerParam &mrk){
            *this = mrk;
        }

        BitMarkerParam& operator = (const BitMarkerParam &mrk){
            this->img = mrk.img;
            this->length = mrk.length;
            this->offset = mrk.offset;
            return *this;
        }

        void setImg(const Mem2<Col3> &img){
            const double sigma = static_cast<double>(img.dsize[0]) / _SP_BITMARKERSIZE;

            Mem2<Col3> tmp;
            gaussianFilter<Col3, Byte>(tmp, img, sigma / 2.0);
            rescale<Col3, Byte>(tmp, tmp);

            Mem2<Byte> gry;
            cnvImg(gry, tmp);

            const Byte minv = minVal(gry);
            const Byte maxv = maxVal(gry);
            const double rate = SP_BYTEMAX / maxVal(static_cast<double>(maxv - minv), 1.0);

            cnvMem(this->img, gry, rate, minv);
        }

    };

    SP_CPUFUNC BitMarkerParam getBitMarkerParam(const int id, const int block, const double length, const Pose &offset = zeroPose()) {

        Mem2<Col3> img(_SP_BITMARKERSIZE, _SP_BITMARKERSIZE);
        img.zero();

        Mem1<bool> bits(block * block);
        for (int i = 0; i < bits.size(); i++) {
            bits[i] = ((id >> i) & 0x01) ? true : false;
        }

        const int step = _SP_BITMARKERSIZE / (block + 2);

        const Rect rect = getRect2(0, 0, block, block);
        for (int v = 0; v < img.dsize[1]; v++) {
            for (int u = 0; u < img.dsize[0]; u++) {
                const int x = (u / step - 1);
                const int y = (v / step - 1);
                if (inRect2(rect, x, y) == false) continue;

                const Byte val = bits[y * block + x] ? 0 : SP_BYTEMAX;
                img(u, v) = getCol(val, val, val);
            }
        }
        renderPoint(img, getVec(-0.5, _SP_BITMARKERSIZE / 2.0 - 0.5), getCol(SP_BYTEMAX, SP_BYTEMAX, SP_BYTEMAX), _SP_BITMARKERSIZE / 20.0);

        return BitMarkerParam(img, length, offset);
    }


    SP_CPUFUNC Mem1<BitMarkerParam> getBitMarkerParam(const int id, const int block, const double length, const int dsize0, const int dsize1, const double interval) {
        Mem1<BitMarkerParam> mrks;

        for (int y = 0; y < dsize1; y++) {
            for (int x = 0; x < dsize0; x++) {
                const double distance = length + interval;
                const Pose offset = getPose(getVec((dsize0 - 1) / 2.0 - x, (dsize1 - 1) / 2.0 - y, 0.0) * distance);
                
                mrks.push(getBitMarkerParam(id + mrks.size(), block, length, offset));
            }
        }

        return mrks;
    }


    SP_CPUFUNC bool saveBitMarkerParamSVG(const char *path, const int id, const int block, const double length, const int dsize0, const int dsize1, const double interval, const int w = 297, const int h = 210) {

        const double distance = length + interval;
        const double unit = length / (block + 2);

        const Vec2 base = (getVec(w, h) - getVec(length + distance * (dsize0 - 1), length + distance * (dsize1 - 1))) / 2.0;

        string str;
        for (int y = 0; y < dsize1; y++) {
            for (int x = 0; x < dsize0; x++) {
                const Vec2 pos = getVec(x, y) * distance + getVec(round(base.x), round(base.y));
                str += _svg::rect(pos.x, pos.y, length, length, "fill='#000000'");

                str += _svg::rect(pos.x + unit, pos.y + unit, unit * block, unit * block, "fill='#FFFFFF'");

                for(int by = 0; by < block; by++){
                    for (int bx = 0; bx < block; bx++) {
                        const bool bit = (((id + y * dsize0 + x) >> (by * block + bx)) & 0x01) ? true : false;
                        if (bit == true) {
                            str += _svg::rect(pos.x + (bx + 1) * unit, pos.y + (by + 1) * unit, unit, unit, "fill='#000000' stroke='#000000' stroke-width='1'");
                        }
                    }
                }
                str += _svg::circle(pos.x, pos.y + length / 2.0, length / 20.0, "fill='#FFFFFF'");
            }
        }
        saveSVG(path, str.c_str(), w, h);
        return true;
    }

    //--------------------------------------------------------------------------------
    // bit marker pose estimater
    //--------------------------------------------------------------------------------

    class BitMarker {
    private:
        // input camera parameter
        CamParam m_cam;

        // input marker parameter
        Mem1<Mem1<BitMarkerParam> > m_mrks;

        // marker to camera poses
        Mem1<Pose> m_poses;

        // crsp pnts
        Mem1<Mem1<Vec2> > m_cpixs;
        Mem1<Mem1<Vec3> > m_cobjs;

    public:

        BitMarker(){
            m_cam = getCamParam(0, 0);
        }

        //--------------------------------------------------------------------------------
        // input parameter
        //--------------------------------------------------------------------------------
    
        void setCam(const CamParam &cam) {
            m_cam = cam;
        }

        void addMrks(const BitMarkerParam &mrk) {
            m_mrks.push(Mem1<BitMarkerParam>(1, &mrk));
        }

        void addMrks(const Mem1<BitMarkerParam> &mrks){
            m_mrks.push(mrks);
        }

        const CamParam& getCam() const {
            return m_cam;
        }
        
        const int size() const {
            return m_mrks.size();
        }
  
        const Mem1<BitMarkerParam>& getMrks(const int i) const{
            return m_mrks[i];
        }


        //--------------------------------------------------------------------------------
        // output parameter
        //--------------------------------------------------------------------------------

        const Pose* getPose(const int i) const {
            if (i < 0 || i >= m_poses.size()) return NULL;
            return (cmpPose(m_poses[i], zeroPose()) == false) ? &m_poses[i] : NULL;
        }

        const Mem1<Vec2>* getCrspPixs(const int i) const {
            if (i < 0 || i >= m_cpixs.size()) return NULL;
            return (m_cpixs[i].size() > 0) ? &m_cpixs[i] : NULL;
        }

        const Mem1<Vec3>* getCrspObjs(const int i) const {
            if (i < 0 || i >= m_cobjs.size()) return NULL;
            return (m_cobjs[i].size() > 0) ? &m_cobjs[i] : NULL;
        }


        //--------------------------------------------------------------------------------
        // execute pose estimation
        //--------------------------------------------------------------------------------

        bool execute(const void *img, const int dsize0, const int dsize1, const int ch) {
            Mem2<Byte> gry;
            cnvPtrToImg(gry, img, dsize0, dsize1, ch);
            return _execute(gry);
        }

        bool execute(const Mem2<Col3> &img) {
            Mem2<Byte> gry;
            cnvImg(gry, img);
            return _execute(gry);
        }

        bool execute(const Mem2<Byte> &img) {
            return _execute(img);
        }

    private:
        bool _execute(const Mem2<Byte> &img){

            // set default camera parameter
            if (cmpSize(2, m_cam.dsize, img.dsize) == false) {
                m_cam = getCamParam(img.dsize);
            }

            // clear output
            m_poses.clear();
            m_cpixs.clear();
            m_cobjs.clear();

            try{
                if (img.size() == 0 || m_mrks.size() == 0) throw "input size";

                // detect corners
                const Mem1<Mem1<Vec2> > corners = detect(img);

                // estimate marker pose
                estimate(img, corners);

            }
            catch (const char *str){
                SP_PRINTD("BitMarker::execute [%s]\n", str);
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

        double MRK_MATCHRATE = 0.75;

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

            Mem1<Mem1<Vec2> > contours;
            {
                contours = getContour(minImg, labelMap);
                contours *= (1.0 / getMinScale());

                SP_HOLDER_SET("contours", contours);
                if (contours.size() == 0) throw "contours";
            }

            Mem1<Mem1<Vec2> > corners;
            {
                corners = getCorner(img, contours);

                SP_HOLDER_SET("corners", corners);
                if (corners.size() == 0) throw "corners";
            }

            return corners;
        }

        void estimate(const Mem2<Byte> &img, const Mem1<Mem1<Vec2> > corners) {

            Mem1<Pose> poses;
            Mem1<Mem1<Vec2> > dpixs;
            {
                if (calcMrk(poses, dpixs, img, corners) == false) throw "calcMrk";
            }

            Mem1<Mem1<int> > crsps;
            {
                if (crspMrk(crsps, poses, img, m_mrks) == false) throw "crspMrk";
            }

            {
                if (postProc(m_poses, m_cpixs, m_cobjs, m_mrks, crsps, dpixs, poses) == false) throw "postProc";
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

        bool calcMrk(Mem1<Pose> &poses, Mem1<Mem1<Vec2> > &dpixs, const Mem2<Byte> &img, const Mem1<Mem1<Vec2> > corners) {

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

                    if (fit2D(pose, img, m_cam, objs, drcs) == false) continue;

                    bool check = true;
                    for (int c = 0; c < unit.size(); c++){
                        const Vec2 pix = mulCamD(m_cam, prjVec(pose * unit[c]));
                        if (normVec(pix - corners[i][c]) > 5.0){
                            check = false;
                        }
                    }
                    if (check == true){
                        poses.push(pose);
                        dpixs.push(corners[i]);
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

                        const Mem1<Vec2> tmp = dpixs[i];
                        for (int j = 0; j < 4; j++) {
                            dpixs[i][j] = tmp[(j + id) % 4];
                        }
                    }
                }
            }
            return true;
        }

        bool crspMrk(Mem1<Mem1<int> > &crsp, const Mem1<Pose> &poses, const Mem2<Byte> &img, const Mem1<Mem1<BitMarkerParam> > &mrks){
            const int *dsize = mrks[0][0].img.dsize;
            const int margin = round(dsize[0] * 0.1);

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

            Mem1<Mem1<Mem1<double> > > evals(mrks.size());
            for (int i = 0; i < mrks.size(); i++) {
                evals[i].resize(mrks[i].size());

                for (int j = 0; j < mrks[i].size(); j++) {
                    const Mem2<Byte> &mimg = mrks[i][j].img;

                    for (int p = 0; p < pimgs.size(); p++) {
                        const Mem2<Byte> &pimg = pimgs[p];

                        int cnt = 0;
                        double sqsum = 0.0;

                        for (int v = margin; v < mimg.dsize[1] - margin; v++) {
                            for (int u = margin; u < mimg.dsize[0] - margin; u++) {
                                sqsum += square(mimg(u, v) - pimg(u, v));
                                cnt++;
                            }
                        }
                        const double eval = 1.0 - 1.0 * sqrt(sqsum / cnt) / SP_BYTEMAX;
                        evals[i][j].push(eval);
                    }
                }
            }

            crsp.resize(mrks.size());
            for (int i = 0; i < mrks.size(); i++) {
                crsp[i].resize(mrks[i].size());
                for (int j = 0; j < mrks[i].size(); j++) {
                    crsp[i][j] = -1;
                }
            }

            int cnt = 0;
            for (int p = 0; p < pimgs.size(); p++){
                int id[2] = { -1, -1 };
                int c = -1;

                double maxEval = -1.0;
                for (int i = 0; i < mrks.size(); i++){
                    for (int j = 0; j < mrks[i].size(); j++) {
                        for (int q = 0; q < pimgs.size(); q++) {
                            const double eval = evals[i][j][q];

                            if (eval > maxEval) {
                                maxEval = eval;
                                id[0] = i;
                                id[1] = j;
                                c = q;
                            }
                        }
                    }
                }
                if (c < 0 || maxEval < MRK_MATCHRATE) break;

                cnt++;
                crsp[id[0]][id[1]] = c;

                for (int i = 0; i < mrks.size(); i++) {
                    for (int j = 0; j < mrks[i].size(); j++) {
                        for (int q = 0; q < pimgs.size(); q++) {
                            if ((i == id[0] && j == id[1]) || q == c) {
                                evals[i][j][q] = -1.0;
                            }
                        }
                    }
                }
            }

            return (cnt > 0) ? true : false;
        }

        bool postProc(Mem1<Pose> &poses, Mem1<Mem1<Vec2> > &cpixs, Mem1<Mem1<Vec3> > &cobjs, const Mem1<Mem1<BitMarkerParam> > &mrks, const Mem1<Mem1<int> > &crsps, Mem1<Mem1<Vec2> > &dpixs, const Mem1<Pose> &cposes) {
            poses.resize(mrks.size());
            cpixs.resize(mrks.size());
            cobjs.resize(mrks.size());

            const Vec2 _unit[4] = { getVec(-0.5, -0.5), getVec(+0.5, -0.5), getVec(+0.5, +0.5), getVec(-0.5, +0.5) };
            const Mem1<Vec2> unit(4, _unit);

            for (int i = 0; i < mrks.size(); i++) {
                poses[i] = zeroPose();

                if (maxVal(crsps[i]) < 0) continue;

                Mem1<Pose> tposes;
                Mem1<Vec2> tcpixs;
                Mem1<Vec3> tcobjs;


                for (int j = 0; j < mrks[i].size(); j++) {
                    if (crsps[i][j] < 0) continue;
                    Pose pose;
                    pose = cposes[crsps[i][j]];
                    pose.trn *= mrks[i][j].length;
                    pose = pose * mrks[i][j].offset;

                    tposes.push(pose);
                    tcpixs.push(dpixs[crsps[i][j]]);
                    tcobjs.push(invPose(mrks[i][j].offset) * (unit * mrks[i][j].length));
                }

                if (mrks[i].size() == 1) {
                    poses[i] = tposes[0];
                    cpixs[i] = tcpixs;
                    cobjs[i] = tcobjs;
                }
                else {
                    if (tposes.size() == 0) continue;

                    Pose pose;

                    double minv = SP_INFINITY;
                    for (int j = 0; j < tposes.size(); j++) {
                        Pose &tmp = tposes[j];
                        if (refinePose(tmp, m_cam, tcpixs, tcobjs) == false) continue;

                        const double err = medianVal(calcPrjErr(tmp, m_cam, tcpixs, tcobjs));

                        if (err < minv) {
                            minv = err;
                            pose = tmp;
                        }
                    }
                    if (minv == SP_INFINITY) continue;

                    const Mem1<double> errs = calcPrjErr(pose, m_cam, tcpixs, tcobjs);
                    poses[i] = pose;
                    cpixs[i] = denoise(tcpixs, errs, minv * 3.0);
                    cobjs[i] = denoise(tcobjs, errs, minv * 3.0);
                }

            }
            return true;
        }
    };


}
#endif