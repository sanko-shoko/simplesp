﻿//--------------------------------------------------------------------------------
// Copyright (c) 2017-2020, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

// [reference]
// R.Achanta, A.Shaji, K.Smith, A.Lucchi, P.Fua, and S.Susstrunk, 
// "SLIC superpixels compared to state-of-the-art superpixel methods", 
// IEEE Transactions on Pattern Analysis and Machine Intelligence(PAMI), 2011

#ifndef __SP_SLIC_H__
#define __SP_SLIC_H__

#include "spcore/spcore.h"
#include "spapp/spimg/spimg.h"

namespace sp{
    
    SP_CPUFUNC void slic(Mem2<int> &map, const Mem2<Col3> &img, const int step = 20, const int maxit = 5){
        
        Mem2<Col3> smooth;
        gaussianFilter<Col3, Byte>(smooth, img);

        // convert color space
        Mem2<Vec3> labImg(img.dsize);
        for (int i = 0; i < labImg.size(); i++){
            cnvColToLab(labImg[i], smooth[i]);
        }

        // initalize
        Mem1<Vec2> pixs;
        Mem1<Vec3> labs;
        {
            for (int v = step / 2; v < img.dsize[1]; v += step){
                for (int u = step / 2; u < img.dsize[0]; u += step){
                    int su, sv;
                    SP_REAL minv = SP_INFINITY;
                    for (int y = -1; y <= 1; y++){
                        for (int x = -1; x <= 1; x++){
                            const Vec3 dx = labImg(u + x + 1, v + y) - labImg(u + x - 1, v + y);
                            const Vec3 dy = labImg(u + x, v + y + 1) - labImg(u + x, v + y - 1);
                            const SP_REAL val = sqVec(dx) + sqVec(dy);
                            if (val < minv){
                                minv = val;
                                su = u + x;
                                sv = v + y;
                            }
                        }
                    }
                    pixs.push(getVec2(su, sv));
                    labs.push(labImg(su, sv));
                }
            }
        }

        Mem2<int> &clsMap = map;
        Mem2<SP_REAL> dstMap;
        {
            clsMap.resize(img.dsize);
            dstMap.resize(img.dsize);

            setElm(clsMap, -1);
        }

        // iteration
        for (int it = 0; it < maxit; it++){
            for (int i = 0; i < map.size(); i++){
                dstMap[i] = SP_INFINITY;
            }

            // normalize value
            const SP_REAL Nc = 30;
            const SP_REAL Ns = step;

            // assign label
            for (int i = 0; i < pixs.size(); i++){
                Rect2 rect = getRect2(pixs[i]) + step;
                rect = andRect(rect, getRect2(img.dsize));

                for (int v = rect.dbase[1]; v < rect.dbase[1] + rect.dsize[1]; v++){
                    for (int u = rect.dbase[0]; u < rect.dbase[0] + rect.dsize[0]; u++){
                        const Vec2 pix = getVec2(u, v);
                        const Vec3 lab = labImg(u, v);
                        const SP_REAL d = sqVec((lab - labs[i]) / Nc) + sqVec((pix - pixs[i]) / Ns);
                        if (d < dstMap(u, v)){
                            dstMap(u, v) = d;
                            clsMap(u, v) = i;
                        }
                    }
                }
            }
            if (it == maxit - 1) break;

            // update clusters
            {
                Mem1<int> cnts(pixs.dsize);
                cnts.zero();
                labs.zero();
                pixs.zero();

                for (int v = 0; v < img.dsize[1]; v++){
                    for (int u = 0; u < img.dsize[0]; u++){
                        const int label = map(u, v);
                        cnts[label]++;
                        labs[label] += labImg(u, v);
                        pixs[label] += getVec2(u, v);
                    }
                }

                for (int i = 0; i < pixs.size(); i++){
                    const int cnt = cnts[i];
                    if (cnt == 0) continue;
                    pixs[i] /= cnt;
                    labs[i] /= cnt;
                }
            }
        }

        // post process
        {
            Mem2<int> tmpMap(clsMap.dsize);
            for (int i = 0; i < tmpMap.size(); i++){
                tmpMap[i] = -1;
            }

            const int ref[][2] = {
                { -1, 0 }, { 0, -1 }, { +1, 0 }, { 0, +1 }
            };

            int crntLabel = 0;
            const SP_REAL minSize = step * step * 0.2;

            for (int v = 0; v < img.dsize[1]; v++){
                for (int u = 0; u < img.dsize[0]; u++){
                    if (tmpMap(u, v) >= 0) continue;
                    
                    // get near label
                    int nearLabel = -1;
                    {
                        for (int n = 0; n < 4; n++) {
                            const int x = u + ref[n][0];
                            const int y = v + ref[n][1];

                            if (inRect(img.dsize, x, y) == false) continue;
                            if (tmpMap(x, y) < 0) continue;
                            nearLabel = tmpMap(x, y);
                            break;
                        }
                    }

                    Mem1<Vec2> list;
                    {
                        list.push(getVec2(u, v));
                        tmpMap(u, v) = crntLabel;
                    }

                    for (int i = 0; i < list.size(); i++) {
                        for (int n = 0; n < 4; n++) {
                            const int x = round(list[i].x + ref[n][0]);
                            const int y = round(list[i].y + ref[n][1]);
                            if (inRect(img.dsize, x, y) == false) continue;
                            if (tmpMap(x, y) >= 0 || clsMap(u, v) != clsMap(x, y)) continue;

                            tmpMap(x, y) = crntLabel;
                            list.push(getVec2(x, y));
                        }
                    }

                    crntLabel++;
                    if (list.size() > minSize) continue;

                    // assign near label
                    for (int i = 0; i < list.size(); i++) {
                        tmpMap(round(list[i].x), round(list[i].y)) = nearLabel;
                    }
                    crntLabel--;
                }
            }
            clsMap = tmpMap;
        }
    }

}

#endif