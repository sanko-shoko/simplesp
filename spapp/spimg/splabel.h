//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_LABEL_H__
#define __SP_LABEL_H__

#include "spcore/spcore.h"
#include "spapp/spimg/spbin.h"

namespace sp{

    //--------------------------------------------------------------------------------
    // labeling
    //--------------------------------------------------------------------------------

    SP_CPUFUNC int labeling(Mem2<int> &map, const Mem2<Byte> &bin, const bool near8 = false){
        
        const int step = bin.dsize[0];

        map.resize(bin.dsize);

        Mem1<int> table;
        table.reserve(bin.size());

        const Rect rect = getRect2(bin.dsize);

        const int linkNum = (near8 == true) ? 4 : 2;
        const int link[][2] = { { -1, 0 }, { 0, -1 }, { -1, -1 }, { +1, -1 } };

        const Byte *pBin = bin.ptr;
        int *pMap = map.ptr;
        int *pTable = table.ptr;

        for (int v = 0; v < bin.dsize[1]; v++){
            for (int u = 0; u < bin.dsize[0]; u++){
                pMap[v * step + u] = -1;
                if (pBin[v * step + u] == 0) continue;

                int crntLabel = table.size();

                int *pTmp[4] = { 0 };

                // check min label
                for (int i = 0; i < linkNum; i++){
                    const int ru = u + link[i][0];
                    const int rv = v + link[i][1];

                    if (inRect2(rect, ru, rv) == false) continue;
                    if (pBin[rv * step + ru] == 0) continue;

                    const int refLabel = table[pMap[rv * step + ru]];
                    pTmp[i] = &pMap[rv * step + ru];
                    if (refLabel < crntLabel){
                        crntLabel = refLabel;
                    }
                }

                if (crntLabel == table.size()){
                    table.push(crntLabel);
                }
                else{
                    for (int i = 0; i < linkNum; i++){
                        if (pTmp[i] != NULL) {
                            pTable[*pTmp[i]] = crntLabel;
                        }
                    }
                }

                pMap[v * step + u] = crntLabel;
            }
        }

        const int tableNum = table.size();

        // update table
        for (int i = 0; i < tableNum; i++){
            int p = i;
            while (pTable[p] != p){
                p = pTable[p];
            }
            pTable[i] = p;
        }

        int labelNum = 0;

        // update label
        int maxv = -1;
        for (int i = 0; i < tableNum; i++){
            if (pTable[i] > maxv){
                maxv = pTable[i];

                for (int j = i; j < tableNum; j++){
                    if (pTable[j] == maxv){
                        pTable[j] = labelNum;
                    }
                }
                labelNum++;
            }
        }

        // update map
        for (int i = 0; i < bin.size(); i++){
            int &id = pMap[i];
            if (id < 0) continue;

            id = pTable[id];
        }

        return labelNum;
    }

    SP_CPUFUNC Mem1<int> getLabelCount(const Mem2<int> &map) {
        const int labelNum = round(maxval(map) + 1);
        //const int step = map.dsize[0];

        Mem1<int> dst(labelNum);
        dst.zero();

        const int *pMap = map.ptr;
        int *pDst = dst.ptr;

        for (int v = 0; v < map.dsize[1]; v++) {
            for (int u = 0; u < map.dsize[0]; u++) {
                const int id = *pMap++;
                if (id < 0) continue;
                pDst[id]++;
            }
        }
        return dst;
    }

    SP_CPUFUNC Mem1<Rect> getLabelRect(const Mem2<int> &map){
        const int labelNum = round(maxval(map) + 1);
        const int step = map.dsize[0];

        Mem1<Rect> dst(labelNum);
        dst.zero();

        const int *pMap = map.ptr;
        Rect *pDst = dst.ptr;

        for (int v = 0; v < map.dsize[1]; v++){
            for (int u = 0; u < map.dsize[0]; u++){
                const int id = *pMap++;
                if (id < 0) continue;
                Rect &rect = pDst[id];
                if (rect.dim == 0) {
                    rect = getRect2(u, v, 1, 1);
                }
                else {
                    const int ru = rect.dbase[0];
                    const int rv = rect.dbase[1];
                    const int rw = rect.dsize[0];
                    const int rh = rect.dsize[1];

                    rect.dbase[0] = minval(u, ru);
                    rect.dsize[0] = maxval(u + 1, ru + rw) - rect.dbase[0];

                    rect.dbase[1] = minval(v, rv);
                    rect.dsize[1] = maxval(v + 1, rv + rh) - rect.dbase[1];
                }
            }
        }
        return dst;
    }

    SP_CPUFUNC Mem1<Mem1<Vec2> > getLabelContour(const Mem2<int> &map, const bool onPixel = true, const bool useImgFrame = true){

        const Mem1<Rect> rects = getLabelRect(map);

        // 8 nears clockwise search
        const int order8[8][2] = {
            { -1, -1 }, { 0, -1 }, { +1, -1 }, { +1, 0 },
            { +1, +1 }, { 0, +1 }, { -1, +1 }, { -1, 0 }
        };
        const int start8[3][3] = {
            { 0, 1, 2 },
            { 7, 0, 3 },
            { 6, 5, 4 }
        };

        // 4 nears clockwise search
        const int order4[8][2] = {
            { 0, -1 }, { +1, 0 }, { 0, +1 }, { -1, 0 }
        };
        const int start4[3][3] = {
            { 0, 0, 0 },
            { 3, 0, 1 },
            { 0, 2, 0 }
        };

        const int(*order)[2] = (onPixel == true) ? order8 : order4;
        const int(*start)[3] = (onPixel == true) ? start8 : start4;

        Mem1< Mem1<Vec2> > dst(rects.size());

        for (int i = 0; i < dst.size(); i++){
            const Rect rect = rects[i];

            if (useImgFrame == false){
                if (inRect(getRect2(map.dsize) - 1, rect) == false) continue;
            }

            int sx = 0;
            int sy = 0;
            for (int y = rect.dbase[1]; y < rect.dbase[1] + rect.dsize[1]; y++){
                for (int x = rect.dbase[0]; x < rect.dbase[0] + rect.dsize[0]; x++){
                    if (map(x, y) == i){
                        sx = x;
                        sy = y;
                        goto _exit;
                    }
                }
            }
        _exit:;

            int vec[2] = { +1, 0 };
            int cx = sx;
            int cy = sy;

            int cnt = 0;
            while (1){

                if (onPixel == true) {
                    const int s = (start[vec[1] + 1][vec[0] + 1] + 8 - 2) % 8;
                    
                    for (int j = 0; j < 8; j++) {
                        const int t = (s + j) % 8;
                        const int x = cx + order[t][0];
                        const int y = cy + order[t][1];
                        if (inRect2(rect, x, y) == false) continue;

                        if (map(x, y) == i) {
                            vec[0] = x - cx;
                            vec[1] = y - cy;

                            cx = x;
                            cy = y;
                            break;
                        }
                    }
                    dst[i].push(getVec2(cx, cy));
                }
                else {
                    const int s = (start[vec[1] + 1][vec[0] + 1] + 4 - 1) % 4;
                   
                    Rect trect = rect;
                    trect.dsize[0]++;
                    trect.dsize[1]++;

                    // clockwise
                    bool list[4] = { map(cx - 1, cy - 1) == i, map(cx, cy - 1) == i, map(cx, cy) == i, map(cx - 1, cy) == i };
                  
                    if (cx == 0) {
                        list[0] = false;
                        list[3] = false;
                    }
                    if (cy == 0) {
                        list[0] = false;
                        list[1] = false;
                    }
                    if (cx == map.dsize[0]) {
                        list[1] = false;
                        list[2] = false;
                    }
                    if (cy == map.dsize[1]) {
                        list[2] = false;
                        list[3] = false;
                    }

                    bool edge[4] = { list[0] != list[1], list[1] != list[2], list[2] != list[3], list[3] != list[0] };

                    for (int j = 0; j < 4; j++) {
                        const int t = (s + j) % 4;
                        const int x = cx + order[t][0];
                        const int y = cy + order[t][1];
                        if (inRect2(trect, x, y) == false) continue;

                        if (edge[t] == true) {
                            vec[0] = x - cx;
                            vec[1] = y - cy;

                            cx = x;
                            cy = y;

                            break;
                        }
                    }

                    dst[i].push(getVec2(cx - 0.5, cy - 0.5));
                }

                if (cx == sx && cy == sy){
                    break;
                }
            }
        }

        return dst;
    }


    SP_CPUFUNC Mem1<Mem1<Vec2> > getLabelMesh(const Mem2<int> &map, const bool useImgFrame = true) {

        const Mem1<Rect> rects = getLabelRect(map);

        const Mem1<Mem1<Vec2 > > contours = getLabelContour(map, false, useImgFrame);

        Mem1<Mem1<Vec2> > dst(rects.size());

        for (int i = 0; i < dst.size(); i++) {
            const Rect rect = rects[i];

            if (useImgFrame == false) {
                if (inRect(getRect2(map.dsize) - 1, rect) == false) continue;
            }

            const Mem1<Vec2> &contour = contours[i];

            Mem1<Vec2> tmps;
            for (int j = 0; j < contour.size(); j++) {
                tmps.push(contour[j]);
                tmps.push((contour[j] + contour[(j + 1) % contour.size()]) * 0.5);
            }

            Mem1<int> vtxs;

            for (int j = 0; j < tmps.size(); j += 2) {

                const int x = floor(tmps[j].x + 1.0);
                const int y = floor(tmps[j].y + 1.0);

                int list[4] = { map(x - 1, y - 1), map(x, y - 1), map(x, y), map(x - 1, y) };

                if (x == 0) {
                    list[0] = -2;
                    list[3] = -2;
                }
                if (y == 0) {
                    list[0] = -2;
                    list[1] = -2;
                }
                if (x == map.dsize[0]) {
                    list[1] = -2;
                    list[2] = -2;
                }
                if (y == map.dsize[1]) {
                    list[2] = -2;
                    list[3] = -2;
                }

                if (x == 0 && y == 0) {
                    list[0] = -3;
                }
                if (x == map.dsize[0] && y == 0) {
                    list[1] = -3;
                }
                if (x == map.dsize[0] && y == map.dsize[1]) {
                    list[2] = -3;
                }
                if (x == 0 && y == map.dsize[1]) {
                    list[3] = -3;
                }

                if (list[0] < 0 && list[1] < 0 && list[2] < 0 && list[3] < 0) continue;

                const bool h0 = (list[0] != list[1]);
                const bool h1 = (list[2] != list[3]);
                const bool v0 = (list[0] != list[3]);
                const bool v1 = (list[1] != list[2]);

                if ((h0 && h1 && (v0 || v1)) || (v0 && v1 && (h0 || h1))) {
                    vtxs.push(j);
                }
            }

            bool loop = true;
            while (loop) {
                loop = false;

                for (int j = 0; j < vtxs.size(); j++) {
                    const int a = vtxs[(j + 0) % vtxs.size()];
                    const int b = vtxs[(j + 1) % vtxs.size()];

                    const Vec2 A = tmps[a];
                    const Vec2 B = tmps[b];
                    const Vec2 v = unitVec(A - B);
                    const Vec2 n = getVec2(-v.y, v.x);

                    const Vec2 t = getVec2(1.0, 1.0);

                    SP_REAL maxv = 0.0;
                    int id = -1;

                    const SP_REAL dA = dotVec(A, t);
                    const SP_REAL dB = dotVec(B, t);

                    if (dA < dB || (cmpVal(dA, dB) && (A.y < B.y)) ) {
                        for (int k = a + 1; ; k++) {
                            const int c = k % tmps.size();
                            if (c == b) break;

                            const Vec2 C = tmps[c];

                            const SP_REAL len = fabs(dotVec(n, A - C));

                            if (len > maxv) {
                                maxv = len;
                                id = c;
                            }
                        }
                    }
                    else {
                        for (int k = b - 1; ; k--) {
                            const int c = (k + tmps.size()) % tmps.size();
                            if (c == a) break;

                            const Vec2 C = tmps[c];

                            const SP_REAL len = fabs(dotVec(n, B - C));

                            if (len > maxv) {
                                maxv = len;
                                id = c;
                            }
                        }
                    }

                    if (maxv >= 2.0) {
                        vtxs.add(j + 1, id);
                        loop = true;
                        break;
                    }
                }
            }

            for (int j = 0; j < vtxs.size(); j++) {
                dst[i].push(tmps[vtxs[j]]);
            }
        }
        return dst;
    }
}

#endif