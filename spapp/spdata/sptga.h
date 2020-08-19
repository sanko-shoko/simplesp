//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_TGA_H__
#define __SP_TGA_H__

#include "spapp/spdata/spfile.h"
#include "spapp/spimg/spimg.h"

namespace sp{
    // http://www.paulbourke.net/dataformats/tga/

    SP_CPUFUNC bool saveTGA(const char *path, const Mem<Col4> &src){
        File file;
        if (file.open(path, "wb") == false) return false;

        const int w = src.dsize[0];
        const int h = src.dsize[1];
        const int c = 4;

        BYTE header[18] = { 0,0,2,0,0,0,0,0,0,0,0,0, (BYTE)(w % 256), (BYTE)(w / 256), (BYTE)(h % 256), (BYTE)(h / 256), (BYTE)(c * 8), 0x20 };
        file.write(header, 18);

        for (int i = 0; i < src.size(); i++) {
            file.write(&src[i].b, 1);
            file.write(&src[i].g, 1);
            file.write(&src[i].r, 1);
            file.write(&src[i].a, 1);
        }

        return true;
    }


}

#endif