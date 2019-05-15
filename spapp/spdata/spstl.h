//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_STL_H__
#define __SP_STL_H__

#include "spapp/spdata/spfile.h"

namespace sp{

    SP_CPUFUNC bool saveSTL(const char *path, Mem1<Mesh3> &meshes){
        File file;
        if (file.open(path, "wb", ByteOrder::LittleEndian) == false) return false;

        char buf[80] = { 0 };
        file.write(buf, 80);

        unsigned int size = static_cast<unsigned int>(meshes.size());
        file.write(&size, 1);

        for (int i = 0; i < meshes.size(); i++){
            float f[3];

            const Vec3 nrm = getMeshNrm(meshes[i]);
            f[0] = static_cast<float>(nrm.x);
            f[1] = static_cast<float>(nrm.y);
            f[2] = static_cast<float>(nrm.z);
            file.write(f, 3);

            for (int j = 0; j < 3; j++) {
                f[0] = static_cast<float>(meshes[i].pos[j].x);
                f[1] = static_cast<float>(meshes[i].pos[j].y);
                f[2] = static_cast<float>(meshes[i].pos[j].z);
                file.write(f, 3);
            }
            file.write(buf, 2);
        }
        return true;
    }
}

#endif