//--------------------------------------------------------------------------------
// Copyright (c) 2017-2020, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_DATASET_H__
#define __SP_DATASET_H__

#include "spcore/spcore.h"
#include "spapp/spdata/spfile.h"
#include "spapp/splearn/spneubase.h"

namespace sp{

    //--------------------------------------------------------------------------------
    // mnist
    //--------------------------------------------------------------------------------

    namespace _mnist{
        template<typename TYPE>
        SP_CPUFUNC bool loadImages(Mem1<TYPE> &images, const char *dir, const char *name, const double scale = 1.0){
            File file;
            char path[512];
            
            sprintf(path, "%s/%s%s", dir, name, "-images.idx3-ubyte");
            if (file.open(path, "rb", BigEndian) == false) return false;

            int buf, num;
            file.read(&buf, 1);
            file.read(&num, 1);

            int x, y;
            file.read(&x, 1);
            file.read(&y, 1);

            images.resize(num);
            for (int i = 0; i < num; i++){
                Mem2<Byte> gry(x, y);
                file.read(gry.ptr, x * y);
                cnvMem(images[i], gry, scale);
            }

            return true;
        }

        SP_CPUFUNC bool loadLabels(Mem1<int> &labels, const char *dir, const char *name){
            File file;
            char path[512];
            sprintf(path, "%s/%s%s", dir, name, "-labels.idx1-ubyte");
            if (file.open(path, "rb", BigEndian) == false) return false;

            int buf, num;
            file.read(&buf, 1);
            file.read(&num, 1);

            labels.resize(num);
            for (int i = 0; i < num; i++){
                Byte label;
                file.read(&label, 1);
                labels[i] = label;
            }
            return true;
        }
    }

    SP_CPUFUNC bool loadMNIST(
        Mem1<Mem<SP_REAL> > &trainImages, Mem1<int> &trainLabels,
        Mem1<Mem<SP_REAL> > &testImages, Mem1<int> &testLabels, const char *dir){

        if (_mnist::loadImages(trainImages, dir, "train", 1.0 / 255.0) == false) return false;
        if (_mnist::loadLabels(trainLabels, dir, "train") == false) return false;

        if (_mnist::loadImages(testImages, dir, "train", 1.0 / 255.0) == false) return false;
        if (_mnist::loadLabels(testLabels, dir, "train") == false) return false;
        return true;
    }

    SP_CPUFUNC bool loadMNIST(
        Mem1<Mem2<Byte> > &trainImages, Mem1<int> &trainLabels,
        Mem1<Mem2<Byte> > &testImages, Mem1<int> &testLabels, const char *dir){

        if (_mnist::loadImages(trainImages, dir, "train") == false) return false;
        if (_mnist::loadLabels(trainLabels, dir, "train") == false) return false;

        if (_mnist::loadImages(testImages, dir, "train") == false) return false;
        if (_mnist::loadLabels(testLabels, dir, "train") == false) return false;
        return true;
    }


    //--------------------------------------------------------------------------------
    // cifar
    //--------------------------------------------------------------------------------

    namespace _cifar{

        template<typename TYPE>
        SP_CPUFUNC bool load10(Mem1<TYPE> &images, Mem1<int> &labels, const char *dir, const char *name, const int num, const double scale = 1.0){
            
            File file;
            char path[512];

            sprintf(path, "%s/%s", dir, name);
            if (file.open(path, "rb") == false) return false;

            for (int i = 0; i < num; i++){
                Byte label;
                file.read(&label, 1);

                Mem3<Byte> data(32, 32, 3);
                file.read(data.ptr, data.size());

                TYPE image;
                cnvMem(image, data, scale);
                
                labels.push(label);
                images.push(image);
            }

            return true;
        }

        template<typename TYPE>
        SP_CPUFUNC bool loadTrain10(Mem1<Mem<TYPE> > &images, Mem1<int> &labels, const char *dir, const double scale = 1.0){
            const int num = 10000;

            images.clear();
            labels.clear();
            images.reserve(num * 5);
            labels.reserve(num * 5);

            if (load10(images, labels, dir, "data_batch_1.bin", num, scale) == false) return false;
            if (load10(images, labels, dir, "data_batch_2.bin", num, scale) == false) return false;
            if (load10(images, labels, dir, "data_batch_3.bin", num, scale) == false) return false;
            if (load10(images, labels, dir, "data_batch_4.bin", num, scale) == false) return false;
            if (load10(images, labels, dir, "data_batch_5.bin", num, scale) == false) return false;

            return true;
        }

        template<typename TYPE>
        SP_CPUFUNC bool loadTest10(Mem1<Mem<TYPE> > &images, Mem1<int> &labels, const char *dir, const double scale = 1.0){
            const int num = 10000;

            images.clear();
            labels.clear();
            images.reserve(num);
            labels.reserve(num);
            if (load10(images, labels, dir, "test_batch.bin", num, scale) == false) return false;

            return true;
        }
    }

    SP_CPUFUNC bool loadCIFAR10(
        Mem1<Mem<SP_REAL> > &trainImg, Mem1<int> &trainLabel,
        Mem1<Mem<SP_REAL> > &testImg, Mem1<int> &testLabel, const char *dir){

        if (_cifar::loadTrain10(trainImg, trainLabel, dir, 1.0 / 255.0) == false) return false;
        if (_cifar::loadTest10(testImg, testLabel, dir, 1.0 / 255.0) == false) return false;
        return true;
    }


}
#endif