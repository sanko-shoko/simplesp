//--------------------------------------------------------------------------------
// Copyright (c) 2017-2020, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_FILE_H__
#define __SP_FILE_H__

#include "spcore/spgen/spbase.h"
#include "spcore/spcpu/spmem.h"

//--------------------------------------------------------------------------------
// format
//--------------------------------------------------------------------------------

namespace sp {

    //--------------------------------------------------------------------------------
    // text
    //--------------------------------------------------------------------------------

    SP_CPUFUNC bool ftextf(FILE *fp, const char *mode, const char *format, const char *option = NULL) {
        if (format == NULL) return false;

        {
            if (*mode == 'w') {
                ::fprintf(fp, "%s", format);
            }
            else {
                const int size = strlen(format);
                char buf[SP_STRMAX];
                ::fread(buf, 1, size, fp);
            }
        }
        ftextf(fp, mode, option);
        return true;
    }

    template <typename TYPE>
    SP_CPUFUNC bool ftextf(FILE *fp, const char *mode, const char *format, const TYPE *val, const int num, const char *option = NULL) {
        if (format == NULL) return false;

        for (int i = 0; i < num; i++) {
            if (*mode == 'w') {
                ::fprintf(fp, format, val[i]);
            }
            else {
                ::fscanf(fp, format, &val[i]);
            }
            if (i < num - 1) {
                ftextf(fp, mode, ",");
            }
        }
        ftextf(fp, mode, option);
        return true;
    }

    //--------------------------------------------------------------------------------
    // binanary
    //--------------------------------------------------------------------------------

    template <typename TYPE>
    SP_CPUFUNC bool fbin(FILE *fp, const char *mode, const TYPE *val, const int num) {

        TYPE *ptr = (TYPE*)val;
        if (*mode == 'w') {
            ::fwrite(ptr, sizeof(TYPE), num, fp);
        }
        else {
            ::fread(ptr, sizeof(TYPE), num, fp);
        }

        return true;
    }


    //--------------------------------------------------------------------------------
    // bool
    //--------------------------------------------------------------------------------

    SP_CPUFUNC bool ftext(FILE *fp, const char *mode, const bool *val, const int num) {
        return ftextf(fp, mode, "%d", val, num, "\n");
    }


    //--------------------------------------------------------------------------------
    // char
    //--------------------------------------------------------------------------------

    SP_CPUFUNC bool ftext(FILE *fp, const char *mode, const char *val, const int num) {
        return ftextf(fp, mode, "%d", val, num, "\n");
    }


    //--------------------------------------------------------------------------------
    // unsigned char
    //--------------------------------------------------------------------------------

    SP_CPUFUNC bool ftext(FILE *fp, const char *mode, const unsigned char *val, const int num) {
        return ftextf(fp, mode, "%d", val, num, "\n");
    }


    //--------------------------------------------------------------------------------
    // short
    //--------------------------------------------------------------------------------

    SP_CPUFUNC bool ftext(FILE *fp, const char *mode, const short *val, const int num) {
        return ftextf(fp, mode, "%d", val, num, "\n");
    }


    //--------------------------------------------------------------------------------
    // unsigned short
    //--------------------------------------------------------------------------------

    SP_CPUFUNC bool ftext(FILE *fp, const char *mode, const unsigned short *val, const int num) {
        return ftextf(fp, mode, "%d", val, num, "\n");
    }


    //--------------------------------------------------------------------------------
    // int
    //--------------------------------------------------------------------------------

    SP_CPUFUNC bool ftext(FILE *fp, const char *mode, const int *val, const int num) {
        return ftextf(fp, mode, "%d", val, num, "\n");
    }


    //--------------------------------------------------------------------------------
    // unsigned int
    //--------------------------------------------------------------------------------

    SP_CPUFUNC bool ftext(FILE *fp, const char *mode, const unsigned int *val, const int num) {
        return ftextf(fp, mode, "%d", val, num, "\n");
    }


    //--------------------------------------------------------------------------------
    // float
    //--------------------------------------------------------------------------------

    SP_CPUFUNC bool ftext(FILE *fp, const char *mode, const float *val, const int num) {
        return ftextf(fp, mode, "%f", val, num, "\n");
    }


    //--------------------------------------------------------------------------------
    // double
    //--------------------------------------------------------------------------------

    SP_CPUFUNC bool ftext(FILE *fp, const char *mode, const double *val, const int num) {
        const char *format = (*mode == 'w') ? "%e" : "%lf";
        return ftextf(fp, mode, format, val, num, "\n");
    }


    //--------------------------------------------------------------------------------
    // with name
    //--------------------------------------------------------------------------------

    template <typename TYPE>
    SP_CPUFUNC bool ftextn(FILE *fp, const char *mode, const TYPE *val, const int num, const char *name) {
        bool ret = true;
        ret &= ftextf(fp, mode, name, ",");
        ret &= ftext(fp, mode, val, num);
        return ret;
    }


    //--------------------------------------------------------------------------------
    // camera parameter
    //--------------------------------------------------------------------------------

    SP_CPUFUNC bool ftext(FILE *fp, const char *mode, const CamParam *val, const int num) {

        for (int i = 0; i < num; i++) {
            ftextf(fp, mode, "CamParam\n");

            ftextn(fp, mode, val[i].dsize, 2, "dsize");

            ftextn(fp, mode, &val[i].fx, 1, "fx");
            ftextn(fp, mode, &val[i].fy, 1, "fy");
            ftextn(fp, mode, &val[i].cx, 1, "cx");
            ftextn(fp, mode, &val[i].cy, 1, "cy");

            ftextn(fp, mode, &val[i].k1, 1, "k1");
            ftextn(fp, mode, &val[i].k2, 1, "k2");
            ftextn(fp, mode, &val[i].k3, 1, "k3");
            ftextn(fp, mode, &val[i].p1, 1, "p1");
            ftextn(fp, mode, &val[i].p2, 1, "p2");
        }
        return true;
    }


    //--------------------------------------------------------------------------------
    // pose
    //--------------------------------------------------------------------------------

    SP_CPUFUNC bool ftext(FILE *fp, const char *mode, const Pose *val, const int num) {

        for (int i = 0; i < num; i++) {
            ftextf(fp, mode, "Pose\n");

            ftextn(fp, mode, &val[i].rot.qx, 1, "qx");
            ftextn(fp, mode, &val[i].rot.qy, 1, "qy");
            ftextn(fp, mode, &val[i].rot.qz, 1, "qz");
            ftextn(fp, mode, &val[i].rot.qw, 1, "qw");

            ftextn(fp, mode, &val[i].pos.x, 1, "tx");
            ftextn(fp, mode, &val[i].pos.y, 1, "ty");
            ftextn(fp, mode, &val[i].pos.z, 1, "tz");
        }
        return true;
    }


    //--------------------------------------------------------------------------------
    // vector
    //--------------------------------------------------------------------------------

    SP_CPUFUNC bool ftext(FILE *fp, const char *mode, const Vec2 *val, const int num) {

        for (int i = 0; i < num; i++) {
            ftextf(fp, mode, "Vec2");
            ftextf(fp, mode, ",%lf", &val[i].x, 1);
            ftextf(fp, mode, ",%lf", &val[i].y, 1);
            ftextf(fp, mode, "\n");
        }
        return true;
    }

    SP_CPUFUNC bool ftext(FILE *fp, const char *mode, const VecPD2 *val, const int num) {

        for (int i = 0; i < num; i++) {
            ftextf(fp, mode, "VecPD2");
            ftextf(fp, mode, ",%lf", &val[i].pos.x, 1);
            ftextf(fp, mode, ",%lf", &val[i].pos.y, 1);
            ftextf(fp, mode, ",%lf", &val[i].drc.x, 1);
            ftextf(fp, mode, ",%lf", &val[i].drc.y, 1);
            ftextf(fp, mode, "\n");
        }
        return true;
    }

    SP_CPUFUNC bool ftext(FILE *fp, const char *mode, const Vec3 *val, const int num) {

        for (int i = 0; i < num; i++) {
            ftextf(fp, mode, "Vec3");
            ftextf(fp, mode, ",%lf", &val[i].x, 1);
            ftextf(fp, mode, ",%lf", &val[i].y, 1);
            ftextf(fp, mode, ",%lf", &val[i].z, 1);
            ftextf(fp, mode, "\n");
        }
        return true;
    }

    SP_CPUFUNC bool ftext(FILE *fp, const char *mode, const VecPD3 *val, const int num) {

        for (int i = 0; i < num; i++) {
            ftextf(fp, mode, "VecPD3");
            ftextf(fp, mode, ",%lf", &val[i].pos.x, 1);
            ftextf(fp, mode, ",%lf", &val[i].pos.y, 1);
            ftextf(fp, mode, ",%lf", &val[i].pos.z, 1);
            ftextf(fp, mode, ",%lf", &val[i].drc.x, 1);
            ftextf(fp, mode, ",%lf", &val[i].drc.y, 1);
            ftextf(fp, mode, ",%lf", &val[i].drc.z, 1);
            ftextf(fp, mode, "\n");
        }
        return true;
    }

    //--------------------------------------------------------------------------------
    // mem
    //--------------------------------------------------------------------------------

    template <typename TYPE>
    SP_CPUFUNC bool ftext(FILE *fp, const char *mode, const Mem<TYPE> *val, const int num) {

        for (int i = 0; i < num; i++) {
            ftextf(fp, mode, "Mem\n");
            ftextn(fp, mode, &val[i].dim, 1, "dim");
            ftextn(fp, mode, val[i].dsize, val[i].dim, "dsize");

            if (*mode == 'r') {
                Mem<TYPE> *m = const_cast<Mem<TYPE>*>(&val[i]);
                m->resize(val[i].dim, val[i].dsize);
            }

            for (int j = 0; j < val[i].size(); j += val[i].dsize[0]) {
                ftext(fp, mode, &val[i].ptr[j], val[i].dsize[0]);
            }
        }
        return true;
    }
}

//--------------------------------------------------------------------------------
// file
//--------------------------------------------------------------------------------

namespace sp {

    class File {
    private:
        FILE *m_fp;

        // file mode
        const char *m_mode;

        // file byte order
        ByteOrder m_endian;

        // file size
        int m_size;

    private:

        // reset member
        void reset() {
            memset(this, 0, sizeof(File));
        }

    public:

        File() {
            reset();
        }

        File(const char *path, const char *mode, const ByteOrder endian = ByteOrder::LittleEndian) {
            reset();
            open(path, mode, endian);
        }

        ~File() {
            close();
        }

        bool open(const char *path, const char *mode, const ByteOrder endian = ByteOrder::LittleEndian) {
            close();

            m_fp = ::fopen(path, mode);

            if (m_fp != NULL) {
                SP_PRINTF("file open %s [%s]\n", mode, path);

                m_mode = mode;
                m_endian = endian;

                seek(SEEK_END);
                m_size = static_cast<int>(::ftell(m_fp));
                seek(SEEK_SET);
                return true;
            }
            else {
                SP_PRINTF("file error %s [%s]\n", mode, path);
                return false;
            }

        }
        
        void close() {
            if (m_fp == NULL) return;
            
            ::fclose(m_fp);
            reset();
        }

        FILE* fp() {
            return m_fp;
        }

        void seek(const int origin) {
            if (m_fp == NULL) return;

            ::fseek(m_fp, 0, origin);
        }
        int tell() const {
            return static_cast<int>(::ftell(m_fp));
        }
        int residual() const {
            return m_size - tell();
        }

    public:

        //--------------------------------------------------------------------------------
        // base method
        //--------------------------------------------------------------------------------

        template <typename TYPE>
        bool write(const TYPE *src, const int count) {
            if (m_fp == NULL) return false;

            Mem1<TYPE> tmp(count, src);
            if (m_endian != getByteOrder()) {
                revByteOrder(tmp.ptr, count);
            }

            const bool ret = (::fwrite(tmp.ptr, sizeof(TYPE), count, m_fp) == count) ? true : false;

            return ret;
        }

        template <typename TYPE>
        bool read(TYPE *dst, const int count) {
            if (m_fp == NULL) return false;

            const bool ret = (::fread(dst, sizeof(TYPE), count, m_fp) == count) ? true : false;

            if (ret == true && m_endian != getByteOrder()) {
                revByteOrder(dst, count);
            }
            return ret;
        }

        bool gets(char *str) {
            if (m_fp == NULL) return false;

            const bool ret = (::fgets(str, SP_STRMAX - 1, m_fp) != NULL) ? true : false;
            return ret;
        }

        bool printf(const char* format, ...) {
            if (m_fp == NULL) return false;

            va_list arg;
            va_start(arg, format);
            const bool ret = (::vfprintf(m_fp, format, arg) >= 0) ? true : false;
            va_end(arg);

            return ret;
        }

        bool scanf(const char* format, ...) {
            if (m_fp == NULL) return false;

            va_list arg;
            va_start(arg, format);
            const bool ret = (::vfscanf(m_fp, format, arg) >= 0) ? true : false;
            va_end(arg);

            return ret;
        }

    public:

        //--------------------------------------------------------------------------------
        // binnary
        //--------------------------------------------------------------------------------

        template <typename TYPE>
        bool bin(const TYPE *val, const int num = 1) {
            const bool ret = fbin(m_fp, m_mode, val, num);

            if (ret == true && m_endian != getByteOrder()) {
                revByteOrder(val, num);
            }
            return ret;
        }


        //--------------------------------------------------------------------------------
        // text
        //--------------------------------------------------------------------------------

        void textf(const char* format) {
            ftextf(m_fp, m_mode, format);
        }

        template <typename TYPE>
        void textf(const char* format, const TYPE *val, const int num = 1, const char *name = NULL) {
            ftextf(m_fp, m_mode, name, ",");
            ftextf(m_fp, m_mode, format, val, num);
        }

        template <typename TYPE>
        void text(const TYPE *val, const int num = 1, const char *name = NULL) {
            ftextf(m_fp, m_mode, name, ",");
            ftext(m_fp, m_mode, val, num);
        }


        //--------------------------------------------------------------------------------
        // textex
        //--------------------------------------------------------------------------------

#define SP_TEXTEX() friend class File; virtual void textex(File &file) const

        template <typename TYPE>
        void textex(const TYPE *val, const int num = 1, const char *name = NULL) {
            ftextf(m_fp, m_mode, name);
            for (int i = 0; i < num; i++) {
                val[i].textex(*this);
            }
        }

    };

    //--------------------------------------------------------------------------------
    // save / load text
    //--------------------------------------------------------------------------------

    template <typename TYPE>
    SP_CPUFUNC bool saveText(const char *path, const TYPE &data) {
        File file;
        if (file.open(path, "w") == false) return false;

        file.text(&data);
        return true;
    }

    template <typename TYPE>
    SP_CPUFUNC bool loadText(const char *path, TYPE &data) {
        File file;
        if (file.open(path, "r") == false) return false;

        file.text(&data);
        return true;
    }


    //--------------------------------------------------------------------------------
    // save / load mem
    //--------------------------------------------------------------------------------

    template <typename TYPE>
    SP_CPUFUNC bool saveMem(const char *path, const Mem<TYPE> &mem) {
        File file;
        if (file.open(path, "wb") == false) return false;

        file.textf("Mem\n");
        file.text(&mem.dim, 1, "dim");
        file.text(mem.dsize, mem.dim, "dsize");

        file.write(mem.ptr, mem.size());
        return true;
    }

    template <typename TYPE>
    SP_CPUFUNC bool loadMem(const char *path, Mem<TYPE> &mem) {
        File file;
        if (file.open(path, "rb") == false) return false;

        file.textf("Mem\n");
        file.text(&mem.dim, 1, "dim");
        file.text(mem.dsize, mem.dim, "dsize");

        mem.resize(mem.dim, mem.dsize);
        file.read(mem.ptr, mem.size());
        return true;
    }
}

#endif