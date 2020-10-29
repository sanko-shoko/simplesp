//--------------------------------------------------------------------------------
// Copyright (c) 2017-2020, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_FILE_H__
#define __SP_FILE_H__

#include "spcore/spcore.h"
#include "spapp/spdata/spformat.h"

namespace sp {

    //--------------------------------------------------------------------------------
    // file
    //--------------------------------------------------------------------------------

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