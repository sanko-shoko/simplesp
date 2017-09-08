//--------------------------------------------------------------------------------
// Copyright (c) 2017, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_FILE_H__
#define __SP_FILE_H__

#include "spcore/spcore.h"
#include "spapp/spdata/spstr.h"
#include "spapp/spdata/spformat.h"

namespace sp {

	//--------------------------------------------------------------------------------
	// file util
	//--------------------------------------------------------------------------------

	using namespace std;

	SP_CPUFUNC bool findFile(const char *path) {
		FILE *fp = ::fopen(path, "r");
		if (fp != NULL) {
			::fclose(fp);
			return true;
		}
		else {
			return false;
		}
	}

	SP_CPUFUNC bool checkFileExt(const char *path, const char *ext) {
		Mem1<string> exts = strSplit(ext);

		bool ret = false;

		for (int i = 0; i < exts.size(); i++) {
			if (strstr(path, exts[i].c_str()) != NULL) {
				ret = true;
				break;
			}
		}
		return ret;
	}

	//--------------------------------------------------------------------------------
	// file
	//--------------------------------------------------------------------------------

	class File {
	private:
		FILE *m_fp;

		// file mode
		const char *m_mode;

		// file status
		bool m_status;

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

		File(const char *path, const char *mode, const ByteOrder endian = getByteOrder()) {
			reset();
			open(path, mode);
		}

		~File() {
			close();
		}

		bool open(const char *path, const char *mode, const ByteOrder endian = getByteOrder()) {
			close();

			m_fp = ::fopen(path, mode);

			if (m_fp != NULL) {
				SP_PRINTF("file open %s [%s]\n", mode, path);

				m_mode = mode;
				m_status = true;
				m_endian = endian;

				::fseek(m_fp, 0, SEEK_END);
				m_size = static_cast<int>(::ftell(m_fp));
				::fseek(m_fp, 0, SEEK_SET);
			}
			else {
				SP_PRINTF("file error %s [%s]\n", mode, path);
			}

			return m_status;
		}
		
		void close() {
			if (m_fp != NULL) {
				::fclose(m_fp);
			}
			reset();
		}

		bool status() const {
			return m_status;
		}

		int residual() const {
			return m_size - static_cast<int>(::ftell(m_fp));
		}

	public:

		//--------------------------------------------------------------------------------
		// base method
		//--------------------------------------------------------------------------------

		template <typename TYPE>
		bool write(const TYPE *src, const int count) {
			if (m_status == false) return false;

			m_status = (::fwrite(src, sizeof(TYPE), count, m_fp) == count) ? true : false;
		
			if (m_status == true && m_endian != getByteOrder()) {
				revByteOrder(src, count);
			}
			return m_status;
		}

		template <typename TYPE>
		bool read(TYPE *dst, const int count) {
			if (m_status == false) return false;

			m_status = (::fread(dst, sizeof(TYPE), count, m_fp) == count) ? true : false;

			if (m_status == true && m_endian != getByteOrder()) {
				revByteOrder(dst, count);
			}
			return m_status;
		}

		bool gets(char *str) {
			if (m_status == false) return false;

			m_status = (::fgets(str, SP_STRMAX - 1, m_fp) != NULL) ? true : false;
			return m_status;
		}

		bool printf(const char* format, ...) {
			if (m_status == false) return false;

			va_list arg;
			va_start(arg, format);
			m_status = (::vfprintf(m_fp, format, arg) >= 0) ? true : false;
			va_end(arg);

			return m_status;
		}

		bool scanf(const char* format, ...) {
			if (m_status == false) return false;

			va_list arg;
			va_start(arg, format);
			m_status = (::vfscanf(m_fp, format, arg) >= 0) ? true : false;
			va_end(arg);

			return m_status;
		}

	public:

		//--------------------------------------------------------------------------------
		// binnary
		//--------------------------------------------------------------------------------

		template <typename TYPE>
		bool bin(const TYPE *val, const int num = 1) {
			m_status = fbin(m_fp, m_mode, val, num);

			if (m_status == true && m_endian != getByteOrder()) {
				revByteOrder(val, num);
			}
			return m_status;
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
	SP_CPUFUNC bool saveText(const TYPE &data, const char *path) {
		File file;
		if (file.open(path, "w") == false) return false;

		file.text(&data);
		return true;
	}

	template <typename TYPE>
	SP_CPUFUNC bool loadText(TYPE &data, const char *path) {
		File file;
		if (file.open(path, "r") == false) return false;

		file.text(&data);
		return true;
	}


	//--------------------------------------------------------------------------------
	// save / load mem
	//--------------------------------------------------------------------------------

	template <typename TYPE>
	SP_CPUFUNC bool saveMem(const Mem<TYPE> &mem, const char *path) {
		File file;
		if (file.open(path, "wb") == false) return false;

		file.textf("Mem,\n");
		file.text(&mem.dim, 1, "dim");
		file.text(mem.dsize, mem.dim, "dsize");

		file.write(mem.ptr, mem.size());
		return true;
	}

	template <typename TYPE>
	SP_CPUFUNC bool loadMem(Mem<TYPE> &mem, const char *path) {
		File file;
		if (file.open(path, "rb") == false) return false;

		file.textf("Mem,\n");
		file.text(&mem.dim, 1, "dim");
		file.text(mem.dsize, mem.dim, "dsize");

		mem.resize(mem.dim, mem.dsize);
		file.read(mem.ptr, mem.size());
		return true;
	}





}
#endif