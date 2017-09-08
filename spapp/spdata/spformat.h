//--------------------------------------------------------------------------------
// Copyright (c) 2017, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_FORMAT_H__
#define __SP_FORMAT_H__

#include "spcore/spcore.h"
#include "spapp/spdata/spfile.h"

namespace sp{

	//--------------------------------------------------------------------------------
	// text
	//--------------------------------------------------------------------------------

	SP_CPUFUNC bool ftextf(FILE *fp, const char *mode, const char *format, const char *option = NULL) {
		if (format == NULL) return false;

		{
			if (*mode == 'w') {
				::fprintf(fp, format);
			}
			else {
				::fscanf(fp, format);
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
			ftextf(fp, mode, ",");
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
			ftextf(fp, mode, "CamParam,\n");
			
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
			ftextf(fp, mode, "Pose,\n");

			ftextn(fp, mode, &val[i].rot.qx, 1, "qx");
			ftextn(fp, mode, &val[i].rot.qy, 1, "qy");
			ftextn(fp, mode, &val[i].rot.qz, 1, "qz");
			ftextn(fp, mode, &val[i].rot.qw, 1, "qw");

			ftextn(fp, mode, &val[i].trn.x, 1, "tx");
			ftextn(fp, mode, &val[i].trn.y, 1, "ty");
			ftextn(fp, mode, &val[i].trn.z, 1, "tz");
		}
		return true;
	}


	//--------------------------------------------------------------------------------
	// vector
	//--------------------------------------------------------------------------------

	SP_CPUFUNC bool ftext(FILE *fp, const char *mode, const Vec2 *val, const int num) {
		
		for (int i = 0; i < num; i++) {
			ftextf(fp, mode, "Vec2,");

			ftext(fp, mode, &val[i].x, 1);
			ftext(fp, mode, &val[i].y, 1);
		}
		return true;
	}

	SP_CPUFUNC bool ftext(FILE *fp, const char *mode, const VecVN2 *val, const int num) {

		for (int i = 0; i < num; i++) {
			ftextf(fp, mode, "VecVN2,");

			ftext(fp, mode, &val[i].vtx.x, 1);
			ftext(fp, mode, &val[i].vtx.y, 1);
			ftext(fp, mode, &val[i].nrm.x, 1);
			ftext(fp, mode, &val[i].nrm.y, 1);
		}
		return true;
	}

	SP_CPUFUNC bool ftext(FILE *fp, const char *mode, const Vec3 *val, const int num) {

		for (int i = 0; i < num; i++) {
			ftextf(fp, mode, "Vec3,");

			ftext(fp, mode, &val[i].x, 1);
			ftext(fp, mode, &val[i].y, 1);
			ftext(fp, mode, &val[i].z, 1);
		}
		return true;
	}

	SP_CPUFUNC bool ftext(FILE *fp, const char *mode, const VecVN3 *val, const int num) {

		for (int i = 0; i < num; i++) {
			ftextf(fp, mode, "VecVN3,");

			ftext(fp, mode, &val[i].vtx.x, 1);
			ftext(fp, mode, &val[i].vtx.y, 1);
			ftext(fp, mode, &val[i].vtx.z, 1);
			ftext(fp, mode, &val[i].nrm.x, 1);
			ftext(fp, mode, &val[i].nrm.y, 1);
			ftext(fp, mode, &val[i].nrm.z, 1);
		}
		return true;
	}

	//--------------------------------------------------------------------------------
	// mem
	//--------------------------------------------------------------------------------
	
	template <typename TYPE>
	SP_CPUFUNC bool ftext(FILE *fp, const char *mode, const Mem<TYPE> *val, const int num) {

		for (int i = 0; i < num; i++) {
			ftextf(fp, mode, "Mem,\n");
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
#endif