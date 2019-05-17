//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_PATH_H__
#define __SP_PATH_H__

#include "spcore/spcore.h"

#if defined(_WIN32)
#include <windows.h>
#include <direct.h>

#elif defined(__unix__)
#include <sys/stat.h>
#include <dirent.h>
#include <dlfcn.h>
#include <unistd.h>
#include <libgen.h>

#else
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>
#include <libgen.h>
#endif


//--------------------------------------------------------------------------------
// path util
//--------------------------------------------------------------------------------

namespace sp {

    SP_CPUFUNC int makeDir(const char *dir) {
        int ret = 0;
#if defined(_WIN32)
        ret = _mkdir(dir);
#else
        ret = mkdir(dir, 0755);
#endif
        return ret;
    }

    SP_CPUFUNC char* getCrntDir() {
        static char dir[512] = { 0 };
#if defined(_WIN32)
        GetCurrentDirectory(sizeof(dir) - 1, dir);
#else
        getcwd(dir, sizeof(dir) - 1);
#endif
        return dir;
    }

    SP_CPUFUNC char* getModulePath() {
        static char path[512] = { 0 };
#if defined(_WIN32)
        GetModuleFileName(NULL, path, MAX_PATH);
#elif defined(__unix__)
        // non
#elif defined(__APPLE__)
        Dl_info module_info;
        if (dladdr(reinterpret_cast<void*>(getModulePath), &module_info) != 0) {
            strcpy(path, module_info.dli_fname);
        }
#else
        //readlink("/proc/self/exe", path, sizeof(path) - 1);
#endif
        return path;
    }

    SP_CPUFUNC char* getModuleDir() {
        static char dir[512] = { 0 };
        const char *path = getModulePath();
        
        char tmp[512] = { 0 };
        split(tmp, path, -1, "\\/");

        const int size = strlen(path) - strlen(tmp) - 1;

        memcpy(dir, path, size);

        return dir;
    }

    SP_CPUFUNC char* getModuleName() {
        static char name[512] = { 0 };
        const char *path = getModulePath();
        split(name, path, -1, "\\/");
        return name;
    }

    SP_CPUFUNC bool extcmp(const char *path, const char *ext) {
        if (ext == NULL) return false;

        bool ret = false;
        const std::string a = split(path, -1, ".");
        for (int i = 0; ; i++) {
            const std::string b = split(ext, i);
            if (b.size() == 0) break;

            if (a == b) {
                ret = true;
                break;
            }
        }
        return ret;
    }

    SP_CPUFUNC bool searchPath(char *dst, const char *dir, const int x, const char *ext = NULL) {
        bool ret = false;

#if defined(_WIN32)
        WIN32_FIND_DATA fd;

        char format[SP_STRMAX];
        sprintf(format, "%s\\*.*", dir);

        const HANDLE handle = FindFirstFile(format, &fd);
        SP_ASSERT(handle != INVALID_HANDLE_VALUE);

        int c = 0;
        do {
            const char *name = fd.cFileName;
            if (fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
                // directory
            }
            if (fd.dwFileAttributes & FILE_ATTRIBUTE_ARCHIVE) {
                // file
                if (ext == NULL || extcmp(name, ext) == true) {
                    if (x == c++) {
                        strcpy(dst, name);
                        ret = true;
                    }
                }
            }

        } while (FindNextFile(handle, &fd) == TRUE);

        FindClose(handle);

#elif defined(__APPLE__)
        string search_path;

        struct stat stat_buf;

        struct dirent **namelist = NULL;
        const int dirElements = scandir(dir, &namelist, NULL, NULL);

        int c = 0;
        for (int i = 0; i < dirElements; i += 1) {
            const char *name = namelist[i]->d_name;
            // skip . and ..
            if ((strcmp(name, ".\0") != 0) && (strcmp(name, "..\0") != 0)) {

                string path = dir + string(name);

                if (stat(path.c_str(), &stat_buf) == 0) {

                    if ((stat_buf.st_mode & S_IFMT) == S_IFDIR) {
                        // directory
                    }
                    if ((stat_buf.st_mode & S_IFMT) == S_IFREG) {
                        // file
                        if (x == c++ && (ext == NULL || extcmp(name, ext) == true)) {
                            strcpy(dst, name);
                            ret = true;
                        }
                    }
                }
                else {
                    // error
                }
            }
        }
        if (namelist != NULL) {
            free(namelist);
        }
#else

#endif
        return ret;
    }

    SP_CPUFUNC std::string searchPath(const char *dir, const int x, const char *ext = NULL) {
        std::string ret;

        char name[SP_STRMAX];
        if (searchPath(name, dir, x, ext) == true) {
            ret = name;
        }

        return ret;
    }




}

#endif

