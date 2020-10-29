//--------------------------------------------------------------------------------
// Copyright (c) 2017-2020, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_SYSTEM_H__
#define __SP_SYSTEM_H__

#include "spcore/spcpu/spstr.h"


//--------------------------------------------------------------------------------
// resource
//--------------------------------------------------------------------------------

#if defined(_WIN32)
#include <windows.h>
#include <psapi.h>

#elif defined(__APPLE__) || defined(__linux__) || defined(__unix__)
#include <sys/time.h>
#include <sys/resource.h>

#else

#endif

namespace sp {

    // get memory usage [KB]
    static int getMemoryUsage() {
        int m = -1;
#if defined(_WIN32)
        HANDLE h = GetCurrentProcess();
        PROCESS_MEMORY_COUNTERS_EX pmc;
        if (GetProcessMemoryInfo(h, (PROCESS_MEMORY_COUNTERS*)&pmc, sizeof(pmc))) {
            m = static_cast<int>(pmc.PrivateUsage / (1000.0));
        }
        CloseHandle(h);

#elif defined(__APPLE__) || defined(__linux__) || defined(__unix__)

        rusage usage;
        if (getrusage(RUSAGE_SELF, &usage) == 0) {
            m = static_cast<int>(usage.ru_maxrss);
        }

#else

#endif
        return m;
    }
}


//--------------------------------------------------------------------------------
// path util
//--------------------------------------------------------------------------------

#if defined(_WIN32)
#include <windows.h>
#include <direct.h>

#elif defined(__APPLE__)
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <unistd.h>
#include <libgen.h>
#include <dlfcn.h>

#elif defined(__linux__)
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <unistd.h>
#include <libgen.h>

#elif defined(__unix__)

#else

#endif

namespace sp {

    SP_CPUFUNC int makeDir(const char *dir) {
        int ret = 0;
#if defined(_WIN32)
        ret = _mkdir(dir);
#elif defined(__APPLE__) || defined(__linux__)
        ret = mkdir(dir, 0755);
#else

#endif
        return ret;
    }

    SP_CPUFUNC char* getCrntDir() {
        static char dir[512] = { 0 };
#if defined(_WIN32)
        GetCurrentDirectory(sizeof(dir) - 1, dir);
#elif defined(__APPLE__) || defined(__linux__)
        getcwd(dir, sizeof(dir) - 1);
#else

#endif
        return dir;
    }

    SP_CPUFUNC char* getModulePath() {
        static char path[512] = { 0 };
#if defined(_WIN32)
        GetModuleFileName(NULL, path, MAX_PATH);
#elif defined(__APPLE__)
        Dl_info module_info;
        if (dladdr(reinterpret_cast<void*>(getModulePath), &module_info) != 0) {
            strcpy(path, module_info.dli_fname);
        }
#elif defined(__linux__)
        //readlink("/proc/self/exe", path, sizeof(path) - 1);
#elif defined(__unix__)
        // non
#endif
        return path;
    }

    SP_CPUFUNC char* getModuleDir() {
        static char dir[512] = { 0 };
        const char *path = getModulePath();

        char tmp[512] = { 0 };
        strget(tmp, path, -1, "\\/");

        const int size = strlen(path) - strlen(tmp) - 1;

        memcpy(dir, path, size);

        return dir;
    }

    SP_CPUFUNC char* getModuleName() {
        static char name[512] = { 0 };
        const char *path = getModulePath();
        strget(name, path, -1, "\\/");
        return name;
    }

    SP_CPUFUNC bool extcmp(const char *path, const char *ext) {
        if (ext == NULL) return false;

        char ext0[SP_STRMAX];
        if (strget(ext0, path, -1, ".") == NULL) return false;

        bool ret = false;

        for (int i = 0; ; i++) {
            char ext1[SP_STRMAX];
            if (strget(ext1, ext, i) == NULL) break;

            if (strcmp(ext0, ext1) == 0) {
                ret = true;
                break;
            }
        }
        return ret;
    }

    SP_CPUFUNC char* extset(char *path, const char *ext) {
        if (ext == NULL) return NULL;
        if (extcmp(path, ext) == false) {
            char tmp[SP_STRMAX];
            sprintf(tmp, "%s.%s", path, ext);
            strcpy(path, tmp);
        }
        return path;
    }

    SP_CPUFUNC char* searchPath(char *dst, const char *dir, const int x, const char *ext = NULL) {
        char *ret = NULL;

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
                        ret = dst;
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
                            ret = dst;
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


}

#endif

