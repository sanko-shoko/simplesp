//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_TIME_H__
#define __SP_TIME_H__

#include "spcore/spwrap.h"

#ifndef SP_USE_TIMER
#define SP_USE_TIMER 1
#endif

#ifndef SP_USE_DEBUG
#define SP_USE_DEBUG 0
#endif

#ifndef SP_USE_LOGGER
#define SP_USE_LOGGER SP_USE_DEBUG
#endif


//--------------------------------------------------------------------------------
// additional include
//--------------------------------------------------------------------------------

#if SP_USE_TIMER
#if WIN32
#include <Windows.h>
#else
#include <chrono>
#endif
#endif

#if SP_USE_LOGGER
#include <vector>
#include <string>
#endif


namespace sp {

    class Timer {

    public:

#if SP_USE_TIMER
#if WIN32
        typedef LARGE_INTEGER tpoint;
#else
        typedef std::chrono::system_clock::time_point tpoint;
#endif
#else
        typedef SP_REAL tpoint;
#endif


    public:

        static tpoint now() {
            tpoint n;

#if SP_USE_TIMER
#if WIN32
            QueryPerformanceCounter(&n);
#else
            n = std::chrono::system_clock::now();
#endif
#else
            n = 0.0;
#endif
            return n;
        }

        static SP_REAL dif(const tpoint &tp0, const tpoint &tp1) {

            SP_REAL ms = 0.0;
#if SP_USE_TIMER
#if WIN32
            tpoint freq;
            QueryPerformanceFrequency(&freq);

            ms = static_cast<SP_REAL>((tp1.QuadPart - tp0.QuadPart) * 1000.0 / freq.QuadPart);
#else
            ms = static_cast<SP_REAL>(std::chrono::duration_cast<std::chrono::microseconds>(tp1 - tp0).count() / 1000.0);
#endif
#endif
            return (ms > 0) ? +ms : -ms;
        }


    private:

        tpoint tp[2];

    public:

        void start() {
            tp[0] = now();
        }

        void stop() {
            tp[1] = now();
        }

        SP_REAL getms() {
            return dif(tp[0], tp[1]);
        }

        void print(const char *name = "time") {
            SP_PRINTF("%s: %.1lf[ms]\n", name, getms());
        }

    };

}


//--------------------------------------------------------------------------------
// time logger
//--------------------------------------------------------------------------------

#if SP_USE_LOGGER

#define SP_LOGGER_SET(NAME) sp::LoggerUnit _lunit(sp::Logger::root(), NAME);
#define SP_LOGGER_START(NAME) sp::Logger::root()->start(NAME);
#define SP_LOGGER_STOP(NAME) sp::Logger::root()->stop(NAME);
#define SP_LOGGER_PRINT(NAME) sp::Logger::root()->print(NAME);
#else

#define SP_LOGGER_SET(NAME) 
#define SP_LOGGER_START(NAME) 
#define SP_LOGGER_STOP(NAME) 
#define SP_LOGGER_PRINT(NAME)
#endif


namespace sp {
    using namespace std;

    class Logger {

    public:

#if SP_USE_LOGGER

        vector<string> names;

        vector<Timer::tpoint> tpnts;

        vector<SP_REAL> times;
        vector<bool> flags;
#endif

    public:

        void reset() {

#if SP_USE_LOGGER
            names.clear();

            tpnts.clear();

            times.clear();
            flags.clear();
#endif
        }

        void start(const char* name) {

#if SP_USE_LOGGER

            int id = -1;
            for (int i = 0; i < names.size(); i++) {
                if (name == names[i]) {
                    id = i;
                    break;
                }
            }

            const Timer::tpoint tp = Timer::now();

            if (id < 0) {
                names.push_back(name);
                tpnts.push_back(tp);

                times.push_back(0.0);
                flags.push_back(false);
            }
            else {
                names[id] = name;
                tpnts[id] = tp;

                times[id] = 0.0;
                flags[id] = false;
            }
#endif
        }

        void stop(const char *name) {

#if SP_USE_LOGGER
            const Timer::tpoint tp = Timer::now();

            int id = -1;
            for (int i = 0; i < names.size(); i++) {
                if (name == names[i]) {
                    id = i;
                    break;
                }
            }

            if (id >= 0 && flags[id] == false) {
                times[id] = Timer::dif(tpnts[id], tp);
                flags[id] = true;
            }
#endif
        }

        void print(const char *name = NULL) {

#if SP_USE_LOGGER
            for (int i = 0; i < names.size(); i++) {
                if (name == NULL || name == names[i]) {
                    if (flags[i] == false) stop(names[i].c_str());
                    SP_PRINTF("%s : %.3lf [ms]\n", names[i].c_str(), times[i]);
                }
            }
            SP_PRINTF("\n");
#endif
        }

        static Logger *root() {
            static Logger logger;
            return &logger;
        }
    };

    class LoggerUnit {

    private:
        Logger * logger;
        const char *name;

    public:
        LoggerUnit(Logger *logger, const char *name) {
            //SP_PRINTF("%s\n", str);

            this->logger = logger;
            this->name = name;

            logger->start(name);
        }

        ~LoggerUnit() {
            logger->stop(name);
        }
    };
}

#endif

