//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_TIME_H__
#define __SP_TIME_H__

#include "spcore/spsys.h"
#include "spcore/spwrap.h"


#ifndef SP_USE_DEBUG
#define SP_USE_DEBUG 0
#endif

#ifndef SP_USE_LOGGER
#define SP_USE_LOGGER SP_USE_DEBUG
#endif

//--------------------------------------------------------------------------------
// time logger
//--------------------------------------------------------------------------------

#if SP_USE_LOGGER
#include <vector>
#include <string>
#endif


#if SP_USE_LOGGER

#define SP_LOGGER_SET(NAME) sp::LoggerUnit _lunit(sp::Logger::instance(), NAME);
#define SP_LOGGER_START(NAME) sp::Logger::instance()->start(NAME);
#define SP_LOGGER_STOP(NAME) sp::Logger::instance()->stop(NAME);
#define SP_LOGGER_PRINT(NAME) sp::Logger::instance()->print(NAME);
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
        vector<Timer> timers;
        vector<bool> flags;
#endif

    public:

        void reset() {

#if SP_USE_LOGGER
            names.clear();
            timers.clear();
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

            const Timer timer = Timer();

            if (id < 0) {
                names.push_back(name);
                timers.push_back(Timer());

                flags.push_back(false);
            }
            else {
                names[id] = name;
                timers[id] = Timer();

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
                timers[id].stop();
                flags[id] = true;
            }
#endif
        }

        void print(const char *name = NULL) {

#if SP_USE_LOGGER
            for (int i = 0; i < names.size(); i++) {
                if (name == NULL || name == names[i]) {
                    if (flags[i] == false) stop(names[i].c_str());
                    SP_PRINTF("%s : %.3lf [ms]\n", names[i].c_str(), timers[i].getms());
                }
            }
            SP_PRINTF("\n");
#endif
        }

        static Logger *instance() {
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

