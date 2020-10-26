//--------------------------------------------------------------------------------
// Copyright (c) 2017-2020, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_DEBUG_H__
#define __SP_DEBUG_H__

#include "spcore/spsystem.h"
#include "spcore/sptime.h"
#include "spcore/spprint.h"

#include <string>
#include <vector>


//--------------------------------------------------------------------------------
// time logger
//--------------------------------------------------------------------------------

#ifndef SP_USE_LOGGER
#define SP_USE_LOGGER SP_USE_DEBUG
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

    class Logger {
    public:

#if SP_USE_LOGGER

        std::vector<std::string> names;
        std::vector<Timer> timers;
        std::vector<bool> flags;
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


//--------------------------------------------------------------------------------
// data holder
//--------------------------------------------------------------------------------

#ifndef SP_USE_HOLDER
#define SP_USE_HOLDER SP_USE_DEBUG
#endif

#if SP_USE_HOLDER

#define SP_HOLDER_SET(NAME, DATA) Holder::instance()->set(NAME, DATA);
#define SP_HOLDER_GET(NAME, TYPE) Holder::instance()->get<TYPE>(NAME);
#else

#define SP_HOLDER_SET(NAME, DATA)
#define SP_HOLDER_GET(NAME, TYPE) NULL;
#endif

namespace sp {
    using namespace std;

    class Holder {

    private:

#if SP_USE_HOLDER
        vector<string> names;
        vector<void *> ptrs;
#endif

    public:

        ~Holder() {
            reset();
        }

        void reset() {

#if SP_USE_HOLDER
            names.clear();
            for (int i = 0; i < ptrs.size(); i++) {
                delete ptrs[i];
            }
            ptrs.clear();
#endif

        }

        template <typename TYPE>
        void set(const char *name, const TYPE &data) {
            TYPE *ptr = NULL;

#if SP_USE_HOLDER
            for (int i = 0; i < names.size(); i++) {
                if (name == names[i]) {
                    ptr = (TYPE*)ptrs[i];
                    break;
                }
            }
            if (ptr != NULL) {
                *ptr = data;
            }
            else {
                ptr = new TYPE();
                *ptr = data;

                names.push_back(name);
                ptrs.push_back(ptr);
            }
#endif
        }

        template <typename TYPE>
        const TYPE* get(const char *name) {
            TYPE *ptr = NULL;

#if SP_USE_HOLDER
            for (int i = 0; i < names.size(); i++) {
                if (name == names[i]) {
                    ptr = (TYPE*)ptrs[i];
                    break;
                }
            }
#endif
            return ptr;
        }

        static Holder *instance() {
            static Holder holder;
            return &holder;
        }
    };
}


#endif

