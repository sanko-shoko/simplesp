//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_SYS_H__
#define __SP_SYS_H__

#ifndef SP_USE_SYS
#define SP_USE_SYS 1
#endif

#if SP_USE_SYS

//--------------------------------------------------------------------------------
// timer
//--------------------------------------------------------------------------------

#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#else
#include <chrono>
#endif

namespace sp {

    class Timer {
    public:

#if defined(_WIN32) || defined(_WIN64)
        typedef LARGE_INTEGER tpoint;
#else
        typedef std::chrono::system_clock::time_point tpoint;
#endif

        static tpoint now() {
            tpoint n;
#if defined(_WIN32) || defined(_WIN64)
            QueryPerformanceCounter(&n);
#else
            n = std::chrono::system_clock::now();
#endif
            return n;
        }

        static double dif(const tpoint &tp0, const tpoint &tp1) {
            double ms = 0.0;
#if defined(_WIN32) || defined(_WIN64)
            tpoint freq;
            QueryPerformanceFrequency(&freq);
            ms = static_cast<double>((tp1.QuadPart - tp0.QuadPart) * 1000.0 / freq.QuadPart);
#else
            ms = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(tp1 - tp0).count() / 1000.0);
#endif
            return (ms > 0) ? +ms : -ms;
        }

    private:

        tpoint tp[2];

    public:

        Timer() {
            tp[0] = now();
            tp[1] = tp[0];
        }

        Timer(const Timer &timer) {
            *this = timer;
        }

        Timer & operator = (const Timer &timer) {
            tp[0] = timer.tp[0];
            tp[1] = timer.tp[1];
            return *this;
        }

        void start() {
            tp[0] = now();
        }

        void stop() {
            tp[1] = now();
        }

        double getms() {
            return dif(tp[0], tp[1]);
        }
    };
}


//--------------------------------------------------------------------------------
// resource
//--------------------------------------------------------------------------------

#if WIN32
#include <windows.h>
#include <psapi.h>
#else
#include <sys/time.h>
#include <sys/resource.h>
#endif

namespace sp {

    // get memory usage [KB]
    static int getMemoryUsage() {
        int m = -1;
#if defined(_WIN32) || defined(_WIN64)
        HANDLE h = GetCurrentProcess();
        PROCESS_MEMORY_COUNTERS_EX pmc;
        if (GetProcessMemoryInfo(h, (PROCESS_MEMORY_COUNTERS*)&pmc, sizeof(pmc))) {
            m = static_cast<int>(pmc.PrivateUsage / (1000.0));
        }
        CloseHandle(h);
#else
        rusage usage;
        if (getrusage(RUSAGE_SELF, &usage) == 0) {
            m = static_cast<int>(usage.ru_maxrss);
        }
#endif
        return m;
    }
}

//--------------------------------------------------------------------------------
// thread
//--------------------------------------------------------------------------------

#include <thread>
#include <mutex>
#include <functional>

namespace sp {

    SP_CPUFUNC void sleep(const long long ms) {
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }

    class Thread {
    private:
        bool m_used;
        std::mutex m_mtx;

    public:
        Thread() {
            m_used = false;
        }

        bool used() {
            return m_used;
        }

        template<class Class, void (Class::*Func)()>
        bool run(Class *ptr, const bool wait = true) {
            if (wait == false && m_used == true) return false;

            std::thread th([this, ptr, wait] {
                m_mtx.lock();
                m_used = true;
                (ptr->*Func)();
                m_used = false;
                m_mtx.unlock();
            });
            th.detach();
            return true;
        }

        bool run(std::function<void()> func, const bool wait = true) {
            if (wait == false && m_used == true) return false;

            std::thread th([this, func, wait] {
                m_mtx.lock();
                m_used = true;
                func();
                m_used = false;
                m_mtx.unlock();
            });
            th.detach();
            return true;
        }

        bool run(void (*func)(), const bool wait = true) {
            if (wait == false && m_used == true) return false;

            std::thread th([this, func, wait] {
                m_mtx.lock();
                m_used = true;
                func();
                m_used = false;
                m_mtx.unlock();
            });
            th.detach();
            return true;
        }
    };
}

#endif //SP_USE_SYS

#endif

