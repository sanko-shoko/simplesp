//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_THREAD_H__
#define __SP_THREAD_H__


//--------------------------------------------------------------------------------
// thread
//--------------------------------------------------------------------------------

#include <thread>
#include <mutex>
#include <functional>

namespace sp {

    static void sleep(const long long ms) {
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

#endif

