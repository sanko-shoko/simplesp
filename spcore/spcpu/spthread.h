//--------------------------------------------------------------------------------
// Copyright (c) 2017-2020, sanko-shoko. All rights reserved.
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

    class Thread {
    private:
        bool m_used;
        bool m_init;
        std::mutex m_mtx;

    public:
        Thread() {
            init();
            m_used = false;
        }

        void init() {
            m_init = true;
        }

        void freeze() {
            lock();
            m_init = false;
            unlock();
        }

        bool used() {
            return (m_init == false) | (m_used == true);
        }

        void lock() {
            m_mtx.lock();
        }

        void unlock() {
            m_mtx.unlock();
        }

        template<class Class, void (Class::*Func)()>
        bool run(Class *ptr, const bool wait = true) {
            if (m_init == false) return false;
            if (wait == false && m_used == true) return false;

            std::thread th([this, ptr, wait] {
                lock();
                m_used = true;
                (ptr->*Func)();
                m_used = false;
                unlock();
            });
            th.detach();
            return true;
        }

        bool run(std::function<void()> func, const bool wait = true) {
            if (m_init == false) return false;
            if (wait == false && m_used == true) return false;

            std::thread th([this, func, wait] {
                lock();
                m_used = true;
                func();
                m_used = false;
                unlock();
            });
            th.detach();
            return true;
        }

        bool run(void (*func)(), const bool wait = true) {
            if (m_init == false) return false;
            if (wait == false && m_used == true) return false;

            std::thread th([this, func, wait] {
                lock();
                m_used = true;
                func();
                m_used = false;
                unlock();
            });
            th.detach();
            return true;
        }
    };
}

#endif

