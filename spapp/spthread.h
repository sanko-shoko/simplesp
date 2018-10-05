//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_THREAD_H__
#define __SP_THREAD_H__

#include <thread>
#include <mutex>

#if SP_USE_THREAD

namespace sp {

    void sleep(const long long ms) {
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

        template<class Class, void (Class::*Func)()>
        bool run(Class *ptr, const bool wait = true) {
            if (wait == false && m_used == true) return false;

            thread th([this, ptr, wait] {
                m_mtx.lock();
                while (m_used == true) {
                    sleep(10);
                }
                m_used = true;
                m_mtx.unlock();

                (static_cast<Class*>(ptr)->*Func)();
                m_used = false;
            });
            th.detach();
            return true;
        }

        bool run(void (*func)(), const bool wait = true) {
            if (wait == false && m_used == true) return false;

            thread th([this, func, wait] {
                m_mtx.lock();
                while (m_used == true) {
                    sleep(10);
                }
                m_used = true;
                m_mtx.unlock();

                func();
                m_used = false;
            });
            th.detach();
            return true;
        }
    };
}
#endif

#endif
