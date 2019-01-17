//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_HOLD_H__
#define __SP_HOLD_H__

#include "spcore/spcom.h"

#ifndef SP_USE_DEBUG
#define SP_USE_DEBUG 0
#endif

#ifndef SP_USE_HOLDER
#define SP_USE_HOLDER SP_USE_DEBUG
#endif


//--------------------------------------------------------------------------------
// additional include
//--------------------------------------------------------------------------------

#if SP_USE_HOLDER
#include <vector>
#include <string>
#endif


//--------------------------------------------------------------------------------
// data holder
//--------------------------------------------------------------------------------

#if SP_USE_HOLDER

#define SP_HOLDER_SET(NAME, DATA) Holder::root()->set(NAME, DATA);
#define SP_HOLDER_GET(NAME, TYPE) Holder::root()->get<TYPE>(NAME);
#else

#define SP_HOLDER_SET(NAME, DATA)
#define SP_HOLDER_GET(NAME, TYPE) NULL;
#endif

namespace sp {
    using namespace std;

    class Holder{

    private:

#if SP_USE_HOLDER
        vector<string> names;
        vector<void *> ptrs;
#endif

    public:

        ~Holder(){
            reset();
        }
   
        void reset(){

#if SP_USE_HOLDER
            names.clear();
            for (int i = 0; i < ptrs.size(); i++) {
                delete ptrs[i];
            }
            ptrs.clear();
#endif

        }

        template <typename TYPE>
        void set(const char *name, const TYPE &data){
            TYPE *ptr = NULL;

#if SP_USE_HOLDER
            for (int i = 0; i < names.size(); i++){
                if (name == names[i]){
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
        const TYPE* get(const char *name){
            TYPE *ptr = NULL;

#if SP_USE_HOLDER
            for (int i = 0; i < names.size(); i++){
                if (name == names[i]) {
                    ptr = (TYPE*)ptrs[i];
                    break;
                }
            }
#endif
            return ptr;
        }

        static Holder *root() {
            static Holder holder;
            return &holder;
        }
    };
}



#endif
