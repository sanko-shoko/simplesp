//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_DEBUG_H__
#define __SP_DEBUG_H__


#include "spcore/spcom.h"
#include "spcore/spwrap.h"

#if SP_USE_DEBUG
#define SP_USE_HOLDER 1
#define SP_USE_LOGGER 1
#endif


//--------------------------------------------------------------------------------
// additional include
//--------------------------------------------------------------------------------

#if SP_USE_DEBUG

#include <string>
#include <vector>
#include <chrono>

#if WIN32
#include <Windows.h>
#endif

#endif


namespace sp {

    //--------------------------------------------------------------------------------
    // stdio 
    //--------------------------------------------------------------------------------

#if    SP_USE_CONSOLE
#define SP_PRINTF(...) ::printf(__VA_ARGS__);
#else
#define SP_PRINTF(...) if(0){ ::printf(__VA_ARGS__); }
#endif

#if SP_USE_DEBUG
#define SP_PRINTD(...) SP_PRINTF(__VA_ARGS__);
#else
#define SP_PRINTD(...) if(0){ SP_PRINTF(__VA_ARGS__); }
#endif

#if SP_USE_ASSERT
#define SP_ASSERT(EXP) if(!(EXP)){ ::printf(" assert (%s)\n file: %s\n line: %d\n", #EXP, __FILE__, __LINE__); ::exit(0); }
#else
#define SP_ASSERT(EXP) (EXP);
#endif


    //--------------------------------------------------------------------------------
    // print 
    //--------------------------------------------------------------------------------

    SP_CPUFUNC void print(const int dim, const int *dsize) {
        SP_PRINTF("dim %d, ", dim);

        SP_PRINTF("dsize [ ");
        for (int d = 0; d < dim; d++){
            SP_PRINTF("%d ", dsize[d]);
        }
        SP_PRINTF("]\n");
    }

    SP_CPUFUNC void print(const Rect &rect) {
        SP_PRINTF("Rect dim %d, ", rect.dim);

        SP_PRINTF("dbase [ ");
        for (int i = 0; i < rect.dim; i++) {
            SP_PRINTF("%d ", rect.dbase[i]);
        }
        SP_PRINTF("], ");

        SP_PRINTF("dsize [ ");
        for (int i = 0; i < rect.dim; i++) {
            SP_PRINTF("%d ", rect.dsize[i]);
        }
        SP_PRINTF("]\n");
    }

    SP_CPUFUNC void print(const Vec2 &vec) {
        SP_PRINTF("Vec2 [ %+.5lf %+.5lf ]\n", vec.x, vec.y);
    }

    SP_CPUFUNC void print(const Vec3 &vec) {
        SP_PRINTF("Vec3 [ %+.5lf %+.5lf %+.5lf ]\n", vec.x, vec.y, vec.z);
    }

    SP_CPUFUNC void print(const Col3 &col) {
        SP_PRINTF("Col3 [ %d %d %d ]\n", col.r, col.g, col.b);
    }

    SP_CPUFUNC void print(const VecPN2 &vec) {
        SP_PRINTF("VecPN2 [\n");
        print(vec.pos);
        print(vec.nrm);
        SP_PRINTF("]\n");
    }

    SP_CPUFUNC void print(const VecPN3 &vec) {
        SP_PRINTF("VecPN3 [\n");
        print(vec.pos);
        print(vec.nrm);
        SP_PRINTF("]\n");
    }

    SP_CPUFUNC void print(const Mesh3 &mesh) {
        SP_PRINTF("Mesh3 [\n");
        print(mesh.pos[0]);
        print(mesh.pos[1]);
        print(mesh.pos[2]);
        SP_PRINTF("]\n");
    }

    SP_CPUFUNC void print(const double *mat, const int rows, const int cols) {
        SP_PRINTF("Mat %d %d [\n", rows, cols);
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                SP_PRINTF("%g ", mat[r * cols + c]);
            }
            SP_PRINTF("\n");
        }
        SP_PRINTF("]\n");
    }

    template <typename TYPE>
    SP_CPUFUNC void print(const ExPtr<TYPE> &mem) {
        SP_PRINTF("Mem ");

        print(mem.dim, mem.dsize);

        int size = 1;
        int unit[SP_DIMMAX] = { 0 };
        for (int i = 0; i < mem.dim; i++){
            size *= mem.dsize[i];
            unit[i] = size;
        }

        for (int i = 0; i < size; i++){
            for (int j = mem.dim - 1; j >= 0; j--){
                if ((i + 0) % unit[j] == 0){
                    SP_PRINTF((j == 0) ? " [ " : "[\n");
                }
            }
            
            const double val = mem.ptr[i];
            SP_PRINTF("%g ", val);

            for (int j = mem.dim - 1; j >= 0; j--){
                if ((i + 1) % unit[j] == 0){
                    SP_PRINTF("]\n");
                }
            }
        }
    }

    SP_CPUFUNC void print(const Rot &rot) {
        SP_PRINTF("Rot [ %+.5lf %+.5lf %+.5lf %+.5lf ]\n", rot.qx, rot.qy, rot.qz, rot.qw);
    }

    SP_CPUFUNC void print(const Pose &pose) {
        SP_PRINTF("Pose [\n");
        print(pose.rot);
        print(pose.trn);
        SP_PRINTF("]\n");
    }

    SP_CPUFUNC void print(const CamParam &cam) {
        SP_PRINTF("CamParam [\n");
        SP_PRINTF("size %d %d\n", cam.dsize[0], cam.dsize[1]);
        SP_PRINTF("fx %.5lf, fy %.5lf\n", cam.fx, cam.fy);
        SP_PRINTF("cx %.5lf, cy %.5lf\n", cam.cx, cam.cy);
        SP_PRINTF("k1 %+.5lf, k2 %+.5lf, k3 %+.5lf\n", cam.k1, cam.k2, cam.k3);
        SP_PRINTF("p1 %+.5lf, p2 %+.5lf\n", cam.p1, cam.p2);
        SP_PRINTF("]\n");
    }

    template<typename TYPE>
    SP_CPUFUNC void print(const TYPE &data, const char *name) {
        SP_PRINTF("%s ", name);
        print(data);
    }


    //--------------------------------------------------------------------------------
    // util
    //--------------------------------------------------------------------------------

    SP_CPUFUNC char* progressBar(const double crnt, const double size){
        const double rate = (size > 1) ? crnt / (size - 1.0) : 1.0;

        static char bar[11] = { 0 };
        for (int i = 0; i < 10; i++){
            bar[i] = (i <= 10 * rate) ? '>' : '-';
        }
        return bar;
    }
}

//--------------------------------------------------------------------------------
// data holder
//--------------------------------------------------------------------------------

#if SP_USE_HOLDER

#define SP_HOLDER_INSTANCE sp::Holder holder;
#define SP_HOLDER_RESET() holder.reset();
#define SP_HOLDER_SET(STR, DATA) { holder.set(STR, DATA); }
#define SP_HOLDER_GET(STR, CLASS) (CLASS.holder.get(STR));
#else

#define SP_HOLDER_INSTANCE
#define SP_HOLDER_RESET()
#define SP_HOLDER_SET(STR, DATA)
#define SP_HOLDER_GET(STR, CLASS) NULL;
#endif


#if SP_USE_HOLDER

namespace sp {
    using namespace std;

    class Holder{

    private:

        vector<string> strs;
        vector<void *> ptrs;

    public:

        void reset(){
            strs.clear();
            for (int i = 0; i < ptrs.size(); i++) {
                delete ptrs[i];
            }
            ptrs.clear();
        }

        ~Holder(){
            reset();
        }

        template <typename TYPE>
        void set(const char *str, const TYPE &data){

            for (int i = 0; i < strs.size(); i++){
                if (str == strs[i]){
                    *((TYPE*)ptrs[i]) = data;
                    return;
                }
            }

            {
                TYPE *ptr = new TYPE();
                *ptr = data;
                strs.push_back(str);
                ptrs.push_back(ptr);
            }
        }

        const void* get(const char *str){

            for (int i = 0; i < strs.size(); i++){
                if (str == strs[i]){
                    return ptrs[i];
                }
            }
            return NULL;
        }
    };
}
#endif


//--------------------------------------------------------------------------------
// time logger
//--------------------------------------------------------------------------------

#if SP_USE_LOGGER

#define SP_LOGGER_INSTANCE sp::Logger logger;
#define SP_LOGGER_SET(STR) sp::LoggerUnit loggerUnit(&logger,STR);
#else

#define SP_LOGGER_INSTANCE
#define SP_LOGGER_SET(STR)
#endif


#if SP_USE_LOGGER

namespace sp {
    using namespace std;

    class Logger{

    private:

#if WIN32
        typedef LARGE_INTEGER sptime;
        sptime freq;

        LARGE_INTEGER getNow(){
            LARGE_INTEGER v;
            QueryPerformanceCounter(&v);
            return v;
        }

        double getTime(const sptime start, const sptime end){
            return static_cast<double>(end.QuadPart - start.QuadPart) * 1000.0 / freq.QuadPart;
        }

    public:

        Logger(){
            QueryPerformanceFrequency(&freq);
        }
#else
        typedef chrono::system_clock::time_point sptime;

        sptime getNow(){
            return chrono::system_clock::now();
        }

        double getTime(const sptime start, const sptime end){
            return static_cast<double>(chrono::duration_cast<chrono::microseconds>(end - start).count() / 1000.0);
        }
    public:

        Logger(){
        }
#endif

        vector<sptime> cnts;
        vector<string> strs;

        vector<double> times;
        vector<string> disps;

        void reset(){
            cnts.clear();
            strs.clear();

            times.clear();
            disps.clear();
        }

        void start(const char* str){
            for (int i = 0; i < strs.size(); i++){
                if (strs[i] == str){
                    reset();
                    break;
                }
            }
            
            cnts.push_back(getNow());
            strs.push_back(str);
        }

        void stop(){
            const int stack = static_cast<int>(cnts.size());
            if (stack > 0){
                times.push_back(getTime(cnts[stack - 1], getNow()));
                disps.push_back(strs[stack - 1]);
                cnts.pop_back();
                strs.pop_back();
            }
            print();
        }

        void print(){
            const int stack = static_cast<int>(cnts.size());
            if (stack > 0) return;

            const int num = static_cast<int>(times.size());
            for (int i = 0; i < num; i++){
                SP_PRINTF("%s : %.3lf [ms]\n", disps[i].c_str(), times[i]);
            }
            SP_PRINTF("\n");

            reset();
        }
    };

    class LoggerUnit{
        Logger *logger;
    public:
        LoggerUnit(Logger *logger, const char *str){
            //SP_PRINTF("%s\n", str);

            this->logger = logger;
            logger->start(str);
        }

        ~LoggerUnit(){
            logger->stop();    
        }
    };
}
#endif


#endif
