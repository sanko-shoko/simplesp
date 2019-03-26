//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_MEM_H__
#define __SP_MEM_H__

#include "spcore/spcom.h"
#include "spcore/spgen/spacs.h"

namespace sp{

    //--------------------------------------------------------------------------------
    // mem base class 
    //--------------------------------------------------------------------------------

    template<typename TYPE> class Mem : public ExPtr<TYPE>{

    protected:

        // memory size
        int msize;

        void init(TYPE *ptr, const int dim, const int *dsize, const int msize){
            this->ptr = ptr;
            this->dim = dim;
            this->msize = msize;

            setMem(this->dsize, dim, dsize);
        }

        void reset(){
            const int dsize[SP_DIMMAX] = { 0 };
            init(NULL, 0, dsize, 0);
        }

        void free(){
            delete[]this->ptr;
            reset();
        }

        void move(Mem<TYPE> &mem){
            init(mem.ptr, mem.dim, mem.dsize, mem.msize);
            mem.reset();
        }
        
        void malloc(const int msize, const void *cpy) {
            const TYPE *tmp = this->ptr;

            if (msize > this->msize) {
                this->msize = msize;
                this->ptr = new TYPE[msize];
            }

            if (cpy != NULL) {
                copy(cpy, size());
            }

            if (this->ptr != tmp && (cpy == tmp || cpy == NULL)) {
                delete[]tmp;
            }
        }

        virtual void copy(const void *cpy, const int csize){
            memcpy(this->ptr, cpy, csize * sizeof(TYPE));
        }

    public:

        //--------------------------------------------------------------------------------
        // constructor / destructor
        //--------------------------------------------------------------------------------
        
        Mem() {
            reset();
        }

        Mem(const int dim, const int *dsize = NULL, const void *cpy = NULL){
            reset();
            resize(dim, dsize, cpy);
        }

        Mem(const Mem<TYPE> &mem){
            reset();
            resize(mem.dim, mem.dsize, mem.ptr);
        }

        Mem(Mem<TYPE> &&mem){
            reset();
            move(mem);
        }

        ~Mem(){
            free();
        }


        //--------------------------------------------------------------------------------
        // operator
        //--------------------------------------------------------------------------------

        Mem& operator = (const Mem<TYPE> &mem){
            resize(mem.dim, mem.dsize, mem.ptr);
            return *this;
        }

        Mem& operator = (Mem<TYPE> &&mem){
            free();
            move(mem);

            return *this;
        }

        TYPE& operator [](const int x){
            return (x >= 0) ? this->ptr[x] : this->ptr[size() - x];
        }

        const TYPE& operator [](const int x) const{
            return (x >= 0) ? this->ptr[x] : this->ptr[size() - x];
        }


        //--------------------------------------------------------------------------------
        // resize
        //--------------------------------------------------------------------------------

        void resize(const int dim, const int *dsize, const void *cpy = NULL){
            this->dim = maxVal(dim, 1);

            for (int i = 0; i < SP_DIMMAX; i++){
                this->dsize[i] = (dsize != NULL && i < dim) ? dsize[i] : 0;
            }

            malloc(size(), cpy);
        }


        //--------------------------------------------------------------------------------
        // base method
        //--------------------------------------------------------------------------------

        int size() const{
            int tsize = (this->dim > 0) ? 1 : 0;
            for (int i = 0; i < this->dim; i++){
                tsize *= this->dsize[i];
            }
            return tsize;
        }

        void clear(){
            free();
            resize(1, NULL);
        }

        void zero(){
            memset(this->ptr, 0, size() * sizeof(TYPE));
        }


        //--------------------------------------------------------------------------------
        // util
        //--------------------------------------------------------------------------------

        //Mem slice(const int axis, const int start, const int end) const{
        //    const int s = start;
        //    const int e = minVal(end, this->dsize[axis]);
        //    const int num = e - s;

        //    int dsize[SP_DIMMAX] = { 0 };
        //    memcpy(dsize, this->dsize, SP_DIMMAX);
        //    dsize[axis] = num;

        //    Mem<TYPE> ret(this->dim, dsize);

        //    int block = 1;
        //    for (int i = 0; i < axis; i++){
        //        block *= this->dsize[i];
        //    }

        //    TYPE *ptr = ret.ptr;
        //    for (int i = 0; i < size(); i += (block * this->dsize[axis])){
        //        for (int j = i + s * block; j < i + e * block; j++){
        //            *ptr++ = this->ptr[j];
        //        }
        //    }

        //    return ret;
        //}

    };



    //--------------------------------------------------------------------------------
    // mem 1d 
    //--------------------------------------------------------------------------------

    template<typename TYPE> class Mem1 : public Mem <TYPE> {

    protected:

        virtual void copy(const void *cpy, const int csize){
            const TYPE *src = static_cast<const TYPE*>(cpy);
            TYPE *dst = this->ptr;
            
            // work constructor
            for (int i = 0; i < csize; i++){
                *(dst++) = *(src++);
            }
        }

    public:

        //--------------------------------------------------------------------------------
        // constructor
        //--------------------------------------------------------------------------------

        Mem1(const Mem1<TYPE> &mem) : Mem<TYPE>(){
            resize(mem.dsize, mem.ptr);
        }

        Mem1(const Mem<TYPE> &mem) : Mem<TYPE>() {
            resize(mem.dsize, mem.ptr);
        }

        Mem1(Mem1<TYPE> &&mem) : Mem<TYPE>() {
            Mem<TYPE>::move(mem);
        }

        Mem1(Mem<TYPE> &&mem) : Mem<TYPE>() {
            Mem<TYPE>::move(mem);
        }

        Mem1(const int *dsize = NULL, const void *cpy = NULL) : Mem<TYPE>() {
            resize(dsize, cpy);
        }

        Mem1(const int dsize0, const void *cpy = NULL) : Mem<TYPE>() {
            resize(dsize0, cpy);
        }


        //--------------------------------------------------------------------------------
        // resize
        //--------------------------------------------------------------------------------

        void resize(const int dsize0, const void *cpy = NULL){
            const int dsize[] = { dsize0 };
            resize(dsize, cpy);
        }

        void resize(const int *dsize, const void *cpy = NULL){
            Mem<TYPE>::resize(1, dsize, cpy);
        }


        //--------------------------------------------------------------------------------
        // operator
        //--------------------------------------------------------------------------------

        Mem1& operator = (const Mem1<TYPE> &mem){
            Mem<TYPE>::resize(mem.dim, mem.dsize, mem.ptr);
            return *this;
        }

        Mem1& operator = (Mem1<TYPE> &&mem){
            Mem<TYPE>::free();
            Mem<TYPE>::move(mem);

            return *this;
        }

        TYPE& operator () (const int d0, const bool loop = false){
            return (loop == false) ? acs1(*this, d0) : lacs1(*this, d0);
        }

        const TYPE& operator () (const int d0, const bool loop = false) const{
            return (loop == false) ? acs1(*this, d0) : lacs1(*this, d0);
        }


        //--------------------------------------------------------------------------------
        // util
        //--------------------------------------------------------------------------------
        
        void reserve(const int msize) {
            this->dsize[0] = 0;
            if (msize > this->msize) {
                Mem<TYPE>::malloc(msize, NULL);
            }
        }

        TYPE* extend(const int ex = 1) {
            const int s = this->dsize[0];

            if (s + ex > this->msize) {
                Mem<TYPE>::malloc((s + ex) * 2, this->ptr);
            }
            this->dsize[0] += ex;

            return &this->ptr[s];
        }

        void push(const TYPE &data){
            *extend() = data;
        }

        void push(const TYPE *data, const int num) {
            for (int i = 0; i < num; i++) {
                push(data[i]);
            }
        }

        void push(const Mem<TYPE> &data){
            push(data.ptr, data.size());
        }

        void pop() {
            if (this->dsize[0] > 0) {
                this->dsize[0]--;
            }
        }

        void add(const int x, const TYPE &data) {
            extend();
            for (int i = this->dsize[0] - 1; i > x; i--) {
                this->ptr[i] = this->ptr[i - 1];
            }
            this->ptr[x] = data;
        }

        void del(const int x) {
            for (int i = x; i < this->dsize[0] - 1; i++) {
                this->ptr[i] = this->ptr[i + 1];
            }
            pop();
        }

        TYPE* last() {
            return  (this->dsize[0] > 0) ? &this->ptr[this->dsize[0] - 1] : NULL;
        }

        const TYPE* last() const {
            return  (this->dsize[0] > 0) ? &this->ptr[this->dsize[0] - 1] : NULL;
        }

        Mem1 part(const int dbase0, const int dsize0) const {
            Mem1<TYPE> ret(minVal(dsize0, this->dsize[0] - dbase0));
            TYPE *ptr = ret.ptr;

            for (int d0 = 0; d0 < ret.dsize[0]; d0++){
                *ptr++ = (*this)(dbase0 + d0);
            }

            return ret;
        }
    };


    //--------------------------------------------------------------------------------
    // mem 2d 
    //--------------------------------------------------------------------------------

    template<typename TYPE> class Mem2 : public Mem <TYPE> {

    public:
        
        //--------------------------------------------------------------------------------
        // constructor
        //--------------------------------------------------------------------------------

        Mem2(const Mem2<TYPE> &mem) : Mem<TYPE>() {
            resize(mem.dsize, mem.ptr);
        }
        
        Mem2(const Mem<TYPE> &mem) : Mem<TYPE>() {
            resize(mem.dsize, mem.ptr);
        }

        Mem2(Mem2<TYPE> &&mem) : Mem<TYPE>() {
             Mem<TYPE>::move(mem);
        }

        Mem2(Mem<TYPE> &&mem) : Mem<TYPE>() {
            Mem<TYPE>::move(mem);
        }

        Mem2(const int *dsize = NULL, const void *cpy = NULL) : Mem<TYPE>() {
            resize(dsize, cpy);
        }

        Mem2(const int dsize0, const int dsize1, const void *cpy = NULL) : Mem<TYPE>() {
            resize(dsize0, dsize1, cpy);
        }


        //--------------------------------------------------------------------------------
        // resize
        //--------------------------------------------------------------------------------

        void resize(const int dsize0, const int dsize1, const void *cpy = NULL){
            const int dsize[] = { dsize0, dsize1 };
            resize(dsize, cpy);
        }

        void resize(const int *dsize, const void *cpy = NULL){
            Mem<TYPE>::resize(2, dsize, cpy);
        }


        //--------------------------------------------------------------------------------
        // operator
        //--------------------------------------------------------------------------------
    
        Mem2& operator = (const Mem2<TYPE> &mem){
            Mem<TYPE>::resize(mem.dim, mem.dsize, mem.ptr);
            return *this;
        }

        Mem2& operator = (Mem2<TYPE> &&mem){
            Mem<TYPE>::free();
            Mem<TYPE>::move(mem);

            return *this;
        }

        TYPE& operator () (const int d0, const int d1){
            return acs2(*this, d0, d1);
        }

        const TYPE& operator () (const int d0, const int d1) const{
            return acs2(*this, d0, d1);
        }


        //--------------------------------------------------------------------------------
        // util
        //--------------------------------------------------------------------------------
        
        Mem2 part(const int dbase0, const int dbase1, const int dsize0, const int dsize1) const {
            Mem2<TYPE> ret(minVal(dsize0, this->dsize[0] - dbase0), minVal(dsize1, this->dsize[1] - dbase1));
            TYPE *ptr = ret.ptr;

            for (int d1 = 0; d1 < ret.dsize[1]; d1++) {
                for (int d0 = 0; d0 < ret.dsize[0]; d0++) {
                    *ptr++ = (*this)(dbase0 + d0, dbase1 + d1);
                }
            }

            return ret;
        }
    };


    //--------------------------------------------------------------------------------
    // mem 3d 
    //--------------------------------------------------------------------------------

    template<typename TYPE> class Mem3 : public Mem <TYPE> {

    public:

        //--------------------------------------------------------------------------------
        // constructor
        //--------------------------------------------------------------------------------
        
        Mem3(const Mem3<TYPE> &mem) : Mem<TYPE>() {
            resize(mem.dsize, mem.ptr);
        }

        Mem3(const Mem<TYPE> &mem) : Mem<TYPE>() {
            resize(mem.dsize, mem.ptr);
        }

        Mem3(Mem3<TYPE> &&mem) : Mem<TYPE>() {
            Mem<TYPE>::move(mem);
        }
        Mem3(Mem<TYPE> &&mem) : Mem<TYPE>() {
            Mem<TYPE>::move(mem);
        }

        Mem3(const int *dsize = NULL, const void *cpy = NULL) : Mem<TYPE>() {
            resize(dsize, cpy);
        }

        Mem3(const int dsize0, const int dsize1, const int dsize2, const void *cpy = NULL) : Mem<TYPE>() {
            resize(dsize0, dsize1, dsize2, cpy);
        }


        //--------------------------------------------------------------------------------
        // resize
        //--------------------------------------------------------------------------------

        void resize(const int dsize0, const int dsize1, const int dsize2, const void *cpy = NULL){
            const int dsize[] = { dsize0, dsize1, dsize2 };
            resize(dsize, cpy);
        }

        void resize(const int *dsize, const void *cpy = NULL){
            Mem<TYPE>::resize(3, dsize, cpy);
        }


        //--------------------------------------------------------------------------------
        // operator
        //--------------------------------------------------------------------------------
     
        Mem3& operator = (const Mem3<TYPE> &mem){
            Mem<TYPE>::resize(mem.dim, mem.dsize, mem.ptr);
            return *this;
        }

        Mem3& operator = (Mem3<TYPE> &&mem){
            Mem<TYPE>::free();
            Mem<TYPE>::move(mem);

            return *this;
        }

        TYPE& operator () (const int d0, const int d1, const int d2){
            return acs3(*this, d0, d1, d2);
        }

        const TYPE& operator () (const int d0, const int d1, const int d2) const{
            return acs3(*this, d0, d1, d2);
        }


        //--------------------------------------------------------------------------------
        // util
        //--------------------------------------------------------------------------------

        Mem3 part(const int dbase0, const int dbase1, const int dbase2, const int dsize0, const int dsize1, const int dsize2) const {
            Mem3<TYPE> ret(minVal(dsize0, this->dsize[0] - dbase0), minVal(dsize1, this->dsize[1] - dbase1), minVal(dsize2, this->dsize[2] - dbase2));
            TYPE *ptr = ret.ptr;

            for (int d2 = 0; d2 < ret.dsize[2]; d2++) {
                for (int d1 = 0; d1 < ret.dsize[1]; d1++) {
                    for (int d0 = 0; d0 < ret.dsize[0]; d0++) {
                        *ptr++ = (*this)(dbase0 + d0, dbase1 + d1, dbase2 + d2);
                    }
                }
            }

            return ret;
        }
    };


    //--------------------------------------------------------------------------------
    // matrix
    //--------------------------------------------------------------------------------

    class Mat : public Mem <double> {
        typedef double TYPE;
    public:

        Mat(const Mat &mat){
            resize(mat.dsize, mat.ptr);
        }
        
        Mat(Mat &&mat){
            Mem<TYPE>::move(mat);
        }

        Mat(const int *dsize = NULL, const void *cpy = NULL){
            resize(dsize, cpy);
        }

        Mat(const int rows, const int cols, const void *cpy = NULL){
            resize(rows, cols, cpy);
        }


        //--------------------------------------------------------------------------------
        // resize
        //--------------------------------------------------------------------------------

        void resize(const int rows, const int cols, const void *cpy = NULL){
            const int dsize[] = { cols, rows };
            resize(dsize, cpy);
        }

        void resize(const int *dsize, const void *cpy = NULL){
            Mem<TYPE>::resize(2, dsize, cpy);
        }


        //--------------------------------------------------------------------------------
        // operator
        //--------------------------------------------------------------------------------
        
        Mat& operator = (const Mat &mem){
            Mem<TYPE>::resize(mem.dim, mem.dsize, mem.ptr);
            return *this;
        }

        Mat& operator = (Mat &&mem){
            Mem<TYPE>::free();
            Mem<TYPE>::move(mem);

            return *this;
        }

        double& operator () (const int r, const int c){
            return acsm(*this, r, c);
        }

        const double& operator () (const int r, const int c) const{
            return acsm(*this, r, c);
        }


        //--------------------------------------------------------------------------------
        // util
        //--------------------------------------------------------------------------------

        int rows() const{
            return this->dsize[1];
        }

        int cols() const{
            return this->dsize[0];
        }

        Mat part(const int rbase, const int cbase, const int rsize, const int csize) const {
            Mat mat(minVal(rsize, rows() - rbase), minVal(csize, cols() - cbase));
            for (int r = 0; r < mat.rows(); r++) {
                for (int c = 0; c < mat.cols(); c++) {
                    mat(r, c) = (*this)(rbase + r, cbase + c);
                }
            }
            return mat;
        }

        Mat row(const int rbase, const int rsize = 1) const {
            return part(rbase, 0, rsize, cols());
        }

        Mat col(const int cbase, const int csize = 1) const {
            return part(0, cbase, rows(), csize);
        }

    };


    //--------------------------------------------------------------------------------
    // mem array
    //--------------------------------------------------------------------------------

    template<typename TYPE, int SIZE> class MemA {
   
    public:
        TYPE ptr[SIZE];

    public:
        MemA() {
        }

        MemA(const MemA<TYPE, SIZE> &mem) {
            set(mem.ptr);
        }

        MemA(const TYPE data0, ...) {
            va_list arg;
            va_start(arg, data0);
            ptr[0] = data0;
            for (int i = 1; i < SIZE; i++) {
                ptr[i] = va_arg(arg, TYPE);
            }
            va_end(arg);
        }

        MemA& operator = (const MemA<TYPE, SIZE> &mem) {
            set(mem.ptr);
            return *this;
        }

         //--------------------------------------------------------------------------------
        // operator
        //--------------------------------------------------------------------------------
        
        TYPE& operator [] (const int i) {
            return ptr[i];
        }
        const TYPE& operator [] (const int i) const {
            return ptr[i];
        }

        //--------------------------------------------------------------------------------
        // util
        //--------------------------------------------------------------------------------
     
        void set(const TYPE data0, ...) {
            va_list arg;
            va_start(arg, data0);
            ptr[0] = data0;
            for (int i = 1; i < SIZE; i++) {
                ptr[i] = va_arg(arg, TYPE);
            }
            va_end(arg);
        }

        void set(const TYPE *ptr) {
            for (int i = 0; i < SIZE; i++) {
                this->ptr[i] = ptr[i];
            }
        }

        int size() const {
            return SIZE;
        }

    };


    //--------------------------------------------------------------------------------
    // mem pool 
    //--------------------------------------------------------------------------------

    template<typename TYPE> class MemP {

        // data unit size (ex TYPE[2] -> 2)
        int m_unit;

        // pool block size
        int m_block;

        // pool block ptr
        Mem1<TYPE*> m_ptrs;

        // reference to next 
        Mem1<int*> m_refs;
        
        // next id
        int m_next;

        // crnt size
        int m_size;

        // max size
        int m_maxs;

    public:

        MemP(){
            init();
        }
        
        MemP(const MemP &mem) {
            *this = mem;
        }

        MemP& operator = (const MemP &mem) {
            init(mem.m_unit, mem.m_block);

            for (int i = 0; i < mem.size(); i++) {
                *malloc() = mem[i];
            }
            return *this;
        }

        ~MemP(){
            clear();
        }

        void init(const int unit = 1, const int block = 100){
            clear();

            m_unit = maxVal(1, unit);
            m_block = block;
        }

        void clear(){
            for (int i = 0; i < m_ptrs.size(); i++){
                delete[]m_ptrs[i];
            }
            m_ptrs.clear();

            for (int i = 0; i < m_refs.size(); i++){
                delete[]m_refs[i];
            }
            m_refs.clear();

            m_next = 0;
            m_maxs = 0;
            m_size = 0;
        }

        //--------------------------------------------------------------------------------
        // util
        //--------------------------------------------------------------------------------
    
        int size() const {
            return m_size;
        }

        TYPE& operator[](const int x) {
            const int crnt = search(x);
            const int m = crnt / m_block;
            const int n = crnt % m_block;
            return m_ptrs[m][n * m_unit];
        }

        const TYPE& operator[](const int x) const {
            const int crnt = search(x);
            const int m = crnt / m_block;
            const int n = crnt % m_block;
            return m_ptrs[m][n * m_unit];
        }

        TYPE* malloc(){

            if (m_size >= m_ptrs.size() * m_block){
                m_ptrs.push(new TYPE[m_unit * m_block]);
                m_refs.push(new int[m_block]);
                m_next = m_size;
            }

            const int m = m_next / m_block;
            const int n = m_next % m_block;

            if (m_size >= m_maxs){
                m_maxs++;
                m_next++;
            }
            else{
                m_next = m_refs[m][n];
            }
            
            m_refs[m][n] = -1;

            m_size++;

            TYPE *ret = &m_ptrs[m][n * m_unit];
            for (int i = 0; i < m_unit; i++) {
                ret[i] = TYPE();
            }
            return ret;
        }

        void free(TYPE *ptr){
            if (m_size == 0) return;

            int crnt = -1;
            for (int i = 0; i < m_ptrs.size(); i++){
                const TYPE *p = m_ptrs[i];
                if (ptr >= p && ptr < p + m_unit * m_block){

                    crnt = i * m_block + static_cast<int>(ptr - p) / m_unit;
                    break;
                }
            }
            if (crnt < 0) return;

            const int m = crnt / m_block;
            const int n = crnt % m_block;

            m_size--;

            m_refs[m][n] = m_next;
            m_next = crnt;
        }

    private:

        int search(const int x) const {
            int id = 0;

            int cnt = 0;
            for (int i = 0; i < m_refs.size(); i++) {
                for (int j = 0; j < m_block; j++, id++) {
                    if (m_refs[i][j] < 0 && cnt++ == x) goto _exit;
                }
            }
        _exit:;
            return id;
        }
    };

}

#endif