//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_NEUBLOCK_H__
#define __SP_NEUBLOCK_H__

#include "spcore/spcore.h"
#include "spapp/spdata/spfile.h"
#include "spapp/splearn/spneubase.h"

namespace sp{

    //--------------------------------------------------------------------------------
    // base layer
    //--------------------------------------------------------------------------------

    class BaseLayer{

    private:

        //--------------------------------------------------------------------------------
        // data flow 
        //--------------------------------------------------------------------------------

        //
        //            forward->
        //      X      _______     Y
        //   -------> |       | ------->
        //            | layer |
        //   <------- |_______| <-------
        //      B                  A
        //           <-backward 
        //

        // src (mini batch)
        const Mem1<Mem<SP_REAL> > *m_X, *m_A;

        // dst (mini batch)
        Mem1<Mem<SP_REAL> > m_Y, m_B;

    public:

        // initialize flag
        bool m_init;

        // training flag
        bool m_train;

        // node number
        int m_nodeNum;

        BaseLayer(){
            m_init = false;
            m_train = false;
            m_nodeNum = 0;
        }
        ~BaseLayer() {
        }

        // get layer name
        virtual const char* getName() = 0;

        // file text
        SP_TEXTEX(){
            file.text(&m_init, 1, "init");
            file.text(&m_train, 1, "train");
            file.text(&m_nodeNum, 1, "nodeNum");
        }

        //--------------------------------------------------------------------------------
        // execute
        //--------------------------------------------------------------------------------

        // execute foward(true) / backward(false)
        const Mem1<Mem<SP_REAL> >* execute(const Mem1<Mem<SP_REAL> > *src, const bool direct){

            return (direct == true) ? _forward(src) : _backward(src);
        }

        // get result data
        const Mem1<Mem<SP_REAL> >& getResult(const bool direct){

            return (direct == true) ? m_Y : m_B;
        }

    private:

        const Mem1<Mem<SP_REAL> >* _forward(const Mem1<Mem<SP_REAL> > *src){
            m_X = src;
            m_Y.resize(src->size());

            if (m_init == false){
                m_init = true;
                init(*m_X);
            }

            forward(m_Y, *m_X);
            return &m_Y;
        }

        const Mem1<Mem<SP_REAL> >* _backward(const Mem1<Mem<SP_REAL> > *src){
            m_A = src;
            m_B.resize(src->size());

            backward(m_B, *m_A, m_Y, *m_X);

            // reshape
            for (int n = 0; n < m_B.size(); n++){
                m_B[n].resize((*m_X)[n].dim, (*m_X)[n].dsize);
            }
            return &m_B;
        }

        virtual void init(const Mem1<Mem<SP_REAL> > &X){
        }

        virtual void forward(Mem1<Mem<SP_REAL> > &Y, const Mem1<Mem<SP_REAL> > &X){
        }

        virtual void backward(Mem1<Mem<SP_REAL> > &B, const Mem1<Mem<SP_REAL> > &A, const Mem1<Mem<SP_REAL> > &Y, const Mem1<Mem<SP_REAL> > &X){
        }

    };


    //--------------------------------------------------------------------------------
    // parameter layer
    //--------------------------------------------------------------------------------

    class ParamLayer : public BaseLayer{
    public:
        enum UpdataMethod{
            SGD = 0, Momentum = 1,
        };
        UpdataMethod m_update;

    private:

        // update rate
        SP_REAL m_lambda;

    protected:

        // node parameter
        NodeParam m_prm;

        // velocity for momentum
        NodeParam m_vel;

    public:

        ParamLayer(){
            m_update = Momentum;
            m_lambda = 0.1;
        }

        SP_TEXTEX(){
            BaseLayer::textex(file);

            file.textf("%d", &m_update, 1, "update");
            file.text(&m_lambda, 1, "lambda");
            file.text(&m_prm.w, 1, "prm.w");
            file.text(&m_prm.b, 1, "prm.b");

            file.text(&m_vel.w, 1, "vel.w");
            file.text(&m_vel.b, 1, "vel.b");
        }

        void update(const NodeParam &grd){
            switch (m_update){
            default:
            case SGD:
                sgd(grd); break;
            case Momentum:
                momentum(grd); break;
            }
        }

        void update(const Mem1<NodeParam> &grds){
            NodeParam grd = grds[0];
            for (int n = 1; n < grds.size(); n++){
                grd.w += grds[n].w;
                grd.b += grds[n].b;
            }
            update(grd);
        }

    private:
        void sgd(const NodeParam &grd){
            m_prm.w -= grd.w * m_lambda;
            m_prm.b -= grd.b * m_lambda;
        }

        void momentum(const NodeParam &grd){
            if (!cmp(2, m_vel.w.dsize, grd.w.dsize) || !cmp(2, m_vel.b.dsize, grd.b.dsize)){
                m_vel.w = grd.w * m_lambda;
                m_vel.b = grd.b * m_lambda;
            }
            const double momentum = 0.9;

            m_vel.w = m_vel.w * momentum + grd.w * m_lambda;
            m_vel.b = m_vel.b * momentum + grd.b * m_lambda;
            m_prm.w -= m_vel.w;
            m_prm.b -= m_vel.b;

        }

    };


    //--------------------------------------------------------------------------------
    // affine layer
    //--------------------------------------------------------------------------------

    class AffineLayer : public ParamLayer{

    public:
        AffineLayer(){
        }

        AffineLayer(const int nodeNum){
            m_nodeNum = nodeNum;
        }

        virtual const char* getName(){
            return "AffineLayer";
        };

        SP_TEXTEX(){
            ParamLayer::textex(file);
        }

        virtual void init(const Mem1<Mem<SP_REAL> > &X){
            const int dataNum = X[0].size();

            // foward parameter
            m_prm.resize(m_nodeNum, dataNum);
            m_prm.rand();

        }

        virtual void forward(Mem1<Mem<SP_REAL> > &Y, const Mem1<Mem<SP_REAL> > &X){

#if SP_USE_OMP
#pragma omp parallel for
#endif
            for (int n = 0; n < Y.size(); n++){
                const Mat mX = Mat(X[n].size(), 1, X[n].ptr);

                // Y = w * X + b
                Y[n] = m_prm.w * mX + m_prm.b;
            }
        }

        virtual void backward(Mem1<Mem<SP_REAL> > &B, const Mem1<Mem<SP_REAL> > &A, const Mem1<Mem<SP_REAL> > &Y, const Mem1<Mem<SP_REAL> > &X){

            const Mat wt = trnMat(m_prm.w);

            Mem1<NodeParam> m_grds(B.size());

#if SP_USE_OMP
#pragma omp parallel for
#endif
            for (int n = 0; n < B.size(); n++){
                const Mat mA(A[n].size(), 1, A[n].ptr);
                const Mat mX(X[n].size(), 1, X[n].ptr);

                m_grds[n].w = mA * trnMat(mX);
                m_grds[n].b = mA;

                B[n] = wt * mA;
            }

            // update
            update(m_grds);
        }

    };


    //--------------------------------------------------------------------------------
    // soft max layer
    //--------------------------------------------------------------------------------

    class SoftMaxLayer : public BaseLayer{

    public:

        SoftMaxLayer(){
        }

    private:

        virtual void forward(Mem1<Mem<SP_REAL> > &Y, const Mem1<Mem<SP_REAL> > &X){

            for (int n = 0; n < Y.size(); n++){
                const SP_REAL maxv = maxval(X[n]);

                Mem1<SP_REAL> S(X[n].size());
                for (int i = 0; i < S.size(); i++){
                    S[i] = exp(X[n][i] - maxv);
                }

                // Y = S / sum(S)
                divElm(Y[n], S, sumVal(S));
            }
        }

        virtual void backward(Mem1<Mem<SP_REAL> > &B, const Mem1<Mem<SP_REAL> > &A, const Mem1<Mem<SP_REAL> > &Y, const Mem1<Mem<SP_REAL> > &X){

            for (int n = 0; n < B.size(); n++){

                // B = (Y - A) / batch
                subMem(B[n], Y[n], A[n]);
                divElm(B[n], B[n], B.size());
            }
        }

    public:

        //--------------------------------------------------------------------------------
        // text
        //--------------------------------------------------------------------------------

        virtual const char* getName(){
            return "SoftMaxLayer";
        };

        SP_TEXTEX(){
            BaseLayer::textex(file);
        }
    };


    //--------------------------------------------------------------------------------
    // activation layer
    //--------------------------------------------------------------------------------

    class ActivationLayer : public BaseLayer{

    public:
        enum ActivationMethod{
            ReLU = 0, Sigmoid = 1,
        };
        ActivationMethod m_activation;

    public:

        ActivationLayer(const ActivationMethod activation = ReLU){
            m_activation = activation;
        }

        virtual const char* getName(){
            return "ActivationLayer";
        };

        SP_TEXTEX(){
            BaseLayer::textex(file);
            file.textf("%d", &m_activation, 1, "activation");
        }

    private:

        virtual void forward(Mem1<Mem<SP_REAL> > &Y, const Mem1<Mem<SP_REAL> > &X){

            for (int n = 0; n < Y.size(); n++){
                Y[n].resize(X[n].dim, X[n].dsize);

                for (int i = 0; i < Y[n].size(); i++){
                    Y[n][i] = calcFwrd(X[n][i]);
                }
            }
        }

        virtual void backward(Mem1<Mem<SP_REAL> > &B, const Mem1<Mem<SP_REAL> > &A, const Mem1<Mem<SP_REAL> > &Y, const Mem1<Mem<SP_REAL> > &X){

            for (int n = 0; n < B.size(); n++){
                B[n].resize(A[n].dim, A[n].dsize);

                for (int i = 0; i < B[n].size(); i++){
                    B[n][i] = calcBwrd(A[n][i], Y[n][i], X[n][i]);
                }
            }
        }

        SP_REAL calcFwrd(const SP_REAL x){
            switch (m_activation){
            case ReLU:
                return (x > 0) ? x : 0;
            case Sigmoid:
            default:
                return 1.0 / (1.0 + exp(-x));
            }
        }

        SP_REAL calcBwrd(const SP_REAL a, const SP_REAL y, const SP_REAL x){
            switch (m_activation){
            case ReLU:
                return (x > 0) ? a : 0;
            case Sigmoid:
            default:
                return a * y * y * exp(-x);
            }
        }

    };


    //--------------------------------------------------------------------------------
    // convolution layer
    //--------------------------------------------------------------------------------

    class ConvolutionLayer : public ParamLayer{

    private:

        // window size
        int m_winSize;

        // step distance
        int m_stride;

        // margin
        int m_margin;

        // output dsize
        int m_output[3];

        // kernel dsize
        int m_kernel[3];

    public:
        ConvolutionLayer(){
            m_nodeNum = 0;
            m_winSize = 0;
            m_stride = 0;
            m_margin = 0;
        }

        ConvolutionLayer(const int convNum, const int winSize, const int stride, const int margin = 0){
            m_nodeNum = convNum;
            m_winSize = winSize;
            m_stride = stride;
            m_margin = margin;
        }

        virtual const char* getName(){
            return "ConvolutionLayer";
        };

        SP_TEXTEX(){
            ParamLayer::textex(file);

            file.text(&m_winSize, 1, "winSize");
            file.text(&m_stride, 1, "stride");
            file.text(&m_margin, 1, "margin");
            file.text(m_output, 3, "output");
            file.text(m_kernel, 3, "kernel");
        }

    private:

        virtual void init(const Mem1<Mem<SP_REAL> > &X){

            // output dsize
            m_output[0] = (X[0].dsize[0] - 2 * m_margin) / m_stride;
            m_output[1] = (X[0].dsize[1] - 2 * m_margin) / m_stride;
            m_output[2] = m_nodeNum;

            // kernel dsize
            m_kernel[0] = m_winSize;
            m_kernel[1] = m_winSize;
            m_kernel[2] = maxval(X[0].dsize[2], 1);

            // foward parameter
            m_prm.resize(m_nodeNum, m_kernel[0] * m_kernel[1] * m_kernel[2]);
            m_prm.rand();

        }

        virtual void forward(Mem1<Mem<SP_REAL> > &Y, const Mem1<Mem<SP_REAL> > &X){

#if SP_USE_OMP
#pragma omp parallel for
#endif
            for (int n = 0; n < X.size(); n++){
                Y[n] = cnvFwrd(X[n]);
            }
        }

        virtual void backward(Mem1<Mem<SP_REAL> > &B, const Mem1<Mem<SP_REAL> > &A, const Mem1<Mem<SP_REAL> > &Y, const Mem1<Mem<SP_REAL> > &X){

            Mem1<NodeParam> grds(B.size());

#if SP_USE_OMP
#pragma omp parallel for
#endif
            for (int n = 0; n < B.size(); n++){
                B[n] = cnvBwrd(A[n], X[n]);
                grds[n] = grdBwrd(A[n], X[n]);
            }

            // update
            update(grds);
        }

        Mem<SP_REAL>  cnvFwrd(const Mem<SP_REAL>  &X){
            Mem<SP_REAL>  Y(3, m_output);

            for (int oc = 0; oc < m_output[2]; oc++){
                for (int ov = 0; ov < m_output[1]; ov++){
                    for (int ou = 0; ou < m_output[0]; ou++){

                        const int u = ou * m_stride + m_margin;
                        const int v = ov * m_stride + m_margin;

                        const SP_REAL *pw = &m_prm.w(oc, 0);
                        const int hsize = m_winSize / 2;

                        SP_REAL sum = 0.0;

                        // convolution forward
                        for (int kc = 0; kc < m_kernel[2]; kc++){
                            for (int ky = 0; ky < m_kernel[1]; ky++){
                                for (int kx = 0; kx < m_kernel[0]; kx++){
                                    sum += *(pw++) * acs3(X, u + kx - hsize, v + ky - hsize, kc);
                                }
                            }
                        }
                        acs3(Y, ou, ov, oc) = sum + m_prm.b(oc, 0);
                    }
                }
            }
            return Y;
        }

        Mem<SP_REAL>  cnvBwrd(const Mem<SP_REAL>  &A, const Mem<SP_REAL>  &X){
            Mem<SP_REAL>  B(X.dim, X.dsize);
            B.zero();

            Mat mB = trnMat(Mat(m_nodeNum, m_output[0] * m_output[1], A.ptr)) * m_prm.w;

            const Rect2 rect = getRect2(X.dsize);
            for (int ov = 0; ov < m_output[1]; ov++){
                for (int ou = 0; ou < m_output[0]; ou++){

                    const int u = ou * m_stride + m_margin;
                    const int v = ov * m_stride + m_margin;

                    const SP_REAL *pb = &mB(ov * m_output[0] + ou, 0);
                    const int hsize = m_winSize / 2;

                    // convolution backward
                    for (int kc = 0; kc < m_kernel[2]; kc++){
                        for (int ky = 0; ky < m_kernel[1]; ky++){
                            for (int kx = 0; kx < m_kernel[0]; kx++){
                                const SP_REAL b = *(pb++);
                                if (inRect(rect, u + kx - hsize, v + ky - hsize) == false) continue;

                                acs3(B, u + kx - hsize, v + ky - hsize, kc) += b;
                            }
                        }
                    }
                }
            }
            return B;
        }

        NodeParam grdBwrd(const Mem<SP_REAL>  &A, const Mem<SP_REAL>  &X){
            NodeParam grd;
            grd.resize(m_nodeNum, m_kernel[0] * m_kernel[1] * m_kernel[2]);
            grd.zero();

            for (int oc = 0; oc < m_output[2]; oc++){
                for (int ov = 0; ov < m_output[1]; ov++){
                    for (int ou = 0; ou < m_output[0]; ou++){

                        const int u = ou * m_stride + m_margin;
                        const int v = ov * m_stride + m_margin;

                        const SP_REAL a = acs3(A, ou, ov, oc);
                        const int hsize = m_winSize / 2;

                        // w
                        SP_REAL *pw = &grd.w(oc, 0);

                        for (int kc = 0; kc < m_kernel[2]; kc++){
                            for (int ky = 0; ky < m_kernel[1]; ky++){
                                for (int kx = 0; kx < m_kernel[0]; kx++){
                                    *(pw++) += a * acs3(X, u + kx - hsize, v + ky - hsize, kc);
                                }
                            }
                        }

                        // b
                        grd.b(oc, 0) += a;
                    }
                }
            }
            return grd;
        }

    };


    //--------------------------------------------------------------------------------
    // max pooling layer
    //--------------------------------------------------------------------------------

    class MaxPoolingLayer : public BaseLayer{

    private:

        // window size
        int m_winSize;

        // step distance
        int m_stride;

        // margin
        int m_margin;

        // output dsize
        int m_output[3];

        // kernel dsize
        int m_kernel[2];

        // forward id map
        Mem1<Mem<int> > m_fwrdMap;

    public:
        MaxPoolingLayer(){
            m_winSize = 0;
            m_stride = 0;
            m_margin = 0;
        }

        MaxPoolingLayer(const int winSize, const int stride, const int margin = 0){
            m_winSize = winSize;
            m_stride = stride;
            m_margin = margin;
        }

        virtual const char* getName(){
            return "MaxPoolingLayer";
        };

        SP_TEXTEX(){
            BaseLayer::textex(file);

            file.text(&m_winSize, 1, "winSize");
            file.text(&m_stride, 1, "stride");
            file.text(&m_margin, 1, "margin");
            file.text(m_output, 3, "output");
            file.text(m_kernel, 3, "kernel");
        }

    private:

        virtual void init(const Mem1<Mem<SP_REAL> > &X){

            // output dsize
            m_output[0] = (X[0].dsize[0] - 2 * m_margin) / m_stride;
            m_output[1] = (X[0].dsize[1] - 2 * m_margin) / m_stride;
            m_output[2] = maxval(X[0].dsize[2], 1);

            // kernel dsize
            m_kernel[0] = m_winSize;
            m_kernel[1] = m_winSize;

        }

        virtual void forward(Mem1<Mem<SP_REAL> > &Y, const Mem1<Mem<SP_REAL> > &X){
            m_fwrdMap.resize(Y.size());

#if SP_USE_OMP
#pragma omp parallel for
#endif
            for (int n = 0; n < Y.size(); n++){
                Y[n] = poolFwrd(X[n], m_fwrdMap[n]);
            }
        }

        virtual void backward(Mem1<Mem<SP_REAL> > &B, const Mem1<Mem<SP_REAL> > &A, const Mem1<Mem<SP_REAL> > &Y, const Mem1<Mem<SP_REAL> > &X){

#if SP_USE_OMP
#pragma omp parallel for
#endif
            for (int n = 0; n < B.size(); n++){
                B[n] = poolBwrd(A[n], X[n], m_fwrdMap[n]);
            }

        }

        Mem<SP_REAL>  poolFwrd(const Mem<SP_REAL>  &X, Mem<int> &fwrdMap){
            fwrdMap.resize(3, m_output);

            Mem3<SP_REAL> Y(m_output);

            const Rect2 rect = getRect2(X.dsize);
            for (int oc = 0; oc < m_output[2]; oc++){
                for (int ov = 0; ov < m_output[1]; ov++){
                    for (int ou = 0; ou < m_output[0]; ou++){

                        const int u = ou * m_stride + m_margin;
                        const int v = ov * m_stride + m_margin;

                        const int hsize = m_winSize / 2;

                        int &id = acs3(fwrdMap, ou, ov, oc);

                        SP_REAL maxv = -SP_INFINITY;

                        // max pooling forward
                        for (int ky = 0; ky < m_kernel[1]; ky++){
                            for (int kx = 0; kx < m_kernel[0]; kx++){
                                if (inRect(rect, u + kx - hsize, v + ky - hsize) == false) continue;

                                const SP_REAL val = acs3(X, u + kx - hsize, v + ky - hsize, oc);
                                if (val > maxv){
                                    maxv = val;
                                    id = acsid3(X.dsize, u + kx - hsize, v + ky - hsize, oc);
                                }
                            }
                        }
                        acs3(Y, ou, ov, oc) = maxv;
                    }
                }
            }
            return Y;
        }

        Mem<SP_REAL>  poolBwrd(const Mem<SP_REAL>  &A, const Mem<SP_REAL>  &X, const Mem<int> &fwrdMap){
            Mem<SP_REAL>  B(X.dim, X.dsize);
            B.zero();

            const Rect2 rect = getRect2(X.dsize);
            for (int oc = 0; oc < m_output[2]; oc++){
                for (int ov = 0; ov < m_output[1]; ov++){
                    for (int ou = 0; ou < m_output[0]; ou++){

                        const int id = acs3(fwrdMap, ou, ov, oc);

                        // max pooling backward
                        B[id] += acs3(A, ou, ov, oc);
                    }
                }
            }
            return B;
        }

    };


    //--------------------------------------------------------------------------------
    // drop out layer
    //--------------------------------------------------------------------------------

    class DropOutLayer : public BaseLayer{

    public:
        SP_REAL m_passRate;

        Mem1<Mem<char> > m_mask;
    public:

        DropOutLayer(const SP_REAL passRate = 0.5){
            m_passRate = passRate;
        }

        virtual const char* getName(){
            return "DropOutLayer";
        };

        SP_TEXTEX(){
            BaseLayer::textex(file);
            file.text(&m_passRate, 1, "passRate");
        }

    private:

        virtual void forward(Mem1<Mem<SP_REAL> > &Y, const Mem1<Mem<SP_REAL> > &X){
            if (m_train == true){
                m_mask.resize(Y.size());

                for (int n = 0; n < Y.size(); n++){
                    Y[n].resize(X[n].dim, X[n].dsize);
                    m_mask[n].resize(X[n].dim, X[n].dsize);

                    for (int i = 0; i < Y[n].size(); i++){
                        const char mask = (0.5 * (randu() + 1.0) < m_passRate) ? 1 : 0;

                        Y[n][i] = X[n][i] * mask;
                        m_mask[n][i] = mask;
                    }
                }
            }
            else{
                for (int n = 0; n < Y.size(); n++){
                    mulElm(Y[n], X[n], m_passRate);
                }
            }
        }

        virtual void backward(Mem1<Mem<SP_REAL> > &B, const Mem1<Mem<SP_REAL> > &A, const Mem1<Mem<SP_REAL> > &Y, const Mem1<Mem<SP_REAL> > &X){

            for (int n = 0; n < B.size(); n++){
                mulMem(B[n], A[n], m_mask[n]);
            }
        }

    };


    //--------------------------------------------------------------------------------
    // batch normalization layer
    //--------------------------------------------------------------------------------

    class BatchNormLayer : public ParamLayer{

    private:

        Mem1<Mem1<SP_REAL> > m_cX, m_nX;
        Mem1<SP_REAL> m_mean, m_var, m_std;

    public:

        BatchNormLayer(){
        }
        
        BatchNormLayer(const int nodeNum){
            m_nodeNum = nodeNum;
        }

        virtual const char* getName(){
            return "BatchNormLayer";
        };

        SP_TEXTEX(){
            ParamLayer::textex(file);
            file.text(&m_mean, 1, "mean");
            file.text(&m_var, 1, "var");
            file.text(&m_std, 1, "std");
        }

    private:

        virtual void init(const Mem1<Mem<SP_REAL> > &X){
            if (m_nodeNum == 0){
                m_nodeNum = X[0].size();
            }

            m_prm.resize(m_nodeNum, 1);
            for (int i = 0; i < m_nodeNum; i++){
                m_prm.w[i] = 1.0;
                m_prm.b[i] = 0.0;
            }

            m_mean.resize(m_nodeNum);
            m_var.resize(m_nodeNum);
            m_std.resize(m_nodeNum);
            m_mean.zero();
            m_var.zero();
            m_std.zero();

        }

        virtual void forward(Mem1<Mem<SP_REAL> > &Y, const Mem1<Mem<SP_REAL> > &X){

            m_cX.resize(m_nodeNum);
            m_nX.resize(m_nodeNum);

            Mat M;
            reshape(M, X);

            for (int r = 0; r < M.rows(); r++){
                Mem1<SP_REAL> bX(M.cols(), &M(r, 0));

                Mem1<SP_REAL> cX, nX;
                if (m_train == true){
                    const SP_REAL mean = meanVal(bX);
                    cX = bX - mean;

                    const double var = meanSq(cX);
                    const double std = sqrt(var + 10e-6);
                    nX = cX / std;

                    m_cX[r] = cX;
                    m_nX[r] = nX;

                    m_std[r] = std;

                    const double blend = 0.9;
                    m_mean[r] = blend * m_mean[r] + (1 - blend) * mean;
                    m_var[r] = blend * m_var[r] + (1 - blend) * var;
                }
                else{
                    cX = bX - m_mean[r];
                    nX = cX / sqrt(m_var[r] + 10e-6);
                }

                for (int c = 0; c < M.cols(); c++){
                    M(r, c) = m_prm.w[r] * nX[c] + m_prm.b[r];
                }
            }

            reshape(Y, M);

            for (int n = 0; n < Y.size(); n++){
                Y[n].resize(X[n].dim, X[n].dsize);
            }
        }

        virtual void backward(Mem1<Mem<SP_REAL> > &B, const Mem1<Mem<SP_REAL> > &A, const Mem1<Mem<SP_REAL> > &Y, const Mem1<Mem<SP_REAL> > &X){

            Mat M;
            reshape(M, A);

            NodeParam m_grd;
            m_grd.resize(m_nodeNum, 1);
            m_grd.zero();

            for (int r = 0; r < M.rows(); r++){
                Mem1<SP_REAL> bA(M.cols(), &M(r, 0));

                for (int c = 0; c < M.cols(); c++){
                    m_grd.w[r] += m_nX[r][c] * bA[c];
                    m_grd.b[r] += bA[c];
                }

                Mem1<SP_REAL> dnX(M.cols());
                Mem1<SP_REAL> dcX(M.cols());
                for (int c = 0; c < M.cols(); c++){
                    dnX[c] = m_prm.w[r] * bA[c];
                    dcX[c] = dnX[c] / m_std[r];
                }

                Mem1<SP_REAL> tmp(M.cols());
                for (int c = 0; c < M.cols(); c++){
                    tmp[c] = dnX[c] * m_cX[r][c] / (m_std[r] * m_std[r]);
                }
                const SP_REAL dstd = -sumVal(tmp);
                const SP_REAL dvar = 0.5 * dstd / m_std[r];

                for (int c = 0; c < M.cols(); c++){
                    dcX[c] += (2.0 / M.cols()) * m_cX[r][c] * dvar;
                }
                const SP_REAL dmean = sumVal(dcX);

                for (int c = 0; c < M.cols(); c++){
                    M(r, c) = dcX[c] - dmean / M.cols();
                }
            }

            // update
            update(m_grd);

            reshape(B, M);
        }

    private:

        void reshape(Mat &M, const Mem1<Mem<SP_REAL> > &D){
            const int batchNum = D.size();
            const int dataNum = D[0].size();

            M.resize(m_nodeNum, batchNum * dataNum / m_nodeNum);

            SP_REAL *m = M.ptr;
            for (int i = 0; i < dataNum; i++){
                for (int n = 0; n < batchNum; n++){
                    *m++ = D[n][i];
                }
            }
        }

        void reshape(Mem1<Mem<SP_REAL> > &R, const Mat &M){
            const int batchNum = R.size();
            const int dataNum = M.cols() * m_nodeNum / batchNum;
            
            for (int n = 0; n < batchNum; n++){
                const int dsize[1] = { dataNum };
                R[n].resize(1, dsize);
            }

            SP_REAL *m = M.ptr;
            for (int i = 0; i < dataNum; i++){
                for (int n = 0; n < batchNum; n++){
                    R[n][i] = *m++;
                }
            }
        }

    };
}
#endif
