//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_KALMANFILTER_H__
#define __SP_KALMANFILTER_H__

#include "spcore/spcore.h"


namespace sp{

    class KalmanFilter {

    private:

        Mat m_X;
        Mat m_P;
        Mat m_Q, m_R;

    public:

        void init(const Mat &X, const Mat &P, const Mat &Q, const Mat &R) {
            m_X = X;
            m_P = P;
            m_Q = Q;
            m_R = R;
        }

        const void setX(const Mat &X) {
            m_X = X;
        }

        const void setP(const Mat &P) {
            m_P = P;
        }

        const Mat& getX() {
            return m_X;
        }

        const Mat& getP() {
            return m_P;
        }

        void execute(const Mat &Z, const Mat &F, const Mat &H) {

            // predict
            m_X = F * m_X;
            m_P = F * m_P * trnMat(F) + m_Q;

            // kalman gain
            const Mat K = m_P * trnMat(H) * invMat(H * m_P * trnMat(H) + m_R);

            // update
            m_X = m_X + K * (Z - H * m_X);
            m_P = (eyeMat(m_P.dsize[0], m_P.dsize[1]) - K * H) * m_P;
        }

        void execute(const Mat &Z, const Mat &F, const Mat &H, const Mat &Y) {

            // predict
            m_X = F * m_X;
            m_P = F * m_P * trnMat(F) + m_Q;

            // kalman gain
            const Mat K = m_P * trnMat(H) * invMat(H * m_P * trnMat(H) + m_R);

            // update
            m_X = m_X + K * (Z - Y);
            m_P = (eyeMat(m_P.dsize[0], m_P.dsize[1]) - K * H) * m_P;
        }

    };


}

#endif