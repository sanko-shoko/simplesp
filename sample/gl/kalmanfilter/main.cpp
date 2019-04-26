#include "simplesp.h"
#include "spex/spgl.h"

using namespace sp;

class KalmanfilterGUI : public BaseWindow {

private:

    void help() {
        printf("'ESC' key : exit\n");
        printf("\n");
    }

    virtual void init() {

        help();
    }

    virtual void display() {

        Mem2<Col3> img(320, 240);

        static int cnt = 0;

        // delta time
        const double dt = 0.1;

        // observation noise (sigma)
        const double onoize = 5.0;

        // system noize (sigma)
        const double snoize = 0.1;

        // observation 
        Mat Z(2, 1);
        {
            // circle radius
            const double radius = 100.0;

            const double time = cnt * dt;

            const double angle = time * SP_PI / 180.0;
            const Vec2 pos = getVec2(::cos(angle), ::sin(angle)) * radius + getVec2(img.dsize[0] - 1, img.dsize[1] - 1) * 0.5;

            Z(0, 0) = pos.x + randValGauss() * onoize;
            Z(1, 0) = pos.y + randValGauss() * onoize;
        }

        static Mat X, Q, P, R, F, H;
        if (cnt == 0) {
            // initialize state

            // X (position 2d & velocity 2d)
            {
                double m[]{
                    Z(0, 0),
                    Z(1, 0),
                    0.0,
                    0.0
                };
                X.resize(4, 1, m);
            }

            // Q (system noize covariance)
            {
                const double a = square(dt * dt / 2.0);
                const double b = (dt * dt / 2.0) * dt;
                const double c = dt * dt;

                double m[]{
                    a, 0.0, b, 0.0,
                    0.0, a, 0.0, b,
                    b, 0.0, c, 0.0,
                    0.0, b, 0.0, c
                };
                Q.resize(4, 4, m);

                Q *= square(snoize);
            }

            // P (state covariance)
            P = Q;

            // R (observation noize covariance)
            {
                double m[]{
                    1.0, 0.0,
                    0.0, 1.0
                };
                R.resize(2, 2, m);
                R *= square(onoize);
            }

            // F (prediction matrix)
            {
                double m[]{
                    1.0, 0.0, dt, 0.0,
                    0.0, 1.0, 0.0, dt,
                    0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 1.0
                };

                F.resize(4, 4, m);
            }

            // H (observation matrix)
            {
                double m[]{
                    1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                };

                H.resize(2, 4, m);
            }
        }
        else {

            // predict
            X = F * X;
            P = F * P * trnMat(F) + Q;

            // kalman gain
            const Mat K = P * trnMat(H) * invMat(H * P * trnMat(H) + R);

            // update
            X = X + K * (Z - H * X);
            P = (eyeMat(4, 4) - K * H) * P;

        }

        setElm(img, getCol3(255, 255, 255));
        renderPoint(img, getVec2(Z[0], Z[1]), getCol3(0, 0, 0), 3);
        renderPoint(img, getVec2(X[0], X[1]), getCol3(0, 0, 255), 5);

        glLoadView2D(img.dsize[0], img.dsize[1], m_viewPos, m_viewScale);
        glTexImg(img);

        //char path[256];
        //sprintf(path, "img%03d.bmp", cnt);
        //saveBMP(img, path);

        cnt++;
    }
};


int main(){

    KalmanfilterGUI win;
    win.execute("kalmanfilter", 800, 600);

    return 0;
}