#include "simplesp.h"
using namespace sp;

#include <vector>
#include <complex>
#include <iostream>

typedef complex<double> complexf;

namespace sp{
 

/*!
* ニュートン・ラフソン法(Newton-Raphson method)
* @param[in] b 多項式の係数
* @param[in] n 方程式の次数
* @param[inout] x 探索開始位置を受け取り，解を返す
* @param[inout] max_iter 最大反復数(反復終了後,実際の反復数を返す)
* @param[inout] eps 許容誤差(反復終了後,実際の誤差を返す)
* @return 1:成功,0:失敗
*/
int newton(const vector<double> &b, int n, double &x, int &max_iter, double &eps)
{
    double f, df, dx = 0.0;

    int k;
    for (k = 0; k < max_iter; ++k) {
        f = sp::funcX(x, n + 1, &b[0]);
        double df = dfuncX(x, n + 1, &b[0]);

        x = x - f / df;

        // 収束判定
        dx = ::fabs(f / df);
        if (dx < eps || ::fabs(f) < eps) {
            max_iter = k; eps = dx;
            return 1;
        }
    }

    max_iter = k; eps = dx;

    return 0;
}

/*!
* ホーナー法(組立除法)の係数の値を求める
* @param[in] a 代数方程式の係数
* @param[out] b x=x0のときの組立除法の係数
* @param[in] x0 係数を計算するxの値
*/
template<class T>
inline void horner2(const vector<T> &a, vector<T> &b, int n, T x0)
{
    if (n <= 2) return;

    b = a;
    for (int i = 0; i <= n; ++i) {
        for (int j = 1; j <= n - i; ++j) {
            b[j] += x0 * b[j - 1];
        }
    }
}
inline void horner(const vector<Cmp> &a, vector<Cmp> &b, int n, Cmp x0)
{
    if (n <= 2) return;

    b = a;
    for (int i = 0; i <= n; ++i) {
        for (int j = 1; j <= n - i; ++j) {
            b[j] += x0 * b[j - 1];
        }
    }
}


/*!
* Aberthの方法で初期値を算出
* @param[out] z 解探索のための初期値
* @param[in] c 多項式の係数
* @param[in] n 方程式の次数
* @param[in] max_iter 半径計算のための最大反復数
* @param[in] eps 半径計算のための許容誤差
* @return 1:成功,0:失敗
*/
int aberth2(vector<complexf> &z, const vector<complexf> &c, int n, int max_iter, double eps)
{
    // 半径算出のための方程式の係数
    vector<complexf> a;
    complexf zc = -c[1] / (c[0] * (double)n);
    horner2(c, a, n, zc);

    vector<double> b(a.size());
    b[0] = abs(a[0]);
    for (int i = 1; i <= n; ++i) {
        b[i] = -abs(a[i]);
    }

    // Aberthの初期値の半径をニュートン法で算出
    double r = 100.0;
    newton(b, n, r, max_iter, eps);
    //cout << "r = " << r << endl;

    // Aberthの初期値
    for (int j = 0; j < n; ++j) {
        double theta = (2 * SP_PI / (double)n)*j + SP_PI / (2.0*n);
        z[j] = zc + r * complexf(cos(theta), sin(theta));
    }

    return 1;
}
int aberth(vector<Cmp> &z, const vector<double> &c, int n, int max_iter, double eps)
{
    //// 半径算出のための方程式の係数
    //vector<Cmp> a;
    //Cmp zc = -getCmp(c[1], 0.0) / (c[0] * (double)n) * -1.0;

    //horner(c, a, n, zc);

    //vector<double> b(a.size());
    //b[0] = abs(a[0]);
    //for (int i = 1; i <= n; ++i) {
    //    b[i] = -abs(a[i]);
    //}

    //// Aberthの初期値の半径をニュートン法で算出
    //double r = 100.0;
    //newton(b, n, r, max_iter, eps);
    ////cout << "r = " << r << endl;

    //// Aberthの初期値
    //for (int j = 0; j < n; ++j) {
    //    double theta = (2 * SP_PI / (double)n)*j + SP_PI / (2.0*n);
    //    z[j] = zc + r * complexf(cos(theta), sin(theta));
    //}

    return 1;
}

/*!
* ワイヤストラス法(DK公式)
* @param[inout] z 初期値位置を受け取り，解を返す
* @param[in] c 多項式の係数
* @param[in] n 方程式の次数
* @param[inout] max_iter 最大反復数(反復終了後,実際の反復数を返す)
* @param[inout] eps 許容誤差(反復終了後,実際の誤差を返す)
* @return 1:成功,0:失敗
*/
int weierstrass(vector<complexf> &z, const vector<complexf> &c, int n, int &max_iter, double &eps)
{
    double e = 0.0, ej;

    vector<double> cc;
    for (int i = 0; i < c.size(); i++) {
        cc.push_back(c[i].real());
    }


    vector<complexf> zp;
    complexf f, df;
    int k;
    for (k = 0; k < max_iter; ++k) {
        zp = z;

        // DK式の計算
        for (int j = 0; j < n; ++j) {
            Cmp ff = funcX(getCmp(z[j].real(), z[j].imag()), n + 1, &cc[0]);
            df = c[0];
            for (int i = 0; i < n; ++i) {
                if (i != j) {
                    df *= zp[j] - zp[i];
                }
            }

            z[j] = zp[j] - complexf(ff.re, ff.im) / df;
        }

        // 誤差の算出
        e = 0.0;
        for (int j = 0; j < n; ++j) {
            ej = fabs(funcX(getCmp(z[j].real(), z[j].imag()), n + 1, &cc[0]));
            if (ej > e) {
                e = ej;
            }
        }

        // 収束判定
        if (e < eps) {
            max_iter = k; eps = e;
            return 1;
        }
    }

    eps = e;
    return 0;
}

int eqn2(Cmp xs[], const double nn, const double cs[]) {
    int max_iter = 100;
    double eps = 1e-8;
    vector<complexf> c;

    c.push_back(complex<double>(1));
    c.push_back(complex<double>(2));
    c.push_back(complex<double>(3));
    c.push_back(complex<double>(4));
    c.push_back(complex<double>(5));

    const int n = c.size() - 1;

    // Aberthの初期値
    vector<complexf> z(n);
    aberth2(z, c, n, max_iter, eps);

    //// ワイヤストラス法(DK公式)
    //weierstrass(z, c, n, max_iter, eps);

    //cout << "solutions : " << endl;
    for (int i = 0; i < z.size(); i++){
        cout << "  " << z[i] << endl;
    }
    cout << endl;
    cout << "iter = " << max_iter << ", eps = " << eps << endl;

    return 0;
}
int eqn(Cmp xs[], const double nn, const double cs[]) {
    int max_iter = 100;
    double eps = 1e-8;
    vector<double> c;

    c.push_back((1));
    c.push_back((2));
    c.push_back((3));
    c.push_back((4));
    c.push_back((5));

    const int n = c.size() - 1;

    // Aberthの初期値
    vector<Cmp> z(n);
    //aberth(z, c, n, max_iter, eps);

    return 0;
}
}

int main(){

    print(+getCmp(1, 1));

    Cmp xs[4];

    const int num = eq4(xs, 1, 2, 3, 4, 5);

    for (int i = 0; i < num; i++) {
        print(xs[i]);
    }

    {
        eqn(xs, 5, 0);
        eqn2(xs, 5, 0);
    }


    return 0;
}
