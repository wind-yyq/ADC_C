// spline.h - 自然三次样条插值 (Natural Cubic Spline)
#pragma once
#include <vector>
#include <complex>

class CubicSpline {
public:
    // 从 (x,y) 数据点构建样条. x 必须严格递增.
    CubicSpline(const std::vector<double>& x, const std::vector<double>& y);

    // 在 xx 处求值 (标量)
    double eval(double xx) const;

    // 在 xx 处求值 (向量), 结果写入 out
    void eval_array(const double* xx, int n, double* out) const;

private:
    std::vector<double> x_;   // 节点
    std::vector<double> a_;   // y_i
    std::vector<double> b_;   // 一次系数
    std::vector<double> c_;   // 二次系数
    std::vector<double> d_;   // 三次系数
    int n_;                   // 节点数
};

// 复数样条: 对实部和虚部分别建样条
class ComplexSpline {
public:
    ComplexSpline(const std::vector<double>& t,
                  const std::vector<std::complex<double>>& x);

    std::complex<double> eval(double t) const;
    void eval_array(const double* t, int n, std::complex<double>* out) const;

private:
    CubicSpline real_spline_;
    CubicSpline imag_spline_;
};
