// spline.cpp - 自然三次样条实现
#include "spline.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

// ===================== CubicSpline: 自然三次样条 =====================

CubicSpline::CubicSpline(const std::vector<double>& x,
                         const std::vector<double>& y)
    : x_(x), n_(static_cast<int>(x.size()))
{
    if (n_ < 2)
        throw std::runtime_error("Spline needs at least 2 points");
    a_.resize(n_); b_.resize(n_); c_.resize(n_); d_.resize(n_);

    std::vector<double> h(n_ - 1);
    for (int i = 0; i < n_ - 1; ++i) {
        h[i]  = x_[i + 1] - x_[i];
        a_[i] = y[i];
    }
    a_[n_ - 1] = y[n_ - 1];

    // Thomas 算法解三对角系统求二阶导数 c
    std::vector<double> alpha(n_ - 1);
    for (int i = 1; i < n_ - 1; ++i)
        alpha[i] = 3.0 / h[i] * (a_[i + 1] - a_[i])
                 - 3.0 / h[i - 1] * (a_[i] - a_[i - 1]);

    std::vector<double> l(n_), mu(n_), z(n_);
    l[0] = 1.0; mu[0] = 0.0; z[0] = 0.0;
    for (int i = 1; i < n_ - 1; ++i) {
        l[i]  = 2.0 * (x_[i + 1] - x_[i - 1]) - h[i - 1] * mu[i - 1];
        mu[i] = h[i] / l[i];
        z[i]  = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }
    l[n_ - 1] = 1.0; z[n_ - 1] = 0.0; c_[n_ - 1] = 0.0;

    for (int j = n_ - 2; j >= 0; --j) {
        c_[j] = z[j] - mu[j] * c_[j + 1];
        b_[j] = (a_[j + 1] - a_[j]) / h[j]
                - h[j] * (c_[j + 1] + 2.0 * c_[j]) / 3.0;
        d_[j] = (c_[j + 1] - c_[j]) / (3.0 * h[j]);
    }
}

double CubicSpline::eval(double xx) const
{
    auto it = std::upper_bound(x_.begin(), x_.end(), xx);
    int i = static_cast<int>(it - x_.begin()) - 1;
    if (i < 0) i = 0;
    if (i >= n_ - 1) i = n_ - 2;
    double dx = xx - x_[i];
    return a_[i] + dx * (b_[i] + dx * (c_[i] + dx * d_[i]));
}

void CubicSpline::eval_array(const double* xx, int n, double* out) const
{
    for (int k = 0; k < n; ++k)
        out[k] = eval(xx[k]);
}

// ===================== ComplexSpline: 复信号样条 =====================

namespace {

std::vector<double> get_real(const std::vector<std::complex<double>>& x)
{
    std::vector<double> r(x.size());
    for (size_t i = 0; i < x.size(); ++i) r[i] = x[i].real();
    return r;
}
std::vector<double> get_imag(const std::vector<std::complex<double>>& x)
{
    std::vector<double> r(x.size());
    for (size_t i = 0; i < x.size(); ++i) r[i] = x[i].imag();
    return r;
}

} // namespace

ComplexSpline::ComplexSpline(const std::vector<double>& t,
                             const std::vector<std::complex<double>>& x)
    : real_spline_(t, get_real(x)), imag_spline_(t, get_imag(x))
{}

std::complex<double> ComplexSpline::eval(double t) const
{
    return {real_spline_.eval(t), imag_spline_.eval(t)};
}

void ComplexSpline::eval_array(const double* t, int n,
                               std::complex<double>* out) const
{
    for (int k = 0; k < n; ++k)
        out[k] = eval(t[k]);
}
