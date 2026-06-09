// ADC_model1.cpp — ADC 核心算法实现（AdcModel1 类）
#include "ADC_model1.h"
#include "spline.h"
#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ===================== 内部辅助函数 =====================

double AdcModel1::deg2rad(double deg) { return deg * M_PI / 180.0; }

int AdcModel1::clamp_int(int x, int lo, int hi)
{
	if (x < lo)
		return lo;
	if (x > hi)
		return hi;
	return x;
}

double AdcModel1::clamp_double(double x, double lo, double hi)
{
	if (x < lo)
		return lo;
	if (x > hi)
		return hi;
	return x;
}

int AdcModel1::quantize_one(double x, double VRef, double LSB,
							int levels, int NBits, bool twos_complement)
{
	double xc = clamp_double(x, -VRef, VRef);
	int idx = static_cast<int>(std::floor((xc + VRef) / LSB));
	idx = clamp_int(idx, 0, levels - 1);
	if (twos_complement)
		return idx - (1 << (NBits - 1));
	return idx;
}

// ===================== Catmull-Rom 插值（PCHIP 等价）=====================

double AdcModel1::catmull_rom_1d(const std::vector<double> &x,
								 const std::vector<double> &y, double xx,
								 int &hint)
{
	int n = static_cast<int>(x.size());
	// 滑动窗口：从上次位置向后搜，均摊 O(1)
	int i = hint;
	while (i + 1 < n && x[i + 1] <= xx) ++i;
	if (i >= n - 1) i = n - 2;
	if (i < 0) i = 0;
	hint = i;

	int i0 = std::max(0, i - 1);
	int i1 = i;
	int i2 = std::min(n - 1, i + 1);
	int i3 = std::min(n - 1, i + 2);

	double t = (xx - x[i1]) / (x[i2] - x[i1]);
	double t2 = t * t, t3 = t2 * t;

	double m0 = (y[i2] - y[i0]) / (x[i2] - x[i0]);
	double m1 = (y[i3] - y[i1]) / (x[i3] - x[i1]);

	double h00 = 2 * t3 - 3 * t2 + 1;
	double h10 = t3 - 2 * t2 + t;
	double h01 = -2 * t3 + 3 * t2;
	double h11 = t3 - t2;

	double dx = x[i2] - x[i1];
	return h00 * y[i1] + h10 * dx * m0 + h01 * y[i2] + h11 * dx * m1;
}

void AdcModel1::interp_catmull_rom(const std::vector<double> &t,
								   const std::vector<std::complex<double>> &x,
								   const std::vector<double> &t_samp,
								   std::vector<std::complex<double>> &out)
{
	int n = static_cast<int>(t.size());
	std::vector<double> xr(n), xi(n);
	for (int i = 0; i < n; ++i)
	{
		xr[i] = x[i].real();
		xi[i] = x[i].imag();
	}
	int M = static_cast<int>(t_samp.size());
	int hint_r = 0, hint_i = 0;
	for (int k = 0; k < M; ++k)
		out[k] = {catmull_rom_1d(t, xr, t_samp[k], hint_r),
				  catmull_rom_1d(t, xi, t_samp[k], hint_i)};
}

// ===================== 升余弦滤波器 =====================

void AdcModel1::apply_raised_cosine(std::vector<std::complex<double>> &y,
									double beta, double sps, bool is_complex)
{
	const int span = 20;
	int Nf = static_cast<int>(sps * span);
	std::vector<double> h(Nf + 1);
	double sum_h = 0.0;
	for (int i = 0; i <= Nf; ++i)
	{
		double t = (i - Nf / 2.0) / sps;
		if (std::abs(t) < 1e-12)
			h[i] = 1.0;
		else if (std::abs(std::abs(4.0 * beta * t) - 1.0) < 1e-12)
			h[i] = beta / 2.0 * std::sin(M_PI / (2.0 * beta));
		else
			h[i] = std::sin(M_PI * t) / (M_PI * t) * std::cos(M_PI * beta * t) / (1.0 - 16.0 * beta * beta * t * t);
		sum_h += h[i];
	}
	for (int i = 0; i <= Nf; ++i)
		h[i] /= sum_h;

	int ny = static_cast<int>(y.size());
	std::vector<double> yr_in(ny), yi_in(ny), yr_out(ny, 0.0), yi_out(ny, 0.0);
	if (is_complex)
	{
		for (int i = 0; i < ny; ++i)
		{
			yr_in[i] = y[i].real();
			yi_in[i] = y[i].imag();
		}
		for (int i = 0; i < ny; ++i)
		{
			for (int j = 0; j <= Nf; ++j)
			{
				int k = i - j + Nf / 2;
				if (k >= 0 && k < ny)
				{
					yr_out[i] += yr_in[k] * h[j];
					yi_out[i] += yi_in[k] * h[j];
				}
			}
		}
		for (int i = 0; i < ny; ++i)
			y[i] = {yr_out[i], yi_out[i]};
	}
	else
	{
		for (int i = 0; i < ny; ++i)
			yr_in[i] = y[i].real();
		for (int i = 0; i < ny; ++i)
		{
			for (int j = 0; j <= Nf; ++j)
			{
				int k = i - j + Nf / 2;
				if (k >= 0 && k < ny)
					yr_out[i] += yr_in[k] * h[j];
			}
		}
		for (int i = 0; i < ny; ++i)
			y[i] = {yr_out[i], 0.0};
	}
}

// ===================== Process（主入口）=====================

AdcOutput AdcModel1::Process(const std::vector<std::complex<double>> &x_in,
							 double Ts, double t_start,
							 const AdcParams &params)
{
	int N = static_cast<int>(x_in.size());

	// 生成时间向量
	std::vector<double> t(N);
	for (int i = 0; i < N; ++i)
		t[i] = t_start + static_cast<double>(i) * Ts;

	// 降采样模式 + 抗混叠：才拷贝 + 滤波；否则零拷贝引用
	bool need_filter = (params.conversion_type == Downsampled && params.anti_aliasing_filter == AA_ON);
	if (need_filter)
	{
		std::vector<std::complex<double>> x_filt(x_in);
		double factor = static_cast<double>(params.downsample_factor);
		apply_raised_cosine(x_filt, params.excess_bw, factor, true);
		return ProcessDownsampled(t, x_filt, params);
	}
	else if (params.conversion_type == Downsampled)
		return ProcessDownsampled(t, x_in, params);
	else
		return ProcessClocked(t, x_in, params);
}

// ===================== 时钟采样模式 =====================

AdcOutput AdcModel1::ProcessClocked(const std::vector<double> &t,
									const std::vector<std::complex<double>> &x,
									const AdcParams &params)
{
	int N = static_cast<int>(t.size());
	double Clock = params.clock;
	double Phase = params.phase;
	double SR = params.sr;
	double phi = deg2rad(Phase);
	double VRef = params.v_ref;
	int NBits = params.n_bits;
	int levels = 1 << NBits;
	double LSB = 2.0 * VRef / levels;
	bool twos_comp = (params.output_digital_format == TwosComplement);

	AdcOutput out;
	out.d_i.assign(N, 0);
	out.d_q.assign(N, 0);
	out.a_out.resize(N);
	out.t_out.assign(N, 0.0);

	int last_DI = params.init_DI;
	int last_DQ = params.init_DQ;
	for (int i = 0; i < N; ++i)
	{
		out.d_i[i] = last_DI;
		out.d_q[i] = last_DQ;
		out.t_out[i] = t[i];
	}


	// ---- 预分配可复用向量（避免 chunk 循环内反复分配）----
	std::vector<double> t_samp;
	std::vector<double> t_ov;
	std::vector<std::complex<double>> x_ov;
	std::vector<std::complex<double>> x_samp;
	std::vector<int> DI_blk;
	std::vector<int> DQ_blk;
	std::vector<double> edges;

	t_samp.reserve(static_cast<size_t>(N * Clock / SR) + 100);

	// 分块处理
	const int chunk_size = 100000;
	int chunk_start = 0;

	while (chunk_start < N)
	{
		int chunk_end = std::min(N, chunk_start + chunk_size);

		int pad = std::min(5, chunk_size);
		int ov_start = std::max(0, chunk_start - pad);
		int ov_end = std::min(N, chunk_end + pad);
		int ov_len = ov_end - ov_start;

		// 查找时钟采样点
		t_samp.clear();
		double T_clk = 1.0 / Clock;
		double t0 = (1.5 * M_PI - phi) / (2.0 * M_PI * Clock);
		if (t0 < 0.0)
			t0 += T_clk;
		int k1 = static_cast<int>(std::ceil((t[chunk_start] - t0) / T_clk));
		int k2 = static_cast<int>(std::floor((t[chunk_end - 1] - t0) / T_clk));
		if (k1 < 0)
			k1 = 0;
		for (int k = k1; k <= k2; ++k)
			t_samp.push_back(t0 + k * T_clk);

		if (!t_samp.empty() && ov_len >= 2)
		{
			int M = static_cast<int>(t_samp.size());

			t_ov.resize(ov_len);
			x_ov.resize(ov_len);
			for (int i = 0; i < ov_len; ++i)
			{
				t_ov[i] = t[ov_start + i];
				x_ov[i] = x[ov_start + i];
			}

			// 插值
			x_samp.resize(M);
			if (params.interp_method == Linear || ov_len < 4)
			{
				int hint = 0;  // 滑动窗口
				for (int k = 0; k < M; ++k)
				{
					while (hint + 2 < ov_len && t_ov[hint + 1] <= t_samp[k]) ++hint;
					int i = hint;
					if (i >= ov_len - 1) i = ov_len - 2;
					if (i >= ov_len - 1)
						i = ov_len - 2;
					double frac = (t_samp[k] - t_ov[i]) / (t_ov[i + 1] - t_ov[i]);
					x_samp[k] = x_ov[i] + frac * (x_ov[i + 1] - x_ov[i]);
				}
			}
			else if (params.interp_method == PCHIP)
			{
				interp_catmull_rom(t_ov, x_ov, t_samp, x_samp);
			}
			else
			{
				ComplexSpline cs(t_ov, x_ov);
				cs.eval_array(t_samp.data(), M, x_samp.data());
			}

			// 量化（内联 + 独立限幅，避免 std::abs/sqrt）
			DI_blk.resize(M);
			DQ_blk.resize(M);
			{
				double inv_LSB = 1.0 / LSB;
				int twos_offset = twos_comp ? (1 << (NBits - 1)) : 0;
				double negV = -VRef;
				double posV = VRef;
				int lev1 = levels - 1;
				for (int k = 0; k < M; ++k)
				{
					// I 路：独立限幅（不用 std::abs）
					double xr = x_samp[k].real();
					xr = (xr < negV) ? negV : ((xr > posV) ? posV : xr);
					int idx_r = static_cast<int>(std::floor((xr + posV) * inv_LSB));
					idx_r = (idx_r < 0) ? 0 : ((idx_r >= levels) ? lev1 : idx_r);
					DI_blk[k] = idx_r - twos_offset;

					// Q 路
					double xi = x_samp[k].imag();
					xi = (xi < negV) ? negV : ((xi > posV) ? posV : xi);
					int idx_i = static_cast<int>(std::floor((xi + posV) * inv_LSB));
					idx_i = (idx_i < 0) ? 0 : ((idx_i >= levels) ? lev1 : idx_i);
					DQ_blk[k] = idx_i - twos_offset;
				}
			}

			// ZOH 映射
			edges.resize(M);
			if (params.zoh_mode == Discretize)
			{
				for (int k = 0; k < M; ++k)
					edges[k] = t_samp[k] + 1.0 / SR;
			}
			else
			{
				for (int k = 0; k < M; ++k)
				{
					double pos = t_samp[k] * SR;  // 全局样本索引（避免跨块 ceil 偏差）
					edges[k] = static_cast<int>(std::ceil(pos + 2.0 - 1e-9));
				}
			}

			// 填入输出
			int edge_hint = 0;  // 滑动窗口
			if (params.zoh_mode == Discretize)
			{
				for (int j = chunk_start; j < chunk_end; ++j)
				{
					while (edge_hint < M && edges[edge_hint] <= t[j]) ++edge_hint;
					int bin = edge_hint;
					if (bin == 0)
					{
						out.d_i[j] = last_DI;
						out.d_q[j] = last_DQ;
					}
					else
					{
						out.d_i[j] = DI_blk[bin - 1];
						out.d_q[j] = DQ_blk[bin - 1];
					}
				}
			}
			else
			{
				int int_hint = 0;
				int global_offset = static_cast<int>(std::round(t[0] * SR));
				for (int j = chunk_start; j < chunk_end; ++j)
				{
					double target = static_cast<double>(global_offset + j + 1);
					while (int_hint < M && edges[int_hint] <= target) ++int_hint;
					int bin = int_hint;
					if (bin == 0)
					{
						out.d_i[j] = last_DI;
						out.d_q[j] = last_DQ;
					}
					else
					{
						out.d_i[j] = DI_blk[bin - 1];
						out.d_q[j] = DQ_blk[bin - 1];
					}
				}
			}
		}

		last_DI = out.d_i[chunk_end - 1];
		last_DQ = out.d_q[chunk_end - 1];

		chunk_start = chunk_end;
	}

	// 计算模拟输出幅度
	if (twos_comp)
	{
		double scale = VRef / (1 << (NBits - 1));
		for (int i = 0; i < N; ++i)
		{
			double ai = (out.d_i[i] + 0.5) * scale;
			double aq = (out.d_q[i] + 0.5) * scale;
			out.a_out[i] = {ai, aq};
		}
	}
	else
	{
		for (int i = 0; i < N; ++i)
		{
			double ai = -VRef + (out.d_i[i] + 0.5) * LSB;
			double aq = -VRef + (out.d_q[i] + 0.5) * LSB;
			out.a_out[i] = {ai, aq};
		}
	}

	return out;
}

// ===================== 降采样模式 =====================

AdcOutput AdcModel1::ProcessDownsampled(const std::vector<double> &t,
										const std::vector<std::complex<double>> &x,
										const AdcParams &params)
{
	int N = static_cast<int>(t.size());
	int Factor = std::max(1, params.downsample_factor);
	int ds_phase = std::max(0, std::min(Factor - 1, params.downsample_phase));
	double VRef = params.v_ref;
	int NBits = params.n_bits;
	int levels = 1 << NBits;
	double LSB = 2.0 * VRef / levels;
	bool twos_comp = (params.output_digital_format == TwosComplement);

	AdcOutput out;
	out.d_i.resize(N);
	out.d_q.resize(N);
	out.a_out.resize(N);
	out.t_out.resize(N);

	int last_DI = params.init_DI;
	int last_DQ = params.init_DQ;
	std::complex<double> last_A = 0.0;

	int cnt = 0;
	for (int n = 0; n < N; ++n)
	{
		out.d_i[n] = last_DI;
		out.d_q[n] = last_DQ;
		out.a_out[n] = last_A;
		++cnt;
		if (cnt == Factor)
		{
			int src = n - Factor + 1 + ds_phase;
			if (src < 0)
				src = 0;
			if (src >= N)
				src = N - 1;
			auto xs = x[src];

			// 内联量化（独立限幅，避免 std::abs/sqrt）
			double inv_LSB_ds = 1.0 / LSB;
			int twos_off_ds = twos_comp ? (1 << (NBits - 1)) : 0;
			double xr = xs.real();
			xr = (xr < -VRef) ? -VRef : ((xr > VRef) ? VRef : xr);
			last_DI = static_cast<int>(std::floor((xr + VRef) * inv_LSB_ds)) - twos_off_ds;

			double xi = xs.imag();
			xi = (xi < -VRef) ? -VRef : ((xi > VRef) ? VRef : xi);
			last_DQ = static_cast<int>(std::floor((xi + VRef) * inv_LSB_ds)) - twos_off_ds;

			double ai = -VRef + (last_DI + 0.5) * LSB;
			double aq = -VRef + (last_DQ + 0.5) * LSB;
			last_A = {ai, aq};

			out.d_i[n] = last_DI;
			out.d_q[n] = last_DQ;
			out.a_out[n] = last_A;
			out.t_out[n] = t[src];
			cnt = 0;
		}
	}

	return out;
}
