#include "vsim/driver/qp.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace vsim {

void QuadraticProgram::resize(std::size_t variables, std::size_t rows) {
  n = variables;
  m = rows;
  constexpr double inf = std::numeric_limits<double>::infinity();
  H.assign(n * n, 0.0);
  g.assign(n, 0.0);
  A.assign(m * n, 0.0);
  b.assign(m, 0.0);
  lb.assign(n, -inf);
  ub.assign(n, inf);
}

namespace {

// In-place Cholesky (lower triangle) of a symmetric matrix; false if not positive definite.
bool cholesky(std::vector<double>& M, std::size_t n) {
  for (std::size_t j = 0; j < n; ++j) {
    double d = M[j * n + j];
    for (std::size_t k = 0; k < j; ++k) {
      d -= M[j * n + k] * M[j * n + k];
    }
    if (!(d > 0.0)) {
      return false;
    }
    d = std::sqrt(d);
    M[j * n + j] = d;
    for (std::size_t i = j + 1; i < n; ++i) {
      double s = M[i * n + j];
      for (std::size_t k = 0; k < j; ++k) {
        s -= M[i * n + k] * M[j * n + k];
      }
      M[i * n + j] = s / d;
    }
  }
  return true;
}

void cholesky_solve(const std::vector<double>& L, std::size_t n, std::vector<double>& x) {
  for (std::size_t i = 0; i < n; ++i) {
    double s = x[i];
    for (std::size_t k = 0; k < i; ++k) {
      s -= L[i * n + k] * x[k];
    }
    x[i] = s / L[i * n + i];
  }
  for (std::size_t i = n; i-- > 0;) {
    double s = x[i];
    for (std::size_t k = i + 1; k < n; ++k) {
      s -= L[k * n + i] * x[k];
    }
    x[i] = s / L[i * n + i];
  }
}

// Largest step in (0, 1] keeping v + a dv > 0.
double max_step(const std::vector<double>& v, const std::vector<double>& dv, double limit) {
  double a = limit;
  for (std::size_t i = 0; i < v.size(); ++i) {
    if (dv[i] < 0.0) {
      a = std::min(a, -v[i] / dv[i]);
    }
  }
  return a;
}

double inf_norm(const std::vector<double>& v) {
  double r = 0.0;
  for (const double x : v) {
    r = std::max(r, std::abs(x));
  }
  return r;
}

}  // namespace

QpResult solve_qp(const QuadraticProgram& qp, const QpOptions& options) {
  const std::size_t n = qp.n, m = qp.m;
  QpResult result;

  // Nonzero span of each row.
  std::vector<std::size_t> lo(m, 0), hi(m, 0);
  for (std::size_t i = 0; i < m; ++i) {
    const double* row = &qp.A[i * n];
    std::size_t first = n, last = 0;
    for (std::size_t j = 0; j < n; ++j) {
      if (row[j] != 0.0) {
        first = std::min(first, j);
        last = j + 1;
      }
    }
    lo[i] = first < last ? first : 0;
    hi[i] = first < last ? last : 0;
  }
  std::vector<std::size_t> L, U;  // finite bounds
  for (std::size_t j = 0; j < n; ++j) {
    if (std::isfinite(qp.lb[j])) {
      L.push_back(j);
    }
    if (std::isfinite(qp.ub[j])) {
      U.push_back(j);
    }
  }
  const std::size_t nl = L.size(), nu = U.size();
  const double count = static_cast<double>(m + nl + nu);

  // Start inside the bounds, slacks and multipliers at 1.
  std::vector<double> z(n, 0.0);
  for (std::size_t j = 0; j < n; ++j) {
    const double l = qp.lb[j], u = qp.ub[j];
    if (std::isfinite(l) && std::isfinite(u)) {
      z[j] = std::clamp(0.0, l + 0.1 * (u - l), u - 0.1 * (u - l));
    } else if (std::isfinite(l)) {
      z[j] = std::max(0.0, l + 1.0);
    } else if (std::isfinite(u)) {
      z[j] = std::min(0.0, u - 1.0);
    }
  }
  std::vector<double> s(m), lam(m, 1.0), sl(nl), ml(nl, 1.0), su(nu), mu_u(nu, 1.0);
  for (std::size_t i = 0; i < m; ++i) {
    double az = 0.0;
    for (std::size_t j = lo[i]; j < hi[i]; ++j) {
      az += qp.A[i * n + j] * z[j];
    }
    s[i] = std::max(qp.b[i] - az, 1.0);
  }
  for (std::size_t k = 0; k < nl; ++k) {
    sl[k] = std::max(z[L[k]] - qp.lb[L[k]], 1e-2);
  }
  for (std::size_t k = 0; k < nu; ++k) {
    su[k] = std::max(qp.ub[U[k]] - z[U[k]], 1e-2);
  }

  const double scale_d = 1.0 + inf_norm(qp.g);
  const double scale_p = 1.0 + inf_norm(qp.b);
  std::vector<double> rd(n), rp(m), rl(nl), ru(nu), M(n * n), rhs(n), dz(n);
  std::vector<double> ds(m), dlam(m), dsl(nl), dml(nl), dsu(nu), dmu(nu);
  std::vector<double> cs(m), cl(nl), cu(nu), ds_aff(m), dlam_aff(m), dsl_aff(nl), dml_aff(nl), dsu_aff(nu), dmu_aff(nu);

  const auto residuals = [&]() {
    for (std::size_t i = 0; i < n; ++i) {
      double v = qp.g[i];
      const double* h = &qp.H[i * n];
      for (std::size_t j = 0; j < n; ++j) {
        v += h[j] * z[j];
      }
      rd[i] = v;
    }
    for (std::size_t i = 0; i < m; ++i) {
      double az = 0.0;
      for (std::size_t j = lo[i]; j < hi[i]; ++j) {
        az += qp.A[i * n + j] * z[j];
        rd[j] += qp.A[i * n + j] * lam[i];
      }
      rp[i] = az + s[i] - qp.b[i];
    }
    for (std::size_t k = 0; k < nl; ++k) {
      rd[L[k]] -= ml[k];
      rl[k] = z[L[k]] - qp.lb[L[k]] - sl[k];
    }
    for (std::size_t k = 0; k < nu; ++k) {
      rd[U[k]] += mu_u[k];
      ru[k] = qp.ub[U[k]] - z[U[k]] - su[k];
    }
  };
  // Direction for the complementarity targets cs, cl, cu (factor in M).
  const auto direction = [&]() {
    for (std::size_t j = 0; j < n; ++j) {
      rhs[j] = -rd[j];
    }
    for (std::size_t i = 0; i < m; ++i) {
      const double w = (-cs[i] + lam[i] * rp[i]) / s[i];
      for (std::size_t j = lo[i]; j < hi[i]; ++j) {
        rhs[j] -= qp.A[i * n + j] * w;
      }
    }
    for (std::size_t k = 0; k < nl; ++k) {
      rhs[L[k]] += (-cl[k] - ml[k] * rl[k]) / sl[k];
    }
    for (std::size_t k = 0; k < nu; ++k) {
      rhs[U[k]] -= (-cu[k] - mu_u[k] * ru[k]) / su[k];
    }
    dz = rhs;
    cholesky_solve(M, n, dz);
    for (std::size_t i = 0; i < m; ++i) {
      double adz = 0.0;
      for (std::size_t j = lo[i]; j < hi[i]; ++j) {
        adz += qp.A[i * n + j] * dz[j];
      }
      ds[i] = -rp[i] - adz;
      dlam[i] = (-cs[i] - lam[i] * ds[i]) / s[i];
    }
    for (std::size_t k = 0; k < nl; ++k) {
      dsl[k] = dz[L[k]] + rl[k];
      dml[k] = (-cl[k] - ml[k] * dsl[k]) / sl[k];
    }
    for (std::size_t k = 0; k < nu; ++k) {
      dsu[k] = -dz[U[k]] + ru[k];
      dmu[k] = (-cu[k] - mu_u[k] * dsu[k]) / su[k];
    }
  };
  const auto step_length = [&](double limit) {
    double a = limit;
    a = std::min(a, max_step(s, ds, a));
    a = std::min(a, max_step(lam, dlam, a));
    a = std::min(a, max_step(sl, dsl, a));
    a = std::min(a, max_step(ml, dml, a));
    a = std::min(a, max_step(su, dsu, a));
    a = std::min(a, max_step(mu_u, dmu, a));
    return a;
  };

  for (int it = 0; it < options.max_iterations; ++it) {
    residuals();
    double gap = 0.0;
    for (std::size_t i = 0; i < m; ++i) {
      gap += s[i] * lam[i];
    }
    for (std::size_t k = 0; k < nl; ++k) {
      gap += sl[k] * ml[k];
    }
    for (std::size_t k = 0; k < nu; ++k) {
      gap += su[k] * mu_u[k];
    }
    const double mu = count > 0.0 ? gap / count : 0.0;
    result.dual_residual = inf_norm(rd) / scale_d;
    result.primal_residual = std::max({inf_norm(rp), inf_norm(rl), inf_norm(ru)}) / scale_p;
    result.gap = mu;
    result.iterations = it;
    if (result.dual_residual <= options.tolerance && result.primal_residual <= options.tolerance &&
        mu <= options.tolerance) {
      result.converged = true;
      break;
    }

    // Normal equations: H + A' (lam / s) A + bound terms.
    M = qp.H;
    for (std::size_t i = 0; i < m; ++i) {
      const double d = lam[i] / s[i];
      const double* row = &qp.A[i * n];
      for (std::size_t j = lo[i]; j < hi[i]; ++j) {
        const double aj = d * row[j];
        if (aj == 0.0) {
          continue;
        }
        double* mj = &M[j * n];
        for (std::size_t k = lo[i]; k <= j; ++k) {
          mj[k] += aj * row[k];
        }
      }
    }
    for (std::size_t k = 0; k < nl; ++k) {
      M[L[k] * n + L[k]] += ml[k] / sl[k];
    }
    for (std::size_t k = 0; k < nu; ++k) {
      M[U[k] * n + U[k]] += mu_u[k] / su[k];
    }
    for (std::size_t j = 0; j < n; ++j) {  // mirror the lower triangle, regularise
      for (std::size_t k = 0; k < j; ++k) {
        M[k * n + j] = M[j * n + k];
      }
      M[j * n + j] += 1e-10;
    }
    if (!cholesky(M, n)) {
      // Rebuild with more regularisation (a singular H without bounds on some variables).
      result.converged = false;
      break;
    }

    // Predictor (affine scaling).
    for (std::size_t i = 0; i < m; ++i) {
      cs[i] = s[i] * lam[i];
    }
    for (std::size_t k = 0; k < nl; ++k) {
      cl[k] = sl[k] * ml[k];
    }
    for (std::size_t k = 0; k < nu; ++k) {
      cu[k] = su[k] * mu_u[k];
    }
    direction();
    const double a_aff = step_length(1.0);
    double gap_aff = 0.0;
    for (std::size_t i = 0; i < m; ++i) {
      gap_aff += (s[i] + a_aff * ds[i]) * (lam[i] + a_aff * dlam[i]);
    }
    for (std::size_t k = 0; k < nl; ++k) {
      gap_aff += (sl[k] + a_aff * dsl[k]) * (ml[k] + a_aff * dml[k]);
    }
    for (std::size_t k = 0; k < nu; ++k) {
      gap_aff += (su[k] + a_aff * dsu[k]) * (mu_u[k] + a_aff * dmu[k]);
    }
    const double mu_aff = count > 0.0 ? gap_aff / count : 0.0;
    const double sigma = mu > 0.0 ? std::pow(std::clamp(mu_aff / mu, 0.0, 1.0), 3.0) : 0.0;
    ds_aff = ds;
    dlam_aff = dlam;
    dsl_aff = dsl;
    dml_aff = dml;
    dsu_aff = dsu;
    dmu_aff = dmu;

    // Corrector with centring.
    for (std::size_t i = 0; i < m; ++i) {
      cs[i] = s[i] * lam[i] + ds_aff[i] * dlam_aff[i] - sigma * mu;
    }
    for (std::size_t k = 0; k < nl; ++k) {
      cl[k] = sl[k] * ml[k] + dsl_aff[k] * dml_aff[k] - sigma * mu;
    }
    for (std::size_t k = 0; k < nu; ++k) {
      cu[k] = su[k] * mu_u[k] + dsu_aff[k] * dmu_aff[k] - sigma * mu;
    }
    direction();
    const double a = 0.99 * step_length(1.0 / 0.99);
    for (std::size_t j = 0; j < n; ++j) {
      z[j] += a * dz[j];
    }
    for (std::size_t i = 0; i < m; ++i) {
      s[i] += a * ds[i];
      lam[i] += a * dlam[i];
    }
    for (std::size_t k = 0; k < nl; ++k) {
      sl[k] += a * dsl[k];
      ml[k] += a * dml[k];
    }
    for (std::size_t k = 0; k < nu; ++k) {
      su[k] += a * dsu[k];
      mu_u[k] += a * dmu[k];
    }
    result.iterations = it + 1;
  }

  double objective = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    double hz = 0.0;
    for (std::size_t j = 0; j < n; ++j) {
      hz += qp.H[i * n + j] * z[j];
    }
    objective += z[i] * (0.5 * hz + qp.g[i]);
  }
  result.objective = objective;
  result.z = std::move(z);
  return result;
}

}  // namespace vsim
