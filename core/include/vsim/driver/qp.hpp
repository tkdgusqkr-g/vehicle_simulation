#pragma once

#include <cstddef>
#include <vector>

namespace vsim {

// Convex quadratic program for the MPC driver (design doc §9.5):
//
//   minimize    1/2 z' H z + g' z
//   subject to  A z <= b         (general rows)
//               lb <= z <= ub    (bounds; +-infinity for none)
//
// Dense storage, row-major. H must be symmetric positive semidefinite.
struct QuadraticProgram {
  std::size_t n = 0;  // variables
  std::size_t m = 0;  // general rows
  std::vector<double> H, g;   // n x n, n
  std::vector<double> A, b;   // m x n, m
  std::vector<double> lb, ub; // n

  void resize(std::size_t variables, std::size_t rows);
  double& h(std::size_t i, std::size_t j) { return H[i * n + j]; }
  double& a(std::size_t row, std::size_t j) { return A[row * n + j]; }
};

struct QpResult {
  std::vector<double> z;
  int iterations = 0;
  bool converged = false;
  double objective = 0.0;
  double primal_residual = 0.0, dual_residual = 0.0, gap = 0.0;
};

struct QpOptions {
  int max_iterations = 40;
  double tolerance = 1e-7;  // scaled residuals and complementarity
};

// Primal-dual interior point (Mehrotra predictor-corrector) on the normal
// equations H + A' D A + bound terms, Cholesky-factored each iteration.
// Rows keep track of their nonzero span, so the banded rows of a condensed
// MPC cost less. Deterministic: a fixed sequence of floating-point operations.
QpResult solve_qp(const QuadraticProgram& qp, const QpOptions& options = {});

}  // namespace vsim
