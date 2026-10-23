#include "vsim/elec/circuit.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace vsim::elec {

void Assembler::reset(std::size_t n, int iteration) {
  n_ = n;
  iteration_ = iteration;
  changed_ = false;
  g_.assign(n * n, 0.0);
  rhs_.assign(n, 0.0);
}

void Assembler::conductance(NodeId a, NodeId b, double g) {
  add(a, a, g);
  add(b, b, g);
  add(a, b, -g);
  add(b, a, -g);
}

void Assembler::current(NodeId a, NodeId b, double i) {
  inject(a, -i);
  inject(b, i);
}

void Assembler::transconductance(NodeId a, NodeId b, NodeId c, NodeId d, double g) {
  add(a, c, g);
  add(a, d, -g);
  add(b, c, -g);
  add(b, d, g);
}

bool Circuit::is_ground(const std::string& name) const {
  return std::find(ground_.begin(), ground_.end(), name) != ground_.end();
}

NodeId Circuit::node(const std::string& name) {
  if (is_ground(name)) {
    return kGround;
  }
  if (const auto it = index_.find(name); it != index_.end()) {
    return it->second;
  }
  const auto id = static_cast<NodeId>(names_.size());
  names_.push_back(name);
  index_.emplace(name, id);
  v_.push_back(0.0);
  factored_ = false;
  return id;
}

std::optional<NodeId> Circuit::find(const std::string& name) const {
  if (is_ground(name)) {
    return kGround;
  }
  if (const auto it = index_.find(name); it != index_.end()) {
    return it->second;
  }
  return std::nullopt;
}

Element& Circuit::add(std::unique_ptr<Element> element) {
  nonlinear_ = nonlinear_ || element->nonlinear();
  elements_.push_back(std::move(element));
  factored_ = false;
  return *elements_.back();
}

bool Circuit::factor() {
  const std::size_t n = names_.size();
  lu_ = assembler_.matrix();
  pivot_.resize(n);
  bool ok = true;
  for (std::size_t k = 0; k < n; ++k) {
    std::size_t p = k;
    double best = std::abs(lu_[k * n + k]);
    for (std::size_t i = k + 1; i < n; ++i) {
      const double value = std::abs(lu_[i * n + k]);
      if (value > best) {
        best = value;
        p = i;
      }
    }
    pivot_[k] = p;
    if (p != k) {
      std::swap_ranges(lu_.begin() + static_cast<std::ptrdiff_t>(k * n),
                       lu_.begin() + static_cast<std::ptrdiff_t>((k + 1) * n),
                       lu_.begin() + static_cast<std::ptrdiff_t>(p * n));
    }
    double pivot = lu_[k * n + k];
    if (pivot == 0.0) {
      ok = false;  // gmin keeps this from happening unless a node is shorted to itself
      pivot = lu_[k * n + k] = 1e-30;
    }
    for (std::size_t i = k + 1; i < n; ++i) {
      double& l = lu_[i * n + k];
      if (l == 0.0) {
        continue;
      }
      l /= pivot;
      const double* row_k = &lu_[k * n];
      double* row_i = &lu_[i * n];
      for (std::size_t j = k + 1; j < n; ++j) {
        row_i[j] -= l * row_k[j];
      }
    }
  }
  return ok;
}

void Circuit::substitute(std::vector<double>& x) const {
  const std::size_t n = names_.size();
  for (std::size_t k = 0; k < n; ++k) {
    if (pivot_[k] != k) {
      std::swap(x[k], x[pivot_[k]]);
    }
  }
  for (std::size_t i = 0; i < n; ++i) {
    double sum = x[i];
    const double* row = &lu_[i * n];
    for (std::size_t j = 0; j < i; ++j) {
      sum -= row[j] * x[j];
    }
    x[i] = sum;
  }
  for (std::size_t i = n; i-- > 0;) {
    double sum = x[i];
    const double* row = &lu_[i * n];
    for (std::size_t j = i + 1; j < n; ++j) {
      sum -= row[j] * x[j];
    }
    x[i] = sum / row[i];
  }
}

Circuit::Result Circuit::solve(const Step& step) {
  const std::size_t n = names_.size();
  Result result;
  ++steps;
  if (step.h != last_h_ || step.euler != last_euler_) {
    factored_ = false;  // companion conductances depend on both
    last_h_ = step.h;
    last_euler_ = step.euler;
  }
  work_ = v_;
  double last_change = std::numeric_limits<double>::infinity();
  bool slow = false;
  result.converged = false;
  for (int it = 0; it < max_iterations; ++it) {
    assembler_.reset(n, it);
    const Voltages iterate(work_);
    for (const auto& element : elements_) {
      element->stamp(assembler_, iterate, step);
    }
    std::vector<double>& g = assembler_.matrix();
    for (std::size_t i = 0; i < n; ++i) {
      g[i * n + i] += gmin;
    }
    if (!factored_ || assembler_.was_changed() || slow) {
      factor();
      factored_ = true;
      slow = false;
      ++result.factorizations;
    }
    // Chord step: the residual of the true linearisation, solved with the
    // kept factorisation.
    residual_ = assembler_.rhs();
    for (std::size_t i = 0; i < n; ++i) {
      const double* row = &g[i * n];
      double sum = 0.0;
      for (std::size_t j = 0; j < n; ++j) {
        sum += row[j] * work_[j];
      }
      residual_[i] -= sum;
    }
    substitute(residual_);
    double worst = 0.0, change = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
      work_[i] += residual_[i];
      const double scale = abs_tol + rel_tol * std::abs(work_[i]);
      worst = std::max(worst, std::abs(residual_[i]) / scale);
      change = std::max(change, std::abs(residual_[i]));
    }
    ++result.iterations;
    if (!std::isfinite(worst)) {
      work_ = v_;  // diverged: keep the last good solution
      break;
    }
    if (worst <= 1.0) {
      result.converged = true;
      break;
    }
    slow = it >= 1 && change > 0.25 * last_change;
    last_change = change;
  }
  if (!result.converged) {
    ++failures;
  }
  v_ = work_;
  total_iterations += result.iterations;
  total_factorizations += result.factorizations;
  const Voltages solution(v_);
  for (const auto& element : elements_) {
    element->accept(solution, step);
  }
  return result;
}

Circuit::Result Circuit::operating_point(double t) {
  Step dc;
  dc.t = t;
  dc.h = 0.0;
  dc.euler = true;
  const Result result = solve(dc);
  const Voltages solution(v_);
  for (const auto& element : elements_) {
    element->initialize(solution);
  }
  return result;
}

}  // namespace vsim::elec
