#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace vsim::elec {

// Index of a circuit node; ground is kGround and is not an unknown.
using NodeId = int;
inline constexpr NodeId kGround = -1;

// One integration step (design doc §12.3): capacitors and inductors use
// trapezoidal companion models, backward Euler right after a discontinuity
// (a contact closing, a fuse blowing) so the step does not ring.
struct Step {
  double t = 0.0;      // time at the end of the step [s]
  double h = 0.0;      // step length [s]; 0 solves a DC operating point
  bool euler = true;
  bool dc() const { return h <= 0.0; }
};

// Node voltages of the present iterate; ground reads 0.
class Voltages {
 public:
  explicit Voltages(const std::vector<double>& v) : v_(&v) {}
  double operator()(NodeId n) const { return n < 0 ? 0.0 : (*v_)[static_cast<std::size_t>(n)]; }
  double across(NodeId a, NodeId b) const { return (*this)(a) - (*this)(b); }

 private:
  const std::vector<double>* v_;
};

// Collects the linearised element equations in nodal form, G v = i
// (each row: the currents leaving the node through the elements).
class Assembler {
 public:
  void reset(std::size_t n, int iteration);
  std::size_t size() const { return n_; }
  // Newton iteration within the step; elements stop switching piecewise
  // modes after a number of iterations so the step cannot cycle.
  int iteration() const { return iteration_; }

  // Element current from a to b = g (V(a) - V(b)).
  void conductance(NodeId a, NodeId b, double g);
  // A fixed current i through the element from a to b.
  void current(NodeId a, NodeId b, double i);
  // Element current from a to b = g (V(c) - V(d)).
  void transconductance(NodeId a, NodeId b, NodeId c, NodeId d, double g);
  // The linearisation jumped (a diode turned on, a limit engaged): the
  // solver factors the matrix again instead of reusing the last one.
  void changed() { changed_ = true; }
  bool was_changed() const { return changed_; }

  std::vector<double>& matrix() { return g_; }
  std::vector<double>& rhs() { return rhs_; }

 private:
  void add(NodeId row, NodeId col, double value) {
    if (row >= 0 && col >= 0) {
      g_[static_cast<std::size_t>(row) * n_ + static_cast<std::size_t>(col)] += value;
    }
  }
  void inject(NodeId row, double value) {
    if (row >= 0) {
      rhs_[static_cast<std::size_t>(row)] += value;
    }
  }

  std::size_t n_ = 0;
  int iteration_ = 0;
  bool changed_ = false;
  std::vector<double> g_, rhs_;
};

class Element {
 public:
  explicit Element(std::string id) : id_(std::move(id)) {}
  virtual ~Element() = default;
  const std::string& id() const { return id_; }

  // Linearised equations at the iterate v.
  virtual void stamp(Assembler& a, const Voltages& v, const Step& step) = 0;
  // The step converged at v: charges, currents and internal states move on.
  virtual void accept(const Voltages& /*v*/, const Step& /*step*/) {}
  // Dynamic states from a DC operating point (capacitor voltages, inductor currents).
  virtual void initialize(const Voltages& /*v*/) {}
  // Whether the equations depend on the voltages (Newton iterations needed).
  virtual bool nonlinear() const { return false; }

 private:
  std::string id_;
};

// Modified nodal analysis on a dense matrix with partial-pivoting LU. The
// factorisation is kept between steps (chord Newton) and redone when an
// element reports a jump, the step length changes, or convergence slows.
class Circuit {
 public:
  // Names that mean the reference node (default GND and 0).
  void set_ground(std::vector<std::string> names) { ground_ = std::move(names); }
  bool is_ground(const std::string& name) const;
  NodeId node(const std::string& name);  // creates the node on first use
  std::optional<NodeId> find(const std::string& name) const;
  const std::vector<std::string>& node_names() const { return names_; }
  std::size_t size() const { return names_.size(); }

  template <class T, class... Args>
  T& emplace(Args&&... args) {
    auto element = std::make_unique<T>(std::forward<Args>(args)...);
    T& ref = *element;
    add(std::move(element));
    return ref;
  }
  Element& add(std::unique_ptr<Element> element);
  const std::vector<std::unique_ptr<Element>>& elements() const { return elements_; }

  struct Result {
    int iterations = 0;
    int factorizations = 0;
    bool converged = true;
  };
  // Solves one step and lets every element accept it (also when it did not
  // converge within max_iterations; the result says so).
  Result solve(const Step& step);
  // DC operating point; dynamic elements then start from it.
  Result operating_point(double t);
  // The topology changed between steps (a contact, a fault): factor again.
  void invalidate() { factored_ = false; }

  const std::vector<double>& voltages() const { return v_; }
  Voltages view() const { return Voltages(v_); }
  double voltage(NodeId n) const { return n < 0 ? 0.0 : v_[static_cast<std::size_t>(n)]; }

  // [S] from every node to ground: keeps floating nets (the isolated HV
  // system) defined; 10 MOhm per node, like insulation and measurement leakage.
  double gmin = 1e-7;
  int max_iterations = 60;
  double abs_tol = 1e-6;       // [V]
  double rel_tol = 1e-5;

  std::int64_t total_iterations = 0, total_factorizations = 0, failures = 0, steps = 0;

 private:
  bool factor();
  void substitute(std::vector<double>& x) const;

  std::vector<std::string> ground_ = {"GND", "0"};
  std::vector<std::string> names_;
  std::unordered_map<std::string, NodeId> index_;
  std::vector<std::unique_ptr<Element>> elements_;
  bool nonlinear_ = false;

  std::vector<double> v_, work_, residual_, lu_;
  std::vector<std::size_t> pivot_;
  Assembler assembler_;
  bool factored_ = false;
  double last_h_ = -1.0;
  bool last_euler_ = false;
};

}  // namespace vsim::elec
