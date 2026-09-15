#pragma once

#include <cstddef>
#include <deque>
#include <optional>

namespace gtsam {

/// Robot-agnostic, per-foot stability gate: decides whether a reconstructed
/// vertical ground-reaction force indicates a contact that is stable enough
/// to accept for a kinematic update, rather than the earliest instant of
/// physical touchdown.
///
/// Two modes reproduce the two hysteresis schemes used across the robot
/// adapters:
///  - kWindowedDwell (Spot, ANYmal): activation requires `onSamples`
///    consecutive samples at or above `forceOn` with a stable recent
///    force-delta history; release requires `offSamples` consecutive
///    samples that are either below `forceOff` or show an unstable delta.
///  - kFilteredThreshold (Unitree G1): activation/release is an immediate
///    asymmetric threshold crossing on an exponentially-filtered force,
///    with no dwell/window requirement.
class StabilityGatedContactClassifier {
 public:
  enum class Mode { kWindowedDwell, kFilteredThreshold };

  struct Params {
    Mode mode = Mode::kWindowedDwell;
    double forceOn = 0.0;
    double forceOff = 0.0;

    // kWindowedDwell only.
    double maxForceDelta = 0.0;
    size_t forceWindowSize = 4;
    size_t onSamples = 1;
    bool useForceWindow = false;
    size_t offSamples = 1;

    // kFilteredThreshold only.
    double filterAlpha = 0.2;
  };

  explicit StabilityGatedContactClassifier(Params params);

  /// Feed one new force sample (the signal to gate on, e.g. the absolute
  /// vertical GRF) and return whether the foot is now accepted as a stable
  /// contact. A non-finite sample resets the classifier and returns false.
  bool classify(double force);

  bool active() const { return active_; }

  /// Override the externally-visible active state, e.g. when a caller
  /// combines the force-based decision with additional non-force
  /// conditions (such as a height guard) that should also gate which
  /// threshold/dwell counters the *next* classify() call uses. Only
  /// affects future hysteresis; does not touch the internal filter state.
  void setActive(bool active) { active_ = active; }

  /// kFilteredThreshold only: the current exponentially-filtered force.
  double filteredForce() const { return filtered_; }

 private:
  bool classifyWindowedDwell(double force);
  bool classifyFilteredThreshold(double force);

  Params params_;
  bool active_ = false;

  // kWindowedDwell state.
  std::optional<double> previousForce_;
  std::deque<double> deltas_;
  size_t onCount_ = 0;
  size_t offCount_ = 0;

  // kFilteredThreshold state.
  bool haveFiltered_ = false;
  double filtered_ = 0.0;
};

}  // namespace gtsam
