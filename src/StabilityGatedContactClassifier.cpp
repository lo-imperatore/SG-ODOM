#include "include/StabilityGatedContactClassifier.h"

#include <cmath>

namespace gtsam {

StabilityGatedContactClassifier::StabilityGatedContactClassifier(
    Params params)
    : params_(std::move(params)) {}

bool StabilityGatedContactClassifier::classify(const double force) {
  if (!std::isfinite(force)) {
    previousForce_.reset();
    deltas_.clear();
    onCount_ = 0;
    offCount_ = 0;
    haveFiltered_ = false;
    filtered_ = 0.0;
    active_ = false;
    return false;
  }
  return params_.mode == Mode::kWindowedDwell ? classifyWindowedDwell(force)
                                               : classifyFilteredThreshold(force);
}

bool StabilityGatedContactClassifier::classifyWindowedDwell(
    const double force) {
  std::optional<double> delta;
  if (previousForce_) {
    delta = std::abs(force - *previousForce_);
    deltas_.push_back(*delta);
    while (deltas_.size() > params_.forceWindowSize) {
      deltas_.pop_front();
    }
  }
  previousForce_ = force;

  if (active_) {
    const bool unstable = !delta || *delta > params_.maxForceDelta;
    offCount_ = unstable ? offCount_ + 1 : 0;
    active_ =
        force > params_.forceOff && offCount_ < params_.offSamples;
    if (!active_) {
      onCount_ = 0;
      offCount_ = 0;
    }
    return active_;
  }

  bool stable = delta && *delta <= params_.maxForceDelta;
  if (params_.useForceWindow) {
    stable = deltas_.size() == params_.forceWindowSize;
    for (const double value : deltas_) {
      stable = stable && value <= params_.maxForceDelta;
    }
  }
  onCount_ = force >= params_.forceOn && stable ? onCount_ + 1 : 0;
  active_ = onCount_ >= params_.onSamples;
  if (active_) {
    onCount_ = 0;
    offCount_ = 0;
  }
  return active_;
}

bool StabilityGatedContactClassifier::classifyFilteredThreshold(
    const double force) {
  if (!haveFiltered_) {
    filtered_ = force;
    haveFiltered_ = true;
  } else {
    filtered_ = params_.filterAlpha * force +
               (1.0 - params_.filterAlpha) * filtered_;
  }
  const double threshold = active_ ? params_.forceOff : params_.forceOn;
  active_ = filtered_ >= threshold;
  return active_;
}

}  // namespace gtsam
