#pragma once

#include <anymal_msgs/msg/anymal_state.hpp>
#include <anymal_msgs/msg/contact.hpp>

#include <gtsam/base/Matrix.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "include/LeggedEstimatorReplayUtils.h"
#include "include/JointTorqueGrfEstimator.h"
#include "include/StabilityGatedContactClassifier.h"

namespace gtsam {

class AnymalContactAdapter {
 public:
  struct FootDiagnostic {
    std::string footName;
    bool hasEstimatedGrf = false;
    double estimatedGrfZ = 0.0;
    bool classifiedContact = false;
    bool hasMeasuredContact = false;
    double measuredWrenchZ = 0.0;
    uint8_t measuredContactState = 0;
  };

  struct Diagnostics {
    std::vector<FootDiagnostic> feet;
  };

  struct Options {
    std::vector<std::string> footNames;
    std::vector<Vector3> legImuOffsets;
    bool slippingIsContact = true;
    bool requireStateOk = false;
    std::string contactClassifier = "stability_gate";
    double contactForceOn = 130.0;
    double contactForceOff = 100.0;
    double contactMaxForceDelta = 40.0;
    size_t contactForceWindowSize = 4;
    size_t contactOnSamples = 12;
    bool contactUseForceWindow = false;
    size_t contactForceDeltaOffSamples = 1;
    bool contactUseAbsoluteForceZ = true;
    double jointTorqueGrfDamping = 0.02;
    double jointTorqueScale = 1.0;
    std::string anymalUrdfPath;
  };

  explicit AnymalContactAdapter(Options options);

  std::optional<ContactEvent> makeContactEvent(
      const anymal_msgs::msg::AnymalState& msg, size_t eventIndex,
      Diagnostics* diagnostics = nullptr);

 private:
  static double stampToSeconds(const builtin_interfaces::msg::Time& stamp);
  static std::string canonicalFootName(const std::string& name);
  static std::vector<std::string> aliasesForContactName(
      const std::string& name);
  std::optional<size_t> footIndexForContactName(
      const std::string& name) const;
  bool isActiveContact(const anymal_msgs::msg::Contact& contact) const;
  Vector3 bodyPointFromWorldContact(
      const anymal_msgs::msg::Contact& contact,
      const geometry_msgs::msg::PoseStamped& bodyPose) const;

  Options options_;
  std::vector<std::string> canonicalFootNames_;
  std::vector<bool> previousContactActive_;
  std::vector<bool> havePreviousContactStates_;
  std::vector<StabilityGatedContactClassifier> forceClassifiers_;
  std::unique_ptr<JointTorqueGrfEstimator> jointTorqueGrfEstimator_;
};

}  // namespace gtsam
