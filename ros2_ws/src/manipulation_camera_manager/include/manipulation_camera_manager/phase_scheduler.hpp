#pragma once

#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace manipulation_camera_manager {

struct CameraPhaseRule {
  double decode_hz{0.0};
  double inference_hz{0.0};
};

class PhaseSchedule {
 public:
  void AddRule(
      const std::string& phase, const std::string& camera,
      CameraPhaseRule rule);
  bool HasPhase(const std::string& phase) const;
  const CameraPhaseRule& Rule(
      const std::string& phase, const std::string& camera) const;
  std::vector<std::string> Phases() const;

 private:
  std::map<std::string, std::map<std::string, CameraPhaseRule>> rules_;
};

std::string NormalizePhase(std::string phase);

}  // namespace manipulation_camera_manager
