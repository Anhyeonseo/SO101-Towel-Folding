#include "manipulation_camera_manager/phase_scheduler.hpp"

#include <algorithm>
#include <cctype>

namespace manipulation_camera_manager {

void PhaseSchedule::AddRule(
    const std::string& phase, const std::string& camera,
    CameraPhaseRule rule) {
  if (phase.empty() || camera.empty() || rule.decode_hz < 0.0 ||
      rule.inference_hz < 0.0 || rule.inference_hz > rule.decode_hz) {
    throw std::invalid_argument("invalid camera phase rule");
  }
  rules_[NormalizePhase(phase)][camera] = rule;
}

bool PhaseSchedule::HasPhase(const std::string& phase) const {
  return rules_.find(NormalizePhase(phase)) != rules_.end();
}

const CameraPhaseRule& PhaseSchedule::Rule(
    const std::string& phase, const std::string& camera) const {
  const auto phase_it = rules_.find(NormalizePhase(phase));
  if (phase_it == rules_.end()) {
    throw std::out_of_range("unknown camera phase: " + phase);
  }
  const auto camera_it = phase_it->second.find(camera);
  if (camera_it == phase_it->second.end()) {
    throw std::out_of_range(
        "camera missing from phase " + phase + ": " + camera);
  }
  return camera_it->second;
}

std::vector<std::string> PhaseSchedule::Phases() const {
  std::vector<std::string> result;
  result.reserve(rules_.size());
  for (const auto& [phase, unused] : rules_) {
    (void)unused;
    result.push_back(phase);
  }
  return result;
}

std::string NormalizePhase(std::string phase) {
  phase.erase(
      std::remove_if(
          phase.begin(), phase.end(),
          [](unsigned char value) { return std::isspace(value) != 0; }),
      phase.end());
  std::transform(
      phase.begin(), phase.end(), phase.begin(),
      [](unsigned char value) {
        return static_cast<char>(std::toupper(value));
      });
  return phase;
}

}  // namespace manipulation_camera_manager
