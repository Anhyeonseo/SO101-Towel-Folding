#include "manipulation_camera_manager/rolling_statistics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace manipulation_camera_manager {

RollingStatistics::RollingStatistics(std::size_t capacity)
    : capacity_(capacity) {
  if (capacity_ == 0U) {
    throw std::invalid_argument("statistics capacity must be positive");
  }
}

void RollingStatistics::Add(double value) {
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument("statistics value must be finite and positive");
  }
  if (values_.size() == capacity_) {
    values_.pop_front();
  }
  values_.push_back(value);
}

void RollingStatistics::Clear() { values_.clear(); }

StatisticsSummary RollingStatistics::Summary() const {
  StatisticsSummary result;
  result.count = values_.size();
  if (values_.empty()) {
    return result;
  }
  std::vector<double> sorted(values_.begin(), values_.end());
  std::sort(sorted.begin(), sorted.end());
  const auto percentile = [&sorted](double fraction) {
    const double rank = fraction * static_cast<double>(sorted.size() - 1U);
    const auto lower = static_cast<std::size_t>(std::floor(rank));
    const auto upper = static_cast<std::size_t>(std::ceil(rank));
    const double weight = rank - static_cast<double>(lower);
    return sorted[lower] * (1.0 - weight) + sorted[upper] * weight;
  };
  result.p50 = percentile(0.50);
  result.p95 = percentile(0.95);
  result.maximum = sorted.back();
  return result;
}

}  // namespace manipulation_camera_manager
