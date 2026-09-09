#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>

namespace manipulation_camera_manager {

struct StatisticsSummary {
  std::size_t count{0};
  double p50{0.0};
  double p95{0.0};
  double maximum{0.0};
};

class RollingStatistics {
 public:
  explicit RollingStatistics(std::size_t capacity);

  void Add(double value);
  void Clear();
  StatisticsSummary Summary() const;
  std::size_t Capacity() const { return capacity_; }

 private:
  std::size_t capacity_;
  std::deque<double> values_;
};

}  // namespace manipulation_camera_manager
