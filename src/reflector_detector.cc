#include "reflector_landmark/reflector_detector.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace reflector_landmark {
namespace {

struct BeamPoint {
  std::size_t index;
  double x;
  double y;
  double intensity;
};

double Distance(const BeamPoint& lhs, const BeamPoint& rhs) {
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

}  // namespace

ReflectorDetector::ReflectorDetector(const ReflectorDetectorOptions& options)
    : options_(options) {
  if (options_.intensity_threshold < 0. || options_.min_range < 0. ||
      options_.max_range <= options_.min_range ||
      options_.cluster_distance <= 0. || options_.min_cluster_width < 0. ||
      options_.max_cluster_width < options_.min_cluster_width ||
      options_.min_cluster_points <= 0 ||
      options_.max_cluster_points < options_.min_cluster_points ||
      options_.max_missing_beams < 0) {
    throw std::invalid_argument("Invalid reflector detector options.");
  }
}

std::vector<ReflectorDetection> ReflectorDetector::Detect(
    const sensor_msgs::LaserScan& scan) const {
  std::vector<ReflectorDetection> detections;
  if (scan.ranges.empty() || scan.intensities.size() != scan.ranges.size()) {
    return detections;
  }

  std::vector<BeamPoint> cluster;
  const auto flush_cluster = [&]() {
    if (cluster.size() <
            static_cast<std::size_t>(options_.min_cluster_points) ||
        cluster.size() >
            static_cast<std::size_t>(options_.max_cluster_points)) {
      cluster.clear();
      return;
    }

    const double width = Distance(cluster.front(), cluster.back());
    if (width < options_.min_cluster_width ||
        width > options_.max_cluster_width) {
      cluster.clear();
      return;
    }

    ReflectorDetection detection;
    detection.width = width;
    detection.point_count = static_cast<int>(cluster.size());
    detection.first_beam_index = cluster.front().index;
    detection.last_beam_index = cluster.back().index;
    for (const auto& point : cluster) {
      detection.x += point.x;
      detection.y += point.y;
      detection.mean_intensity += point.intensity;
    }
    const double count = static_cast<double>(cluster.size());
    detection.x /= count;
    detection.y /= count;
    detection.mean_intensity /= count;
    detections.push_back(detection);
    cluster.clear();
  };

  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double range = scan.ranges[i];
    const double intensity = scan.intensities[i];
    if (!std::isfinite(range) || !std::isfinite(intensity) ||
        range < std::max<double>(scan.range_min, options_.min_range) ||
        range > std::min<double>(scan.range_max, options_.max_range) ||
        intensity < options_.intensity_threshold) {
      continue;
    }

    const double angle = scan.angle_min + i * scan.angle_increment;
    BeamPoint point{i, range * std::cos(angle), range * std::sin(angle),
                    intensity};
    if (!cluster.empty()) {
      const std::size_t missing_beams = i - cluster.back().index - 1;
      if (missing_beams >
              static_cast<std::size_t>(options_.max_missing_beams) ||
          Distance(cluster.back(), point) > options_.cluster_distance) {
        flush_cluster();
      }
    }
    cluster.push_back(std::move(point));
  }
  flush_cluster();
  return detections;
}

}  // namespace reflector_landmark
