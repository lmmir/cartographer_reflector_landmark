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
  double z;
  double intensity;
};

double Distance(const BeamPoint& lhs, const BeamPoint& rhs) {
  return std::sqrt((lhs.x - rhs.x) * (lhs.x - rhs.x) +
                   (lhs.y - rhs.y) * (lhs.y - rhs.y) +
                   (lhs.z - rhs.z) * (lhs.z - rhs.z));
}

double Distance(const ReflectorDetection& lhs, const ReflectorDetection& rhs) {
  return std::sqrt((lhs.x - rhs.x) * (lhs.x - rhs.x) +
                   (lhs.y - rhs.y) * (lhs.y - rhs.y) +
                   (lhs.z - rhs.z) * (lhs.z - rhs.z));
}

struct DetectionCluster {
  ReflectorDetection detection;
  BeamPoint first;
  BeamPoint last;
};

void MergeDetection(const DetectionCluster& source_cluster,
                    DetectionCluster* destination_cluster) {
  const auto& source = source_cluster.detection;
  auto* destination = &destination_cluster->detection;
  const int combined_point_count =
      destination->point_count + source.point_count;
  const double destination_weight =
      static_cast<double>(destination->point_count) / combined_point_count;
  const double source_weight =
      static_cast<double>(source.point_count) / combined_point_count;

  destination->x =
      destination_weight * destination->x + source_weight * source.x;
  destination->y =
      destination_weight * destination->y + source_weight * source.y;
  destination->z =
      destination_weight * destination->z + source_weight * source.z;
  destination->mean_intensity =
      destination_weight * destination->mean_intensity +
      source_weight * source.mean_intensity;
  if (source.first_beam_index < destination->first_beam_index) {
    destination_cluster->first = source_cluster.first;
  }
  if (source.last_beam_index > destination->last_beam_index) {
    destination_cluster->last = source_cluster.last;
  }
  // Keep actual endpoints: a weighted centroid need not be the span midpoint.
  destination->width = std::max(
      {destination->width, source.width,
       Distance(destination_cluster->first, destination_cluster->last)});
  destination->point_count = combined_point_count;
  destination->first_beam_index =
      std::min(destination->first_beam_index, source.first_beam_index);
  destination->last_beam_index =
      std::max(destination->last_beam_index, source.last_beam_index);
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
      options_.max_missing_beams < 0 ||
      options_.detection_merge_distance < 0. ||
      options_.detection_merge_distance > options_.max_cluster_width) {
    throw std::invalid_argument("Invalid reflector detector options.");
  }
}

std::vector<ReflectorDetection> ReflectorDetector::Detect(
    const sensor_msgs::LaserScan& scan,
    const BeamTransform& beam_transform) const {
  std::vector<ReflectorDetection> detections;
  if (scan.ranges.empty() || scan.intensities.size() != scan.ranges.size()) {
    return detections;
  }

  std::vector<DetectionCluster> candidate_clusters;
  std::vector<BeamPoint> cluster;
  const auto flush_cluster = [&]() {
    if (cluster.empty()) {
      return;
    }

    const double width = Distance(cluster.front(), cluster.back());

    ReflectorDetection detection;
    detection.width = width;
    detection.point_count = static_cast<int>(cluster.size());
    detection.first_beam_index = cluster.front().index;
    detection.last_beam_index = cluster.back().index;
    for (const auto& point : cluster) {
      detection.x += point.x;
      detection.y += point.y;
      detection.z += point.z;
      detection.mean_intensity += point.intensity;
    }
    const double count = static_cast<double>(cluster.size());
    detection.x /= count;
    detection.y /= count;
    detection.z /= count;
    detection.mean_intensity /= count;
    candidate_clusters.push_back({detection, cluster.front(), cluster.back()});
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
    BeamPosition position{range * std::cos(angle), range * std::sin(angle), 0.};
    if (beam_transform) {
      position = beam_transform(i, position);
    }
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z)) {
      continue;
    }
    BeamPoint point{i, position.x, position.y, position.z, intensity};
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

  std::vector<DetectionCluster> merged_clusters;
  for (const auto& candidate : candidate_clusters) {
    auto closest = merged_clusters.end();
    double closest_distance = options_.detection_merge_distance;
    for (auto it = merged_clusters.begin(); it != merged_clusters.end(); ++it) {
      const double distance = Distance(it->detection, candidate.detection);
      if (options_.detection_merge_distance > 0. &&
          distance <= closest_distance) {
        closest = it;
        closest_distance = distance;
      }
    }
    if (closest == merged_clusters.end()) {
      merged_clusters.push_back(candidate);
    } else {
      MergeDetection(candidate, &*closest);
    }
  }

  // Filtering here lets adjacent single-beam fragments satisfy the minimum
  // together, and still rejects overly wide/large merged reflective objects.
  for (const auto& merged : merged_clusters) {
    const auto& detection = merged.detection;
    if (detection.point_count >= options_.min_cluster_points &&
        detection.point_count <= options_.max_cluster_points &&
        detection.width >= options_.min_cluster_width &&
        detection.width <= options_.max_cluster_width) {
      detections.push_back(detection);
    }
  }
  return detections;
}

}  // namespace reflector_landmark
