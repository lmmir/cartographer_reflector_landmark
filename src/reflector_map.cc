#include "reflector_landmark/reflector_map.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace reflector_landmark {
namespace {

struct AssociationCandidate {
  std::size_t detection_index;
  std::size_t target_index;
  double distance;
};

template <typename Target>
double Distance(const MapPoint& point, const Target& target) {
  return std::hypot(point.x - target.x, point.y - target.y);
}

}  // namespace

ReflectorMap::ReflectorMap(const ReflectorMapOptions& options)
    : options_(options) {
  if (!std::isfinite(options_.association_distance) ||
      !std::isfinite(options_.association_margin) ||
      !std::isfinite(options_.pending_association_distance) ||
      !std::isfinite(options_.duplicate_suppression_distance) ||
      !std::isfinite(options_.map_update_gain) ||
      options_.association_distance <= 0. || options_.association_margin < 0. ||
      options_.pending_association_distance <= 0. ||
      options_.duplicate_suppression_distance <= 0. ||
      options_.minimum_confirmations <= 0 ||
      options_.maximum_pending_missed_scans < 0 ||
      options_.map_update_gain < 0. || options_.map_update_gain > 1. ||
      options_.id_width <= 0) {
    throw std::invalid_argument("Invalid reflector map options.");
  }
}

bool ReflectorMap::AddReflector(const Reflector& reflector) {
  if (reflector.id.empty() || !std::isfinite(reflector.x) ||
      !std::isfinite(reflector.y) || HasReflectorId(reflector.id) ||
      FindCanonicalReflector({reflector.x, reflector.y},
                             options_.duplicate_suppression_distance) >= 0) {
    return false;
  }
  reflectors_.push_back(reflector);
  return true;
}

bool ReflectorMap::HasReflectorId(const std::string& id) const {
  return std::any_of(
      reflectors_.begin(), reflectors_.end(),
      [&id](const Reflector& reflector) { return reflector.id == id; });
}

int ReflectorMap::FindCanonicalReflector(const MapPoint& point,
                                         const double maximum_distance) const {
  for (std::size_t index = 0; index < reflectors_.size(); ++index) {
    if (Distance(point, reflectors_[index]) <= maximum_distance) {
      return static_cast<int>(index);
    }
  }
  return -1;
}

void ReflectorMap::UpdateReflectorFromObservation(
    const std::size_t reflector_index, const MapPoint& point) {
  Reflector& reflector = reflectors_[reflector_index];
  // Optimized poses are authoritative. Noisy observations must not move the
  // association map away from the pose graph between optimization snapshots.
  if (!reflector.optimized) {
    reflector.x = (1. - options_.map_update_gain) * reflector.x +
                  options_.map_update_gain * point.x;
    reflector.y = (1. - options_.map_update_gain) * reflector.y +
                  options_.map_update_gain * point.y;
  }
  ++reflector.observations;
}

std::string ReflectorMap::GenerateReflectorId() {
  while (true) {
    std::ostringstream stream;
    stream << options_.id_prefix << std::setfill('0')
           << std::setw(options_.id_width) << next_reflector_id_++;
    if (!HasReflectorId(stream.str())) {
      return stream.str();
    }
  }
}

std::vector<int> ReflectorMap::AssociateKnownReflectors(
    const std::vector<MapPoint>& map_points) const {
  std::vector<int> associations(map_points.size(), -1);
  if (reflectors_.empty()) {
    return associations;
  }

  std::vector<double> best_distance(map_points.size(),
                                    std::numeric_limits<double>::infinity());
  std::vector<double> second_best_distance(
      map_points.size(), std::numeric_limits<double>::infinity());
  std::vector<std::size_t> best_reflector_index(map_points.size(),
                                                reflectors_.size());
  std::vector<AssociationCandidate> candidates;
  for (std::size_t detection_index = 0; detection_index < map_points.size();
       ++detection_index) {
    for (std::size_t reflector_index = 0; reflector_index < reflectors_.size();
         ++reflector_index) {
      const double distance =
          Distance(map_points[detection_index], reflectors_[reflector_index]);
      if (distance < best_distance[detection_index]) {
        second_best_distance[detection_index] = best_distance[detection_index];
        best_distance[detection_index] = distance;
        best_reflector_index[detection_index] = reflector_index;
      } else if (distance < second_best_distance[detection_index]) {
        second_best_distance[detection_index] = distance;
      }
    }
    const double second_best = second_best_distance[detection_index];
    const bool ambiguous = std::isfinite(second_best) &&
                           second_best - best_distance[detection_index] <
                               options_.association_margin;
    if (!ambiguous &&
        best_distance[detection_index] <= options_.association_distance) {
      candidates.push_back({detection_index,
                            best_reflector_index[detection_index],
                            best_distance[detection_index]});
    }
  }
  std::sort(
      candidates.begin(), candidates.end(),
      [](const AssociationCandidate& lhs, const AssociationCandidate& rhs) {
        return lhs.distance < rhs.distance;
      });

  std::vector<bool> used_reflectors(reflectors_.size(), false);
  for (const auto& candidate : candidates) {
    if (associations[candidate.detection_index] >= 0 ||
        used_reflectors[candidate.target_index]) {
      continue;
    }
    associations[candidate.detection_index] =
        static_cast<int>(candidate.target_index);
    used_reflectors[candidate.target_index] = true;
  }
  return associations;
}

void ReflectorMap::UpdateMapping(const std::vector<MapPoint>& map_points,
                                 std::vector<int>* associations) {
  std::vector<bool> detection_used(map_points.size(), false);
  std::vector<bool> reflector_used(reflectors_.size(), false);
  for (std::size_t i = 0; i < associations->size(); ++i) {
    const int reflector_index = (*associations)[i];
    if (reflector_index < 0) {
      // An ambiguous or already-consumed match is not evidence of a new ID.
      // Suppress it without publishing an arbitrary existing landmark ID.
      detection_used[i] =
          FindCanonicalReflector(
              map_points[i],
              std::max(options_.association_distance,
                       options_.duplicate_suppression_distance)) >= 0;
      continue;
    }
    detection_used[i] = true;
    if (reflector_used[reflector_index]) {
      // A second cluster from the same physical reflector must not create a
      // second landmark observation or a new pending track.
      (*associations)[i] = -1;
      continue;
    }
    (*associations)[i] = reflector_index;
    reflector_used[reflector_index] = true;
    UpdateReflectorFromObservation(reflector_index, map_points[i]);
  }

  std::vector<AssociationCandidate> candidates;
  for (std::size_t detection_index = 0; detection_index < map_points.size();
       ++detection_index) {
    if (detection_used[detection_index]) {
      continue;
    }
    double best_distance = options_.pending_association_distance;
    std::size_t best_track_index = pending_tracks_.size();
    for (std::size_t track_index = 0; track_index < pending_tracks_.size();
         ++track_index) {
      const double distance =
          Distance(map_points[detection_index], pending_tracks_[track_index]);
      if (distance <= best_distance) {
        best_distance = distance;
        best_track_index = track_index;
      }
    }
    if (best_track_index < pending_tracks_.size()) {
      candidates.push_back({detection_index, best_track_index, best_distance});
    }
  }
  std::sort(
      candidates.begin(), candidates.end(),
      [](const AssociationCandidate& lhs, const AssociationCandidate& rhs) {
        return lhs.distance < rhs.distance;
      });

  std::vector<int> pending_detection(pending_tracks_.size(), -1);
  for (const auto& candidate : candidates) {
    if (detection_used[candidate.detection_index] ||
        pending_detection[candidate.target_index] >= 0) {
      continue;
    }
    detection_used[candidate.detection_index] = true;
    pending_detection[candidate.target_index] =
        static_cast<int>(candidate.detection_index);
    PendingTrack& track = pending_tracks_[candidate.target_index];
    track.x = 0.5 * track.x + 0.5 * map_points[candidate.detection_index].x;
    track.y = 0.5 * track.y + 0.5 * map_points[candidate.detection_index].y;
    ++track.hits;
    track.missed_scans = 0;
  }

  for (std::size_t i = 0; i < map_points.size(); ++i) {
    if (detection_used[i]) {
      continue;
    }
    pending_tracks_.push_back({map_points[i].x, map_points[i].y, 1, 0});
    pending_detection.push_back(static_cast<int>(i));
  }

  std::vector<bool> remove_pending(pending_tracks_.size(), false);
  for (std::size_t track_index = 0; track_index < pending_tracks_.size();
       ++track_index) {
    PendingTrack& track = pending_tracks_[track_index];
    const int detection_index = pending_detection[track_index];
    if (detection_index >= 0 && track.hits >= options_.minimum_confirmations) {
      const MapPoint track_point{track.x, track.y};
      const int canonical_index = FindCanonicalReflector(
          track_point, options_.duplicate_suppression_distance);
      if (canonical_index >= 0) {
        // Duplicate suppression is not an association decision. Only the
        // unambiguous one-to-one matcher may attach an existing ID.
        remove_pending[track_index] = true;
        continue;
      }

      Reflector reflector;
      reflector.id = GenerateReflectorId();
      reflector.x = track.x;
      reflector.y = track.y;
      reflector.observations = track.hits;
      reflectors_.push_back(reflector);
      reflector_used.push_back(true);
      (*associations)[detection_index] =
          static_cast<int>(reflectors_.size() - 1);
      remove_pending[track_index] = true;
    } else if (detection_index < 0) {
      ++track.missed_scans;
      if (track.missed_scans > options_.maximum_pending_missed_scans) {
        remove_pending[track_index] = true;
      }
    }
  }

  // A reflector created above may overlap another, less mature pending
  // track. Remove it now so it cannot become a duplicate on a later scan.
  for (std::size_t track_index = 0; track_index < pending_tracks_.size();
       ++track_index) {
    if (remove_pending[track_index]) {
      continue;
    }
    if (FindCanonicalReflector(
            {pending_tracks_[track_index].x, pending_tracks_[track_index].y},
            options_.duplicate_suppression_distance) >= 0) {
      remove_pending[track_index] = true;
    }
  }

  for (std::size_t index = pending_tracks_.size(); index-- > 0;) {
    if (remove_pending[index]) {
      pending_tracks_.erase(pending_tracks_.begin() + index);
    }
  }
}

void ReflectorMap::AgePendingTracks() {
  for (auto& track : pending_tracks_) {
    ++track.missed_scans;
  }
  pending_tracks_.erase(
      std::remove_if(pending_tracks_.begin(), pending_tracks_.end(),
                     [this](const PendingTrack& track) {
                       return track.missed_scans >
                              options_.maximum_pending_missed_scans;
                     }),
      pending_tracks_.end());
}

void ReflectorMap::RebaseUnoptimizedState(const double x, const double y,
                                          const double yaw) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  for (auto& reflector : reflectors_) {
    if (reflector.optimized) {
      continue;
    }
    const double old_x = reflector.x;
    reflector.x = c * old_x - s * reflector.y + x;
    reflector.y = s * old_x + c * reflector.y + y;
  }
  for (auto& track : pending_tracks_) {
    const double old_x = track.x;
    track.x = c * old_x - s * track.y + x;
    track.y = s * old_x + c * track.y + y;
  }
}

void ReflectorMap::SynchronizeOptimizedReflectors(
    const std::vector<Reflector>& optimized) {
  for (const auto& pose : optimized) {
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y)) {
      continue;
    }
    for (auto& reflector : reflectors_) {
      if (reflector.id != pose.id) {
        continue;
      }
      reflector.x = pose.x;
      reflector.y = pose.y;
      reflector.optimized = true;
      break;
    }
  }
  // An optimized existing ID may now overlap a candidate. Suppress only those
  // candidates, so frequent optimization does not starve unrelated new IDs.
  pending_tracks_.erase(
      std::remove_if(pending_tracks_.begin(), pending_tracks_.end(),
                     [this](const PendingTrack& track) {
                       return FindCanonicalReflector(
                                  {track.x, track.y},
                                  options_.duplicate_suppression_distance) >= 0;
                     }),
      pending_tracks_.end());
}

bool ReflectorMap::HasOptimizedReflectors() const {
  return std::any_of(
      reflectors_.begin(), reflectors_.end(),
      [](const Reflector& reflector) { return reflector.optimized; });
}

}  // namespace reflector_landmark
