#ifndef REFLECTOR_LANDMARK_REFLECTOR_MAP_H_
#define REFLECTOR_LANDMARK_REFLECTOR_MAP_H_

#include <cstddef>
#include <string>
#include <vector>

namespace reflector_landmark {

struct MapPoint {
  double x = 0.;
  double y = 0.;
};

struct Reflector {
  std::string id;
  double x = 0.;
  double y = 0.;
  int observations = 0;
  bool optimized = false;
};

struct ReflectorMapOptions {
  double association_distance = 0.3;
  double association_margin = 0.05;
  double pending_association_distance = 0.2;
  double duplicate_suppression_distance = 0.15;
  int minimum_confirmations = 8;
  int maximum_pending_missed_scans = 5;
  double map_update_gain = 0.1;
  std::string id_prefix = "reflector_";
  int id_width = 4;
};

// Association state is independent of ROS so ID stability can be tested
// directly. All supplied points and reflectors belong to the current map epoch.
class ReflectorMap {
 public:
  explicit ReflectorMap(const ReflectorMapOptions& options);

  const std::vector<Reflector>& reflectors() const { return reflectors_; }
  bool AddReflector(const Reflector& reflector);
  bool HasReflectorId(const std::string& id) const;
  int FindCanonicalReflector(const MapPoint& point,
                             double maximum_distance) const;
  std::vector<int> AssociateKnownReflectors(
      const std::vector<MapPoint>& map_points) const;
  void UpdateMapping(const std::vector<MapPoint>& map_points,
                     std::vector<int>* associations);
  void AgePendingTracks();

  // Rebase recent landmarks and pending confirmations together. They have no
  // pose-graph anchor yet and follow the continuous local frame's correction.
  void RebaseUnoptimizedState(double x, double y, double yaw);
  void SynchronizeOptimizedReflectors(const std::vector<Reflector>& optimized);
  bool HasOptimizedReflectors() const;

 private:
  struct PendingTrack {
    double x = 0.;
    double y = 0.;
    int hits = 0;
    int missed_scans = 0;
  };

  void UpdateReflectorFromObservation(std::size_t reflector_index,
                                      const MapPoint& point);
  std::string GenerateReflectorId();

  ReflectorMapOptions options_;
  std::vector<Reflector> reflectors_;
  std::vector<PendingTrack> pending_tracks_;
  int next_reflector_id_ = 0;
};

}  // namespace reflector_landmark

#endif  // REFLECTOR_LANDMARK_REFLECTOR_MAP_H_
