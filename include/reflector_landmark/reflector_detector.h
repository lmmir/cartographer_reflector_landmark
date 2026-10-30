#ifndef REFLECTOR_LANDMARK_REFLECTOR_DETECTOR_H_
#define REFLECTOR_LANDMARK_REFLECTOR_DETECTOR_H_

#include <cstddef>
#include <functional>
#include <vector>

#include "sensor_msgs/LaserScan.h"

namespace reflector_landmark {

struct BeamPosition {
  double x = 0.;
  double y = 0.;
  double z = 0.;
};

struct ReflectorDetection {
  double x = 0.;
  double y = 0.;
  double z = 0.;
  double mean_intensity = 0.;
  double width = 0.;
  int point_count = 0;
  std::size_t first_beam_index = 0;
  std::size_t last_beam_index = 0;
};

struct ReflectorDetectorOptions {
  double intensity_threshold = 1000.;
  double min_range = 0.2;
  double max_range = 20.;
  double cluster_distance = 0.08;
  double min_cluster_width = 0.;
  double max_cluster_width = 0.2;
  int min_cluster_points = 2;
  int max_cluster_points = 30;
  int max_missing_beams = 2;
  // Merges clusters which were split by short intensity dropouts on the same
  // physical reflector. Set to zero to disable merging.
  double detection_merge_distance = 0.1;
};

class ReflectorDetector {
 public:
  // Called only for beams passing the range/intensity filters. The returned
  // position must be expressed at one common reference time (scan start in
  // the ROS node), so clustering and published observations use the same pose.
  using BeamTransform =
      std::function<BeamPosition(std::size_t, const BeamPosition&)>;

  explicit ReflectorDetector(const ReflectorDetectorOptions& options);

  std::vector<ReflectorDetection> Detect(
      const sensor_msgs::LaserScan& scan,
      const BeamTransform& beam_transform = BeamTransform()) const;

 private:
  ReflectorDetectorOptions options_;
};

}  // namespace reflector_landmark

#endif  // REFLECTOR_LANDMARK_REFLECTOR_DETECTOR_H_
