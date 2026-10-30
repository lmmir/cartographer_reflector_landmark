#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "reflector_landmark/reflector_detector.h"
#include "reflector_landmark/reflector_map.h"

namespace reflector_landmark {
namespace {

void Check(const bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void Near(const double actual, const double expected, const double tolerance) {
  Check(std::isfinite(actual) && std::abs(actual - expected) <= tolerance,
        "Expected " + std::to_string(expected) + ", got " +
            std::to_string(actual));
}

sensor_msgs::LaserScan MakeScan(const std::vector<int>& reflective_beams) {
  sensor_msgs::LaserScan scan;
  scan.angle_min = -0.5;
  scan.angle_max = 0.5;
  scan.angle_increment = 0.01f;
  scan.range_min = 0.1f;
  scan.range_max = 30.;
  scan.ranges.assign(101, 5.);
  scan.intensities.assign(101, 10.);
  for (const int beam : reflective_beams) {
    scan.ranges[beam] = 2.;
    scan.intensities[beam] = 1500.;
  }
  return scan;
}

std::vector<int> Observe(ReflectorMap* map,
                         const std::vector<MapPoint>& points) {
  auto associations = map->AssociateKnownReflectors(points);
  map->UpdateMapping(points, &associations);
  return associations;
}

void PreservesNearestId() {
  ReflectorMap map(ReflectorMapOptions{});
  Check(map.AddReflector({"old", 0., 0., 0}), "Load old reflector");
  Check(map.AddReflector({"correct", 0.18, 0., 0}), "Load second reflector");
  const auto associations = Observe(&map, {{0.14, 0.}});
  Check(associations[0] == 1, "Duplicate guard replaced the nearest ID");
  Check(map.reflectors()[0].observations == 0, "Updated wrong reflector");
}

void RejectsAmbiguityWithoutNewIds() {
  ReflectorMap map(ReflectorMapOptions{});
  map.AddReflector({"left", 0., 0., 0});
  map.AddReflector({"right", 0.18, 0., 0});
  for (int i = 0; i < 20; ++i) {
    Check(Observe(&map, {{0.09, 0.}})[0] == -1, "Published ambiguous ID");
  }
  Check(map.reflectors().size() == 2, "Ambiguity created another ID");
  Check(map.reflectors()[0].observations == 0 &&
            map.reflectors()[1].observations == 0,
        "Ambiguity changed known reflectors");
}

void PublishesOneObservationPerId() {
  ReflectorMap map(ReflectorMapOptions{});
  map.AddReflector({"known", 0., 0., 0});
  for (int i = 0; i < 20; ++i) {
    const auto associations = Observe(&map, {{0., 0.}, {0.12, 0.}});
    Check(associations[0] == 0 && associations[1] == -1,
          "Split detections published duplicate observations");
  }
  Check(map.reflectors().size() == 1, "Split detection created another ID");
  Check(map.reflectors()[0].observations == 20, "Incorrect observation count");
}

void ConfirmsNewIdAndKeepsItStable() {
  ReflectorMap map(ReflectorMapOptions{});
  map.AddReflector({"reflector_0000", 0., 0., 0});
  for (int i = 0; i < 7; ++i) {
    Check(Observe(&map, {{2., 0.}})[0] == -1, "Confirmed too early");
  }
  Check(Observe(&map, {{2., 0.}})[0] == 1, "Did not confirm at eighth hit");
  Check(map.reflectors()[1].id == "reflector_0001", "Reused an existing ID");
  Check(Observe(&map, {{2., 0.}})[0] == 1, "Confirmed ID changed");
  Check(map.reflectors().size() == 2, "Created duplicate ID");
}

void SynchronizesOptimizationById() {
  ReflectorMap map(ReflectorMapOptions{});
  map.AddReflector({"first", 1., 0., 0});
  map.AddReflector({"second", 5., 0., 0});
  // Different corrections deliberately cannot be represented by one global
  // translation. Snapshot order also differs from local insertion order.
  map.SynchronizeOptimizedReflectors({{"second", 5.1, 0., 0},
                                      {"unrelated", 8., 0., 0},
                                      {"first", 1.4, 0., 0}});
  for (int i = 0; i < 12; ++i) {
    const auto associations = Observe(&map, {{1.4, 0.}, {5.1, 0.}});
    Check(associations[0] == 0 && associations[1] == 1,
          "Optimized position lost its original ID");
  }
  Check(map.reflectors().size() == 2, "Optimization created duplicate IDs");
  Check(map.HasOptimizedReflectors(), "Missing optimized state");
  Observe(&map, {{1.5, 0.}});
  Near(map.reflectors()[0].x, 1.4, 1e-12);
}

void RebasesOnlyUnoptimizedReflectors() {
  ReflectorMap map(ReflectorMapOptions{});
  map.AddReflector({"optimized", 1., 0., 0});
  map.AddReflector({"recent", 3., 0., 0});
  map.SynchronizeOptimizedReflectors({{"optimized", 1., 0., 0}});
  map.RebaseUnoptimizedState(0.4, 0., std::acos(-1.) / 2.);
  Near(map.reflectors()[0].x, 1., 1e-12);
  Near(map.reflectors()[0].y, 0., 1e-12);
  Near(map.reflectors()[1].x, 0.4, 1e-12);
  Near(map.reflectors()[1].y, 3., 1e-12);
  map.SynchronizeOptimizedReflectors({{"optimized", 1.4, 0., 0}});
  Check(Observe(&map, {{1.4, 0.}, {0.4, 3.}}) == std::vector<int>({0, 1}),
        "Rebased/optimized IDs changed");
}

void RebasesConfirmationsAcrossMapCorrections() {
  ReflectorMap map(ReflectorMapOptions{});
  for (int i = 0; i < 7; ++i) {
    Observe(&map, {{2., 0.}});
  }
  map.RebaseUnoptimizedState(0.4, 0., std::acos(-1.) / 2.);
  Check(Observe(&map, {{0.4, 2.}})[0] == 0,
        "Pending coordinates did not follow the map correction");
  Check(map.reflectors().size() == 1,
        "Incorrect confirmation after correction");
  Near(map.reflectors()[0].x, 0.4, 1e-12);
  Near(map.reflectors()[0].y, 2., 1e-12);
}

void OptimizationPreservesUnrelatedConfirmations() {
  ReflectorMap map(ReflectorMapOptions{});
  map.AddReflector({"known", 0., 0., 0});
  for (int i = 0; i < 7; ++i) {
    Observe(&map, {{2., 0.}});
  }
  map.SynchronizeOptimizedReflectors({{"known", 0.4, 0., 0}});
  Check(Observe(&map, {{2., 0.}})[0] == 1,
        "Non-rigid optimization starved an unrelated candidate");
  Check(map.reflectors().size() == 2, "Lost unrelated confirmations");
}

void OptimizationSuppressesPendingDuplicates() {
  ReflectorMap map(ReflectorMapOptions{});
  map.AddReflector({"known", 0., 0., 0});
  for (int i = 0; i < 7; ++i) {
    Observe(&map, {{2., 0.}});
  }
  map.SynchronizeOptimizedReflectors({{"known", 2., 0., 0}});
  for (int i = 0; i < 10; ++i) {
    Check(Observe(&map, {{2., 0.}})[0] == 0, "Lost optimized ID");
  }
  Check(map.reflectors().size() == 1, "Promoted an optimized landmark twice");
}

void RepeatedSnapshotsAllowNewReflectors() {
  ReflectorMap map(ReflectorMapOptions{});
  map.AddReflector({"known", 0., 0., 0});
  map.SynchronizeOptimizedReflectors({{"known", 0., 0., 0}});
  for (int i = 0; i < 8; ++i) {
    map.SynchronizeOptimizedReflectors({{"known", 0., 0., 0}});
    Observe(&map, {{2., 0.}});
  }
  Check(map.reflectors().size() == 2,
        "Unchanged optimization snapshots prevented new IDs");
}

void DetectsBaselineCluster() {
  ReflectorDetector detector(ReflectorDetectorOptions{});
  const auto detections = detector.Detect(MakeScan({49, 50, 51}));
  Check(detections.size() == 1, "Expected one reflector");
  Check(detections[0].point_count == 3, "Wrong point count");
  Near(detections[0].x, 2., 1e-3);
  Near(detections[0].y, 0., 1e-6);
  Near(detections[0].mean_intensity, 1500., 1e-9);
}

void RejectsTooWideClusters() {
  ReflectorDetectorOptions options;
  options.max_missing_beams = 20;
  options.cluster_distance = 1.;
  options.max_cluster_width = 0.1;
  Check(ReflectorDetector(options).Detect(MakeScan({20, 30})).empty(),
        "Accepted an overly wide cluster");
}

void RequiresIntensityForEveryRange() {
  auto scan = MakeScan({49, 50, 51});
  ReflectorDetector detector(ReflectorDetectorOptions{});
  scan.intensities.clear();
  Check(detector.Detect(scan).empty(), "Accepted missing intensities");
  scan.intensities.assign(100, 1500.f);
  Check(detector.Detect(scan).empty(), "Accepted mismatched intensities");
}

void KeepsSeparatedReflectors() {
  ReflectorDetector detector(ReflectorDetectorOptions{});
  const auto detections = detector.Detect(MakeScan({20, 21, 70, 71}));
  Check(detections.size() == 2, "Merged spatially separated reflectors");
  Check(detections[0].point_count == 2 && detections[1].point_count == 2,
        "Lost points from separated reflectors");
}

void MergesSingletonFragments() {
  ReflectorDetector detector(ReflectorDetectorOptions{});
  const auto detections = detector.Detect(MakeScan({48, 52}));
  Check(detections.size() == 1 && detections[0].point_count == 2,
        "Discarded singleton fragments before merging");
  Check(detections[0].first_beam_index == 48 &&
            detections[0].last_beam_index == 52,
        "Lost original beam indices");
  Near(detections[0].width, 4. * std::sin(0.02), 1e-6);
}

void AppliesMinimumWidthAfterMerging() {
  ReflectorDetectorOptions options;
  options.min_cluster_width = 0.05;
  options.max_missing_beams = 0;
  ReflectorDetector detector(options);
  const auto detections = detector.Detect(MakeScan({48, 49, 51, 52}));
  Check(detections.size() == 1 && detections[0].point_count == 4,
        "Applied minimum width to fragments before merging");
}

void DisabledMergingStillFiltersSmallClusters() {
  ReflectorDetectorOptions options;
  options.detection_merge_distance = 0.;
  ReflectorDetector detector(options);
  Check(detector.Detect(MakeScan({48, 52})).empty(),
        "Disabled merging accepted singletons");
  Check(detector.Detect(MakeScan({48, 49, 53, 54})).size() == 2,
        "Disabled merging unexpectedly combined clusters");
}

void FiltersMergedWidthAndPointCount() {
  ReflectorDetectorOptions options;
  options.max_cluster_width = 0.06;
  options.detection_merge_distance = 0.06;
  options.max_missing_beams = 0;
  ReflectorDetector detector(options);
  Check(detector.Detect(MakeScan({48, 50, 52})).empty(),
        "Merged reflector exceeded maximum width");
  options.max_cluster_width = 0.2;
  options.detection_merge_distance = 0.1;
  options.max_cluster_points = 3;
  Check(ReflectorDetector(options).Detect(MakeScan({48, 49, 51, 52})).empty(),
        "Merged reflector exceeded maximum point count");
}

void CompensatesBeamMotionBeforeClustering() {
  auto scan = MakeScan({49, 50, 51});
  scan.time_increment = 0.001f;
  // A sensor moving in +x at 1 m/s observes the plane x=2. Reconstruct the
  // per-beam ranges, then compensate each beam into the scan-start frame.
  for (const int beam : {49, 50, 51}) {
    const double angle = scan.angle_min + beam * scan.angle_increment;
    scan.ranges[beam] = static_cast<float>(
        (2. - beam * static_cast<double>(scan.time_increment)) /
        std::cos(angle));
  }
  int transformed_beams = 0;
  ReflectorDetector detector(ReflectorDetectorOptions{});
  const auto detections = detector.Detect(
      scan, [&](const std::size_t index, const BeamPosition& point) {
        ++transformed_beams;
        return BeamPosition{
            point.x + index * static_cast<double>(scan.time_increment), point.y,
            0.03};
      });
  Check(transformed_beams == 3, "Transformed non-reflective beams");
  Check(detections.size() == 1, "Deskewed cluster was lost");
  Near(detections[0].x, 2., 1e-6);
  Near(detections[0].z, 0.03, 1e-12);
  const auto uncompensated = detector.Detect(scan);
  Near(uncompensated[0].x, 1.95, 1e-6);
}

void DoesNotReturnPartialDeskewedDetections() {
  ReflectorDetector detector(ReflectorDetectorOptions{});
  bool threw = false;
  try {
    detector.Detect(MakeScan({20, 21, 49, 50}),
                    [](const std::size_t beam, const BeamPosition& point) {
                      if (beam == 49) {
                        throw std::runtime_error("Missing beam transform");
                      }
                      return point;
                    });
  } catch (const std::runtime_error&) {
    threw = true;
  }
  Check(threw, "Returned partial detections after deskew failure");
}

void CompensatesRotatingSensor() {
  auto scan = MakeScan({49, 50, 51});
  scan.time_increment = 0.001f;
  for (const int beam : {49, 50, 51}) {
    scan.ranges[beam] = 10.f;
  }
  ReflectorDetectorOptions options;
  options.cluster_distance = 0.2;
  options.max_cluster_width = 0.3;
  const auto detections = ReflectorDetector(options).Detect(
      scan, [&](const std::size_t beam, const BeamPosition& point) {
        const double yaw = beam * static_cast<double>(scan.time_increment);
        return BeamPosition{std::cos(yaw) * point.x - std::sin(yaw) * point.y,
                            std::sin(yaw) * point.x + std::cos(yaw) * point.y,
                            point.z};
      });
  Check(detections.size() == 1, "Rotational deskew lost the reflector");
  Near(std::atan2(detections[0].y, detections[0].x), 0.05, 1e-6);
  Near(std::hypot(detections[0].x, detections[0].y),
       10. * (1. + 2. * std::cos(0.011)) / 3., 1e-6);
}

}  // namespace
}  // namespace reflector_landmark

int main() {
  using namespace reflector_landmark;
  const std::vector<std::pair<const char*, void (*)()>> tests = {
      {"PreservesNearestId", PreservesNearestId},
      {"RejectsAmbiguityWithoutNewIds", RejectsAmbiguityWithoutNewIds},
      {"PublishesOneObservationPerId", PublishesOneObservationPerId},
      {"ConfirmsNewIdAndKeepsItStable", ConfirmsNewIdAndKeepsItStable},
      {"SynchronizesOptimizationById", SynchronizesOptimizationById},
      {"RebasesOnlyUnoptimizedReflectors", RebasesOnlyUnoptimizedReflectors},
      {"RebasesConfirmationsAcrossMapCorrections",
       RebasesConfirmationsAcrossMapCorrections},
      {"OptimizationPreservesUnrelatedConfirmations",
       OptimizationPreservesUnrelatedConfirmations},
      {"OptimizationSuppressesPendingDuplicates",
       OptimizationSuppressesPendingDuplicates},
      {"RepeatedSnapshotsAllowNewReflectors",
       RepeatedSnapshotsAllowNewReflectors},
      {"DetectsBaselineCluster", DetectsBaselineCluster},
      {"RejectsTooWideClusters", RejectsTooWideClusters},
      {"RequiresIntensityForEveryRange", RequiresIntensityForEveryRange},
      {"KeepsSeparatedReflectors", KeepsSeparatedReflectors},
      {"MergesSingletonFragments", MergesSingletonFragments},
      {"AppliesMinimumWidthAfterMerging", AppliesMinimumWidthAfterMerging},
      {"DisabledMergingStillFiltersSmallClusters",
       DisabledMergingStillFiltersSmallClusters},
      {"FiltersMergedWidthAndPointCount", FiltersMergedWidthAndPointCount},
      {"CompensatesBeamMotionBeforeClustering",
       CompensatesBeamMotionBeforeClustering},
      {"DoesNotReturnPartialDeskewedDetections",
       DoesNotReturnPartialDeskewedDetections},
      {"CompensatesRotatingSensor", CompensatesRotatingSensor},
  };
  int failures = 0;
  for (const auto& test : tests) {
    try {
      test.second();
      std::cout << "PASS " << test.first << '\n';
    } catch (const std::exception& error) {
      ++failures;
      std::cerr << "FAIL " << test.first << ": " << error.what() << '\n';
    }
  }
  std::cout << tests.size() - failures << '/' << tests.size()
            << " regression tests passed.\n";
  return failures == 0 ? 0 : 1;
}
