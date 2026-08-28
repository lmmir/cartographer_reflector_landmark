#include "reflector_landmark/reflector_detector.h"

#include <cmath>

#include "gtest/gtest.h"
#include "sensor_msgs/LaserScan.h"

namespace reflector_landmark {
namespace {

sensor_msgs::LaserScan MakeScan() {
  sensor_msgs::LaserScan scan;
  scan.angle_min = -0.5;
  scan.angle_max = 0.5;
  scan.angle_increment = 0.01;
  scan.range_min = 0.1;
  scan.range_max = 30.;
  scan.ranges.assign(101, 5.);
  scan.intensities.assign(101, 10.);
  return scan;
}

TEST(ReflectorDetectorTest, DetectsReflectiveCluster) {
  auto scan = MakeScan();
  scan.ranges[49] = 2.;
  scan.ranges[50] = 2.;
  scan.ranges[51] = 2.;
  scan.intensities[49] = 1500.;
  scan.intensities[50] = 1600.;
  scan.intensities[51] = 1550.;

  ReflectorDetectorOptions options;
  options.intensity_threshold = 1000.;
  options.min_cluster_points = 2;
  ReflectorDetector detector(options);
  const auto detections = detector.Detect(scan);

  ASSERT_EQ(detections.size(), 1u);
  EXPECT_EQ(detections[0].point_count, 3);
  EXPECT_NEAR(detections[0].x, 2., 1e-3);
  EXPECT_NEAR(detections[0].y, 0., 1e-3);
  EXPECT_NEAR(detections[0].mean_intensity, 1550., 1e-6);
}

TEST(ReflectorDetectorTest, RejectsClusterThatIsTooWide) {
  auto scan = MakeScan();
  scan.ranges[20] = 2.;
  scan.ranges[30] = 2.;
  scan.intensities[20] = 1500.;
  scan.intensities[30] = 1500.;

  ReflectorDetectorOptions options;
  options.intensity_threshold = 1000.;
  options.min_cluster_points = 2;
  options.max_missing_beams = 20;
  options.cluster_distance = 1.;
  options.max_cluster_width = 0.1;
  ReflectorDetector detector(options);

  EXPECT_TRUE(detector.Detect(scan).empty());
}

TEST(ReflectorDetectorTest, RequiresIntensityForEveryRange) {
  auto scan = MakeScan();
  scan.intensities.clear();
  ReflectorDetector detector(ReflectorDetectorOptions{});
  EXPECT_TRUE(detector.Detect(scan).empty());
}

}  // namespace
}  // namespace reflector_landmark
