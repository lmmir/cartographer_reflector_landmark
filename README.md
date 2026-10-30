# reflector_landmark

`reflector_landmark` is an external ROS1 package that detects high-intensity
laser reflectors and sends them to the existing Cartographer landmark input.
It uses the existing Cartographer landmark input. Mapping also requires the
small bundled `cartographer_ros/map_builder_bridge.cc` change: optimized
landmark markers carry their original string IDs in `marker.text`, and all
markers in a snapshot share one timestamp. Rebuild `cartographer_ros` together
with this package. The Cartographer optimization algorithm is unchanged.

## Data flow

```text
sensor_msgs/LaserScan
  -> intensity filter, per-beam motion compensation and clustering
  -> map-frame data association
  -> cartographer_ros_msgs/LandmarkList
  -> /landmark
  -> Cartographer pose graph
```

The node has two modes:

- `mapping`: tracks detections, creates stable IDs after several observations,
  and updates an association map.
- `localization`: loads a fixed reflector map and only publishes unambiguous
  matches. It never creates new IDs.

## Build

Place the package in the same catkin source workspace as Cartographer, then:

```bash
catkin_make_isolated --install --use-ninja
source install_isolated/setup.bash
```

The package builds the detector/association libraries and these executables:

- `reflector_landmark_node`: detector, associator and landmark publisher.
- `reflector_map_exporter`: exports optimized landmark positions from a
  pbstream into the YAML format consumed by the node.
- `reflector_detector_test`: detector unit tests when testing is enabled.
- `reflector_regression_test`: ID association, optimized-map synchronization,
  split-cluster and motion-compensation regression tests.

Run the regression executable after building with testing enabled:

```bash
cd build_isolated/reflector_landmark
ctest -R reflector_regression_test --output-on-failure
```

## Cartographer configuration

The Cartographer trajectory must contain:

```lua
options.use_landmarks = true
options.landmarks_sampling_ratio = 1.0
TRAJECTORY_BUILDER.collate_landmarks = false
```

Landmarks affect the global pose graph, so reduce optimization latency if the
CPU budget permits it:

```lua
POSE_GRAPH.optimize_every_n_nodes = 10
```

Complete integration examples are in `configuration_files/`. They are
templates: adapt frames, laser counts, IMU/odometry use and ranges to the
robot.

Example Cartographer mapping command:

```bash
rosrun cartographer_ros cartographer_node \
  -configuration_directory "$(rospack find reflector_landmark)/configuration_files" \
  -configuration_basename reflector_2d_mapping.lua \
  -save_state_filename /tmp/map_with_reflectors.pbstream
```

## Mapping

First tune `config/reflector_detector.yaml` from recorded laser data. The
default intensity threshold is only a placeholder.

Duplicate-ID protection is controlled by three parameters:

- `max_missing_beams` bridges a short intensity dropout inside one reflector.
- `detection_merge_distance` merges remaining split clusters in the laser
  frame.
- `duplicate_suppression_distance` prevents a nearby mapping candidate from
  creating another ID. Real reflectors must be farther apart than this value.

The shipped values (`2` beams, `0.10 m`, and `0.15 m`) are conservative
starting points. Verify them from a rosbag at the closest and farthest working
distances before building the production map.

Start the detector beside Cartographer:

```bash
roslaunch reflector_landmark reflector_mapping.launch \
  scan_topic:=/scan \
  map_file:=/tmp/reflector_map_online.yaml
```

The node needs a continuous `local_frame` (default `odom`), TF from that frame
to the laser at every reflective beam's acquisition time, and the latest
`map_frame -> local_frame` correction. The supplied Lua examples provide this
TF chain. If an external odometry publisher is used, ensure it owns the
continuous local frame. A moving robot frame or `map` itself is unsuitable.

Each accepted beam is transformed using `header.stamp + index * time_increment`
and deskewed into the laser frame at scan start, before clustering. Both
published landmarks and detection markers therefore retain the original scan
header. A zero `time_increment` is treated as an instantaneous scan. Missing TF
causes the scan to be skipped, with no partially compensated observations.

In mapping mode the node subscribes to `optimized_landmark_topic` (default
`/landmark_poses_list`) and updates coordinates by string ID. After a global
TF correction it pauses association until a current optimized snapshot is
available. Pending confirmations and new landmarks without optimized poses are
rebased together with the global correction; only candidates overlapping an
existing optimized landmark are suppressed, so optimization does not prevent
unrelated new landmarks from being confirmed.
Already optimized landmarks keep their authoritative pose until the next
snapshot. Ambiguous matches are suppressed without publishing an arbitrary ID.
An unmodified Cartographer publisher with unnamed markers is rejected with an
error rather than used as an association map.

Save the online association map on demand:

```bash
rosservice call /reflector_landmark/save_map
```

After Cartographer finishes its final optimization and writes the pbstream,
export the optimized positions. Prefer this file over the online association
map for subsequent localization:

```bash
rosrun reflector_landmark reflector_map_exporter \
  /tmp/map_with_reflectors.pbstream \
  /tmp/reflector_map.yaml \
  map \
  reflector_
```

The final argument filters landmark IDs. Pass an empty string to export every
landmark in a mixed-landmark pbstream.

## Localization

Start Cartographer with the matching pbstream and the localization Lua file:

```bash
rosrun cartographer_ros cartographer_node \
  -configuration_directory "$(rospack find reflector_landmark)/configuration_files" \
  -configuration_basename reflector_2d_localization.lua \
  -load_state_filename /tmp/map_with_reflectors.pbstream
```

Start reflector association with the exported YAML file:

```bash
roslaunch reflector_landmark reflector_localization.launch \
  scan_topic:=/scan \
  map_file:=/tmp/reflector_map.yaml
```

The localization associator requires a sufficiently accurate current
`map -> laser` estimate. Use the existing initial-pose/relocation interface
before enabling landmark association after a cold start. Global reflector-only
RANSAC initialization is intentionally not performed by this package.

## Interfaces

Inputs:

- `scan_topic` (`sensor_msgs/LaserScan`), default `scan`.
- TF from the scan frame to `local_frame` at beam acquisition times, and the
  latest `map_frame -> local_frame` correction.
- `optimized_landmark_topic` (`visualization_msgs/MarkerArray`) in mapping mode,
  with IDs in `marker.text` from the bundled Cartographer ROS change.
- private `reflectors` parameter in localization mode.

Outputs:

- `landmark_topic` (`cartographer_ros_msgs/LandmarkList`), default `landmark`.
- `~detections` (`visualization_msgs/MarkerArray`): green is associated, red is
  rejected or pending.
- `~map` (`visualization_msgs/MarkerArray`): current reflector association map.

Services:

- `~save_map` (`std_srvs/Trigger`).

## Important constraints

- `LaserScan.intensities` must exist and have the same length as `ranges`.
- A physical reflector must keep the same ID. Bad associations create bad
  pose-graph constraints; uncertain detections are therefore discarded.
- If an earlier run created many IDs at the same reflector, discard that
  reflector YAML and pbstream and rebuild the map after applying the duplicate
  guards. Filtering the YAML alone cannot remove the old landmark constraints
  already stored in a pbstream.
- Point reflectors have no reliable orientation. Keep `rotation_weight: 0.0`.
- Plain reflectors are indistinguishable. A cold global initialization needs
  an external initial pose, encoded reflectors, or a separate constellation
  matching step.
