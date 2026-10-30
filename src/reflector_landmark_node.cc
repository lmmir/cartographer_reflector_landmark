#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "XmlRpcValue.h"
#include "cartographer_ros_msgs/LandmarkEntry.h"
#include "cartographer_ros_msgs/LandmarkList.h"
#include "geometry_msgs/TransformStamped.h"
#include "reflector_landmark/reflector_detector.h"
#include "reflector_landmark/reflector_map.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Trigger.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

namespace reflector_landmark {
namespace {

std::string EscapeYamlString(const std::string& value) {
  std::string escaped;
  escaped.reserve(value.size());
  for (const char character : value) {
    if (character == '\\' || character == '"') {
      escaped.push_back('\\');
    }
    escaped.push_back(character);
  }
  return escaped;
}

bool ReadNumber(XmlRpc::XmlRpcValue* object, const std::string& key,
                double* result) {
  if (!object->hasMember(key)) {
    return false;
  }
  XmlRpc::XmlRpcValue& value = (*object)[key];
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    *result = static_cast<double>(value);
    return true;
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    *result = static_cast<int>(value);
    return true;
  }
  return false;
}

}  // namespace

class ReflectorLandmarkNode {
 public:
  ReflectorLandmarkNode()
      : node_handle_(), private_node_handle_("~"), tf_listener_(tf_buffer_) {
    LoadParameters();
    detector_.reset(new ReflectorDetector(detector_options_));
    ReflectorMapOptions map_options;
    map_options.association_distance = association_distance_;
    map_options.association_margin = association_margin_;
    map_options.pending_association_distance = pending_association_distance_;
    map_options.duplicate_suppression_distance =
        duplicate_suppression_distance_;
    map_options.minimum_confirmations = minimum_confirmations_;
    map_options.maximum_pending_missed_scans = maximum_pending_missed_scans_;
    map_options.map_update_gain = map_update_gain_;
    map_options.id_prefix = id_prefix_;
    map_options.id_width = id_width_;
    reflector_map_.reset(new ReflectorMap(map_options));
    LoadReflectorsFromParameters();

    landmark_publisher_ =
        node_handle_.advertise<cartographer_ros_msgs::LandmarkList>(
            landmark_topic_, 10);
    detection_marker_publisher_ =
        private_node_handle_.advertise<visualization_msgs::MarkerArray>(
            "detections", 1);
    map_marker_publisher_ =
        private_node_handle_.advertise<visualization_msgs::MarkerArray>(
            "map", 1, true);
    save_map_service_ = private_node_handle_.advertiseService(
        "save_map", &ReflectorLandmarkNode::HandleSaveMap, this);
    if (mapping_mode_) {
      optimized_landmark_subscriber_ = node_handle_.subscribe(
          optimized_landmark_topic_, 1,
          &ReflectorLandmarkNode::HandleOptimizedLandmarks, this);
    }
    scan_subscriber_ = node_handle_.subscribe(
        scan_topic_, 10, &ReflectorLandmarkNode::HandleScan, this);

    PublishMapMarkers(ros::Time::now());
    ROS_INFO_STREAM("Reflector landmark node started in '"
                    << mode_ << "' mode with "
                    << reflector_map_->reflectors().size()
                    << " known reflectors.");
    ROS_INFO_STREAM(
        "Reflector duplicate guards: max_missing_beams="
        << detector_options_.max_missing_beams << ", detection_merge_distance="
        << detector_options_.detection_merge_distance
        << " m, duplicate_suppression_distance="
        << duplicate_suppression_distance_
        << " m, minimum_confirmations=" << minimum_confirmations_ << ".");
  }

  ~ReflectorLandmarkNode() {
    if (mapping_mode_ && save_on_shutdown_ && !map_file_.empty()) {
      std::string error;
      if (!SaveReflectorMap(&error)) {
        ROS_ERROR_STREAM(
            "Could not save reflector map during shutdown: " << error);
      }
    }
  }

 private:
  void LoadParameters() {
    private_node_handle_.param<std::string>("mode", mode_, "mapping");
    std::transform(mode_.begin(), mode_.end(), mode_.begin(),
                   [](const unsigned char character) {
                     return static_cast<char>(std::tolower(character));
                   });
    if (mode_ != "mapping" && mode_ != "localization") {
      throw std::runtime_error(
          "Parameter '~mode' must be 'mapping' or 'localization'.");
    }
    mapping_mode_ = mode_ == "mapping";

    private_node_handle_.param<std::string>("scan_topic", scan_topic_, "scan");
    private_node_handle_.param<std::string>("landmark_topic", landmark_topic_,
                                            "landmark");
    private_node_handle_.param<std::string>("map_frame", map_frame_, "map");
    private_node_handle_.param<std::string>("local_frame", local_frame_,
                                            "odom");
    private_node_handle_.param<std::string>("optimized_landmark_topic",
                                            optimized_landmark_topic_,
                                            "/landmark_poses_list");
    private_node_handle_.param<std::string>("map_file", map_file_, "");
    private_node_handle_.param<std::string>("id_prefix", id_prefix_,
                                            "reflector_");
    private_node_handle_.param("id_width", id_width_, 4);
    private_node_handle_.param("save_on_shutdown", save_on_shutdown_, true);

    private_node_handle_.param("intensity_threshold",
                               detector_options_.intensity_threshold, 1000.);
    private_node_handle_.param("min_range", detector_options_.min_range, 0.2);
    private_node_handle_.param("max_range", detector_options_.max_range, 20.);
    private_node_handle_.param("cluster_distance",
                               detector_options_.cluster_distance, 0.08);
    private_node_handle_.param("min_cluster_width",
                               detector_options_.min_cluster_width, 0.);
    private_node_handle_.param("max_cluster_width",
                               detector_options_.max_cluster_width, 0.2);
    private_node_handle_.param("min_cluster_points",
                               detector_options_.min_cluster_points, 2);
    private_node_handle_.param("max_cluster_points",
                               detector_options_.max_cluster_points, 30);
    private_node_handle_.param("max_missing_beams",
                               detector_options_.max_missing_beams, 2);
    private_node_handle_.param("detection_merge_distance",
                               detector_options_.detection_merge_distance, 0.1);

    private_node_handle_.param("association_distance", association_distance_,
                               0.3);
    private_node_handle_.param("association_margin", association_margin_, 0.05);
    private_node_handle_.param("pending_association_distance",
                               pending_association_distance_, 0.2);
    private_node_handle_.param("duplicate_suppression_distance",
                               duplicate_suppression_distance_, 0.15);
    private_node_handle_.param("minimum_confirmations", minimum_confirmations_,
                               8);
    private_node_handle_.param("maximum_pending_missed_scans",
                               maximum_pending_missed_scans_, 5);
    private_node_handle_.param("map_update_gain", map_update_gain_, 0.1);
    private_node_handle_.param("translation_weight", translation_weight_, 20.);
    private_node_handle_.param("rotation_weight", rotation_weight_, 0.);
    private_node_handle_.param("tf_lookup_timeout_sec", tf_lookup_timeout_sec_,
                               0.05);
    private_node_handle_.param("detection_marker_scale",
                               detection_marker_scale_, 0.12);
    private_node_handle_.param("map_marker_scale", map_marker_scale_, 0.18);

    if (map_frame_.empty() || local_frame_.empty() ||
        map_frame_ == local_frame_) {
      throw std::runtime_error(
          "Parameters '~map_frame' and '~local_frame' must be nonempty and "
          "distinct; '~local_frame' must be a continuous odometry frame.");
    }
    if (association_distance_ <= 0. || association_margin_ < 0. ||
        pending_association_distance_ <= 0. ||
        duplicate_suppression_distance_ <= 0. || minimum_confirmations_ <= 0 ||
        maximum_pending_missed_scans_ < 0 || map_update_gain_ < 0. ||
        map_update_gain_ > 1. || translation_weight_ <= 0. ||
        rotation_weight_ < 0. || tf_lookup_timeout_sec_ < 0. ||
        id_width_ <= 0) {
      throw std::runtime_error("Invalid reflector landmark parameters.");
    }
  }

  void LoadReflectorsFromParameters() {
    XmlRpc::XmlRpcValue reflector_list;
    if (!private_node_handle_.getParam("reflectors", reflector_list)) {
      if (!mapping_mode_) {
        ROS_WARN("No '~reflectors' parameter was loaded for localization.");
      }
      return;
    }
    if (reflector_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      throw std::runtime_error("Parameter '~reflectors' must be a YAML list.");
    }

    for (int i = 0; i < reflector_list.size(); ++i) {
      XmlRpc::XmlRpcValue& value = reflector_list[i];
      if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
          !value.hasMember("id") ||
          value["id"].getType() != XmlRpc::XmlRpcValue::TypeString) {
        ROS_WARN_STREAM("Skipping invalid reflector map entry " << i << ".");
        continue;
      }
      Reflector reflector;
      reflector.id = static_cast<std::string>(value["id"]);
      if (!ReadNumber(&value, "x", &reflector.x) ||
          !ReadNumber(&value, "y", &reflector.y) || reflector.id.empty() ||
          !std::isfinite(reflector.x) || !std::isfinite(reflector.y) ||
          reflector_map_->HasReflectorId(reflector.id)) {
        ROS_WARN_STREAM("Skipping invalid or duplicate reflector '"
                        << reflector.id << "'.");
        continue;
      }
      const int canonical_index = reflector_map_->FindCanonicalReflector(
          {reflector.x, reflector.y}, duplicate_suppression_distance_);
      if (canonical_index >= 0) {
        ROS_WARN_STREAM("Skipping spatially duplicate reflector '"
                        << reflector.id << "' near canonical reflector '"
                        << reflector_map_->reflectors()[canonical_index].id
                        << "'.");
        continue;
      }
      double observations = 0.;
      if (ReadNumber(&value, "observations", &observations) &&
          std::isfinite(observations) && observations >= 0. &&
          observations <= std::numeric_limits<int>::max()) {
        reflector.observations = std::max(0, static_cast<int>(observations));
      }
      reflector_map_->AddReflector(reflector);
    }
  }

  void UpdateMapFrame() {
    // Use the latest global correction with historical local poses. Looking
    // up map->laser at the old scan stamp would mix optimization epochs when
    // optimized landmark poses have already advanced to the current map.
    const auto transform =
        tf_buffer_.lookupTransform(map_frame_, local_frame_, ros::Time(0),
                                   ros::Duration(tf_lookup_timeout_sec_));
    tf2::Transform map_from_local;
    tf2::fromMsg(transform.transform, map_from_local);
    if (have_map_frame_ && mapping_mode_) {
      const tf2::Transform correction =
          map_from_local * map_from_local_.inverse();
      if (correction.getOrigin().length() > 1e-6 ||
          correction.getRotation().getAngleShortestPath() > 1e-6) {
        reflector_map_->RebaseUnoptimizedState(
            correction.getOrigin().x(), correction.getOrigin().y(),
            tf2::getYaw(correction.getRotation()));
        map_correction_stamp_ = transform.header.stamp;
        // Optimized landmarks need a new authoritative snapshot; merely
        // translating their old poses cannot account for map deformation.
        waiting_for_optimized_map_ = waiting_for_optimized_map_ ||
                                     reflector_map_->HasOptimizedReflectors();
      }
    }
    map_from_local_ = map_from_local;
    have_map_frame_ = true;
  }

  void HandleOptimizedLandmarks(
      const visualization_msgs::MarkerArray::ConstPtr& message) {
    std::vector<Reflector> optimized;
    ros::Time stamp;
    for (const auto& marker : message->markers) {
      if (marker.ns != "Landmarks" ||
          marker.action != visualization_msgs::Marker::ADD) {
        continue;
      }
      if (marker.text.empty()) {
        waiting_for_optimized_map_ = true;
        ROS_ERROR_THROTTLE(
            5.,
            "Optimized landmark markers lack string IDs. Rebuild "
            "cartographer_ros with the landmark marker.text change.");
        return;
      }
      if (marker.header.frame_id != map_frame_ ||
          !std::isfinite(marker.pose.position.x) ||
          !std::isfinite(marker.pose.position.y)) {
        ROS_WARN_THROTTLE(5., "Ignoring invalid optimized landmark snapshot.");
        return;
      }
      if (!optimized.empty() && marker.header.stamp != stamp) {
        return;
      }
      stamp = marker.header.stamp;
      optimized.push_back({marker.text, marker.pose.position.x,
                           marker.pose.position.y, 0, true});
    }
    if (optimized.empty()) {
      return;
    }
    try {
      // Wait for TF to cover the snapshot time, then reject a snapshot from
      // an older correction. TF has its own listener thread during this wait.
      const auto snapshot_transform =
          tf_buffer_.lookupTransform(map_frame_, local_frame_, stamp,
                                     ros::Duration(tf_lookup_timeout_sec_));
      UpdateMapFrame();
      tf2::Transform snapshot_map_from_local;
      tf2::fromMsg(snapshot_transform.transform, snapshot_map_from_local);
      const tf2::Transform difference =
          map_from_local_ * snapshot_map_from_local.inverse();
      if (stamp < map_correction_stamp_ || stamp < optimized_map_stamp_ ||
          difference.getOrigin().length() > 1e-6 ||
          difference.getRotation().getAngleShortestPath() > 1e-6) {
        return;
      }
    } catch (const tf2::TransformException& error) {
      ROS_WARN_THROTTLE(1., "Cannot synchronize optimized reflectors: %s",
                        error.what());
      return;
    }
    reflector_map_->SynchronizeOptimizedReflectors(optimized);
    optimized_map_stamp_ = stamp;
    waiting_for_optimized_map_ = false;
    PublishMapMarkers(stamp);
  }

  std::vector<ReflectorDetection> DetectDeskewed(
      const sensor_msgs::LaserScan& scan, tf2::Transform* local_from_scan) {
    const auto reference = tf_buffer_.lookupTransform(
        local_frame_, scan.header.frame_id, scan.header.stamp,
        ros::Duration(tf_lookup_timeout_sec_));
    tf2::fromMsg(reference.transform, *local_from_scan);
    const tf2::Transform scan_from_local = local_from_scan->inverse();
    return detector_->Detect(
        scan, [&](const std::size_t index, const BeamPosition& point) {
          if (index == 0 || scan.time_increment == 0.f) {
            return point;
          }
          const ros::Time beam_stamp =
              scan.header.stamp +
              ros::Duration(index * static_cast<double>(scan.time_increment));
          const auto beam_transform = tf_buffer_.lookupTransform(
              local_frame_, scan.header.frame_id, beam_stamp,
              ros::Duration(tf_lookup_timeout_sec_));
          tf2::Transform local_from_beam;
          tf2::fromMsg(beam_transform.transform, local_from_beam);
          const tf2::Vector3 deskewed = scan_from_local * local_from_beam *
                                        tf2::Vector3(point.x, point.y, point.z);
          return BeamPosition{deskewed.x(), deskewed.y(), deskewed.z()};
        });
  }

  void PublishLandmarks(const sensor_msgs::LaserScan& scan,
                        const std::vector<ReflectorDetection>& detections,
                        const std::vector<int>& associations) {
    cartographer_ros_msgs::LandmarkList message;
    message.header = scan.header;
    for (std::size_t i = 0; i < associations.size(); ++i) {
      if (associations[i] < 0) {
        continue;
      }
      cartographer_ros_msgs::LandmarkEntry entry;
      entry.id = reflector_map_->reflectors()[associations[i]].id;
      entry.tracking_from_landmark_transform.position.x = detections[i].x;
      entry.tracking_from_landmark_transform.position.y = detections[i].y;
      entry.tracking_from_landmark_transform.position.z = detections[i].z;
      entry.tracking_from_landmark_transform.orientation.w = 1.;
      entry.translation_weight = translation_weight_;
      entry.rotation_weight = rotation_weight_;
      message.landmarks.push_back(entry);
    }
    if (!message.landmarks.empty()) {
      landmark_publisher_.publish(message);
    }
  }

  void PublishDetectionMarkers(
      const sensor_msgs::LaserScan& scan,
      const std::vector<ReflectorDetection>& detections,
      const std::vector<int>& associations) {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(clear);

    for (std::size_t i = 0; i < detections.size(); ++i) {
      visualization_msgs::Marker marker;
      marker.header = scan.header;
      marker.ns = "reflector_detections";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = detections[i].x;
      marker.pose.position.y = detections[i].y;
      marker.pose.position.z = detections[i].z;
      marker.pose.orientation.w = 1.;
      marker.scale.x = detection_marker_scale_;
      marker.scale.y = detection_marker_scale_;
      marker.scale.z = detection_marker_scale_;
      marker.color.a = 1.;
      if (i < associations.size() && associations[i] >= 0) {
        marker.color.g = 1.;
      } else {
        marker.color.r = 1.;
      }
      marker.lifetime = ros::Duration(0.3);
      marker_array.markers.push_back(marker);
    }
    detection_marker_publisher_.publish(marker_array);
  }

  void PublishMapMarkers(const ros::Time& stamp) {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(clear);

    for (std::size_t i = 0; i < reflector_map_->reflectors().size(); ++i) {
      const Reflector& reflector = reflector_map_->reflectors()[i];
      visualization_msgs::Marker sphere;
      sphere.header.frame_id = map_frame_;
      sphere.header.stamp = stamp;
      sphere.ns = "reflector_map";
      sphere.id = static_cast<int>(2 * i);
      sphere.type = visualization_msgs::Marker::SPHERE;
      sphere.action = visualization_msgs::Marker::ADD;
      sphere.pose.position.x = reflector.x;
      sphere.pose.position.y = reflector.y;
      sphere.pose.orientation.w = 1.;
      sphere.scale.x = map_marker_scale_;
      sphere.scale.y = map_marker_scale_;
      sphere.scale.z = map_marker_scale_;
      sphere.color.b = 1.;
      sphere.color.a = 1.;
      marker_array.markers.push_back(sphere);

      visualization_msgs::Marker text = sphere;
      text.ns = "reflector_ids";
      text.id = static_cast<int>(2 * i + 1);
      text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text.pose.position.z = 0.25;
      text.scale.x = 0.;
      text.scale.y = 0.;
      text.scale.z = 0.15;
      text.color.r = 1.;
      text.color.g = 1.;
      text.color.b = 1.;
      text.text = reflector.id;
      marker_array.markers.push_back(text);
    }
    map_marker_publisher_.publish(marker_array);
  }

  void HandleScan(const sensor_msgs::LaserScan::ConstPtr& scan) {
    if (scan->header.frame_id.empty()) {
      ROS_WARN_THROTTLE(1., "Ignoring LaserScan with an empty frame_id.");
      return;
    }
    if (scan->intensities.size() != scan->ranges.size()) {
      ROS_WARN_THROTTLE(
          1., "LaserScan intensities are empty or do not match ranges.");
      return;
    }
    if (!std::isfinite(scan->time_increment) || scan->time_increment < 0.f ||
        (scan->time_increment > 0.f && scan->header.stamp.isZero())) {
      ROS_WARN_THROTTLE(1., "Ignoring LaserScan with invalid beam timestamps.");
      return;
    }

    std::vector<ReflectorDetection> detections;
    tf2::Transform local_from_scan;
    try {
      UpdateMapFrame();
      detections = DetectDeskewed(*scan, &local_from_scan);
    } catch (const tf2::TransformException& error) {
      if (mapping_mode_) {
        reflector_map_->AgePendingTracks();
      }
      ROS_WARN_THROTTLE(1., "Cannot transform/deskew reflectors: %s",
                        error.what());
      return;
    }
    if (detections.empty()) {
      if (mapping_mode_) {
        reflector_map_->AgePendingTracks();
      }
      PublishDetectionMarkers(*scan, detections, {});
      return;
    }

    if (mapping_mode_ && waiting_for_optimized_map_) {
      ROS_WARN_THROTTLE(1., "Waiting for optimized reflector coordinates.");
      PublishDetectionMarkers(*scan, detections,
                              std::vector<int>(detections.size(), -1));
      return;
    }

    std::vector<MapPoint> map_points;
    map_points.reserve(detections.size());
    const tf2::Transform map_from_scan = map_from_local_ * local_from_scan;
    for (const auto& detection : detections) {
      const tf2::Vector3 point =
          map_from_scan * tf2::Vector3(detection.x, detection.y, detection.z);
      map_points.push_back({point.x(), point.y()});
    }
    std::vector<int> associations =
        reflector_map_->AssociateKnownReflectors(map_points);
    if (mapping_mode_) {
      reflector_map_->UpdateMapping(map_points, &associations);
    }
    PublishLandmarks(*scan, detections, associations);
    PublishDetectionMarkers(*scan, detections, associations);
    PublishMapMarkers(scan->header.stamp);
  }

  bool SaveReflectorMap(std::string* error) const {
    if (map_file_.empty()) {
      *error = "parameter '~map_file' is empty";
      return false;
    }
    std::ofstream output(map_file_);
    if (!output.is_open()) {
      *error = "cannot open '" + map_file_ + "' for writing";
      return false;
    }

    std::vector<Reflector> sorted_reflectors = reflector_map_->reflectors();
    std::sort(sorted_reflectors.begin(), sorted_reflectors.end(),
              [](const Reflector& lhs, const Reflector& rhs) {
                return lhs.id < rhs.id;
              });
    output << std::setprecision(17);
    output << "map_frame: \"" << EscapeYamlString(map_frame_) << "\"\n";
    output << (sorted_reflectors.empty() ? "reflectors: []\n"
                                         : "reflectors:\n");
    for (const auto& reflector : sorted_reflectors) {
      output << "  - id: \"" << EscapeYamlString(reflector.id) << "\"\n"
             << "    x: " << reflector.x << "\n"
             << "    y: " << reflector.y << "\n"
             << "    observations: " << reflector.observations << "\n";
    }
    output.close();
    if (!output) {
      *error = "failed while writing '" + map_file_ + "'";
      return false;
    }
    return true;
  }

  bool HandleSaveMap(std_srvs::Trigger::Request& /*request*/,
                     std_srvs::Trigger::Response& response) {
    std::string error;
    response.success = SaveReflectorMap(&error);
    if (response.success) {
      response.message = "Saved " +
                         std::to_string(reflector_map_->reflectors().size()) +
                         " reflectors to '" + map_file_ + "'.";
      ROS_INFO_STREAM(response.message);
    } else {
      response.message = error;
      ROS_ERROR_STREAM("Could not save reflector map: " << error);
    }
    return true;
  }

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ReflectorDetectorOptions detector_options_;
  std::unique_ptr<ReflectorDetector> detector_;
  std::unique_ptr<ReflectorMap> reflector_map_;

  ros::Subscriber scan_subscriber_;
  ros::Subscriber optimized_landmark_subscriber_;
  ros::Publisher landmark_publisher_;
  ros::Publisher detection_marker_publisher_;
  ros::Publisher map_marker_publisher_;
  ros::ServiceServer save_map_service_;

  std::string mode_;
  bool mapping_mode_ = true;
  std::string scan_topic_;
  std::string landmark_topic_;
  std::string map_frame_;
  std::string local_frame_;
  std::string optimized_landmark_topic_;
  tf2::Transform map_from_local_;
  bool have_map_frame_ = false;
  bool waiting_for_optimized_map_ = false;
  ros::Time map_correction_stamp_;
  ros::Time optimized_map_stamp_;
  std::string map_file_;
  std::string id_prefix_;
  int id_width_ = 4;
  bool save_on_shutdown_ = true;

  double association_distance_ = 0.3;
  double association_margin_ = 0.05;
  double pending_association_distance_ = 0.2;
  double duplicate_suppression_distance_ = 0.15;
  int minimum_confirmations_ = 8;
  int maximum_pending_missed_scans_ = 5;
  double map_update_gain_ = 0.1;
  double translation_weight_ = 20.;
  double rotation_weight_ = 0.;
  double tf_lookup_timeout_sec_ = 0.05;
  double detection_marker_scale_ = 0.12;
  double map_marker_scale_ = 0.18;
};

}  // namespace reflector_landmark

int main(int argc, char** argv) {
  ros::init(argc, argv, "reflector_landmark");
  try {
    reflector_landmark::ReflectorLandmarkNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("Reflector landmark node failed: " << error.what());
    return 1;
  }
  return 0;
}
