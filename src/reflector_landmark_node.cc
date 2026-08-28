#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "XmlRpcValue.h"
#include "cartographer_ros_msgs/LandmarkEntry.h"
#include "cartographer_ros_msgs/LandmarkList.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "reflector_landmark/reflector_detector.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Trigger.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

namespace reflector_landmark {
namespace {

struct MapPoint {
  double x = 0.;
  double y = 0.;
};

struct Reflector {
  std::string id;
  double x = 0.;
  double y = 0.;
  int observations = 0;
};

struct PendingTrack {
  double x = 0.;
  double y = 0.;
  int hits = 0;
  int missed_scans = 0;
};

struct AssociationCandidate {
  std::size_t detection_index;
  std::size_t target_index;
  double distance;
};

double Distance(const MapPoint& point, const Reflector& reflector) {
  return std::hypot(point.x - reflector.x, point.y - reflector.y);
}

double Distance(const MapPoint& point, const PendingTrack& track) {
  return std::hypot(point.x - track.x, point.y - track.y);
}

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
      : node_handle_(),
        private_node_handle_("~"),
        tf_listener_(tf_buffer_) {
    LoadParameters();
    detector_.reset(new ReflectorDetector(detector_options_));
    LoadReflectorsFromParameters();

    landmark_publisher_ = node_handle_.advertise<
        cartographer_ros_msgs::LandmarkList>(landmark_topic_, 10);
    detection_marker_publisher_ = private_node_handle_.advertise<
        visualization_msgs::MarkerArray>("detections", 1);
    map_marker_publisher_ = private_node_handle_.advertise<
        visualization_msgs::MarkerArray>("map", 1, true);
    save_map_service_ = private_node_handle_.advertiseService(
        "save_map", &ReflectorLandmarkNode::HandleSaveMap, this);
    scan_subscriber_ = node_handle_.subscribe(
        scan_topic_, 10, &ReflectorLandmarkNode::HandleScan, this);

    PublishMapMarkers(ros::Time::now());
    ROS_INFO_STREAM("Reflector landmark node started in '" << mode_
                    << "' mode with " << reflectors_.size()
                    << " known reflectors.");
  }

  ~ReflectorLandmarkNode() {
    if (mapping_mode_ && save_on_shutdown_ && !map_file_.empty()) {
      std::string error;
      if (!SaveReflectorMap(&error)) {
        ROS_ERROR_STREAM("Could not save reflector map during shutdown: "
                         << error);
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

    private_node_handle_.param<std::string>("scan_topic", scan_topic_,
                                             "scan");
    private_node_handle_.param<std::string>("landmark_topic", landmark_topic_,
                                             "landmark");
    private_node_handle_.param<std::string>("map_frame", map_frame_, "map");
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
                               detector_options_.max_missing_beams, 0);

    private_node_handle_.param("association_distance", association_distance_,
                               0.3);
    private_node_handle_.param("association_margin", association_margin_,
                               0.05);
    private_node_handle_.param("pending_association_distance",
                               pending_association_distance_, 0.2);
    private_node_handle_.param("minimum_confirmations",
                               minimum_confirmations_, 3);
    private_node_handle_.param("maximum_pending_missed_scans",
                               maximum_pending_missed_scans_, 5);
    private_node_handle_.param("map_update_gain", map_update_gain_, 0.1);
    private_node_handle_.param("translation_weight", translation_weight_,
                               20.);
    private_node_handle_.param("rotation_weight", rotation_weight_, 0.);
    private_node_handle_.param("tf_lookup_timeout_sec", tf_lookup_timeout_sec_,
                               0.05);
    private_node_handle_.param("detection_marker_scale",
                               detection_marker_scale_, 0.12);
    private_node_handle_.param("map_marker_scale", map_marker_scale_, 0.18);

    if (association_distance_ <= 0. || association_margin_ < 0. ||
        pending_association_distance_ <= 0. || minimum_confirmations_ <= 0 ||
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
          !ReadNumber(&value, "y", &reflector.y) ||
          HasReflectorId(reflector.id)) {
        ROS_WARN_STREAM("Skipping invalid or duplicate reflector '"
                        << reflector.id << "'.");
        continue;
      }
      double observations = 0.;
      if (ReadNumber(&value, "observations", &observations)) {
        reflector.observations = std::max(0, static_cast<int>(observations));
      }
      reflectors_.push_back(reflector);
    }
  }

  bool HasReflectorId(const std::string& id) const {
    return std::any_of(reflectors_.begin(), reflectors_.end(),
                       [&id](const Reflector& reflector) {
                         return reflector.id == id;
                       });
  }

  std::string GenerateReflectorId() {
    while (true) {
      std::ostringstream stream;
      stream << id_prefix_ << std::setfill('0') << std::setw(id_width_)
             << next_reflector_id_++;
      if (!HasReflectorId(stream.str())) {
        return stream.str();
      }
    }
  }

  bool TransformToMap(
      const sensor_msgs::LaserScan& scan,
      const std::vector<ReflectorDetection>& detections,
      std::vector<MapPoint>* map_points) {
    geometry_msgs::TransformStamped map_from_scan;
    try {
      map_from_scan = tf_buffer_.lookupTransform(
          map_frame_, scan.header.frame_id, scan.header.stamp,
          ros::Duration(tf_lookup_timeout_sec_));
    } catch (const tf2::TransformException& error) {
      ROS_WARN_THROTTLE(1., "Cannot transform reflector detections to %s: %s",
                        map_frame_.c_str(), error.what());
      return false;
    }

    map_points->clear();
    map_points->reserve(detections.size());
    for (const auto& detection : detections) {
      geometry_msgs::PointStamped point_in_scan;
      point_in_scan.header = scan.header;
      point_in_scan.point.x = detection.x;
      point_in_scan.point.y = detection.y;
      point_in_scan.point.z = 0.;
      geometry_msgs::PointStamped point_in_map;
      tf2::doTransform(point_in_scan, point_in_map, map_from_scan);
      map_points->push_back({point_in_map.point.x, point_in_map.point.y});
    }
    return true;
  }

  std::vector<int> AssociateKnownReflectors(
      const std::vector<MapPoint>& map_points) const {
    std::vector<int> associations(map_points.size(), -1);
    if (reflectors_.empty()) {
      return associations;
    }

    std::vector<double> best_distance(
        map_points.size(), std::numeric_limits<double>::infinity());
    std::vector<double> second_best_distance(
        map_points.size(), std::numeric_limits<double>::infinity());
    std::vector<std::size_t> best_reflector_index(
        map_points.size(), reflectors_.size());
    std::vector<AssociationCandidate> candidates;
    for (std::size_t detection_index = 0;
         detection_index < map_points.size(); ++detection_index) {
      for (std::size_t reflector_index = 0;
           reflector_index < reflectors_.size(); ++reflector_index) {
        const double distance = Distance(map_points[detection_index],
                                         reflectors_[reflector_index]);
        if (distance < best_distance[detection_index]) {
          second_best_distance[detection_index] =
              best_distance[detection_index];
          best_distance[detection_index] = distance;
          best_reflector_index[detection_index] = reflector_index;
        } else if (distance < second_best_distance[detection_index]) {
          second_best_distance[detection_index] = distance;
        }
      }
      const double second_best = second_best_distance[detection_index];
      const bool ambiguous =
          std::isfinite(second_best) &&
          second_best - best_distance[detection_index] < association_margin_;
      if (!ambiguous && best_distance[detection_index] <=
                            association_distance_) {
        candidates.push_back({detection_index,
                              best_reflector_index[detection_index],
                              best_distance[detection_index]});
      }
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const AssociationCandidate& lhs,
                 const AssociationCandidate& rhs) {
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

  void UpdateMapping(const std::vector<MapPoint>& map_points,
                     std::vector<int>* associations) {
    std::vector<bool> detection_used(map_points.size(), false);
    for (std::size_t i = 0; i < associations->size(); ++i) {
      const int reflector_index = (*associations)[i];
      if (reflector_index < 0) {
        continue;
      }
      detection_used[i] = true;
      Reflector& reflector = reflectors_[reflector_index];
      reflector.x = (1. - map_update_gain_) * reflector.x +
                    map_update_gain_ * map_points[i].x;
      reflector.y = (1. - map_update_gain_) * reflector.y +
                    map_update_gain_ * map_points[i].y;
      ++reflector.observations;
    }

    std::vector<AssociationCandidate> candidates;
    for (std::size_t detection_index = 0;
         detection_index < map_points.size(); ++detection_index) {
      if (detection_used[detection_index]) {
        continue;
      }
      double best_distance = pending_association_distance_;
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
        candidates.push_back(
            {detection_index, best_track_index, best_distance});
      }
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const AssociationCandidate& lhs,
                 const AssociationCandidate& rhs) {
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

    std::vector<std::size_t> tracks_to_remove;
    for (std::size_t track_index = 0; track_index < pending_tracks_.size();
         ++track_index) {
      PendingTrack& track = pending_tracks_[track_index];
      const int detection_index = pending_detection[track_index];
      if (detection_index >= 0 && track.hits >= minimum_confirmations_) {
        Reflector reflector;
        reflector.id = GenerateReflectorId();
        reflector.x = track.x;
        reflector.y = track.y;
        reflector.observations = track.hits;
        reflectors_.push_back(reflector);
        (*associations)[detection_index] =
            static_cast<int>(reflectors_.size() - 1);
        tracks_to_remove.push_back(track_index);
        ROS_INFO_STREAM("Created reflector '" << reflector.id << "' at ["
                        << reflector.x << ", " << reflector.y << "].");
      } else if (detection_index < 0) {
        ++track.missed_scans;
        if (track.missed_scans > maximum_pending_missed_scans_) {
          tracks_to_remove.push_back(track_index);
        }
      }
    }

    std::sort(tracks_to_remove.rbegin(), tracks_to_remove.rend());
    for (const std::size_t index : tracks_to_remove) {
      pending_tracks_.erase(pending_tracks_.begin() + index);
    }
  }

  void AgePendingTracks() {
    for (auto& track : pending_tracks_) {
      ++track.missed_scans;
    }
    pending_tracks_.erase(
        std::remove_if(pending_tracks_.begin(), pending_tracks_.end(),
                       [this](const PendingTrack& track) {
                         return track.missed_scans >
                                maximum_pending_missed_scans_;
                       }),
        pending_tracks_.end());
  }

  void PublishLandmarks(
      const sensor_msgs::LaserScan& scan,
      const std::vector<ReflectorDetection>& detections,
      const std::vector<int>& associations) {
    cartographer_ros_msgs::LandmarkList message;
    message.header = scan.header;
    for (std::size_t i = 0; i < associations.size(); ++i) {
      if (associations[i] < 0) {
        continue;
      }
      cartographer_ros_msgs::LandmarkEntry entry;
      entry.id = reflectors_[associations[i]].id;
      entry.tracking_from_landmark_transform.position.x = detections[i].x;
      entry.tracking_from_landmark_transform.position.y = detections[i].y;
      entry.tracking_from_landmark_transform.position.z = 0.;
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

    for (std::size_t i = 0; i < reflectors_.size(); ++i) {
      const Reflector& reflector = reflectors_[i];
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

    const std::vector<ReflectorDetection> detections = detector_->Detect(*scan);
    if (detections.empty()) {
      if (mapping_mode_) {
        AgePendingTracks();
      }
      PublishDetectionMarkers(*scan, detections, {});
      return;
    }

    std::vector<MapPoint> map_points;
    if (!TransformToMap(*scan, detections, &map_points)) {
      PublishDetectionMarkers(
          *scan, detections, std::vector<int>(detections.size(), -1));
      return;
    }

    std::vector<int> associations = AssociateKnownReflectors(map_points);
    if (mapping_mode_) {
      UpdateMapping(map_points, &associations);
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

    std::vector<Reflector> sorted_reflectors = reflectors_;
    std::sort(sorted_reflectors.begin(), sorted_reflectors.end(),
              [](const Reflector& lhs, const Reflector& rhs) {
                return lhs.id < rhs.id;
              });
    output << std::setprecision(17);
    output << "map_frame: \"" << EscapeYamlString(map_frame_) << "\"\n";
    output << "reflectors:\n";
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
      response.message = "Saved " + std::to_string(reflectors_.size()) +
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
  std::vector<Reflector> reflectors_;
  std::vector<PendingTrack> pending_tracks_;

  ros::Subscriber scan_subscriber_;
  ros::Publisher landmark_publisher_;
  ros::Publisher detection_marker_publisher_;
  ros::Publisher map_marker_publisher_;
  ros::ServiceServer save_map_service_;

  std::string mode_;
  bool mapping_mode_ = true;
  std::string scan_topic_;
  std::string landmark_topic_;
  std::string map_frame_;
  std::string map_file_;
  std::string id_prefix_;
  int id_width_ = 4;
  int next_reflector_id_ = 0;
  bool save_on_shutdown_ = true;

  double association_distance_ = 0.3;
  double association_margin_ = 0.05;
  double pending_association_distance_ = 0.2;
  int minimum_confirmations_ = 3;
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
