#include "gp11_perception/cable_tracker_node.h"
#include "gp11_perception/native_curve_optimizer.h"

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <XmlRpcValue.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace gp11_perception {
namespace {

template <typename T>
double Median(std::vector<T> values) {
  if (values.empty()) {
    return 0.0;
  }
  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), middle, values.end());
  double median = static_cast<double>(*middle);
  if ((values.size() % 2) == 0) {
    const auto lower_middle = std::max_element(values.begin(), middle);
    median = 0.5 * (median + static_cast<double>(*lower_middle));
  }
  return median;
}

template <typename T>
void LoadParam(const ros::NodeHandle& pnh, const std::string& name, T* value) {
  T loaded = *value;
  pnh.param(name, loaded, *value);
  *value = loaded;
}

double Clamp(double value, double low, double high) {
  return std::max(low, std::min(value, high));
}

double Sigmoid(double value) {
  if (value >= 0.0) {
    const double exp_neg = std::exp(-value);
    return 1.0 / (1.0 + exp_neg);
  }
  const double exp_pos = std::exp(value);
  return exp_pos / (1.0 + exp_pos);
}

double Percentile(std::vector<double> values, double ratio) {
  if (values.empty()) {
    return 0.0;
  }
  ratio = Clamp(ratio, 0.0, 1.0);
  const double scaled = ratio * static_cast<double>(std::max<std::size_t>(values.size() - 1, 0));
  const std::size_t low_index = static_cast<std::size_t>(std::floor(scaled));
  const std::size_t high_index = static_cast<std::size_t>(std::ceil(scaled));
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(low_index), values.end());
  const double low_value = values[low_index];
  if (high_index == low_index) {
    return low_value;
  }
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(high_index), values.end());
  const double high_value = values[high_index];
  const double blend = scaled - static_cast<double>(low_index);
  return (1.0 - blend) * low_value + blend * high_value;
}

bool HasArea(const cv::Rect& rect) {
  return rect.width > 0 && rect.height > 0;
}

Eigen::MatrixXd PointsToMatrix2d(const std::vector<cv::Point2d>& points) {
  Eigen::MatrixXd matrix(points.size(), 2);
  for (std::size_t index = 0; index < points.size(); ++index) {
    matrix(static_cast<Eigen::Index>(index), 0) = points[index].x;
    matrix(static_cast<Eigen::Index>(index), 1) = points[index].y;
  }
  return matrix;
}

std::vector<double> FlattenPoints3d(const std::vector<Eigen::Vector3d>& points) {
  std::vector<double> flattened;
  flattened.reserve(points.size() * 3);
  for (const auto& point : points) {
    flattened.push_back(point.x());
    flattened.push_back(point.y());
    flattened.push_back(point.z());
  }
  return flattened;
}

std::vector<double> FlattenPoints2d(const std::vector<cv::Point2d>& points) {
  std::vector<double> flattened;
  flattened.reserve(points.size() * 2);
  for (const auto& point : points) {
    flattened.push_back(point.x);
    flattened.push_back(point.y);
  }
  return flattened;
}

std::vector<Eigen::Vector3d> UnflattenPoints3d(const std::vector<double>& flattened) {
  std::vector<Eigen::Vector3d> points;
  points.reserve(flattened.size() / 3);
  for (std::size_t index = 0; index + 2 < flattened.size(); index += 3) {
    points.emplace_back(flattened[index + 0], flattened[index + 1], flattened[index + 2]);
  }
  return points;
}

struct VoxelKey {
  int x = 0;
  int y = 0;
  int z = 0;

  bool operator==(const VoxelKey& other) const { return x == other.x && y == other.y && z == other.z; }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& key) const {
    const std::size_t h1 = std::hash<int>()(key.x);
    const std::size_t h2 = std::hash<int>()(key.y);
    const std::size_t h3 = std::hash<int>()(key.z);
    return h1 ^ (h2 << 1) ^ (h3 << 7);
  }
};

std::vector<Eigen::Vector3d> VoxelDownsample(const std::vector<Eigen::Vector3d>& points, double voxel_size) {
  if (points.empty() || voxel_size <= 1e-6) {
    return points;
  }

  struct VoxelAccum {
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    int count = 0;
  };

  std::unordered_map<VoxelKey, VoxelAccum, VoxelKeyHash> accumulators;
  accumulators.reserve(points.size());
  for (const auto& point : points) {
    const VoxelKey key{static_cast<int>(std::floor(point.x() / voxel_size)),
                       static_cast<int>(std::floor(point.y() / voxel_size)),
                       static_cast<int>(std::floor(point.z() / voxel_size))};
    VoxelAccum& accum = accumulators[key];
    accum.sum += point;
    ++accum.count;
  }

  std::vector<Eigen::Vector3d> downsampled;
  downsampled.reserve(accumulators.size());
  for (const auto& entry : accumulators) {
    downsampled.push_back(entry.second.sum / static_cast<double>(std::max(entry.second.count, 1)));
  }
  return downsampled;
}

double ClosestArcLengthOnPolyline(const std::vector<Eigen::Vector3d>& polyline,
                                  const Eigen::Vector3d& point,
                                  double* distance_to_curve) {
  if (distance_to_curve != nullptr) {
    *distance_to_curve = std::numeric_limits<double>::infinity();
  }
  if (polyline.size() < 2) {
    if (polyline.empty()) {
      return 0.0;
    }
    if (distance_to_curve != nullptr) {
      *distance_to_curve = (point - polyline.front()).norm();
    }
    return 0.0;
  }

  double best_distance = std::numeric_limits<double>::infinity();
  double best_arc_length = 0.0;
  double accumulated_length = 0.0;
  for (std::size_t index = 0; index + 1 < polyline.size(); ++index) {
    const Eigen::Vector3d start = polyline[index];
    const Eigen::Vector3d end = polyline[index + 1];
    const Eigen::Vector3d segment = end - start;
    const double segment_length_sq = segment.squaredNorm();
    const double segment_length = std::sqrt(std::max(segment_length_sq, 0.0));
    if (segment_length < 1e-9) {
      accumulated_length += segment_length;
      continue;
    }
    const double ratio = Clamp((point - start).dot(segment) / segment_length_sq, 0.0, 1.0);
    const Eigen::Vector3d projection = start + ratio * segment;
    const double distance = (point - projection).norm();
    if (distance < best_distance) {
      best_distance = distance;
      best_arc_length = accumulated_length + ratio * segment_length;
    }
    accumulated_length += segment_length;
  }

  if (distance_to_curve != nullptr) {
    *distance_to_curve = best_distance;
  }
  return best_arc_length;
}

}  // namespace

CableTrackerNode::CableTrackerNode()
    : nh_(),
      pnh_("~"),
      tf_buffer_(ros::Duration(10.0)),
      tf_listener_(tf_buffer_),
      use_lsd_(false),
      rng_(7),
      has_last_segment_(false),
      consecutive_local_misses_(0),
      has_last_track_(false),
      has_last_camera_track_(false),
      last_length_(0.0),
      has_length_track_(false),
      tracked_length_estimate_(0.0),
      tracked_length_confidence_(0.0),
      has_endpoint_track_(false),
      tracked_endpoint_low_(Eigen::Vector3d::Zero()),
      tracked_endpoint_high_(Eigen::Vector3d::Zero()),
      tracked_endpoint_confidence_low_(0.0),
      tracked_endpoint_confidence_high_(0.0),
      has_centerline_state_(false) {
  loadParameters();
  initializeLineDetector();
  initializePublishers();
  initializeSubscribers();

  ROS_INFO_STREAM("GP11 D435i cable tracker (C++) ready: color=" << config_.color_topic
                  << ", depth=" << config_.depth_topic << ", info=" << config_.camera_info_topic
                  << ", output_frame=" << (config_.output_frame.empty() ? "<camera>" : config_.output_frame));
}

void CableTrackerNode::loadParameters() {
  LoadParam(pnh_, "color_topic", &config_.color_topic);
  LoadParam(pnh_, "depth_topic", &config_.depth_topic);
  LoadParam(pnh_, "camera_info_topic", &config_.camera_info_topic);
  LoadParam(pnh_, "marker_topic", &config_.marker_topic);
  LoadParam(pnh_, "target_point_topic", &config_.target_point_topic);
  LoadParam(pnh_, "moveit_target_point_topic", &config_.moveit_target_point_topic);
  LoadParam(pnh_, "debug_image_topic", &config_.debug_image_topic);
  LoadParam(pnh_, "status_topic", &config_.status_topic);
  LoadParam(pnh_, "output_frame", &config_.output_frame);
  LoadParam(pnh_, "publish_target_to_moveit", &config_.publish_target_to_moveit);
  LoadParam(pnh_, "publish_debug_image", &config_.publish_debug_image);
  LoadParam(pnh_, "debug_publish_max_rate_hz", &config_.debug_publish_max_rate_hz);
  LoadParam(pnh_, "debug_image_scale", &config_.debug_image_scale);
  LoadParam(pnh_, "debug_candidate_limit", &config_.debug_candidate_limit);
  LoadParam(pnh_, "debug_draw_sample_points", &config_.debug_draw_sample_points);
  LoadParam(pnh_, "debug_require_subscriber", &config_.debug_require_subscriber);
  LoadParam(pnh_, "debug_antialiasing", &config_.debug_antialiasing);
  LoadParam(pnh_, "silver_prefilter_enabled", &config_.silver_prefilter_enabled);
  LoadParam(pnh_, "silver_hsv_s_max", &config_.silver_hsv_s_max);
  LoadParam(pnh_, "silver_hsv_v_min", &config_.silver_hsv_v_min);
  LoadParam(pnh_, "silver_hsv_v_max", &config_.silver_hsv_v_max);
  LoadParam(pnh_, "silver_lab_l_min", &config_.silver_lab_l_min);
  LoadParam(pnh_, "silver_lab_l_max", &config_.silver_lab_l_max);
  LoadParam(pnh_, "silver_lab_chroma_max", &config_.silver_lab_chroma_max);
  LoadParam(pnh_, "silver_mask_close_px", &config_.silver_mask_close_px);
  LoadParam(pnh_, "silver_mask_open_px", &config_.silver_mask_open_px);
  LoadParam(pnh_, "silver_line_support_ratio", &config_.silver_line_support_ratio);
  LoadParam(pnh_, "silver_hard_reject_support_ratio", &config_.silver_hard_reject_support_ratio);
  LoadParam(pnh_, "silver_prefilter_fallback_enabled", &config_.silver_prefilter_fallback_enabled);
  LoadParam(pnh_, "silver_score_weight", &config_.silver_score_weight);
  LoadParam(pnh_, "silver_support_penalty_weight", &config_.silver_support_penalty_weight);
  LoadParam(pnh_, "min_depth_m", &config_.min_depth_m);
  LoadParam(pnh_, "max_depth_m", &config_.max_depth_m);
  LoadParam(pnh_, "cable_depth_filter_enabled", &config_.cable_depth_filter_enabled);
  LoadParam(pnh_, "cable_depth_min_m", &config_.cable_depth_min_m);
  LoadParam(pnh_, "cable_depth_max_m", &config_.cable_depth_max_m);
  LoadParam(pnh_, "min_line_length_px", &config_.min_line_length_px);
  LoadParam(pnh_, "line_width_px", &config_.line_width_px);
  LoadParam(pnh_, "depth_patch_radius_px", &config_.depth_patch_radius_px);
  LoadParam(pnh_, "min_valid_depth_ratio", &config_.min_valid_depth_ratio);
  LoadParam(pnh_, "tracking_search_margin_px", &config_.tracking_search_margin_px);
  LoadParam(pnh_, "smoothing_alpha", &config_.smoothing_alpha);
  LoadParam(pnh_, "target_ratio", &config_.target_ratio);
  LoadParam(pnh_, "candidate_top_k", &config_.candidate_top_k);
  LoadParam(pnh_, "raw_line_top_k", &config_.raw_line_top_k);
  LoadParam(pnh_, "segment_sample_count", &config_.segment_sample_count);
  LoadParam(pnh_, "line_band_half_width_px", &config_.line_band_half_width_px);
  LoadParam(pnh_, "line_band_step_px", &config_.line_band_step_px);
  LoadParam(pnh_, "depth_discontinuity_thresh_m", &config_.depth_discontinuity_thresh_m);
  LoadParam(pnh_, "line_inlier_thresh_m", &config_.line_inlier_thresh_m);
  LoadParam(pnh_, "min_inlier_count", &config_.min_inlier_count);
  LoadParam(pnh_, "ransac_iterations", &config_.ransac_iterations);
  LoadParam(pnh_, "temporal_position_gate_m", &config_.temporal_position_gate_m);
  LoadParam(pnh_, "temporal_angle_gate_deg", &config_.temporal_angle_gate_deg);
  LoadParam(pnh_, "lost_track_hold_time_s", &config_.lost_track_hold_time_s);
  LoadParam(pnh_, "global_reacquire_missed_frames", &config_.global_reacquire_missed_frames);
  LoadParam(pnh_, "parallel_edge_angle_tol_deg", &config_.parallel_edge_angle_tol_deg);
  LoadParam(pnh_, "parallel_edge_min_separation_px", &config_.parallel_edge_min_separation_px);
  LoadParam(pnh_, "parallel_edge_max_separation_px", &config_.parallel_edge_max_separation_px);
  LoadParam(pnh_, "parallel_edge_min_overlap_ratio", &config_.parallel_edge_min_overlap_ratio);
  LoadParam(pnh_, "parallel_edge_depth_tol_m", &config_.parallel_edge_depth_tol_m);
  LoadParam(pnh_, "parallel_edge_max_width_std_px", &config_.parallel_edge_max_width_std_px);
  LoadParam(pnh_, "curve_tracking_enabled", &config_.curve_tracking_enabled);
  LoadParam(pnh_, "curve_cross_section_samples", &config_.curve_cross_section_samples);
  LoadParam(pnh_, "curve_min_valid_points", &config_.curve_min_valid_points);
  LoadParam(pnh_, "curve_spatial_smooth_window", &config_.curve_spatial_smooth_window);
  LoadParam(pnh_, "curve_resample_point_count", &config_.curve_resample_point_count);
  LoadParam(pnh_, "curve_length_change_ratio", &config_.curve_length_change_ratio);
  LoadParam(pnh_, "support_percentile_low", &config_.support_percentile_low);
  LoadParam(pnh_, "support_percentile_high", &config_.support_percentile_high);
  LoadParam(pnh_, "support_axis_inlier_radius_m", &config_.support_axis_inlier_radius_m);
  LoadParam(pnh_, "support_min_points", &config_.support_min_points);
  LoadParam(pnh_, "support_endpoint_window_m", &config_.support_endpoint_window_m);
  LoadParam(pnh_, "support_outside_window_m", &config_.support_outside_window_m);
  LoadParam(pnh_, "support_temporal_sigma_m", &config_.support_temporal_sigma_m);
  LoadParam(pnh_, "support_cover_sigma_m", &config_.support_cover_sigma_m);
  LoadParam(pnh_, "length_expand_alpha", &config_.length_expand_alpha);
  LoadParam(pnh_, "length_shrink_alpha", &config_.length_shrink_alpha);
  LoadParam(pnh_, "length_min_endpoint_confidence", &config_.length_min_endpoint_confidence);
  LoadParam(pnh_, "endpoint_confident_alpha", &config_.endpoint_confident_alpha);
  LoadParam(pnh_, "endpoint_uncertain_alpha", &config_.endpoint_uncertain_alpha);
  LoadParam(pnh_, "endpoint_max_adjust_m", &config_.endpoint_max_adjust_m);
  LoadParam(pnh_, "centerline_state_enabled", &config_.centerline_state_enabled);
  LoadParam(pnh_, "centerline_state_node_count", &config_.centerline_state_node_count);
  LoadParam(pnh_, "centerline_measurement_alpha", &config_.centerline_measurement_alpha);
  LoadParam(pnh_, "centerline_velocity_decay", &config_.centerline_velocity_decay);
  LoadParam(pnh_, "centerline_velocity_gain", &config_.centerline_velocity_gain);
  LoadParam(pnh_, "centerline_spatial_smooth_alpha", &config_.centerline_spatial_smooth_alpha);
  LoadParam(pnh_, "centerline_max_node_step_m", &config_.centerline_max_node_step_m);
  LoadParam(pnh_, "centerline_hidden_node_alpha", &config_.centerline_hidden_node_alpha);
  LoadParam(pnh_, "centerline_length_scale_limit", &config_.centerline_length_scale_limit);
  LoadParam(pnh_, "point_cloud_topic", &config_.point_cloud_topic);
  LoadParam(pnh_, "use_point_cloud_observation", &config_.use_point_cloud_observation);
  LoadParam(pnh_, "point_cloud_max_age_s", &config_.point_cloud_max_age_s);
  LoadParam(pnh_, "point_cloud_voxel_size_m", &config_.point_cloud_voxel_size_m);
  LoadParam(pnh_, "point_cloud_assign_distance_m", &config_.point_cloud_assign_distance_m);
  LoadParam(pnh_, "point_cloud_min_points", &config_.point_cloud_min_points);
  LoadParam(pnh_, "point_cloud_mask_thickness_px", &config_.point_cloud_mask_thickness_px);
  LoadParam(pnh_, "anchor_constraint_enabled", &config_.anchor_constraint_enabled);
  LoadParam(pnh_, "anchor_endpoint", &config_.anchor_endpoint);
  LoadParam(pnh_, "anchor_frame", &config_.anchor_frame);
  LoadParam(pnh_, "anchor_alpha", &config_.anchor_alpha);
  LoadParam(pnh_, "anchor_neighbor_alpha", &config_.anchor_neighbor_alpha);
  LoadParam(pnh_, "anchor_max_pull_m", &config_.anchor_max_pull_m);
  LoadParam(pnh_, "joint_optimization_enabled", &config_.joint_optimization_enabled);
  LoadParam(pnh_, "native_backend_enabled", &config_.native_backend_enabled);
  LoadParam(pnh_, "native_optimizer_max_iterations", &config_.native_optimizer_max_iterations);
  LoadParam(pnh_, "native_point_assign_distance_m", &config_.native_point_assign_distance_m);
  LoadParam(pnh_, "optimizer_candidate_top_k", &config_.optimizer_candidate_top_k);
  LoadParam(pnh_, "optimizer_control_point_count", &config_.optimizer_control_point_count);
  LoadParam(pnh_, "optimizer_curve_sample_count", &config_.optimizer_curve_sample_count);
  LoadParam(pnh_, "optimizer_curve_image_sigma_px", &config_.optimizer_curve_image_sigma_px);
  LoadParam(pnh_, "optimizer_curve_smooth_sigma_m", &config_.optimizer_curve_smooth_sigma_m);
  LoadParam(pnh_, "optimizer_curve_length_sigma_m", &config_.optimizer_curve_length_sigma_m);
  LoadParam(pnh_, "optimizer_temporal_curve_sigma_m", &config_.optimizer_temporal_curve_sigma_m);
  LoadParam(pnh_, "optimizer_control_point_bound_m", &config_.optimizer_control_point_bound_m);
  LoadParam(pnh_, "optimizer_curve_point_sigma_m", &config_.optimizer_curve_point_sigma_m);
  LoadParam(pnh_, "optimizer_point_sigma_m", &config_.optimizer_point_sigma_m);
  LoadParam(pnh_, "optimizer_temporal_position_sigma_m", &config_.optimizer_temporal_position_sigma_m);

  XmlRpc::XmlRpcValue roi_value;
  if (pnh_.getParam("search_roi", roi_value) && roi_value.getType() == XmlRpc::XmlRpcValue::TypeArray &&
      roi_value.size() == 4) {
    config_.has_search_roi = true;
    for (int index = 0; index < 4; ++index) {
      config_.search_roi[static_cast<std::size_t>(index)] = static_cast<int>(roi_value[index]);
    }
  }

  XmlRpc::XmlRpcValue anchor_point_value;
  if (pnh_.getParam("anchor_point", anchor_point_value) &&
      anchor_point_value.getType() == XmlRpc::XmlRpcValue::TypeArray && anchor_point_value.size() == 3) {
    for (int index = 0; index < 3; ++index) {
      config_.anchor_point[static_cast<std::size_t>(index)] = static_cast<double>(anchor_point_value[index]);
    }
  }

  config_.debug_image_scale = Clamp(config_.debug_image_scale, 0.1, 1.0);
  config_.global_reacquire_missed_frames = std::max(1, config_.global_reacquire_missed_frames);
  config_.min_depth_m = std::max(0.02, config_.min_depth_m);
  config_.max_depth_m = std::max(config_.min_depth_m, config_.max_depth_m);
  config_.cable_depth_min_m = std::max(config_.min_depth_m, config_.cable_depth_min_m);
  config_.cable_depth_max_m = std::min(config_.max_depth_m, config_.cable_depth_max_m);
  if (config_.cable_depth_max_m < config_.cable_depth_min_m) {
    config_.cable_depth_filter_enabled = false;
    config_.cable_depth_min_m = config_.min_depth_m;
    config_.cable_depth_max_m = config_.max_depth_m;
  }
  config_.target_ratio = Clamp(config_.target_ratio, 0.0, 1.0);
  config_.support_percentile_low = Clamp(config_.support_percentile_low, 0.0, 0.45);
  config_.support_percentile_high = Clamp(config_.support_percentile_high, 0.55, 1.0);
  if (config_.support_percentile_high <= config_.support_percentile_low) {
    config_.support_percentile_low = 0.05;
    config_.support_percentile_high = 0.95;
  }
  config_.support_axis_inlier_radius_m = std::max(0.002, config_.support_axis_inlier_radius_m);
  config_.support_min_points = std::max(6, config_.support_min_points);
  config_.support_endpoint_window_m = std::max(0.005, config_.support_endpoint_window_m);
  config_.support_outside_window_m = std::max(0.005, config_.support_outside_window_m);
  config_.support_temporal_sigma_m = std::max(0.005, config_.support_temporal_sigma_m);
  config_.support_cover_sigma_m = std::max(1e-3, config_.support_cover_sigma_m);
  config_.length_expand_alpha = Clamp(config_.length_expand_alpha, 0.01, 1.0);
  config_.length_shrink_alpha = Clamp(config_.length_shrink_alpha, 0.0, 1.0);
  config_.length_min_endpoint_confidence = Clamp(config_.length_min_endpoint_confidence, 0.0, 1.0);
  config_.endpoint_confident_alpha = Clamp(config_.endpoint_confident_alpha, 0.01, 1.0);
  config_.endpoint_uncertain_alpha = Clamp(config_.endpoint_uncertain_alpha, 0.0, 0.50);
  config_.endpoint_max_adjust_m = std::max(0.01, config_.endpoint_max_adjust_m);
  config_.centerline_state_node_count = std::max(5, config_.centerline_state_node_count);
  config_.centerline_measurement_alpha = Clamp(config_.centerline_measurement_alpha, 0.05, 1.0);
  config_.centerline_velocity_decay = Clamp(config_.centerline_velocity_decay, 0.0, 0.99);
  config_.centerline_velocity_gain = Clamp(config_.centerline_velocity_gain, 0.0, 1.0);
  config_.centerline_spatial_smooth_alpha = Clamp(config_.centerline_spatial_smooth_alpha, 0.0, 0.95);
  config_.centerline_max_node_step_m = std::max(0.005, config_.centerline_max_node_step_m);
  config_.centerline_hidden_node_alpha = Clamp(config_.centerline_hidden_node_alpha, 0.0, 0.50);
  config_.centerline_length_scale_limit = Clamp(config_.centerline_length_scale_limit, 0.0, 0.30);
  config_.point_cloud_max_age_s = std::max(0.0, config_.point_cloud_max_age_s);
  config_.point_cloud_voxel_size_m = std::max(0.0, config_.point_cloud_voxel_size_m);
  config_.point_cloud_assign_distance_m = std::max(0.005, config_.point_cloud_assign_distance_m);
  config_.point_cloud_min_points = std::max(4, config_.point_cloud_min_points);
  config_.point_cloud_mask_thickness_px = std::max(1, config_.point_cloud_mask_thickness_px);
  config_.anchor_alpha = Clamp(config_.anchor_alpha, 0.0, 1.0);
  config_.anchor_neighbor_alpha = Clamp(config_.anchor_neighbor_alpha, 0.0, 1.0);
  config_.anchor_max_pull_m = std::max(0.0, config_.anchor_max_pull_m);
  config_.curve_cross_section_samples = std::max(3, config_.curve_cross_section_samples);
  if ((config_.curve_cross_section_samples % 2) == 0) {
    ++config_.curve_cross_section_samples;
  }
}

void CableTrackerNode::initializeLineDetector() {
  try {
    lsd_detector_ = cv::createLineSegmentDetector(cv::LSD_REFINE_STD);
    use_lsd_ = static_cast<bool>(lsd_detector_);
    ROS_INFO("Using OpenCV LSD line detector in C++ tracker.");
  } catch (const cv::Exception& exc) {
    use_lsd_ = false;
    ROS_WARN("OpenCV LSD is unavailable (%s). Falling back to Canny + HoughLinesP.", exc.what());
  }
}

void CableTrackerNode::initializePublishers() {
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>(config_.marker_topic, 1);
  target_pub_ = nh_.advertise<geometry_msgs::PointStamped>(config_.target_point_topic, 1);
  debug_pub_ = nh_.advertise<sensor_msgs::Image>(config_.debug_image_topic, 1);
  status_pub_ = nh_.advertise<std_msgs::String>(config_.status_topic, 1, true);
  if (config_.publish_target_to_moveit) {
    moveit_target_pub_ = nh_.advertise<geometry_msgs::PointStamped>(config_.moveit_target_point_topic, 1);
  }
}

void CableTrackerNode::initializeSubscribers() {
  color_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh_, config_.color_topic, 1));
  depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh_, config_.depth_topic, 1));
  info_sub_.reset(new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, config_.camera_info_topic, 1));
  sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(8), *color_sub_, *depth_sub_, *info_sub_));
  sync_->registerCallback(boost::bind(&CableTrackerNode::rgbdCallback, this, _1, _2, _3));
  if (config_.use_point_cloud_observation && !config_.point_cloud_topic.empty()) {
    point_cloud_sub_ = nh_.subscribe(config_.point_cloud_topic, 1, &CableTrackerNode::pointCloudCallback, this);
  }
}

void CableTrackerNode::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  latest_point_cloud_msg_ = cloud_msg;
}

cv::Mat CableTrackerNode::convertDepthToMeters(const sensor_msgs::ImageConstPtr& depth_msg) const {
  cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(depth_msg, depth_msg->encoding);
  if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
      depth_msg->encoding == sensor_msgs::image_encodings::MONO16) {
    cv::Mat depth_m;
    cv_ptr->image.convertTo(depth_m, CV_32FC1, 0.001);
    return depth_m;
  }
  if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    return cv_ptr->image.clone();
  }
  cv::Mat depth_m;
  cv_ptr->image.convertTo(depth_m, CV_32FC1);
  return depth_m;
}

cv::Mat CableTrackerNode::buildSilverMask(const cv::Mat& lab, const cv::Mat& hsv) const {
  cv::Mat mask(lab.rows, lab.cols, CV_8UC1, cv::Scalar(0));
  for (int row = 0; row < lab.rows; ++row) {
    const auto* lab_ptr = lab.ptr<cv::Vec3b>(row);
    const auto* hsv_ptr = hsv.ptr<cv::Vec3b>(row);
    auto* mask_ptr = mask.ptr<uint8_t>(row);
    for (int col = 0; col < lab.cols; ++col) {
      const double l_value = static_cast<double>(lab_ptr[col][0]);
      const double a_value = static_cast<double>(lab_ptr[col][1]) - 128.0;
      const double b_value = static_cast<double>(lab_ptr[col][2]) - 128.0;
      const double chroma = std::sqrt(a_value * a_value + b_value * b_value);
      const double s_value = static_cast<double>(hsv_ptr[col][1]);
      const double v_value = static_cast<double>(hsv_ptr[col][2]);
      const bool is_silver = l_value >= config_.silver_lab_l_min && l_value <= config_.silver_lab_l_max &&
                             chroma <= config_.silver_lab_chroma_max && s_value <= config_.silver_hsv_s_max &&
                             v_value >= config_.silver_hsv_v_min && v_value <= config_.silver_hsv_v_max;
      mask_ptr[col] = is_silver ? static_cast<uint8_t>(255) : static_cast<uint8_t>(0);
    }
  }

  if (config_.silver_mask_close_px > 0) {
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                     cv::Size(2 * config_.silver_mask_close_px + 1,
                                                              2 * config_.silver_mask_close_px + 1));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
  }
  if (config_.silver_mask_open_px > 0) {
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                     cv::Size(2 * config_.silver_mask_open_px + 1,
                                                              2 * config_.silver_mask_open_px + 1));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
  }
  return mask;
}

cv::Mat CableTrackerNode::overlayColorMask(const cv::Mat& image, const cv::Mat& mask) const {
  if (image.empty() || mask.empty()) {
    return image.clone();
  }
  cv::Mat resized_mask = mask;
  if (mask.size() != image.size()) {
    cv::resize(mask, resized_mask, image.size(), 0.0, 0.0, cv::INTER_NEAREST);
  }
  cv::Mat highlight(image.rows, image.cols, CV_8UC3, cv::Scalar(0, 0, 0));
  std::vector<cv::Mat> channels(3);
  cv::split(highlight, channels);
  channels[1] = resized_mask;
  cv::merge(channels, highlight);
  cv::Mat overlay;
  cv::addWeighted(image, 0.85, highlight, 0.15, 0.0, overlay);
  return overlay;
}

double CableTrackerNode::maskCoverageRatio(const cv::Mat& mask) const {
  if (mask.empty()) {
    return 1.0;
  }
  return static_cast<double>(cv::countNonZero(mask)) / static_cast<double>(mask.rows * mask.cols);
}

bool CableTrackerNode::shouldRenderDebug(const ros::Time& stamp) const {
  if (!config_.publish_debug_image) {
    return false;
  }
  if (config_.debug_require_subscriber && debug_pub_.getNumSubscribers() <= 0) {
    return false;
  }
  if (config_.debug_publish_max_rate_hz <= 0.0) {
    return true;
  }
  if (last_debug_publish_stamp_.isZero()) {
    return true;
  }
  const ros::Time reference_stamp = stamp.isZero() ? ros::Time::now() : stamp;
  const double elapsed = (reference_stamp - last_debug_publish_stamp_).toSec();
  return elapsed < 0.0 || elapsed >= (1.0 / config_.debug_publish_max_rate_hz);
}

cv::Mat CableTrackerNode::prepareDebugImage(const cv::Mat& color, const ros::Time& stamp) const {
  if (!shouldRenderDebug(stamp)) {
    return cv::Mat();
  }
  if (config_.debug_image_scale >= 0.999) {
    return color.clone();
  }
  cv::Mat resized;
  cv::resize(color, resized, cv::Size(), config_.debug_image_scale, config_.debug_image_scale, cv::INTER_AREA);
  return resized;
}

cv::Point CableTrackerNode::scaleDebugPoint(const cv::Point2d& point) const {
  return cv::Point(static_cast<int>(std::round(point.x * config_.debug_image_scale)),
                   static_cast<int>(std::round(point.y * config_.debug_image_scale)));
}

int CableTrackerNode::debugLineWidth(int width) const {
  return std::max(1, static_cast<int>(std::round(static_cast<double>(width) * config_.debug_image_scale)));
}

int CableTrackerNode::debugCircleRadius(int radius) const {
  return std::max(1, static_cast<int>(std::round(static_cast<double>(radius) * config_.debug_image_scale)));
}

cv::Rect CableTrackerNode::candidateRoi(const cv::Size& image_size, const ros::Time& stamp) const {
  cv::Rect roi;
  if (config_.has_search_roi) {
    roi = cv::Rect(config_.search_roi[0], config_.search_roi[1], config_.search_roi[2] - config_.search_roi[0],
                   config_.search_roi[3] - config_.search_roi[1]);
  }
  if (!has_last_segment_ || !hasRecentTrack(stamp)) {
    return HasArea(roi) ? (roi & cv::Rect(0, 0, image_size.width, image_size.height)) : cv::Rect();
  }

  const double x_min = std::floor(std::min(last_segment_p0_.x, last_segment_p1_.x) - config_.tracking_search_margin_px);
  const double y_min = std::floor(std::min(last_segment_p0_.y, last_segment_p1_.y) - config_.tracking_search_margin_px);
  const double x_max = std::ceil(std::max(last_segment_p0_.x, last_segment_p1_.x) + config_.tracking_search_margin_px);
  const double y_max = std::ceil(std::max(last_segment_p0_.y, last_segment_p1_.y) + config_.tracking_search_margin_px);
  cv::Rect tracked_roi(static_cast<int>(std::max(0.0, x_min)), static_cast<int>(std::max(0.0, y_min)),
                       static_cast<int>(std::min(static_cast<double>(image_size.width), x_max)) -
                           static_cast<int>(std::max(0.0, x_min)),
                       static_cast<int>(std::min(static_cast<double>(image_size.height), y_max)) -
                           static_cast<int>(std::max(0.0, y_min)));
  tracked_roi &= cv::Rect(0, 0, image_size.width, image_size.height);
  if (!HasArea(roi)) {
    return tracked_roi;
  }
  return roi & tracked_roi;
}

std::vector<CableTrackerNode::ParallelCandidate> CableTrackerNode::detectSegments(const cv::Mat& gray, const cv::Mat& depth_m,
                                                                                  const cv::Rect& roi,
                                                                                  const cv::Mat& color_mask,
                                                                                  bool use_tracking_prior) const {
  std::vector<ParallelCandidate> candidates =
      detectSegmentsOnce(gray, depth_m, roi, color_mask, color_mask, use_tracking_prior);
  if (!candidates.empty() || color_mask.empty() || !config_.silver_prefilter_fallback_enabled) {
    return candidates;
  }
  std::vector<ParallelCandidate> fallback_candidates =
      detectSegmentsOnce(gray, depth_m, roi, cv::Mat(), color_mask, use_tracking_prior);
  for (auto& candidate : fallback_candidates) {
    candidate.used_color_mask_fallback = true;
  }
  return fallback_candidates;
}

std::vector<CableTrackerNode::ParallelCandidate> CableTrackerNode::detectSegmentsOnce(const cv::Mat& gray,
                                                                                      const cv::Mat& depth_m,
                                                                                      const cv::Rect& roi,
                                                                                      const cv::Mat& line_mask,
                                                                                      const cv::Mat& color_mask,
                                                                                      bool use_tracking_prior) const {
  cv::Mat work = gray;
  cv::Mat work_mask = line_mask;
  cv::Point2d offset(0.0, 0.0);
  if (HasArea(roi)) {
    if (roi.width <= 2 || roi.height <= 2) {
      return {};
    }
    work = gray(roi);
    if (!line_mask.empty()) {
      work_mask = line_mask(roi);
    }
    offset = cv::Point2d(static_cast<double>(roi.x), static_cast<double>(roi.y));
  }

  const std::vector<cv::Vec4f> lines = detectLines(work, work_mask);
  if (lines.empty()) {
    return {};
  }

  std::vector<EdgeCandidate> edge_candidates;
  edge_candidates.reserve(lines.size());
  for (const auto& line : lines) {
    const cv::Point2d p0 = cv::Point2d(line[0], line[1]) + offset;
    const cv::Point2d p1 = cv::Point2d(line[2], line[3]) + offset;
    const double length_px = cv::norm(p1 - p0);
    if (length_px < config_.min_line_length_px) {
      continue;
    }
    const auto depth_stats = lineDepthStats(p0, p1, depth_m);
    if (depth_stats.first < config_.min_valid_depth_ratio) {
      continue;
    }
    EdgeCandidate candidate;
    candidate.p0 = p0;
    candidate.p1 = p1;
    candidate.center = 0.5 * (p0 + p1);
    candidate.dir = normalize(toEigen(p1 - p0));
    candidate.length_px = length_px;
    candidate.valid_ratio = depth_stats.first;
    candidate.has_mean_depth = depth_stats.second > 0.0;
    candidate.mean_depth = depth_stats.second;
    candidate.score_2d = length_px * (0.5 + depth_stats.first);
    if (use_tracking_prior && has_last_segment_) {
      const cv::Point2d prev_center = 0.5 * (last_segment_p0_ + last_segment_p1_);
      const Eigen::Vector2d prev_dir = normalize(toEigen(last_segment_p1_ - last_segment_p0_));
      const double angle_bonus = std::abs(prev_dir.dot(candidate.dir));
      const double dist_penalty = cv::norm(candidate.center - prev_center);
      candidate.score_2d += 120.0 * angle_bonus - 0.4 * dist_penalty;
    }
    edge_candidates.push_back(candidate);
  }

  std::sort(edge_candidates.begin(), edge_candidates.end(),
            [](const EdgeCandidate& lhs, const EdgeCandidate& rhs) { return lhs.score_2d > rhs.score_2d; });
  if (static_cast<int>(edge_candidates.size()) > config_.raw_line_top_k) {
    edge_candidates.resize(static_cast<std::size_t>(config_.raw_line_top_k));
  }

  std::vector<ParallelCandidate> candidates;
  for (std::size_t first_index = 0; first_index < edge_candidates.size(); ++first_index) {
    for (std::size_t second_index = first_index + 1; second_index < edge_candidates.size(); ++second_index) {
      ParallelCandidate candidate = buildParallelCandidate(edge_candidates[first_index], edge_candidates[second_index], color_mask);
      if (candidate.length_px > 0.0) {
        candidates.push_back(candidate);
      }
    }
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const ParallelCandidate& lhs, const ParallelCandidate& rhs) { return lhs.score_2d > rhs.score_2d; });
  if (static_cast<int>(candidates.size()) > config_.candidate_top_k) {
    candidates.resize(static_cast<std::size_t>(config_.candidate_top_k));
  }
  return candidates;
}

std::vector<cv::Vec4f> CableTrackerNode::detectLines(const cv::Mat& gray, const cv::Mat& mask) const {
  std::vector<cv::Vec4f> lines;
  if (use_lsd_) {
    try {
      cv::Mat work = gray;
      if (!mask.empty()) {
        cv::bitwise_and(gray, gray, work, mask);
      }
      lsd_detector_->detect(work, lines);
      if (!lines.empty()) {
        return lines;
      }
    } catch (const cv::Exception& exc) {
      ROS_WARN_THROTTLE(2.0, "LSD detect failed (%s). Falling back to Hough.", exc.what());
    }
  }

  cv::Mat edges;
  cv::Canny(gray, edges, 50.0, 150.0, 3, true);
  if (!mask.empty()) {
    cv::bitwise_and(edges, edges, edges, mask);
  }
  std::vector<cv::Vec4i> hough_lines;
  cv::HoughLinesP(edges, hough_lines, 1.0, CV_PI / 180.0, 25,
                  static_cast<double>(std::max(1.0, config_.min_line_length_px)), 20.0);
  lines.reserve(hough_lines.size());
  for (const auto& line : hough_lines) {
    lines.emplace_back(static_cast<float>(line[0]), static_cast<float>(line[1]), static_cast<float>(line[2]),
                       static_cast<float>(line[3]));
  }
  return lines;
}

std::vector<cv::Point2d> CableTrackerNode::sampleSegmentPoints(const cv::Point2d& p0, const cv::Point2d& p1,
                                                               int samples) const {
  samples = std::max(8, samples);
  std::vector<cv::Point2d> points;
  points.reserve(static_cast<std::size_t>(samples));
  for (int index = 0; index < samples; ++index) {
    const double ratio = samples == 1 ? 0.0 : static_cast<double>(index) / static_cast<double>(samples - 1);
    points.emplace_back((1.0 - ratio) * p0 + ratio * p1);
  }
  return points;
}

std::pair<double, double> CableTrackerNode::lineDepthStats(const cv::Point2d& p0, const cv::Point2d& p1,
                                                           const cv::Mat& depth_m, int samples) const {
  const std::vector<cv::Point2d> points = sampleSegmentPoints(p0, p1, samples);
  std::vector<double> depths;
  depths.reserve(points.size());
  for (const auto& point : points) {
    double depth = 0.0;
    if (sampleDepth(depth_m, static_cast<int>(std::round(point.x)), static_cast<int>(std::round(point.y)), &depth)) {
      depths.push_back(depth);
    }
  }
  if (depths.empty()) {
    return std::make_pair(0.0, 0.0);
  }
  return std::make_pair(static_cast<double>(depths.size()) / static_cast<double>(points.size()), Median(depths));
}

double CableTrackerNode::maskSupportRatio(const cv::Point2d& p0, const cv::Point2d& p1, const cv::Mat& mask,
                                          int samples) const {
  if (mask.empty()) {
    return 1.0;
  }
  const std::vector<cv::Point2d> points = sampleSegmentPoints(p0, p1, samples);
  int valid = 0;
  for (const auto& point : points) {
    const int x = static_cast<int>(std::round(point.x));
    const int y = static_cast<int>(std::round(point.y));
    if (x >= 0 && x < mask.cols && y >= 0 && y < mask.rows && mask.at<uint8_t>(y, x) > 0) {
      ++valid;
    }
  }
  return static_cast<double>(valid) / static_cast<double>(std::max<std::size_t>(1, points.size()));
}

bool CableTrackerNode::isAcceptedDepth(double depth) const {
  if (!std::isfinite(depth) || depth < config_.min_depth_m || depth > config_.max_depth_m) {
    return false;
  }
  if (!config_.cable_depth_filter_enabled) {
    return true;
  }
  return depth >= config_.cable_depth_min_m && depth <= config_.cable_depth_max_m;
}

CableTrackerNode::ParallelCandidate CableTrackerNode::buildParallelCandidate(const EdgeCandidate& first,
                                                                             const EdgeCandidate& second,
                                                                             const cv::Mat& color_mask) const {
  ParallelCandidate empty_candidate;
  Eigen::Vector2d first_dir = first.dir;
  Eigen::Vector2d second_dir = second.dir;
  cv::Point2d second_p0 = second.p0;
  cv::Point2d second_p1 = second.p1;
  if (first_dir.dot(second_dir) < 0.0) {
    second_dir = -second_dir;
    second_p0 = second.p1;
    second_p1 = second.p0;
  }

  const double cosine = Clamp(std::abs(first_dir.dot(second_dir)), -1.0, 1.0);
  const double angle_deg = std::acos(cosine) * 180.0 / M_PI;
  if (angle_deg > config_.parallel_edge_angle_tol_deg) {
    return empty_candidate;
  }

  const Eigen::Vector2d combined_sum = first_dir + second_dir;
  Eigen::Vector2d combined_dir = Eigen::Vector2d::UnitX();
  if (combined_sum.norm() >= 1e-9) {
    combined_dir = combined_sum / combined_sum.norm();
  }
  const Eigen::Vector2d combined_normal(-combined_dir.y(), combined_dir.x());
  const cv::Point2d first_center = first.center;
  const cv::Point2d second_center = second.center;
  const double separation = std::abs(combined_normal.dot(toEigen(second_center - first_center)));
  if (separation < config_.parallel_edge_min_separation_px || separation > config_.parallel_edge_max_separation_px) {
    return empty_candidate;
  }

  if (first.has_mean_depth && second.has_mean_depth &&
      std::abs(first.mean_depth - second.mean_depth) > config_.parallel_edge_depth_tol_m) {
    return empty_candidate;
  }

  const cv::Point2d origin = 0.5 * (first_center + second_center);
  const auto first_interval = projectInterval(first.p0, first.p1, origin, combined_dir);
  const auto second_interval = projectInterval(second_p0, second_p1, origin, combined_dir);
  const double overlap_start = std::max(first_interval.first, second_interval.first);
  const double overlap_end = std::min(first_interval.second, second_interval.second);
  const double overlap_length = overlap_end - overlap_start;
  const double min_length = std::min(first.length_px, second.length_px);
  if (overlap_length < config_.parallel_edge_min_overlap_ratio * min_length) {
    return empty_candidate;
  }

  const int sample_count = std::max(8, static_cast<int>(overlap_length / 10.0));
  std::vector<cv::Point2d> midpoints;
  std::vector<double> widths;
  std::vector<cv::Point2d> first_points;
  std::vector<cv::Point2d> second_points;
  midpoints.reserve(static_cast<std::size_t>(sample_count));
  widths.reserve(static_cast<std::size_t>(sample_count));
  first_points.reserve(static_cast<std::size_t>(sample_count));
  second_points.reserve(static_cast<std::size_t>(sample_count));

  for (int index = 0; index < sample_count; ++index) {
    const double ratio = sample_count == 1 ? 0.0 : static_cast<double>(index) / static_cast<double>(sample_count - 1);
    const double projection = overlap_start + ratio * (overlap_end - overlap_start);
    const cv::Point2d first_point = pointOnLineAtProjection(first_center, first_dir, origin, combined_dir, projection);
    const cv::Point2d second_point = pointOnLineAtProjection(second_center, second_dir, origin, combined_dir, projection);
    first_points.push_back(first_point);
    second_points.push_back(second_point);
    widths.push_back(cv::norm(second_point - first_point));
    midpoints.push_back(0.5 * (first_point + second_point));
  }

  const double width_std = [&widths]() {
    if (widths.empty()) {
      return 0.0;
    }
    const double mean = std::accumulate(widths.begin(), widths.end(), 0.0) / static_cast<double>(widths.size());
    double variance = 0.0;
    for (const double width : widths) {
      const double delta = width - mean;
      variance += delta * delta;
    }
    variance /= static_cast<double>(widths.size());
    return std::sqrt(std::max(variance, 0.0));
  }();
  if (width_std > config_.parallel_edge_max_width_std_px) {
    return empty_candidate;
  }

  Eigen::Vector2d midpoint_center(0.0, 0.0);
  for (const auto& midpoint : midpoints) {
    midpoint_center += toEigen(midpoint);
  }
  midpoint_center /= static_cast<double>(std::max<std::size_t>(1, midpoints.size()));
  Eigen::MatrixXd centered(static_cast<Eigen::Index>(midpoints.size()), 2);
  for (std::size_t index = 0; index < midpoints.size(); ++index) {
    centered.row(static_cast<Eigen::Index>(index)) = (toEigen(midpoints[index]) - midpoint_center).transpose();
  }
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(centered, Eigen::ComputeThinV);
  const Eigen::Vector2d center_axis = svd.matrixV().col(0);
  Eigen::Vector2d center_dir = Eigen::Vector2d::UnitX();
  if (center_axis.norm() >= 1e-9) {
    center_dir = center_axis / center_axis.norm();
  }
  if (center_dir.dot(combined_dir) < 0.0) {
    center_dir = -center_dir;
  }

  std::vector<double> midpoint_projections;
  midpoint_projections.reserve(midpoints.size());
  for (const auto& midpoint : midpoints) {
    midpoint_projections.push_back((toEigen(midpoint) - midpoint_center).dot(center_dir));
  }
  const double min_projection = *std::min_element(midpoint_projections.begin(), midpoint_projections.end());
  const double max_projection = *std::max_element(midpoint_projections.begin(), midpoint_projections.end());
  const cv::Point2d center_p0 = toCv(midpoint_center + min_projection * center_dir);
  const cv::Point2d center_p1 = toCv(midpoint_center + max_projection * center_dir);
  const double center_length = cv::norm(center_p1 - center_p0);
  if (center_length < config_.min_line_length_px) {
    return empty_candidate;
  }

  const double mask_support_ratio = maskSupportRatio(center_p0, center_p1, color_mask, sample_count);
  if (!color_mask.empty() && mask_support_ratio < config_.silver_hard_reject_support_ratio) {
    return empty_candidate;
  }
  const double support_deficit = std::max(0.0, config_.silver_line_support_ratio - mask_support_ratio);

  ParallelCandidate candidate;
  candidate.p0 = center_p0;
  candidate.p1 = center_p1;
  candidate.center = 0.5 * (center_p0 + center_p1);
  candidate.dir = center_dir;
  candidate.length_px = center_length;
  candidate.valid_ratio = std::min(first.valid_ratio, second.valid_ratio);
  candidate.has_mean_depth = first.has_mean_depth || second.has_mean_depth;
  candidate.mean_depth = first.has_mean_depth && second.has_mean_depth ? 0.5 * (first.mean_depth + second.mean_depth)
                                                                       : (first.has_mean_depth ? first.mean_depth
                                                                                               : second.mean_depth);
  candidate.score_2d = first.score_2d + second.score_2d + 2.0 * overlap_length + 8.0 * separation -
                       12.0 * width_std - 2.0 * angle_deg + config_.silver_score_weight * mask_support_ratio -
                       config_.silver_support_penalty_weight * support_deficit;
  candidate.pair_separation_px = separation;
  candidate.pair_overlap_px = overlap_length;
  candidate.pair_width_std_px = width_std;
  candidate.mask_support_ratio = mask_support_ratio;
  candidate.edge_a_p0 = first.p0;
  candidate.edge_a_p1 = first.p1;
  candidate.edge_b_p0 = second_p0;
  candidate.edge_b_p1 = second_p1;
  candidate.edge_a_samples = first_points;
  candidate.edge_b_samples = second_points;
  candidate.center_samples = midpoints;
  candidate.width_samples_px = widths;
  candidate.edge_normal_px = normalize(toEigen(second_center - first_center));
  candidate.sample_ratios.reserve(midpoints.size());
  for (std::size_t index = 0; index < midpoints.size(); ++index) {
    const double ratio = midpoints.size() <= 1 ? 0.0 : -1.0 + 2.0 * static_cast<double>(index) / static_cast<double>(midpoints.size() - 1);
    candidate.sample_ratios.push_back(ratio);
  }
  return candidate;
}

bool CableTrackerNode::sampleDepth(const cv::Mat& depth_m, int u, int v, double* depth) const {
  const int x0 = std::max(0, u - config_.depth_patch_radius_px);
  const int x1 = std::min(depth_m.cols, u + config_.depth_patch_radius_px + 1);
  const int y0 = std::max(0, v - config_.depth_patch_radius_px);
  const int y1 = std::min(depth_m.rows, v + config_.depth_patch_radius_px + 1);
  if (x0 >= x1 || y0 >= y1) {
    return false;
  }
  std::vector<float> valid_values;
  valid_values.reserve(static_cast<std::size_t>((x1 - x0) * (y1 - y0)));
  for (int row = y0; row < y1; ++row) {
    const float* ptr = depth_m.ptr<float>(row);
    for (int col = x0; col < x1; ++col) {
      const float value = ptr[col];
      if (isAcceptedDepth(static_cast<double>(value))) {
        valid_values.push_back(value);
      }
    }
  }
  if (valid_values.empty()) {
    return false;
  }
  *depth = Median(valid_values);
  return true;
}

void CableTrackerNode::segmentBandPointsTo3d(const ParallelCandidate& candidate, const cv::Mat& depth_m,
                                             std::vector<cv::Point2d>* points_2d,
                                             std::vector<Eigen::Vector3d>* points_3d,
                                             double* band_valid_ratio) const {
  points_2d->clear();
  points_3d->clear();
  *band_valid_ratio = 0.0;
  const double length_px = cv::norm(candidate.p1 - candidate.p0);
  if (length_px < 1.0) {
    return;
  }

  const Eigen::Vector2d tangent = normalize(toEigen(candidate.p1 - candidate.p0));
  const Eigen::Vector2d normal(-tangent.y(), tangent.x());
  std::vector<int> offsets;
  for (int offset = -config_.line_band_half_width_px; offset <= config_.line_band_half_width_px;
       offset += std::max(1, config_.line_band_step_px)) {
    offsets.push_back(offset);
  }
  if (std::find(offsets.begin(), offsets.end(), 0) == offsets.end()) {
    offsets.push_back(0);
    std::sort(offsets.begin(), offsets.end());
  }

  const int sample_count = std::max(config_.segment_sample_count, static_cast<int>(length_px / 6.0));
  const std::vector<cv::Point2d> samples = sampleSegmentPoints(candidate.p0, candidate.p1, sample_count);
  const double fx = camera_model_.fx();
  const double fy = camera_model_.fy();
  const double cx = camera_model_.cx();
  const double cy = camera_model_.cy();
  int valid_rows = 0;
  const int expected_points = std::max(1, static_cast<int>(samples.size() * offsets.size()));

  for (const auto& sample : samples) {
    double center_depth = 0.0;
    if (!sampleDepth(depth_m, static_cast<int>(std::round(sample.x)), static_cast<int>(std::round(sample.y)), &center_depth)) {
      continue;
    }
    bool row_has_point = false;
    for (const int offset : offsets) {
      const Eigen::Vector2d pixel_vec = toEigen(sample) + static_cast<double>(offset) * normal;
      const int u = static_cast<int>(std::round(pixel_vec.x()));
      const int v = static_cast<int>(std::round(pixel_vec.y()));
      double z = 0.0;
      if (!sampleDepth(depth_m, u, v, &z)) {
        continue;
      }
      if (std::abs(z - center_depth) > config_.depth_discontinuity_thresh_m) {
        continue;
      }
      const double x = (static_cast<double>(u) - cx) * z / std::max(fx, 1e-9);
      const double y = (static_cast<double>(v) - cy) * z / std::max(fy, 1e-9);
      points_2d->emplace_back(static_cast<double>(u), static_cast<double>(v));
      points_3d->emplace_back(x, y, z);
      row_has_point = true;
    }
    if (row_has_point) {
      ++valid_rows;
    }
  }

  if (points_3d->empty()) {
    return;
  }
  const double row_valid_ratio = static_cast<double>(valid_rows) / static_cast<double>(std::max<std::size_t>(1, samples.size()));
  const double point_valid_ratio = static_cast<double>(points_3d->size()) / static_cast<double>(expected_points);
  *band_valid_ratio = 0.5 * row_valid_ratio + 0.5 * point_valid_ratio;
}

void CableTrackerNode::candidateCurvePointsTo3d(const ParallelCandidate& candidate, const cv::Mat& depth_m,
                                                std::vector<cv::Point2d>* points_2d,
                                                std::vector<Eigen::Vector3d>* points_3d,
                                                double* valid_ratio) const {
  points_2d->clear();
  points_3d->clear();
  *valid_ratio = 0.0;
  if (!config_.curve_tracking_enabled || candidate.center_samples.empty()) {
    return;
  }

  const double fx = camera_model_.fx();
  const double fy = camera_model_.fy();
  const double cx = camera_model_.cx();
  const double cy = camera_model_.cy();

  for (std::size_t index = 0; index < candidate.center_samples.size(); ++index) {
    std::vector<cv::Point2d> cross_samples;
    if (candidate.edge_a_samples.size() == candidate.center_samples.size() &&
        candidate.edge_b_samples.size() == candidate.center_samples.size()) {
      cross_samples = sampleSegmentPoints(candidate.edge_a_samples[index], candidate.edge_b_samples[index],
                                          config_.curve_cross_section_samples);
    } else {
      cross_samples.push_back(candidate.center_samples[index]);
    }

    std::vector<double> depths;
    for (const auto& sample : cross_samples) {
      double z = 0.0;
      if (sampleDepth(depth_m, static_cast<int>(std::round(sample.x)), static_cast<int>(std::round(sample.y)), &z)) {
        depths.push_back(z);
      }
    }
    if (depths.empty()) {
      continue;
    }
    const double z = Median(depths);
    const double u = candidate.center_samples[index].x;
    const double v = candidate.center_samples[index].y;
    const double x = (u - cx) * z / std::max(fx, 1e-9);
    const double y = (v - cy) * z / std::max(fy, 1e-9);
    points_2d->push_back(candidate.center_samples[index]);
    points_3d->emplace_back(x, y, z);
  }

  if (points_3d->empty()) {
    return;
  }
  if (config_.curve_spatial_smooth_window > 1) {
    *points_3d = smoothPolyline(*points_3d, config_.curve_spatial_smooth_window);
  }
  *valid_ratio = static_cast<double>(points_3d->size()) /
                 static_cast<double>(std::max<std::size_t>(1, candidate.center_samples.size()));
}

CableTrackerNode::TrackFit CableTrackerNode::fitLineRansac(const std::vector<Eigen::Vector3d>& points) const {
  TrackFit empty_fit;
  if (static_cast<int>(points.size()) < config_.min_inlier_count) {
    return empty_fit;
  }

  std::uniform_int_distribution<int> distribution(0, static_cast<int>(points.size()) - 1);
  std::vector<char> best_mask(points.size(), 0);
  bool has_best = false;
  double best_score = -std::numeric_limits<double>::infinity();

  for (int iteration = 0; iteration < config_.ransac_iterations; ++iteration) {
    int index0 = distribution(rng_);
    int index1 = distribution(rng_);
    if (index0 == index1) {
      continue;
    }
    const Eigen::Vector3d anchor = points[static_cast<std::size_t>(index0)];
    Eigen::Vector3d axis = points[static_cast<std::size_t>(index1)] - anchor;
    if (axis.norm() < 1e-6) {
      continue;
    }
    axis = normalize(axis);
    if (has_last_track_ && axis.dot(last_axis_) < 0.0) {
      axis = -axis;
    }

    std::vector<double> distances(points.size(), 0.0);
    std::vector<char> mask(points.size(), 0);
    int inlier_count = 0;
    for (std::size_t point_index = 0; point_index < points.size(); ++point_index) {
      const double distance = ((points[point_index] - anchor).cross(axis)).norm();
      distances[point_index] = distance;
      if (distance < config_.line_inlier_thresh_m) {
        mask[point_index] = 1;
        ++inlier_count;
      }
    }
    if (inlier_count < config_.min_inlier_count) {
      continue;
    }

    double min_projection = std::numeric_limits<double>::infinity();
    double max_projection = -std::numeric_limits<double>::infinity();
    double residual_sum = 0.0;
    for (std::size_t point_index = 0; point_index < points.size(); ++point_index) {
      if (!mask[point_index]) {
        continue;
      }
      const double projection = points[point_index].dot(axis);
      min_projection = std::min(min_projection, projection);
      max_projection = std::max(max_projection, projection);
      residual_sum += distances[point_index];
    }
    const double span = max_projection - min_projection;
    const double residual = residual_sum / static_cast<double>(std::max(1, inlier_count));
    const double score = 5.0 * static_cast<double>(inlier_count) + 20.0 * span - 200.0 * residual;
    if (score > best_score) {
      best_score = score;
      best_mask = mask;
      has_best = true;
    }
  }

  if (!has_best) {
    return empty_fit;
  }

  std::vector<Eigen::Vector3d> inliers;
  inliers.reserve(points.size());
  for (std::size_t index = 0; index < points.size(); ++index) {
    if (best_mask[index]) {
      inliers.push_back(points[index]);
    }
  }
  return finalizeLineFit(inliers, static_cast<int>(points.size()));
}

CableTrackerNode::TrackFit CableTrackerNode::finalizeLineFit(const std::vector<Eigen::Vector3d>& inlier_points,
                                                             int point_count) const {
  TrackFit fit;
  if (inlier_points.size() < 3) {
    return fit;
  }
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  for (const auto& point : inlier_points) {
    center += point;
  }
  center /= static_cast<double>(inlier_points.size());

  Eigen::MatrixXd centered(inlier_points.size(), 3);
  for (std::size_t index = 0; index < inlier_points.size(); ++index) {
    centered.row(static_cast<Eigen::Index>(index)) = (inlier_points[index] - center).transpose();
  }
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(centered, Eigen::ComputeThinV);
  const Eigen::Vector3d axis_basis = svd.matrixV().col(0);
  Eigen::Vector3d axis = Eigen::Vector3d::UnitX();
  if (axis_basis.norm() >= 1e-9) {
    axis = axis_basis / axis_basis.norm();
  }
  if (has_last_track_ && axis.dot(last_axis_) < 0.0) {
    axis = -axis;
  }

  double lower = std::numeric_limits<double>::infinity();
  double upper = -std::numeric_limits<double>::infinity();
  double residual_sum = 0.0;
  for (const auto& point : inlier_points) {
    const Eigen::Vector3d centered_point = point - center;
    const double projection = centered_point.dot(axis);
    lower = std::min(lower, projection);
    upper = std::max(upper, projection);
    residual_sum += centered_point.cross(axis).norm();
  }

  fit.valid = true;
  fit.center = center;
  fit.axis = axis;
  fit.p0 = center + lower * axis;
  fit.p1 = center + upper * axis;
  fit.length = std::max((fit.p1 - fit.p0).norm(), 1e-4);
  fit.residual = residual_sum / static_cast<double>(inlier_points.size());
  fit.inlier_count = static_cast<int>(inlier_points.size());
  fit.point_count = point_count;
  fit.curve_length = fit.length;
  return fit;
}

CableTrackerNode::TrackFit CableTrackerNode::curveTrackFromPoints(const std::vector<Eigen::Vector3d>& curve_points,
                                                                  const TrackFit* fit_template) const {
  TrackFit fit = fit_template != nullptr ? *fit_template : TrackFit();
  if (curve_points.size() < 2) {
    fit.valid = false;
    return fit;
  }

  fit.curve_points = resamplePolyline(curve_points, config_.curve_resample_point_count);
  fit.has_curve = fit.curve_points.size() >= 2;
  fit.curve_length = polylineLength(fit.curve_points);
  if (fit.curve_length < 1e-4) {
    fit.valid = false;
    return fit;
  }

  fit.p0 = fit.curve_points.front();
  fit.p1 = fit.curve_points.back();
  const Eigen::Vector3d chord = fit.p1 - fit.p0;
  if (chord.norm() < 1e-6) {
    fit.center = Eigen::Vector3d::Zero();
    for (const auto& point : fit.curve_points) {
      fit.center += point;
    }
    fit.center /= static_cast<double>(fit.curve_points.size());
    Eigen::MatrixXd centered(fit.curve_points.size(), 3);
    for (std::size_t index = 0; index < fit.curve_points.size(); ++index) {
      centered.row(static_cast<Eigen::Index>(index)) = (fit.curve_points[index] - fit.center).transpose();
    }
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(centered, Eigen::ComputeThinV);
    const Eigen::Vector3d axis_basis = svd.matrixV().col(0);
    fit.axis = Eigen::Vector3d::UnitX();
    if (axis_basis.norm() >= 1e-9) {
      fit.axis = axis_basis / axis_basis.norm();
    }
  } else {
    fit.axis = Eigen::Vector3d::UnitX();
    if (chord.norm() >= 1e-9) {
      fit.axis = chord / chord.norm();
    }
    fit.center = pointAtRatioOnPolyline(fit.curve_points, 0.5);
  }

  const Eigen::Vector3d reference = fit.p1 - fit.p0;
  const double reference_norm = reference.norm();
  fit.curve_residual = 0.0;
  if (reference_norm >= 1e-6) {
    const Eigen::Vector3d reference_axis = reference / reference_norm;
    for (const auto& point : fit.curve_points) {
      fit.curve_residual += (point - fit.p0).cross(reference_axis).norm();
    }
    fit.curve_residual /= static_cast<double>(fit.curve_points.size());
  }
  fit.length = fit.curve_length;
  fit.valid = true;
  return fit;
}

CableTrackerNode::TrackFit CableTrackerNode::applyCurveTrack(const TrackFit& fit,
                                                             const std::vector<Eigen::Vector3d>& curve_points,
                                                             double curve_valid_ratio) const {
  if (curve_points.size() < static_cast<std::size_t>(config_.curve_min_valid_points)) {
    return fit;
  }
  TrackFit curve_fit = curveTrackFromPoints(curve_points, &fit);
  if (!curve_fit.valid) {
    return fit;
  }
  curve_fit.curve_valid_ratio = curve_valid_ratio;
  return curve_fit;
}

cv::Mat CableTrackerNode::buildCandidateObservationMask(const cv::Size& image_size, const ParallelCandidate& candidate,
                                                        const std::vector<cv::Point2d>& curve_points_2d,
                                                        const cv::Mat& color_mask) const {
  cv::Mat mask(image_size, CV_8UC1, cv::Scalar(0));
  const int thickness = std::max(1, config_.point_cloud_mask_thickness_px);
  const int line_type = config_.debug_antialiasing ? cv::LINE_AA : cv::LINE_8;

  auto draw_polyline = [&](const std::vector<cv::Point2d>& points) {
    if (points.size() >= 2) {
      for (std::size_t index = 0; index + 1 < points.size(); ++index) {
        cv::line(mask, points[index], points[index + 1], cv::Scalar(255), thickness, line_type);
      }
    }
  };

  if (curve_points_2d.size() >= 2) {
    draw_polyline(curve_points_2d);
  } else {
    cv::line(mask, candidate.p0, candidate.p1, cv::Scalar(255), thickness, line_type);
  }
  if (candidate.edge_a_samples.size() >= 2) {
    draw_polyline(candidate.edge_a_samples);
  }
  if (candidate.edge_b_samples.size() >= 2) {
    draw_polyline(candidate.edge_b_samples);
  }
  cv::circle(mask, candidate.p0, thickness, cv::Scalar(255), -1, line_type);
  cv::circle(mask, candidate.p1, thickness, cv::Scalar(255), -1, line_type);

  if (!color_mask.empty() && color_mask.size() == mask.size()) {
    cv::Mat dilated_color_mask;
    cv::dilate(color_mask, dilated_color_mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::bitwise_and(mask, dilated_color_mask, mask);
  }
  return mask;
}

bool CableTrackerNode::extractCablePointCloudFromLatestCloud(const cv::Mat& observation_mask,
                                                             const std::string& target_frame,
                                                             const ros::Time& stamp,
                                                             std::vector<Eigen::Vector3d>* cable_points) const {
  if (cable_points == nullptr) {
    return false;
  }
  cable_points->clear();
  if (!config_.use_point_cloud_observation || latest_point_cloud_msg_ == nullptr || observation_mask.empty()) {
    return false;
  }

  const sensor_msgs::PointCloud2ConstPtr cloud_msg = latest_point_cloud_msg_;
  const ros::Time reference_stamp = stamp.isZero() ? ros::Time::now() : stamp;
  const double age = std::abs((reference_stamp - cloud_msg->header.stamp).toSec());
  if (age > config_.point_cloud_max_age_s) {
    return false;
  }

  std::vector<Eigen::Vector3d> points;
  points.reserve(static_cast<std::size_t>(observation_mask.rows * observation_mask.cols / 8));
  const bool organized =
      cloud_msg->height > 1 &&
      static_cast<int>(cloud_msg->height) == observation_mask.rows &&
      static_cast<int>(cloud_msg->width) == observation_mask.cols;

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");

  if (organized) {
    for (int row = 0; row < observation_mask.rows; ++row) {
      const uchar* mask_ptr = observation_mask.ptr<uchar>(row);
      for (int col = 0; col < observation_mask.cols; ++col, ++iter_x, ++iter_y, ++iter_z) {
        if (mask_ptr[col] == 0) {
          continue;
        }
        const double x = static_cast<double>(*iter_x);
        const double y = static_cast<double>(*iter_y);
        const double z = static_cast<double>(*iter_z);
        if (!std::isfinite(x) || !std::isfinite(y) || !isAcceptedDepth(z)) {
          continue;
        }
        points.emplace_back(x, y, z);
      }
    }
  } else {
    const std::string cloud_frame = cloud_msg->header.frame_id;
    if (cloud_frame.empty() || cloud_frame != camera_model_.tfFrame()) {
      return false;
    }
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const double x = static_cast<double>(*iter_x);
      const double y = static_cast<double>(*iter_y);
      const double z = static_cast<double>(*iter_z);
      if (!std::isfinite(x) || !std::isfinite(y) || !isAcceptedDepth(z)) {
        continue;
      }
      const int u = static_cast<int>(std::round(camera_model_.fx() * x / z + camera_model_.cx()));
      const int v = static_cast<int>(std::round(camera_model_.fy() * y / z + camera_model_.cy()));
      if (u < 0 || v < 0 || u >= observation_mask.cols || v >= observation_mask.rows || observation_mask.at<uchar>(v, u) == 0) {
        continue;
      }
      points.emplace_back(x, y, z);
    }
  }

  if (static_cast<int>(points.size()) < config_.point_cloud_min_points) {
    return false;
  }

  if (config_.point_cloud_voxel_size_m > 1e-6) {
    points = VoxelDownsample(points, config_.point_cloud_voxel_size_m);
  }

  if (points.empty()) {
    return false;
  }
  if (cloud_msg->header.frame_id.empty() || cloud_msg->header.frame_id == target_frame || target_frame.empty()) {
    *cable_points = points;
    return true;
  }
  return transformPoints(points, cloud_msg->header.frame_id, target_frame, reference_stamp, cable_points);
}

bool CableTrackerNode::lookupAnchorPoint(const std::string& target_frame, const ros::Time& stamp,
                                         Eigen::Vector3d* anchor_point) const {
  if (anchor_point == nullptr || !config_.anchor_constraint_enabled || config_.anchor_endpoint == "none") {
    return false;
  }

  Eigen::Vector3d point(config_.anchor_point[0], config_.anchor_point[1], config_.anchor_point[2]);
  if (config_.anchor_frame.empty() || target_frame.empty() || config_.anchor_frame == target_frame) {
    *anchor_point = point;
    return true;
  }

  std::vector<Eigen::Vector3d> transformed;
  if (!transformPoints(std::vector<Eigen::Vector3d>{point}, config_.anchor_frame, target_frame, stamp, &transformed) ||
      transformed.empty()) {
    return false;
  }
  *anchor_point = transformed.front();
  return true;
}

std::vector<double> CableTrackerNode::buildCenterlineObservationConfidence(const TrackFit& fit, int node_count) const {
  node_count = std::max(2, node_count);
  const double base_confidence = fit.has_curve ? Clamp(fit.curve_valid_ratio, 0.15, 1.0) : 0.60;
  std::vector<double> confidence(static_cast<std::size_t>(node_count), base_confidence);
  if (!fit.has_support_interval) {
    return confidence;
  }

  const double total_length = std::max(fit.has_curve ? fit.curve_length : fit.length, 1e-6);
  const double support_low = Clamp(fit.support_low, 0.0, total_length);
  const double support_high = Clamp(fit.support_high, support_low, total_length);
  const double shoulder = std::max(config_.support_endpoint_window_m, 0.12 * total_length);

  for (int index = 0; index < node_count; ++index) {
    const double ratio = static_cast<double>(index) / static_cast<double>(node_count - 1);
    const double arc_length = ratio * total_length;
    if (arc_length >= support_low && arc_length <= support_high) {
      confidence[static_cast<std::size_t>(index)] = 1.0;
      continue;
    }
    if (arc_length < support_low) {
      const double falloff = Clamp(1.0 - (support_low - arc_length) / std::max(shoulder, 1e-6), 0.0, 1.0);
      confidence[static_cast<std::size_t>(index)] =
          std::max(base_confidence * 0.20, fit.endpoint_confidence_low * falloff);
    } else {
      const double falloff = Clamp(1.0 - (arc_length - support_high) / std::max(shoulder, 1e-6), 0.0, 1.0);
      confidence[static_cast<std::size_t>(index)] =
          std::max(base_confidence * 0.20, fit.endpoint_confidence_high * falloff);
    }
  }
  return confidence;
}

CableTrackerNode::TrackFit CableTrackerNode::applyCenterlineStateEstimator(
    const TrackFit& observed_fit,
    const std::vector<Eigen::Vector3d>& point_cloud_observation,
    const std::string& frame_id,
    const ros::Time& stamp) {
  if (!observed_fit.valid || !config_.centerline_state_enabled) {
    return observed_fit;
  }

  const int node_count = std::max(5, config_.centerline_state_node_count);
  std::vector<Eigen::Vector3d> observed_nodes;
  if (observed_fit.has_curve && observed_fit.curve_points.size() >= 2) {
    observed_nodes = resamplePolyline(observed_fit.curve_points, node_count);
  } else {
    observed_nodes = resamplePolyline(std::vector<Eigen::Vector3d>{observed_fit.p0, observed_fit.p1}, node_count);
  }
  if (observed_nodes.size() < 2) {
    return observed_fit;
  }

  std::vector<double> observation_confidence = buildCenterlineObservationConfidence(observed_fit, node_count);

  const bool reset_state = !has_centerline_state_ || tracked_centerline_frame_id_ != frame_id || !hasRecentTrack(stamp) ||
                           tracked_centerline_nodes_.size() != observed_nodes.size();
  if (reset_state) {
    has_centerline_state_ = true;
    tracked_centerline_nodes_ = observed_nodes;
    tracked_centerline_velocities_.assign(observed_nodes.size(), Eigen::Vector3d::Zero());
    tracked_centerline_confidence_ = observation_confidence;
    tracked_centerline_frame_id_ = frame_id;
  }

  const double same_cost =
      (observed_nodes.front() - tracked_centerline_nodes_.front()).norm() +
      (observed_nodes.back() - tracked_centerline_nodes_.back()).norm();
  const double reverse_cost =
      (observed_nodes.front() - tracked_centerline_nodes_.back()).norm() +
      (observed_nodes.back() - tracked_centerline_nodes_.front()).norm();
  if (reverse_cost + 1e-6 < same_cost) {
    observed_nodes = std::vector<Eigen::Vector3d>(observed_nodes.rbegin(), observed_nodes.rend());
    observation_confidence = std::vector<double>(observation_confidence.rbegin(), observation_confidence.rend());
  }

  std::vector<Eigen::Vector3d> predicted_nodes(observed_nodes.size(), Eigen::Vector3d::Zero());
  for (std::size_t index = 0; index < observed_nodes.size(); ++index) {
    predicted_nodes[index] =
        tracked_centerline_nodes_[index] + config_.centerline_velocity_decay * tracked_centerline_velocities_[index];
  }

  std::vector<Eigen::Vector3d> updated_nodes = predicted_nodes;
  for (std::size_t index = 0; index < updated_nodes.size(); ++index) {
    const double confidence = Clamp(observation_confidence[index], 0.0, 1.0);
    const double alpha = config_.centerline_hidden_node_alpha +
                         (config_.centerline_measurement_alpha - config_.centerline_hidden_node_alpha) * confidence;
    Eigen::Vector3d delta = observed_nodes[index] - predicted_nodes[index];
    const double delta_norm = delta.norm();
    const double max_step = config_.centerline_max_node_step_m * (0.35 + 0.65 * confidence);
    if (delta_norm > max_step && delta_norm > 1e-9) {
      delta *= max_step / delta_norm;
    }
    updated_nodes[index] = predicted_nodes[index] + alpha * delta;
  }

  if (!point_cloud_observation.empty()) {
    std::vector<Eigen::Vector3d> cloud_sums(updated_nodes.size(), Eigen::Vector3d::Zero());
    std::vector<int> cloud_counts(updated_nodes.size(), 0);
    const double assign_distance =
        std::max(config_.point_cloud_assign_distance_m, 0.5 * config_.point_cloud_voxel_size_m);
    const double assign_distance_sq = assign_distance * assign_distance;

    for (const auto& point : point_cloud_observation) {
      double best_distance_sq = std::numeric_limits<double>::infinity();
      int best_index = -1;
      for (std::size_t index = 0; index < updated_nodes.size(); ++index) {
        const double distance_sq = (point - updated_nodes[index]).squaredNorm();
        if (distance_sq < best_distance_sq) {
          best_distance_sq = distance_sq;
          best_index = static_cast<int>(index);
        }
      }
      if (best_index < 0 || best_distance_sq > assign_distance_sq) {
        continue;
      }
      cloud_sums[static_cast<std::size_t>(best_index)] += point;
      ++cloud_counts[static_cast<std::size_t>(best_index)];
    }

    for (std::size_t index = 0; index < updated_nodes.size(); ++index) {
      if (cloud_counts[index] <= 0) {
        continue;
      }
      const Eigen::Vector3d cloud_mean = cloud_sums[index] / static_cast<double>(cloud_counts[index]);
      const double cloud_confidence = Clamp(static_cast<double>(cloud_counts[index]) / 10.0, 0.0, 1.0);
      const double alpha = 0.5 * config_.centerline_measurement_alpha * cloud_confidence;
      Eigen::Vector3d delta = cloud_mean - updated_nodes[index];
      const double delta_norm = delta.norm();
      const double max_step = config_.centerline_max_node_step_m * (0.60 + 0.40 * cloud_confidence);
      if (delta_norm > max_step && delta_norm > 1e-9) {
        delta *= max_step / delta_norm;
      }
      updated_nodes[index] += alpha * delta;
      observation_confidence[index] = std::max(observation_confidence[index], cloud_confidence);
    }
  }

  if (updated_nodes.size() >= 3) {
    std::vector<Eigen::Vector3d> smoothed_nodes = updated_nodes;
    for (std::size_t index = 1; index + 1 < updated_nodes.size(); ++index) {
      const double confidence = Clamp(observation_confidence[index], 0.0, 1.0);
      const double lambda = config_.centerline_spatial_smooth_alpha * (1.0 - confidence);
      const Eigen::Vector3d neighbor_average = 0.5 * (updated_nodes[index - 1] + updated_nodes[index + 1]);
      smoothed_nodes[index] = (1.0 - lambda) * updated_nodes[index] + lambda * neighbor_average;
    }
    updated_nodes.swap(smoothed_nodes);
  }

  updated_nodes = resamplePolyline(updated_nodes, node_count);

  Eigen::Vector3d anchor_point = Eigen::Vector3d::Zero();
  if (lookupAnchorPoint(frame_id, stamp, &anchor_point)) {
    const bool anchor_low = config_.anchor_endpoint != "high";
    const std::size_t endpoint_index = anchor_low ? 0 : (updated_nodes.size() - 1);
    const std::size_t neighbor_index = anchor_low ? std::min<std::size_t>(1, updated_nodes.size() - 1)
                                                  : (updated_nodes.size() >= 2 ? updated_nodes.size() - 2 : endpoint_index);
    Eigen::Vector3d pull = anchor_point - updated_nodes[endpoint_index];
    const double pull_norm = pull.norm();
    if (pull_norm > config_.anchor_max_pull_m && pull_norm > 1e-9) {
      pull *= config_.anchor_max_pull_m / pull_norm;
    }
    updated_nodes[endpoint_index] += config_.anchor_alpha * pull;
    if (neighbor_index != endpoint_index) {
      updated_nodes[neighbor_index] += config_.anchor_neighbor_alpha * pull;
    }
  }

  if (has_length_track_ && tracked_length_estimate_ > 1e-6) {
    const double current_length = polylineLength(updated_nodes);
    if (current_length > 1e-6) {
      const double target_length = std::max(
          tracked_length_estimate_, observed_fit.has_support_interval ? observed_fit.support_visible_length : 0.0);
      const double raw_scale = target_length / current_length;
      const double scale =
          Clamp(raw_scale, 1.0 - config_.centerline_length_scale_limit, 1.0 + config_.centerline_length_scale_limit);
      double anchor_ratio = 0.5;
      if (observed_fit.has_support_interval) {
        const double support_center = 0.5 * (observed_fit.support_low + observed_fit.support_high);
        const double support_length = std::max(observed_fit.has_curve ? observed_fit.curve_length : observed_fit.length, 1e-6);
        anchor_ratio = Clamp(support_center / support_length, 0.0, 1.0);
      }
      const Eigen::Vector3d anchor = pointAtRatioOnPolyline(updated_nodes, anchor_ratio);
      for (auto& node : updated_nodes) {
        node = anchor + scale * (node - anchor);
      }
    }
  }

  updated_nodes = resamplePolyline(updated_nodes, node_count);

  std::vector<Eigen::Vector3d> new_velocities(updated_nodes.size(), Eigen::Vector3d::Zero());
  std::vector<double> new_confidence(updated_nodes.size(), 0.0);
  for (std::size_t index = 0; index < updated_nodes.size(); ++index) {
    const Eigen::Vector3d innovation = updated_nodes[index] - tracked_centerline_nodes_[index];
    new_velocities[index] = config_.centerline_velocity_decay * tracked_centerline_velocities_[index] +
                            config_.centerline_velocity_gain * innovation;
    new_confidence[index] = 0.85 * tracked_centerline_confidence_[index] + 0.15 * observation_confidence[index];
  }

  tracked_centerline_nodes_ = updated_nodes;
  tracked_centerline_velocities_ = new_velocities;
  tracked_centerline_confidence_ = new_confidence;
  tracked_centerline_frame_id_ = frame_id;

  TrackFit estimated_fit = curveTrackFromPoints(tracked_centerline_nodes_, &observed_fit);
  if (!estimated_fit.valid) {
    return observed_fit;
  }
  estimated_fit.curve_valid_ratio = observed_fit.curve_valid_ratio;
  estimated_fit.curve_points_2d = observed_fit.curve_points_2d;
  estimated_fit.band_valid_ratio = observed_fit.band_valid_ratio;
  estimated_fit.residual = observed_fit.residual;
  estimated_fit.inlier_count = observed_fit.inlier_count;
  estimated_fit.point_count = observed_fit.point_count;
  estimated_fit.candidate = observed_fit.candidate;
  estimated_fit.has_candidate = observed_fit.has_candidate;
  estimated_fit.radius = observed_fit.radius;
  estimated_fit.has_optimizer_cost = observed_fit.has_optimizer_cost;
  estimated_fit.optimizer_cost = observed_fit.optimizer_cost;
  estimated_fit.optimizer_nfev = observed_fit.optimizer_nfev;
  estimated_fit.tracked_length_reference = tracked_length_estimate_;
  if (observed_fit.has_support_interval) {
    const double source_length = std::max(observed_fit.has_curve ? observed_fit.curve_length : observed_fit.length, 1e-6);
    estimated_fit.has_support_interval = true;
    estimated_fit.support_low = Clamp(observed_fit.support_low / source_length, 0.0, 1.0) * estimated_fit.curve_length;
    estimated_fit.support_high = Clamp(observed_fit.support_high / source_length, 0.0, 1.0) * estimated_fit.curve_length;
    estimated_fit.support_visible_length = std::max(0.0, estimated_fit.support_high - estimated_fit.support_low);
    estimated_fit.endpoint_confidence_low = observed_fit.endpoint_confidence_low;
    estimated_fit.endpoint_confidence_high = observed_fit.endpoint_confidence_high;
  }
  return estimated_fit;
}

bool CableTrackerNode::estimateSupportInterval(const TrackFit& fit, const std::vector<Eigen::Vector3d>& support_points,
                                               const TrackFit* previous_state, TrackFit* fit_with_support) const {
  if (fit_with_support == nullptr) {
    return false;
  }
  *fit_with_support = fit;
  fit_with_support->has_support_interval = false;
  fit_with_support->support_low = 0.0;
  fit_with_support->support_high = 0.0;
  fit_with_support->support_visible_length = 0.0;
  fit_with_support->endpoint_confidence_low = 0.0;
  fit_with_support->endpoint_confidence_high = 0.0;

  if (!fit.valid || support_points.size() < 2) {
    return false;
  }

  const double radial_gate =
      std::max(config_.support_axis_inlier_radius_m, fit.radius > 1e-4 ? 2.5 * fit.radius : config_.support_axis_inlier_radius_m);

  std::vector<double> support_coordinates;
  support_coordinates.reserve(support_points.size());
  if (fit.has_curve && fit.curve_points.size() >= 2) {
    for (const auto& point : support_points) {
      double distance_to_curve = std::numeric_limits<double>::infinity();
      const double arc_length = ClosestArcLengthOnPolyline(fit.curve_points, point, &distance_to_curve);
      if (distance_to_curve <= radial_gate) {
        support_coordinates.push_back(arc_length);
      }
    }
  } else {
    const Eigen::Vector3d axis = normalize(fit.axis);
    for (const auto& point : support_points) {
      const Eigen::Vector3d delta = point - fit.center;
      const double projection = delta.dot(axis);
      const double radial_error = (delta - projection * axis).norm();
      if (radial_error <= radial_gate) {
        support_coordinates.push_back(projection + 0.5 * fit.length);
      }
    }
  }

  if (static_cast<int>(support_coordinates.size()) < config_.support_min_points) {
    return false;
  }

  const double support_low = Percentile(support_coordinates, config_.support_percentile_low);
  const double support_high = Percentile(support_coordinates, config_.support_percentile_high);
  if (support_high - support_low < 1e-4) {
    return false;
  }

  fit_with_support->has_support_interval = true;
  fit_with_support->support_low = support_low;
  fit_with_support->support_high = support_high;
  fit_with_support->support_visible_length = support_high - support_low;
  fit_with_support->endpoint_confidence_low =
      computeEndpointConfidence(fit, support_coordinates, support_low, true, previous_state);
  fit_with_support->endpoint_confidence_high =
      computeEndpointConfidence(fit, support_coordinates, support_high, false, previous_state);
  return true;
}

double CableTrackerNode::computeEndpointConfidence(const TrackFit& fit, const std::vector<double>& support_coordinates, double boundary,
                                                   bool is_low_endpoint, const TrackFit* previous_state) const {
  if (!fit.valid || support_coordinates.empty()) {
    return 0.0;
  }

  const double inside_window = std::max(0.5 * config_.support_endpoint_window_m, config_.support_endpoint_window_m);
  const double outside_window = std::max(0.5 * config_.support_outside_window_m, config_.support_outside_window_m);
  int inside_count = 0;
  int outside_count = 0;
  constexpr int kOutsideBins = 6;
  std::array<int, kOutsideBins> outside_bins{};
  outside_bins.fill(0);

  for (const double coordinate : support_coordinates) {
    if (is_low_endpoint) {
      if (coordinate >= boundary && coordinate <= boundary + inside_window) {
        ++inside_count;
      } else if (coordinate < boundary && coordinate >= boundary - outside_window) {
        ++outside_count;
        const double ratio = Clamp((coordinate - (boundary - outside_window)) / outside_window, 0.0, 0.999999);
        const int bin = std::min(kOutsideBins - 1, static_cast<int>(ratio * static_cast<double>(kOutsideBins)));
        ++outside_bins[static_cast<std::size_t>(bin)];
      }
    } else {
      if (coordinate <= boundary && coordinate >= boundary - inside_window) {
        ++inside_count;
      } else if (coordinate > boundary && coordinate <= boundary + outside_window) {
        ++outside_count;
        const double ratio = Clamp((coordinate - boundary) / outside_window, 0.0, 0.999999);
        const int bin = std::min(kOutsideBins - 1, static_cast<int>(ratio * static_cast<double>(kOutsideBins)));
        ++outside_bins[static_cast<std::size_t>(bin)];
      }
    }
  }

  int occupied_outside_bins = 0;
  for (const int count : outside_bins) {
    if (count > 0) {
      ++occupied_outside_bins;
    }
  }

  const double eps = 1e-3;
  const double inside_density = static_cast<double>(inside_count) / std::max(inside_window, 1e-6);
  const double outside_density = static_cast<double>(outside_count) / std::max(outside_window, 1e-6);
  const double density_ll = std::log((inside_density + eps) / (outside_density + eps));
  const double outside_gap_ratio =
      1.0 - static_cast<double>(occupied_outside_bins) / static_cast<double>(std::max(1, kOutsideBins));
  const double clamped_gap = Clamp(outside_gap_ratio, 0.05, 0.95);
  const double gap_ll = std::log(clamped_gap / (1.0 - clamped_gap));

  double temporal_ll = 0.0;
  if (previous_state != nullptr && previous_state->valid) {
    const double current_length = std::max(fit.has_curve ? fit.curve_length : fit.length, 1e-6);
    const double previous_length =
        std::max(previous_state->has_curve ? previous_state->curve_length : previous_state->length, 1e-6);
    const Eigen::Vector3d current_endpoint =
        fit.has_curve && fit.curve_points.size() >= 2
            ? pointAtRatioOnPolyline(fit.curve_points, Clamp(boundary / current_length, 0.0, 1.0))
            : (fit.center + (boundary - 0.5 * fit.length) * normalize(fit.axis));
    Eigen::Vector3d previous_low = previous_state->p0;
    Eigen::Vector3d previous_high = previous_state->p1;
    if (previous_state->has_support_interval) {
      if (previous_state->has_curve && previous_state->curve_points.size() >= 2) {
        previous_low = pointAtRatioOnPolyline(previous_state->curve_points,
                                              Clamp(previous_state->support_low / previous_length, 0.0, 1.0));
        previous_high = pointAtRatioOnPolyline(previous_state->curve_points,
                                               Clamp(previous_state->support_high / previous_length, 0.0, 1.0));
      } else {
        const Eigen::Vector3d previous_axis = normalize(previous_state->axis);
        previous_low = previous_state->center + (previous_state->support_low - 0.5 * previous_state->length) * previous_axis;
        previous_high = previous_state->center + (previous_state->support_high - 0.5 * previous_state->length) * previous_axis;
      }
    }
    const Eigen::Vector3d current_axis = normalize(fit.axis);
    const bool same_direction = current_axis.dot(normalize(previous_state->axis)) >= 0.0;
    const Eigen::Vector3d reference_endpoint =
        is_low_endpoint ? (same_direction ? previous_low : previous_high)
                        : (same_direction ? previous_high : previous_low);
    const double temporal_error = (current_endpoint - reference_endpoint).norm();
    const double sigma = std::max(config_.support_temporal_sigma_m, 1e-6);
    temporal_ll = 0.6 - 0.5 * (temporal_error * temporal_error) / (sigma * sigma);
  }

  const double logit = 1.35 * density_ll + 0.95 * gap_ll + temporal_ll;
  return Sigmoid(logit);
}

void CableTrackerNode::updateTrackedLengthState(const TrackFit& fit) {
  if (!fit.valid) {
    return;
  }

  const double measured_length = std::max(fit.has_curve ? fit.curve_length : fit.length, 1e-4);
  const double visible_length = fit.has_support_interval ? fit.support_visible_length : measured_length;
  const double min_endpoint_conf = config_.length_min_endpoint_confidence;
  const bool confident_endpoints =
      fit.has_support_interval && fit.endpoint_confidence_low >= min_endpoint_conf &&
      fit.endpoint_confidence_high >= min_endpoint_conf;
  double length_measurement = std::max(measured_length, visible_length);

  constexpr std::size_t kLengthWindow = 5;
  if (confident_endpoints) {
    recent_length_measurements_.push_back(length_measurement);
    while (recent_length_measurements_.size() > kLengthWindow) {
      recent_length_measurements_.pop_front();
    }
    length_measurement = Median(std::vector<double>(recent_length_measurements_.begin(), recent_length_measurements_.end()));
  }

  if (!has_length_track_) {
    tracked_length_estimate_ = length_measurement;
    tracked_length_confidence_ = confident_endpoints ? 1.0 : 0.35;
    has_length_track_ = true;
    return;
  }

  if (confident_endpoints) {
    const double alpha = length_measurement >= tracked_length_estimate_ ? config_.length_expand_alpha
                                                                        : config_.length_shrink_alpha;
    tracked_length_estimate_ =
        (1.0 - alpha) * tracked_length_estimate_ + alpha * std::max(length_measurement, visible_length);
    tracked_length_confidence_ = 0.8 * tracked_length_confidence_ + 0.2;
  } else if (visible_length > tracked_length_estimate_) {
    const double alpha = 0.5 * config_.length_expand_alpha;
    tracked_length_estimate_ = (1.0 - alpha) * tracked_length_estimate_ + alpha * visible_length;
    tracked_length_confidence_ = 0.9 * tracked_length_confidence_ + 0.1 * 0.6;
  } else {
    tracked_length_estimate_ = std::max(tracked_length_estimate_, visible_length);
    tracked_length_confidence_ = 0.95 * tracked_length_confidence_;
  }
}

CableTrackerNode::TrackFit CableTrackerNode::applyTrackedLength(const TrackFit& fit) const {
  if (!fit.valid || !has_length_track_ || tracked_length_estimate_ <= 1e-6) {
    return fit;
  }

  const double lower_bound = fit.has_support_interval ? fit.support_visible_length : 0.0;
  const double target_length = std::max(tracked_length_estimate_, lower_bound);
  if (fit.has_curve && fit.curve_points.size() >= 2) {
    const double current_length = polylineLength(fit.curve_points);
    if (current_length > 1e-6) {
      std::vector<Eigen::Vector3d> stabilized_curve = fit.curve_points;
      Eigen::Vector3d anchor = pointAtRatioOnPolyline(stabilized_curve, 0.5);
      if (fit.has_support_interval) {
        const double low_ratio = Clamp(fit.support_low / current_length, 0.0, 1.0);
        const double high_ratio = Clamp(fit.support_high / current_length, 0.0, 1.0);
        const bool trust_low =
            fit.endpoint_confidence_low >= config_.length_min_endpoint_confidence &&
            fit.endpoint_confidence_high < config_.length_min_endpoint_confidence;
        const bool trust_high =
            fit.endpoint_confidence_high >= config_.length_min_endpoint_confidence &&
            fit.endpoint_confidence_low < config_.length_min_endpoint_confidence;
        if (trust_low) {
          anchor = pointAtRatioOnPolyline(stabilized_curve, low_ratio);
        } else if (trust_high) {
          anchor = pointAtRatioOnPolyline(stabilized_curve, high_ratio);
        }
      }
      const double scale = Clamp(target_length / current_length, 0.85, 1.15);
      for (auto& point : stabilized_curve) {
        point = anchor + scale * (point - anchor);
      }
      TrackFit stabilized = curveTrackFromPoints(stabilized_curve, &fit);
      if (stabilized.valid) {
        stabilized.curve_valid_ratio = fit.curve_valid_ratio;
        stabilized.has_support_interval = fit.has_support_interval;
        stabilized.support_low = fit.support_low;
        stabilized.support_high = fit.support_high;
        stabilized.support_visible_length = fit.support_visible_length;
        stabilized.endpoint_confidence_low = fit.endpoint_confidence_low;
        stabilized.endpoint_confidence_high = fit.endpoint_confidence_high;
        stabilized.tracked_length_reference = target_length;
        return stabilized;
      }
    }
  }

  TrackFit stabilized = fit;
  stabilized.length = target_length;
  stabilized.p0 = stabilized.center - 0.5 * target_length * normalize(stabilized.axis);
  stabilized.p1 = stabilized.center + 0.5 * target_length * normalize(stabilized.axis);
  stabilized.tracked_length_reference = target_length;
  stabilized.valid = true;
  return stabilized;
}

void CableTrackerNode::updateTrackedEndpointState(const TrackFit& fit, const std::string& frame_id, const ros::Time& stamp) {
  if (!fit.valid || frame_id.empty()) {
    return;
  }

  Eigen::Vector3d measured_low = fit.p0;
  Eigen::Vector3d measured_high = fit.p1;
  double measured_conf_low = fit.has_support_interval ? fit.endpoint_confidence_low : 1.0;
  double measured_conf_high = fit.has_support_interval ? fit.endpoint_confidence_high : 1.0;

  if (has_endpoint_track_) {
    const double same_cost =
        (measured_low - tracked_endpoint_low_).norm() + (measured_high - tracked_endpoint_high_).norm();
    const double reverse_cost =
        (measured_low - tracked_endpoint_high_).norm() + (measured_high - tracked_endpoint_low_).norm();
    if (reverse_cost + 1e-6 < same_cost) {
      std::swap(measured_low, measured_high);
      std::swap(measured_conf_low, measured_conf_high);
    }
  }

  if (!has_endpoint_track_ || tracked_endpoint_frame_id_ != frame_id || !hasRecentTrack(stamp)) {
    has_endpoint_track_ = true;
    tracked_endpoint_low_ = measured_low;
    tracked_endpoint_high_ = measured_high;
    tracked_endpoint_confidence_low_ = measured_conf_low;
    tracked_endpoint_confidence_high_ = measured_conf_high;
    tracked_endpoint_frame_id_ = frame_id;
    return;
  }

  auto update_one_endpoint = [&](const Eigen::Vector3d& measured, double measured_conf, Eigen::Vector3d* tracked,
                                 double* tracked_confidence) {
    const bool confident = measured_conf >= config_.length_min_endpoint_confidence;
    const double alpha = confident ? config_.endpoint_confident_alpha
                                   : config_.endpoint_uncertain_alpha * Clamp(measured_conf, 0.0, 1.0);
    Eigen::Vector3d delta = measured - *tracked;
    const double delta_norm = delta.norm();
    const double max_step = confident ? config_.endpoint_max_adjust_m : 0.5 * config_.endpoint_max_adjust_m;
    if (delta_norm > max_step && delta_norm > 1e-9) {
      delta *= max_step / delta_norm;
    }
    *tracked += alpha * delta;
    *tracked_confidence = 0.85 * (*tracked_confidence) + 0.15 * measured_conf;
  };

  update_one_endpoint(measured_low, measured_conf_low, &tracked_endpoint_low_, &tracked_endpoint_confidence_low_);
  update_one_endpoint(measured_high, measured_conf_high, &tracked_endpoint_high_, &tracked_endpoint_confidence_high_);
  tracked_endpoint_frame_id_ = frame_id;
}

CableTrackerNode::TrackFit CableTrackerNode::applyTrackedEndpoints(const TrackFit& fit) const {
  if (!fit.valid || !has_endpoint_track_) {
    return fit;
  }

  if (fit.has_curve && fit.curve_points.size() >= 2) {
    TrackFit working_fit = fit;
    std::vector<Eigen::Vector3d> working_curve = fit.curve_points;
    const double same_cost =
        (working_curve.front() - tracked_endpoint_low_).norm() + (working_curve.back() - tracked_endpoint_high_).norm();
    const double reverse_cost =
        (working_curve.front() - tracked_endpoint_high_).norm() + (working_curve.back() - tracked_endpoint_low_).norm();
    if (reverse_cost + 1e-6 < same_cost) {
      working_curve = std::vector<Eigen::Vector3d>(working_curve.rbegin(), working_curve.rend());
      working_fit = curveTrackFromPoints(working_curve, &fit);
      if (!working_fit.valid) {
        return fit;
      }
      if (fit.has_support_interval) {
        const double total_length = std::max(fit.curve_length, 1e-6);
        working_fit.has_support_interval = true;
        working_fit.support_low = total_length - fit.support_high;
        working_fit.support_high = total_length - fit.support_low;
        working_fit.support_visible_length = working_fit.support_high - working_fit.support_low;
        working_fit.endpoint_confidence_low = fit.endpoint_confidence_high;
        working_fit.endpoint_confidence_high = fit.endpoint_confidence_low;
      }
      working_fit.curve_valid_ratio = fit.curve_valid_ratio;
      working_fit.curve_points_2d = std::vector<cv::Point2d>(fit.curve_points_2d.rbegin(), fit.curve_points_2d.rend());
      working_fit.candidate = fit.candidate;
      working_fit.has_candidate = fit.has_candidate;
      working_curve = working_fit.curve_points;
    }

    const Eigen::Vector3d current_low = working_curve.front();
    const Eigen::Vector3d current_high = working_curve.back();
    Eigen::Vector3d low_shift = tracked_endpoint_low_ - current_low;
    Eigen::Vector3d high_shift = tracked_endpoint_high_ - current_high;
    const double low_shift_norm = low_shift.norm();
    const double high_shift_norm = high_shift.norm();
    if (low_shift_norm > config_.endpoint_max_adjust_m && low_shift_norm > 1e-9) {
      low_shift *= config_.endpoint_max_adjust_m / low_shift_norm;
    }
    if (high_shift_norm > config_.endpoint_max_adjust_m && high_shift_norm > 1e-9) {
      high_shift *= config_.endpoint_max_adjust_m / high_shift_norm;
    }

    const double total_length = std::max(polylineLength(working_curve), 1e-6);
    std::vector<double> cumulative(working_curve.size(), 0.0);
    for (std::size_t index = 1; index < working_curve.size(); ++index) {
      cumulative[index] = cumulative[index - 1] + (working_curve[index] - working_curve[index - 1]).norm();
    }

    std::vector<Eigen::Vector3d> warped_curve = working_curve;
    for (std::size_t index = 0; index < warped_curve.size(); ++index) {
      const double ratio = Clamp(cumulative[index] / total_length, 0.0, 1.0);
      warped_curve[index] += (1.0 - ratio) * low_shift + ratio * high_shift;
    }

    TrackFit warped_fit = curveTrackFromPoints(warped_curve, &working_fit);
    if (!warped_fit.valid) {
      return fit;
    }
    warped_fit.curve_valid_ratio = working_fit.curve_valid_ratio;
    warped_fit.curve_points_2d = working_fit.curve_points_2d;
    warped_fit.candidate = working_fit.candidate;
    warped_fit.has_candidate = working_fit.has_candidate;
    warped_fit.has_optimizer_cost = working_fit.has_optimizer_cost;
    warped_fit.optimizer_cost = working_fit.optimizer_cost;
    warped_fit.optimizer_nfev = working_fit.optimizer_nfev;
    warped_fit.radius = working_fit.radius;
    warped_fit.tracked_length_reference = working_fit.tracked_length_reference;
    if (working_fit.has_support_interval) {
      const double low_ratio = Clamp(working_fit.support_low / std::max(working_fit.curve_length, 1e-6), 0.0, 1.0);
      const double high_ratio = Clamp(working_fit.support_high / std::max(working_fit.curve_length, 1e-6), 0.0, 1.0);
      warped_fit.has_support_interval = true;
      warped_fit.support_low = low_ratio * warped_fit.curve_length;
      warped_fit.support_high = high_ratio * warped_fit.curve_length;
      warped_fit.support_visible_length = std::max(0.0, warped_fit.support_high - warped_fit.support_low);
      warped_fit.endpoint_confidence_low = working_fit.endpoint_confidence_low;
      warped_fit.endpoint_confidence_high = working_fit.endpoint_confidence_high;
    }
    return warped_fit;
  }

  TrackFit stabilized = fit;
  stabilized.p0 = tracked_endpoint_low_;
  stabilized.p1 = tracked_endpoint_high_;
  stabilized.center = 0.5 * (stabilized.p0 + stabilized.p1);
  const Eigen::Vector3d axis = stabilized.p1 - stabilized.p0;
  stabilized.axis = normalize(axis);
  stabilized.length = axis.norm();
  stabilized.valid = stabilized.length > 1e-6;
  return stabilized;
}

bool CableTrackerNode::optimizeCurve(const ParallelCandidate& candidate, const TrackFit& fit_camera,
                                     const std::vector<Eigen::Vector3d>& points_camera, const TrackFit* previous_state,
                                     TrackFit* optimized_fit) const {
  if (!config_.joint_optimization_enabled || !config_.native_backend_enabled || fit_camera.curve_points.size() < 2) {
    return false;
  }

  Gp11NativeConfig native_config;
  native_config.control_point_count = std::max(3, config_.optimizer_control_point_count);
  native_config.curve_sample_count = std::max(native_config.control_point_count + 2, config_.optimizer_curve_sample_count);
  native_config.max_iterations = std::max(1, config_.native_optimizer_max_iterations);
  native_config.point_sigma_m = std::max(1e-4, config_.optimizer_point_sigma_m);
  native_config.curve_point_sigma_m = std::max(1e-4, config_.optimizer_curve_point_sigma_m);
  native_config.curve_image_sigma_px = std::max(1e-4, config_.optimizer_curve_image_sigma_px);
  native_config.curve_smooth_sigma_m = std::max(1e-4, config_.optimizer_curve_smooth_sigma_m);
  native_config.curve_length_sigma_m = std::max(1e-4, config_.optimizer_curve_length_sigma_m);
  native_config.temporal_curve_sigma_m = std::max(1e-4, config_.optimizer_temporal_curve_sigma_m);
  native_config.control_point_bound_m = std::max(0.01, config_.optimizer_control_point_bound_m);
  native_config.min_depth_m =
      std::max(0.02, config_.cable_depth_filter_enabled ? config_.cable_depth_min_m : config_.min_depth_m);
  native_config.point_assign_distance_m = std::max(0.01, config_.native_point_assign_distance_m);
  native_config.support_cover_sigma_m = std::max(1e-4, config_.support_cover_sigma_m);

  const std::vector<double> observed_curve_3d = FlattenPoints3d(fit_camera.curve_points);
  const std::vector<double> observed_curve_2d = FlattenPoints2d(fit_camera.curve_points_2d);
  const std::vector<double> point_cloud = FlattenPoints3d(points_camera);
  std::vector<Eigen::Vector3d> previous_curve;
  if (previous_state != nullptr) {
    if (previous_state->has_curve && !previous_state->curve_points.empty()) {
      previous_curve = previous_state->curve_points;
    } else if (previous_state->valid) {
      previous_curve = {previous_state->p0, previous_state->p1};
    }
  }
  const std::vector<double> previous_points = FlattenPoints3d(previous_curve);
  const double reference_length =
      has_length_track_ && tracked_length_estimate_ > 1e-6 ? tracked_length_estimate_ : fit_camera.curve_length;

  std::vector<double> output_control_points(static_cast<std::size_t>(native_config.control_point_count * 3), 0.0);
  std::vector<double> output_curve_points(static_cast<std::size_t>(native_config.curve_sample_count * 3), 0.0);
  Gp11NativeResult result;
  const int success = gp11_optimize_curve(
      &native_config, observed_curve_3d.data(), static_cast<int>(fit_camera.curve_points.size()),
      observed_curve_2d.empty() ? nullptr : observed_curve_2d.data(), static_cast<int>(fit_camera.curve_points_2d.size()),
      point_cloud.empty() ? nullptr : point_cloud.data(), static_cast<int>(points_camera.size()),
      previous_points.empty() ? nullptr : previous_points.data(), static_cast<int>(previous_curve.size()), camera_model_.fx(),
      camera_model_.fy(), camera_model_.cx(), camera_model_.cy(), reference_length, nullptr, nullptr, 0.0, 0.0, 0.0,
      0.0, fit_camera.radius,
      output_control_points.data(), output_curve_points.data(), &result);
  if (success == 0 || result.success == 0) {
    return false;
  }

  TrackFit optimized = fit_camera;
  optimized.curve_points = UnflattenPoints3d(output_curve_points);
  optimized.has_curve = optimized.curve_points.size() >= 2;
  optimized.radius = result.radius;
  optimized.has_optimizer_cost = true;
  optimized.optimizer_cost = result.optimizer_cost;
  optimized.optimizer_nfev = result.iterations;
  optimized = applyCurveTrack(optimized, optimized.curve_points, fit_camera.curve_valid_ratio);
  optimized.valid = optimized.has_curve;
  optimized.curve_points_2d = fit_camera.curve_points_2d;
  optimized.has_support_interval = fit_camera.has_support_interval;
  optimized.support_low = fit_camera.support_low;
  optimized.support_high = fit_camera.support_high;
  optimized.support_visible_length = fit_camera.support_visible_length;
  optimized.endpoint_confidence_low = fit_camera.endpoint_confidence_low;
  optimized.endpoint_confidence_high = fit_camera.endpoint_confidence_high;
  optimized.tracked_length_reference = reference_length;
  optimized.candidate = candidate;
  optimized.has_candidate = true;
  *optimized_fit = optimized;
  return optimized_fit->valid;
}

bool CableTrackerNode::transformPoints(const std::vector<Eigen::Vector3d>& points, const std::string& source_frame,
                                       const std::string& target_frame, const ros::Time& stamp,
                                       std::vector<Eigen::Vector3d>* transformed) const {
  if (source_frame.empty() || source_frame == target_frame) {
    *transformed = points;
    return true;
  }
  geometry_msgs::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(target_frame, source_frame, stamp, ros::Duration(0.2));
  } catch (const tf2::TransformException& exc) {
    ROS_WARN_THROTTLE(2.0, "TF lookup failed: %s -> %s (%s)", source_frame.c_str(), target_frame.c_str(), exc.what());
    return false;
  }

  const Eigen::Quaterniond rotation(transform.transform.rotation.w, transform.transform.rotation.x,
                                    transform.transform.rotation.y, transform.transform.rotation.z);
  const Eigen::Vector3d translation(transform.transform.translation.x, transform.transform.translation.y,
                                    transform.transform.translation.z);
  const Eigen::Matrix3d rotation_matrix = rotation.normalized().toRotationMatrix();

  transformed->clear();
  transformed->reserve(points.size());
  for (const auto& point : points) {
    transformed->push_back(rotation_matrix * point + translation);
  }
  return true;
}

bool CableTrackerNode::transformFitToFrame(const TrackFit& fit, const std::string& source_frame,
                                           const std::string& target_frame, const ros::Time& stamp,
                                           TrackFit* transformed) const {
  if (!fit.valid) {
    return false;
  }
  *transformed = fit;
  if (source_frame.empty() || source_frame == target_frame) {
    if (fit.has_curve && !fit.curve_points.empty()) {
      *transformed = applyCurveTrack(*transformed, fit.curve_points, fit.curve_valid_ratio);
    }
    transformed->valid = true;
    return true;
  }

  std::vector<Eigen::Vector3d> base_points = {fit.center, fit.p0, fit.p1};
  std::vector<Eigen::Vector3d> transformed_points;
  if (!transformPoints(base_points, source_frame, target_frame, stamp, &transformed_points) || transformed_points.size() != 3) {
    return false;
  }
  transformed->center = transformed_points[0];
  transformed->p0 = transformed_points[1];
  transformed->p1 = transformed_points[2];
  const Eigen::Vector3d transformed_axis = transformed->p1 - transformed->p0;
  transformed->axis = Eigen::Vector3d::UnitX();
  if (transformed_axis.norm() >= 1e-9) {
    transformed->axis = transformed_axis / transformed_axis.norm();
  }
  transformed->length = (transformed->p1 - transformed->p0).norm();
  if (fit.has_curve && !fit.curve_points.empty()) {
    std::vector<Eigen::Vector3d> transformed_curve;
    if (transformPoints(fit.curve_points, source_frame, target_frame, stamp, &transformed_curve) &&
        transformed_curve.size() >= 2) {
      *transformed = applyCurveTrack(*transformed, transformed_curve, fit.curve_valid_ratio);
    }
  }
  transformed->valid = true;
  return true;
}

bool CableTrackerNode::passesTemporalGate(const TrackFit& fit, const ros::Time& stamp) const {
  if (!hasRecentTrack(stamp) || !has_last_track_) {
    return true;
  }
  const double position_error = (fit.center - last_center_).norm();
  const double cosine = Clamp(std::abs(fit.axis.dot(last_axis_)), -1.0, 1.0);
  const double angle_error_deg = std::acos(cosine) * 180.0 / M_PI;
  return position_error <= config_.temporal_position_gate_m && angle_error_deg <= config_.temporal_angle_gate_deg;
}

double CableTrackerNode::scoreCandidate(const ParallelCandidate& candidate, const TrackFit& fit, const ros::Time& stamp,
                                        bool use_tracking_prior) const {
  double score = 6.0 * static_cast<double>(fit.inlier_count) + 80.0 * candidate.valid_ratio +
                 120.0 * fit.band_valid_ratio + 0.25 * candidate.length_px + 25.0 * fit.length - 350.0 * fit.residual;
  if (fit.has_curve) {
    score += 120.0 * fit.curve_valid_ratio;
    score -= 180.0 * fit.curve_residual;
  }
  if (use_tracking_prior && hasRecentTrack(stamp) && has_last_track_) {
    const double position_error = (fit.center - last_center_).norm();
    const double alignment = std::abs(fit.axis.dot(last_axis_));
    score += 140.0 * alignment - 50.0 * position_error;
    score -= 35.0 * std::abs(fit.length - last_length_);
    if (!last_curve_points_.empty() && fit.has_curve) {
      std::vector<Eigen::Vector3d> current_curve = resamplePolyline(fit.curve_points, config_.curve_resample_point_count);
      std::vector<Eigen::Vector3d> reference_curve = resamplePolyline(last_curve_points_, config_.curve_resample_point_count);
      current_curve = alignCurveDirection(current_curve, reference_curve);
      double curve_distance = 0.0;
      for (std::size_t index = 0; index < current_curve.size() && index < reference_curve.size(); ++index) {
        curve_distance += (current_curve[index] - reference_curve[index]).norm();
      }
      curve_distance /= static_cast<double>(std::max<std::size_t>(1, std::min(current_curve.size(), reference_curve.size())));
      score -= 400.0 * curve_distance;
    }
  }
  return score;
}

CableTrackerNode::TrackFit CableTrackerNode::smoothTrack(const TrackFit& fit, const ros::Time& stamp) const {
  const double smoothing_reference_length =
      has_length_track_ && tracked_length_estimate_ > 1e-6 ? tracked_length_estimate_ : last_length_;
  if (fit.has_curve && fit.curve_points.size() >= 2) {
    std::vector<Eigen::Vector3d> current_curve = fit.curve_points;
    if (hasRecentTrack(stamp) && !last_curve_points_.empty()) {
      const double alpha = config_.smoothing_alpha;
      std::vector<Eigen::Vector3d> reference_curve = last_curve_points_;
      current_curve = alignCurveDirection(current_curve, reference_curve);
      current_curve = resamplePolyline(current_curve, config_.curve_resample_point_count);
      reference_curve = resamplePolyline(reference_curve, config_.curve_resample_point_count);
      std::vector<Eigen::Vector3d> smoothed_curve;
      smoothed_curve.reserve(current_curve.size());
      for (std::size_t index = 0; index < current_curve.size(); ++index) {
        smoothed_curve.push_back(alpha * current_curve[index] + (1.0 - alpha) * reference_curve[index]);
      }
      TrackFit smoothed_fit = curveTrackFromPoints(smoothed_curve, &fit);
      if (smoothed_fit.valid && smoothing_reference_length > 1e-6) {
        const double min_length = (1.0 - config_.curve_length_change_ratio) * smoothing_reference_length;
        const double max_length = (1.0 + config_.curve_length_change_ratio) * smoothing_reference_length;
        const double clamped_length = Clamp(smoothed_fit.length, min_length, max_length);
        if (std::abs(clamped_length - smoothed_fit.length) > 1e-6) {
          const Eigen::Vector3d center_point = pointAtRatioOnPolyline(smoothed_curve, 0.5);
          const double scale = clamped_length / std::max(smoothed_fit.length, 1e-6);
          for (auto& point : smoothed_curve) {
            point = center_point + scale * (point - center_point);
          }
          smoothed_fit = curveTrackFromPoints(smoothed_curve, &fit);
        }
      }
      if (smoothed_fit.valid) {
        smoothed_fit.curve_valid_ratio = fit.curve_valid_ratio;
        return smoothed_fit;
      }
    }

    TrackFit curve_fit = curveTrackFromPoints(current_curve, &fit);
    if (curve_fit.valid) {
      curve_fit.curve_valid_ratio = fit.curve_valid_ratio;
      return curve_fit;
    }
  }

  if (!hasRecentTrack(stamp) || !has_last_track_) {
    return fit;
  }
  const double alpha = config_.smoothing_alpha;
  Eigen::Vector3d axis = fit.axis;
  if (axis.dot(last_axis_) < 0.0) {
    axis = -axis;
  }
  TrackFit smoothed = fit;
  smoothed.center = alpha * fit.center + (1.0 - alpha) * last_center_;
  const Eigen::Vector3d smoothed_axis = alpha * axis + (1.0 - alpha) * last_axis_;
  smoothed.axis = Eigen::Vector3d::UnitX();
  if (smoothed_axis.norm() >= 1e-9) {
    smoothed.axis = smoothed_axis / smoothed_axis.norm();
  }
  smoothed.length = alpha * fit.length + (1.0 - alpha) * smoothing_reference_length;
  smoothed.p0 = smoothed.center - 0.5 * smoothed.length * smoothed.axis;
  smoothed.p1 = smoothed.center + 0.5 * smoothed.length * smoothed.axis;
  smoothed.tracked_length_reference = smoothing_reference_length;
  smoothed.valid = true;
  return smoothed;
}

void CableTrackerNode::updateTrack(const TrackFit& track, const Eigen::Vector3d& target, const std::string& frame_id,
                                   const ros::Time& stamp, const TrackFit* camera_track,
                                   const std::string& camera_frame_id) {
  has_last_track_ = true;
  last_center_ = track.center;
  last_axis_ = Eigen::Vector3d::UnitX();
  if (track.axis.norm() >= 1e-9) {
    last_axis_ = track.axis / track.axis.norm();
  }
  last_length_ = track.length;
  last_p0_ = track.p0;
  last_p1_ = track.p1;
  last_target_ = target;
  last_frame_id_ = frame_id;
  last_track_stamp_ = stamp.isZero() ? ros::Time::now() : stamp;
  last_curve_points_ = track.has_curve ? track.curve_points : std::vector<Eigen::Vector3d>();
  if (camera_track != nullptr) {
    has_last_camera_track_ = camera_track->valid;
    last_camera_track_storage_ = *camera_track;
    last_camera_frame_id_ = camera_frame_id;
  }
}

const CableTrackerNode::TrackFit* CableTrackerNode::getPreviousCameraTrack(const std::string& frame_id,
                                                                           const ros::Time& stamp) const {
  if (!has_last_camera_track_ || last_camera_frame_id_ != frame_id || !hasRecentTrack(stamp)) {
    return nullptr;
  }
  return &last_camera_track_storage_;
}

bool CableTrackerNode::hasRecentTrack(const ros::Time& stamp) const {
  if (!has_last_track_ || last_track_stamp_.isZero()) {
    return false;
  }
  const ros::Time reference_stamp = stamp.isZero() ? ros::Time::now() : stamp;
  double age = (reference_stamp - last_track_stamp_).toSec();
  if (age < 0.0) {
    age = 0.0;
  }
  return age <= config_.lost_track_hold_time_s;
}

void CableTrackerNode::handleMissingTrack(cv::Mat* debug_image, const std_msgs::Header& header,
                                          const std::string& status_text) {
  if (hasRecentTrack(header.stamp) && has_last_track_ && !last_frame_id_.empty()) {
    if (has_last_segment_) {
      drawDebugSegment(debug_image, last_segment_p0_, last_segment_p1_, sampleSegmentPoints(last_segment_p0_, last_segment_p1_, 16));
    }
    publishMarker(last_p0_, last_p1_, last_target_, last_frame_id_, header.stamp, last_curve_points_);
    publishTarget(last_target_, last_frame_id_, header.stamp);
    if (debug_image != nullptr && !debug_image->empty()) {
      publishDebugImage(*debug_image, header);
    }
    publishStatus(status_text + " Holding last stable track.");
    return;
  }
  has_last_segment_ = false;
  consecutive_local_misses_ = 0;
  if (debug_image != nullptr && !debug_image->empty()) {
    publishDebugImage(*debug_image, header);
  }
  publishStatus(status_text);
}

void CableTrackerNode::drawCandidateSegments(cv::Mat* image, const std::vector<ParallelCandidate>& candidates) const {
  if (image == nullptr || image->empty()) {
    return;
  }
  const int limit = std::min<int>(config_.debug_candidate_limit, candidates.size());
  for (int index = 0; index < limit; ++index) {
    const ParallelCandidate& candidate = candidates[static_cast<std::size_t>(index)];
    cv::line(*image, scaleDebugPoint(candidate.edge_a_p0), scaleDebugPoint(candidate.edge_a_p1), cv::Scalar(255, 160, 0), 1,
             config_.debug_antialiasing ? cv::LINE_AA : cv::LINE_8);
    cv::line(*image, scaleDebugPoint(candidate.edge_b_p0), scaleDebugPoint(candidate.edge_b_p1), cv::Scalar(255, 160, 0), 1,
             config_.debug_antialiasing ? cv::LINE_AA : cv::LINE_8);
    cv::line(*image, scaleDebugPoint(candidate.p0), scaleDebugPoint(candidate.p1), cv::Scalar(255, 180, 0), 1,
             config_.debug_antialiasing ? cv::LINE_AA : cv::LINE_8);
  }
}

void CableTrackerNode::drawDebugSegment(cv::Mat* image, const cv::Point2d& p0, const cv::Point2d& p1,
                                        const std::vector<cv::Point2d>& points_2d) const {
  if (image == nullptr || image->empty()) {
    return;
  }
  const int line_type = config_.debug_antialiasing ? cv::LINE_AA : cv::LINE_8;
  if (points_2d.size() >= 2) {
    for (std::size_t index = 0; index + 1 < points_2d.size(); ++index) {
      cv::line(*image, scaleDebugPoint(points_2d[index]), scaleDebugPoint(points_2d[index + 1]), cv::Scalar(0, 255, 0),
               debugLineWidth(config_.line_width_px), line_type);
    }
  } else {
    cv::line(*image, scaleDebugPoint(p0), scaleDebugPoint(p1), cv::Scalar(0, 255, 0), debugLineWidth(config_.line_width_px),
             line_type);
  }
  if (config_.debug_draw_sample_points && !points_2d.empty()) {
    const std::size_t stride = std::max<std::size_t>(1, points_2d.size() / 12);
    for (std::size_t index = 0; index < points_2d.size(); index += stride) {
      cv::circle(*image, scaleDebugPoint(points_2d[index]), debugCircleRadius(2), cv::Scalar(0, 0, 255), -1, line_type);
    }
  }
}

void CableTrackerNode::publishMarker(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& target,
                                     const std::string& frame_id, const ros::Time& stamp,
                                     const std::vector<Eigen::Vector3d>& curve_points) const {
  visualization_msgs::Marker line;
  line.header.frame_id = frame_id;
  line.header.stamp = stamp;
  line.ns = "gp11_perception";
  line.id = 0;
  line.type = visualization_msgs::Marker::LINE_STRIP;
  line.action = visualization_msgs::Marker::ADD;
  line.scale.x = 0.008;
  line.color.r = 0.1F;
  line.color.g = 0.9F;
  line.color.b = 0.2F;
  line.color.a = 0.95F;
  if (curve_points.size() >= 2) {
    line.points.reserve(curve_points.size());
    for (const auto& point : curve_points) {
      geometry_msgs::Point msg_point;
      msg_point.x = point.x();
      msg_point.y = point.y();
      msg_point.z = point.z();
      line.points.push_back(msg_point);
    }
  } else {
    geometry_msgs::Point msg_p0;
    msg_p0.x = p0.x();
    msg_p0.y = p0.y();
    msg_p0.z = p0.z();
    geometry_msgs::Point msg_p1;
    msg_p1.x = p1.x();
    msg_p1.y = p1.y();
    msg_p1.z = p1.z();
    line.points.push_back(msg_p0);
    line.points.push_back(msg_p1);
  }
  marker_pub_.publish(line);

  visualization_msgs::Marker sphere;
  sphere.header.frame_id = frame_id;
  sphere.header.stamp = stamp;
  sphere.ns = "gp11_perception";
  sphere.id = 1;
  sphere.type = visualization_msgs::Marker::SPHERE;
  sphere.action = visualization_msgs::Marker::ADD;
  sphere.pose.position.x = target.x();
  sphere.pose.position.y = target.y();
  sphere.pose.position.z = target.z();
  sphere.pose.orientation.w = 1.0;
  sphere.scale.x = 0.03;
  sphere.scale.y = 0.03;
  sphere.scale.z = 0.03;
  sphere.color.r = 1.0F;
  sphere.color.g = 0.2F;
  sphere.color.b = 0.2F;
  sphere.color.a = 0.95F;
  marker_pub_.publish(sphere);
}

void CableTrackerNode::publishTarget(const Eigen::Vector3d& point_xyz, const std::string& frame_id,
                                     const ros::Time& stamp) const {
  geometry_msgs::PointStamped msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.point.x = point_xyz.x();
  msg.point.y = point_xyz.y();
  msg.point.z = point_xyz.z();
  target_pub_.publish(msg);
  if (config_.publish_target_to_moveit) {
    moveit_target_pub_.publish(msg);
  }
}

void CableTrackerNode::publishDebugImage(const cv::Mat& image, const std_msgs::Header& header) {
  if (!config_.publish_debug_image || image.empty()) {
    return;
  }
  cv_bridge::CvImage cv_image(header, sensor_msgs::image_encodings::BGR8, image);
  debug_pub_.publish(cv_image.toImageMsg());
  last_debug_publish_stamp_ = header.stamp.isZero() ? ros::Time::now() : header.stamp;
}

void CableTrackerNode::publishStatus(const std::string& text) const {
  ROS_INFO_STREAM_THROTTLE(1.0, text);
  std_msgs::String msg;
  msg.data = text;
  status_pub_.publish(msg);
}

Eigen::Vector2d CableTrackerNode::toEigen(const cv::Point2d& point) { return Eigen::Vector2d(point.x, point.y); }

cv::Point2d CableTrackerNode::toCv(const Eigen::Vector2d& point) { return cv::Point2d(point.x(), point.y()); }

Eigen::Vector3d CableTrackerNode::normalize(const Eigen::Vector3d& vector) {
  const double norm = vector.norm();
  if (norm < 1e-9) {
    return Eigen::Vector3d::UnitX();
  }
  return vector / norm;
}

Eigen::Vector2d CableTrackerNode::normalize(const Eigen::Vector2d& vector) {
  const double norm = vector.norm();
  if (norm < 1e-9) {
    return Eigen::Vector2d::UnitX();
  }
  return vector / norm;
}

double CableTrackerNode::polylineLength(const std::vector<Eigen::Vector3d>& points) {
  if (points.size() < 2) {
    return 0.0;
  }
  double length = 0.0;
  for (std::size_t index = 0; index + 1 < points.size(); ++index) {
    length += (points[index + 1] - points[index]).norm();
  }
  return length;
}

std::vector<Eigen::Vector3d> CableTrackerNode::resamplePolyline(const std::vector<Eigen::Vector3d>& points,
                                                                int sample_count) {
  if (points.empty()) {
    return {};
  }
  if (points.size() == 1) {
    return std::vector<Eigen::Vector3d>(static_cast<std::size_t>(std::max(2, sample_count)), points.front());
  }
  sample_count = std::max(2, sample_count);
  std::vector<double> cumulative(points.size(), 0.0);
  for (std::size_t index = 1; index < points.size(); ++index) {
    cumulative[index] = cumulative[index - 1] + (points[index] - points[index - 1]).norm();
  }
  const double total_length = cumulative.back();
  if (total_length < 1e-9) {
    return std::vector<Eigen::Vector3d>(static_cast<std::size_t>(sample_count), points.front());
  }
  std::vector<Eigen::Vector3d> result;
  result.reserve(static_cast<std::size_t>(sample_count));
  for (int sample_index = 0; sample_index < sample_count; ++sample_index) {
    const double distance = (static_cast<double>(sample_index) / static_cast<double>(sample_count - 1)) * total_length;
    auto upper = std::lower_bound(cumulative.begin(), cumulative.end(), distance);
    std::size_t segment_index = static_cast<std::size_t>(std::distance(cumulative.begin(), upper));
    segment_index = std::max<std::size_t>(1, std::min<std::size_t>(segment_index, points.size() - 1));
    const double segment_start = cumulative[segment_index - 1];
    const double segment_length = std::max(cumulative[segment_index] - segment_start, 1e-9);
    const double ratio = (distance - segment_start) / segment_length;
    result.push_back((1.0 - ratio) * points[segment_index - 1] + ratio * points[segment_index]);
  }
  return result;
}

std::vector<cv::Point2d> CableTrackerNode::resamplePolyline(const std::vector<cv::Point2d>& points, int sample_count) {
  if (points.empty()) {
    return {};
  }
  if (points.size() == 1) {
    return std::vector<cv::Point2d>(static_cast<std::size_t>(std::max(2, sample_count)), points.front());
  }
  sample_count = std::max(2, sample_count);
  std::vector<double> cumulative(points.size(), 0.0);
  for (std::size_t index = 1; index < points.size(); ++index) {
    cumulative[index] = cumulative[index - 1] + cv::norm(points[index] - points[index - 1]);
  }
  const double total_length = cumulative.back();
  if (total_length < 1e-9) {
    return std::vector<cv::Point2d>(static_cast<std::size_t>(sample_count), points.front());
  }
  std::vector<cv::Point2d> result;
  result.reserve(static_cast<std::size_t>(sample_count));
  for (int sample_index = 0; sample_index < sample_count; ++sample_index) {
    const double distance = (static_cast<double>(sample_index) / static_cast<double>(sample_count - 1)) * total_length;
    auto upper = std::lower_bound(cumulative.begin(), cumulative.end(), distance);
    std::size_t segment_index = static_cast<std::size_t>(std::distance(cumulative.begin(), upper));
    segment_index = std::max<std::size_t>(1, std::min<std::size_t>(segment_index, points.size() - 1));
    const double segment_start = cumulative[segment_index - 1];
    const double segment_length = std::max(cumulative[segment_index] - segment_start, 1e-9);
    const double ratio = (distance - segment_start) / segment_length;
    result.push_back((1.0 - ratio) * points[segment_index - 1] + ratio * points[segment_index]);
  }
  return result;
}

std::vector<Eigen::Vector3d> CableTrackerNode::smoothPolyline(const std::vector<Eigen::Vector3d>& points,
                                                              int window_size) {
  if (points.size() < 3 || window_size <= 1) {
    return points;
  }
  window_size = std::max(1, window_size);
  if ((window_size % 2) == 0) {
    ++window_size;
  }
  const int radius = window_size / 2;
  std::vector<Eigen::Vector3d> smoothed(points.size(), Eigen::Vector3d::Zero());
  for (std::size_t index = 0; index < points.size(); ++index) {
    Eigen::Vector3d accum = Eigen::Vector3d::Zero();
    int count = 0;
    for (int offset = -radius; offset <= radius; ++offset) {
      const int clamped = std::max<int>(0, std::min<int>(static_cast<int>(points.size()) - 1, static_cast<int>(index) + offset));
      accum += points[static_cast<std::size_t>(clamped)];
      ++count;
    }
    smoothed[index] = accum / static_cast<double>(count);
  }
  smoothed.front() = points.front();
  smoothed.back() = points.back();
  return smoothed;
}

std::vector<Eigen::Vector3d> CableTrackerNode::alignCurveDirection(const std::vector<Eigen::Vector3d>& points,
                                                                   const std::vector<Eigen::Vector3d>& reference_points) {
  if (points.size() < 2 || reference_points.size() < 2) {
    return points;
  }
  const double same_cost = (points.front() - reference_points.front()).norm() +
                           (points.back() - reference_points.back()).norm();
  const double reverse_cost = (points.front() - reference_points.back()).norm() +
                              (points.back() - reference_points.front()).norm();
  if (reverse_cost >= same_cost) {
    return points;
  }
  return std::vector<Eigen::Vector3d>(points.rbegin(), points.rend());
}

Eigen::Vector3d CableTrackerNode::pointAtRatioOnPolyline(const std::vector<Eigen::Vector3d>& points, double ratio) {
  if (points.empty()) {
    return Eigen::Vector3d::Zero();
  }
  if (points.size() == 1) {
    return points.front();
  }
  ratio = Clamp(ratio, 0.0, 1.0);
  std::vector<double> cumulative(points.size(), 0.0);
  for (std::size_t index = 1; index < points.size(); ++index) {
    cumulative[index] = cumulative[index - 1] + (points[index] - points[index - 1]).norm();
  }
  const double total_length = cumulative.back();
  if (total_length < 1e-9) {
    return points.front();
  }
  const double distance = ratio * total_length;
  auto upper = std::lower_bound(cumulative.begin(), cumulative.end(), distance);
  std::size_t segment_index = static_cast<std::size_t>(std::distance(cumulative.begin(), upper));
  segment_index = std::max<std::size_t>(1, std::min<std::size_t>(segment_index, points.size() - 1));
  const double segment_start = cumulative[segment_index - 1];
  const double segment_length = std::max(cumulative[segment_index] - segment_start, 1e-9);
  const double local_ratio = (distance - segment_start) / segment_length;
  return (1.0 - local_ratio) * points[segment_index - 1] + local_ratio * points[segment_index];
}

std::pair<double, double> CableTrackerNode::projectInterval(const cv::Point2d& p0, const cv::Point2d& p1,
                                                            const cv::Point2d& origin,
                                                            const Eigen::Vector2d& direction) {
  const double a = (toEigen(p0 - origin)).dot(direction);
  const double b = (toEigen(p1 - origin)).dot(direction);
  return std::make_pair(std::min(a, b), std::max(a, b));
}

cv::Point2d CableTrackerNode::pointOnLineAtProjection(const cv::Point2d& base_point,
                                                      const Eigen::Vector2d& line_direction,
                                                      const cv::Point2d& origin,
                                                      const Eigen::Vector2d& projection_direction,
                                                      double target_projection) {
  const double denom = line_direction.dot(projection_direction);
  if (std::abs(denom) < 1e-6) {
    return base_point;
  }
  const double base_projection = toEigen(base_point - origin).dot(projection_direction);
  const double distance = (target_projection - base_projection) / denom;
  return base_point + toCv(distance * line_direction);
}

void CableTrackerNode::rgbdCallback(const sensor_msgs::ImageConstPtr& color_msg,
                                    const sensor_msgs::ImageConstPtr& depth_msg,
                                    const sensor_msgs::CameraInfoConstPtr& info_msg) {
  if (!ros::ok()) {
    return;
  }

  cv::Mat color;
  cv::Mat depth_m;
  try {
    color = cv_bridge::toCvShare(color_msg, sensor_msgs::image_encodings::BGR8)->image.clone();
    depth_m = convertDepthToMeters(depth_msg);
  } catch (const std::exception& exc) {
    publishStatus(std::string("RGB-D conversion failed: ") + exc.what());
    return;
  }

  camera_model_.fromCameraInfo(*info_msg);
  cv::Mat lab;
  cv::cvtColor(color, lab, cv::COLOR_BGR2Lab);
  cv::Mat hsv;
  if (config_.silver_prefilter_enabled) {
    cv::cvtColor(color, hsv, cv::COLOR_BGR2HSV);
  }
  cv::Mat gray;
  cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0.0);
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
  clahe->apply(gray, gray);

  cv::Mat color_mask;
  if (config_.silver_prefilter_enabled) {
    color_mask = buildSilverMask(lab, hsv);
  }
  const double mask_coverage = maskCoverageRatio(color_mask);

  cv::Mat debug = prepareDebugImage(color, color_msg->header.stamp);
  const bool use_local_tracking = has_last_segment_ && hasRecentTrack(color_msg->header.stamp);
  const cv::Rect tracked_roi = candidateRoi(gray.size(), color_msg->header.stamp);
  cv::Rect global_roi;
  if (config_.has_search_roi) {
    global_roi = cv::Rect(config_.search_roi[0], config_.search_roi[1], config_.search_roi[2] - config_.search_roi[0],
                          config_.search_roi[3] - config_.search_roi[1]);
    global_roi &= cv::Rect(0, 0, gray.cols, gray.rows);
  }
  if (!debug.empty() && HasArea(tracked_roi)) {
    cv::rectangle(debug, scaleDebugPoint(cv::Point2d(tracked_roi.x, tracked_roi.y)),
                  scaleDebugPoint(cv::Point2d(tracked_roi.x + tracked_roi.width, tracked_roi.y + tracked_roi.height)),
                  cv::Scalar(255, 255, 0), 1, config_.debug_antialiasing ? cv::LINE_AA : cv::LINE_8);
  }
  if (!debug.empty() && !color_mask.empty()) {
    debug = overlayColorMask(debug, color_mask);
  }

  const std::string source_frame =
      !color_msg->header.frame_id.empty()
          ? color_msg->header.frame_id
          : (!info_msg->header.frame_id.empty() ? info_msg->header.frame_id : camera_model_.tfFrame());
  const std::string target_frame = config_.output_frame.empty() ? source_frame : config_.output_frame;

  const TrackFit* previous_camera_track = getPreviousCameraTrack(source_frame, color_msg->header.stamp);

  auto select_best_track = [&](const std::vector<ParallelCandidate>& candidates, bool enforce_temporal_gate,
                               bool use_tracking_prior, TrackFit* selected_track,
                               TrackFit* selected_camera_track) -> bool {
    if (selected_track == nullptr || selected_camera_track == nullptr || candidates.empty()) {
      return false;
    }
    const TrackFit* temporal_reference = use_tracking_prior ? previous_camera_track : nullptr;

    const int candidate_limit = (config_.joint_optimization_enabled && config_.native_backend_enabled)
                                    ? std::min<int>(config_.optimizer_candidate_top_k, candidates.size())
                                    : static_cast<int>(candidates.size());

    bool has_selected = false;
    double best_rank = std::numeric_limits<double>::infinity();
    for (int index = 0; index < candidate_limit; ++index) {
      const ParallelCandidate& candidate = candidates[static_cast<std::size_t>(index)];

      std::vector<cv::Point2d> curve_points_2d;
      std::vector<Eigen::Vector3d> curve_points_3d;
      double curve_valid_ratio = 0.0;
      candidateCurvePointsTo3d(candidate, depth_m, &curve_points_2d, &curve_points_3d, &curve_valid_ratio);

      std::vector<cv::Point2d> band_points_2d;
      std::vector<Eigen::Vector3d> points_3d_camera;
      double band_valid_ratio = 0.0;
      segmentBandPointsTo3d(candidate, depth_m, &band_points_2d, &points_3d_camera, &band_valid_ratio);

      TrackFit fit_camera;
      if (static_cast<int>(points_3d_camera.size()) < config_.min_inlier_count) {
        if (static_cast<int>(curve_points_3d.size()) < config_.curve_min_valid_points) {
          continue;
        }
        fit_camera.valid = true;
        fit_camera.residual = 0.0;
        fit_camera.inlier_count = static_cast<int>(curve_points_3d.size());
        fit_camera.point_count = static_cast<int>(curve_points_3d.size());
        fit_camera.band_valid_ratio = curve_valid_ratio;
        fit_camera.curve_points_2d = curve_points_2d;
      } else {
        fit_camera = fitLineRansac(points_3d_camera);
        if (!fit_camera.valid) {
          continue;
        }
        fit_camera.band_valid_ratio = band_valid_ratio;
        fit_camera.curve_points_2d = curve_points_2d;
      }

      fit_camera = applyCurveTrack(fit_camera, curve_points_3d, curve_valid_ratio);
      fit_camera.candidate = candidate;
      fit_camera.has_candidate = true;
      fit_camera.curve_points_2d = curve_points_2d;
      std::vector<Eigen::Vector3d> support_points_camera = points_3d_camera;
      support_points_camera.insert(support_points_camera.end(), curve_points_3d.begin(), curve_points_3d.end());
      TrackFit supported_fit_camera;
      if (estimateSupportInterval(fit_camera, support_points_camera, temporal_reference, &supported_fit_camera)) {
        supported_fit_camera.candidate = candidate;
        supported_fit_camera.has_candidate = true;
        supported_fit_camera.curve_points_2d = curve_points_2d;
        fit_camera = supported_fit_camera;
      }

      TrackFit optimized_camera_track;
      if (optimizeCurve(candidate, fit_camera, points_3d_camera, temporal_reference, &optimized_camera_track)) {
        TrackFit optimized_supported_fit;
        if (estimateSupportInterval(optimized_camera_track, support_points_camera, temporal_reference,
                                    &optimized_supported_fit)) {
          optimized_supported_fit.candidate = candidate;
          optimized_supported_fit.has_candidate = true;
          optimized_supported_fit.curve_points_2d = curve_points_2d;
          optimized_camera_track = optimized_supported_fit;
        }
        optimized_camera_track.candidate = candidate;
        optimized_camera_track.has_candidate = true;
        fit_camera = optimized_camera_track;
      }

      TrackFit fit_world;
      if (!transformFitToFrame(fit_camera, source_frame, target_frame, color_msg->header.stamp, &fit_world)) {
        continue;
      }
      fit_world.candidate = candidate;
      fit_world.has_candidate = true;
      if (enforce_temporal_gate && !passesTemporalGate(fit_world, color_msg->header.stamp)) {
        continue;
      }

      const double rank = fit_camera.has_optimizer_cost
                              ? fit_camera.optimizer_cost
                              : -scoreCandidate(candidate, fit_world, color_msg->header.stamp, use_tracking_prior);
      if (!has_selected || rank < best_rank) {
        has_selected = true;
        best_rank = rank;
        *selected_track = fit_world;
        *selected_camera_track = fit_camera;
      }
    }
    return has_selected;
  };

  bool has_best = false;
  bool used_global_reacquire = false;
  bool had_any_candidates = false;
  bool local_tracking_failed = false;
  TrackFit best_track;
  TrackFit best_camera_track;

  if (use_local_tracking) {
    const std::vector<ParallelCandidate> local_candidates =
        detectSegments(gray, depth_m, tracked_roi, color_mask, true);
    if (!debug.empty()) {
      drawCandidateSegments(&debug, local_candidates);
    }
    had_any_candidates = had_any_candidates || !local_candidates.empty();
    has_best = select_best_track(local_candidates, true, true, &best_track, &best_camera_track);
    local_tracking_failed = !has_best;
    if (has_best) {
      consecutive_local_misses_ = 0;
    } else {
      ++consecutive_local_misses_;
    }
  }

  const bool allow_global_reacquire =
      !use_local_tracking || consecutive_local_misses_ >= config_.global_reacquire_missed_frames;
  if (use_local_tracking && local_tracking_failed && !allow_global_reacquire) {
    std::ostringstream status;
    status << "Local cable track missing. Waiting before global reacquire ("
           << consecutive_local_misses_ << "/" << config_.global_reacquire_missed_frames << ").";
    handleMissingTrack(&debug, color_msg->header, status.str());
    return;
  }

  if (!has_best && allow_global_reacquire) {
    const std::vector<ParallelCandidate> global_candidates =
        detectSegments(gray, depth_m, global_roi, color_mask, false);
    if (!debug.empty() && !global_candidates.empty()) {
      drawCandidateSegments(&debug, global_candidates);
    }
    had_any_candidates = had_any_candidates || !global_candidates.empty();
    has_best = select_best_track(global_candidates, false, false, &best_track, &best_camera_track);
    used_global_reacquire = has_best && use_local_tracking;
  }

  if (!has_best) {
    const std::string status_text =
        had_any_candidates ? "All cable candidates were rejected by 3D fitting or temporal gating."
                           : std::string("No stable cable segment detected. silver_mask=") +
                                 std::to_string(mask_coverage * 100.0) + "%";
    handleMissingTrack(&debug, color_msg->header, status_text);
    return;
  }

  consecutive_local_misses_ = 0;

  std::vector<Eigen::Vector3d> point_cloud_observation;
  if (config_.use_point_cloud_observation) {
    const cv::Mat observation_mask =
        buildCandidateObservationMask(color.size(), best_track.candidate, best_camera_track.curve_points_2d, color_mask);
    if (extractCablePointCloudFromLatestCloud(observation_mask, target_frame, color_msg->header.stamp,
                                              &point_cloud_observation)) {
      TrackFit refined_support_track;
      if (estimateSupportInterval(best_track, point_cloud_observation, nullptr, &refined_support_track)) {
        refined_support_track.candidate = best_track.candidate;
        refined_support_track.has_candidate = best_track.has_candidate;
        refined_support_track.curve_points_2d = best_track.curve_points_2d;
        refined_support_track.curve_valid_ratio = best_track.curve_valid_ratio;
        best_track = refined_support_track;
      }
    }
  }

  updateTrackedLengthState(best_camera_track);
  TrackFit smoothed_track;
  if (config_.centerline_state_enabled) {
    smoothed_track = applyCenterlineStateEstimator(best_track, point_cloud_observation, target_frame,
                                                   color_msg->header.stamp);
  } else {
    best_track = applyTrackedLength(best_track);
    const TrackFit length_stabilized_track = applyTrackedLength(smoothTrack(best_track, color_msg->header.stamp));
    updateTrackedEndpointState(length_stabilized_track, target_frame, color_msg->header.stamp);
    smoothed_track = applyTrackedEndpoints(length_stabilized_track);
  }
  const Eigen::Vector3d target = smoothed_track.has_curve
                                     ? pointAtRatioOnPolyline(smoothed_track.curve_points, config_.target_ratio)
                                     : smoothed_track.p0 + config_.target_ratio * (smoothed_track.p1 - smoothed_track.p0);

  has_last_segment_ = true;
  last_segment_p0_ = best_track.candidate.p0;
  last_segment_p1_ = best_track.candidate.p1;
  updateTrack(smoothed_track, target, target_frame, color_msg->header.stamp, &best_camera_track, source_frame);

  if (!debug.empty()) {
    drawDebugSegment(&debug, best_track.candidate.p0, best_track.candidate.p1, best_track.curve_points_2d);
  }
  publishMarker(smoothed_track.p0, smoothed_track.p1, target, target_frame, color_msg->header.stamp,
                smoothed_track.has_curve ? smoothed_track.curve_points : std::vector<Eigen::Vector3d>());
  publishTarget(target, target_frame, color_msg->header.stamp);
  if (!debug.empty()) {
    publishDebugImage(debug, color_msg->header);
  }

  std::ostringstream status;
  status.setf(std::ios::fixed);
  status.precision(3);
  status << "Cable tracked: len=" << smoothed_track.length << "m, target=(" << target.x() << ", " << target.y()
         << ", " << target.z() << ") in " << target_frame << " | inliers=" << best_track.inlier_count << "/"
         << best_track.point_count;
  if (used_global_reacquire) {
    status << " | reacquired=global";
  }
  if (best_track.has_candidate && best_track.candidate.has_mean_depth) {
    status.precision(3);
    status << " | depth=" << best_track.candidate.mean_depth << "m";
  }
  status.precision(4);
  status << " residual=" << best_track.residual;
  if (smoothed_track.has_curve) {
    status << " | curve_pts=" << smoothed_track.curve_points.size() << " curve_res=" << smoothed_track.curve_residual;
  }
  if (!color_mask.empty()) {
    status.precision(1);
    status << " | silver_mask=" << (mask_coverage * 100.0) << "% support="
           << (best_track.candidate.mask_support_ratio * 100.0) << "%";
  }
  if (best_camera_track.has_optimizer_cost) {
    status.precision(3);
    status << " | opt_cost=" << best_camera_track.optimizer_cost << " nfev=" << best_camera_track.optimizer_nfev;
  }
  if (smoothed_track.has_support_interval) {
    status.precision(3);
    status << " | visible=" << smoothed_track.support_visible_length << " conf=("
           << smoothed_track.endpoint_confidence_low << ", " << smoothed_track.endpoint_confidence_high << ")";
  }
  if (has_length_track_) {
    status.precision(3);
    status << " | len_track=" << tracked_length_estimate_ << " lc=" << tracked_length_confidence_;
  }
  if (config_.centerline_state_enabled && has_centerline_state_) {
    const double mean_node_confidence =
        tracked_centerline_confidence_.empty()
            ? 0.0
            : std::accumulate(tracked_centerline_confidence_.begin(), tracked_centerline_confidence_.end(), 0.0) /
                  static_cast<double>(tracked_centerline_confidence_.size());
    status.precision(3);
    status << " | node_state=" << tracked_centerline_nodes_.size() << " nc=" << mean_node_confidence;
  } else if (has_endpoint_track_) {
    status.precision(3);
    status << " | endpoint_track_conf=(" << tracked_endpoint_confidence_low_ << ", "
           << tracked_endpoint_confidence_high_ << ")";
  }
  publishStatus(status.str());
}

}  // namespace gp11_perception
