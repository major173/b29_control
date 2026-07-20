#pragma once

#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <array>
#include <deque>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace gp11_perception {

class CableTrackerNode {
 public:
  CableTrackerNode();

 private:
 using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,
                                                                     sensor_msgs::CameraInfo>;

  struct TrackerConfig {
    std::string color_topic = "/camera/color/image_raw";
    std::string depth_topic = "/camera/aligned_depth_to_color/image_raw";
    std::string camera_info_topic = "/camera/color/camera_info";
    std::string marker_topic = "/gp11_perception/cable_marker";
    std::string target_point_topic = "/gp11_perception/grasp_point";
    std::string moveit_target_point_topic = "/gp11_moveit/reach_arm/target_point";
    std::string debug_image_topic = "/gp11_perception/debug_tracking";
    std::string status_topic = "/gp11_perception/status";
    std::string output_frame;
    bool publish_target_to_moveit = false;
    bool publish_debug_image = false;
    double debug_publish_max_rate_hz = 10.0;
    double debug_image_scale = 0.5;
    int debug_candidate_limit = 3;
    bool debug_draw_sample_points = false;
    bool debug_require_subscriber = true;
    bool debug_antialiasing = false;
    bool silver_prefilter_enabled = false;
    double silver_hsv_s_max = 90.0;
    double silver_hsv_v_min = 45.0;
    double silver_hsv_v_max = 255.0;
    double silver_lab_l_min = 90.0;
    double silver_lab_l_max = 235.0;
    double silver_lab_chroma_max = 30.0;
    int silver_mask_close_px = 2;
    int silver_mask_open_px = 1;
    double silver_line_support_ratio = 0.40;
    double silver_hard_reject_support_ratio = 0.12;
    bool silver_prefilter_fallback_enabled = true;
    double silver_score_weight = 60.0;
    double silver_support_penalty_weight = 90.0;
    double min_depth_m = 0.10;
    double max_depth_m = 2.50;
    bool cable_depth_filter_enabled = false;
    double cable_depth_min_m = 0.30;
    double cable_depth_max_m = 1.50;
    double min_line_length_px = 80.0;
    int line_width_px = 3;
    int depth_patch_radius_px = 2;
    double min_valid_depth_ratio = 0.55;
    int tracking_search_margin_px = 60;
    double smoothing_alpha = 0.20;
    double target_ratio = 0.50;
    bool has_search_roi = false;
    std::array<int, 4> search_roi{{0, 0, 0, 0}};
    int candidate_top_k = 4;
    int raw_line_top_k = 12;
    int segment_sample_count = 32;
    int line_band_half_width_px = 6;
    int line_band_step_px = 3;
    double depth_discontinuity_thresh_m = 0.04;
    double line_inlier_thresh_m = 0.015;
    int min_inlier_count = 18;
    int ransac_iterations = 24;
    double temporal_position_gate_m = 0.06;
    double temporal_angle_gate_deg = 12.0;
    double lost_track_hold_time_s = 0.40;
    int global_reacquire_missed_frames = 5;
    double parallel_edge_angle_tol_deg = 8.0;
    double parallel_edge_min_separation_px = 4.0;
    double parallel_edge_max_separation_px = 30.0;
    double parallel_edge_min_overlap_ratio = 0.6;
    double parallel_edge_depth_tol_m = 0.08;
    double parallel_edge_max_width_std_px = 4.0;
    bool curve_tracking_enabled = true;
    int curve_cross_section_samples = 7;
    int curve_min_valid_points = 6;
    int curve_spatial_smooth_window = 7;
    int curve_resample_point_count = 17;
    double curve_length_change_ratio = 0.10;
    double support_percentile_low = 0.05;
    double support_percentile_high = 0.95;
    double support_axis_inlier_radius_m = 0.02;
    int support_min_points = 20;
    double support_endpoint_window_m = 0.03;
    double support_outside_window_m = 0.025;
    double support_temporal_sigma_m = 0.04;
    double support_cover_sigma_m = 0.015;
    double length_expand_alpha = 0.35;
    double length_shrink_alpha = 0.05;
    double length_min_endpoint_confidence = 0.60;
    double endpoint_confident_alpha = 0.28;
    double endpoint_uncertain_alpha = 0.03;
    double endpoint_max_adjust_m = 0.10;
    bool centerline_state_enabled = true;
    int centerline_state_node_count = 17;
    double centerline_measurement_alpha = 0.45;
    double centerline_velocity_decay = 0.55;
    double centerline_velocity_gain = 0.35;
    double centerline_spatial_smooth_alpha = 0.20;
    double centerline_max_node_step_m = 0.035;
    double centerline_hidden_node_alpha = 0.08;
    double centerline_length_scale_limit = 0.08;
    std::string point_cloud_topic = "/camera/depth/color/points";
    bool use_point_cloud_observation = true;
    double point_cloud_max_age_s = 0.12;
    double point_cloud_voxel_size_m = 0.01;
    double point_cloud_assign_distance_m = 0.03;
    int point_cloud_min_points = 24;
    int point_cloud_mask_thickness_px = 10;
    bool anchor_constraint_enabled = false;
    std::string anchor_endpoint = "none";
    std::string anchor_frame;
    std::array<double, 3> anchor_point{{0.0, 0.0, 0.0}};
    double anchor_alpha = 0.85;
    double anchor_neighbor_alpha = 0.35;
    double anchor_max_pull_m = 0.06;
    bool joint_optimization_enabled = true;
    bool native_backend_enabled = true;
    int native_optimizer_max_iterations = 4;
    double native_point_assign_distance_m = 0.05;
    int optimizer_candidate_top_k = 2;
    int optimizer_control_point_count = 5;
    int optimizer_curve_sample_count = 17;
    double optimizer_curve_image_sigma_px = 4.0;
    double optimizer_curve_smooth_sigma_m = 0.02;
    double optimizer_curve_length_sigma_m = 0.025;
    double optimizer_temporal_curve_sigma_m = 0.020;
    double optimizer_control_point_bound_m = 0.06;
    double optimizer_curve_point_sigma_m = 0.008;
    double optimizer_point_sigma_m = 0.012;
    double optimizer_temporal_position_sigma_m = 0.04;
  };

  struct EdgeCandidate {
    cv::Point2d p0;
    cv::Point2d p1;
    cv::Point2d center;
    Eigen::Vector2d dir = Eigen::Vector2d::UnitX();
    double length_px = 0.0;
    double valid_ratio = 0.0;
    double mean_depth = 0.0;
    bool has_mean_depth = false;
    double score_2d = 0.0;
  };

  struct ParallelCandidate {
    cv::Point2d p0;
    cv::Point2d p1;
    cv::Point2d center;
    Eigen::Vector2d dir = Eigen::Vector2d::UnitX();
    double length_px = 0.0;
    double valid_ratio = 0.0;
    double mean_depth = 0.0;
    bool has_mean_depth = false;
    double score_2d = 0.0;
    double pair_separation_px = 0.0;
    double pair_overlap_px = 0.0;
    double pair_width_std_px = 0.0;
    double mask_support_ratio = 1.0;
    cv::Point2d edge_a_p0;
    cv::Point2d edge_a_p1;
    cv::Point2d edge_b_p0;
    cv::Point2d edge_b_p1;
    std::vector<cv::Point2d> edge_a_samples;
    std::vector<cv::Point2d> edge_b_samples;
    std::vector<cv::Point2d> center_samples;
    std::vector<double> width_samples_px;
    std::vector<double> sample_ratios;
    Eigen::Vector2d edge_normal_px = Eigen::Vector2d::UnitY();
    bool used_color_mask_fallback = false;
  };

  struct TrackFit {
    bool valid = false;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Vector3d axis = Eigen::Vector3d::UnitX();
    double length = 0.0;
    Eigen::Vector3d p0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d p1 = Eigen::Vector3d::Zero();
    double residual = 0.0;
    int inlier_count = 0;
    int point_count = 0;
    double band_valid_ratio = 0.0;
    double curve_valid_ratio = 0.0;
    double curve_length = 0.0;
    double curve_residual = 0.0;
    double radius = 0.0;
    bool has_curve = false;
    std::vector<Eigen::Vector3d> curve_points;
    std::vector<cv::Point2d> curve_points_2d;
    bool has_optimizer_cost = false;
    double optimizer_cost = 0.0;
    int optimizer_nfev = 0;
    bool has_support_interval = false;
    double support_low = 0.0;
    double support_high = 0.0;
    double support_visible_length = 0.0;
    double endpoint_confidence_low = 0.0;
    double endpoint_confidence_high = 0.0;
    double tracked_length_reference = 0.0;
    ParallelCandidate candidate;
    bool has_candidate = false;
  };

  void loadParameters();
  void initializeSubscribers();
  void initializePublishers();
  void initializeLineDetector();
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

  void rgbdCallback(const sensor_msgs::ImageConstPtr& color_msg, const sensor_msgs::ImageConstPtr& depth_msg,
                    const sensor_msgs::CameraInfoConstPtr& info_msg);

  cv::Mat convertDepthToMeters(const sensor_msgs::ImageConstPtr& depth_msg) const;
  cv::Mat buildSilverMask(const cv::Mat& lab, const cv::Mat& hsv) const;
  cv::Mat overlayColorMask(const cv::Mat& image, const cv::Mat& mask) const;
  double maskCoverageRatio(const cv::Mat& mask) const;

  bool shouldRenderDebug(const ros::Time& stamp) const;
  cv::Mat prepareDebugImage(const cv::Mat& color, const ros::Time& stamp) const;
  cv::Point scaleDebugPoint(const cv::Point2d& point) const;
  int debugLineWidth(int width) const;
  int debugCircleRadius(int radius) const;

  cv::Rect candidateRoi(const cv::Size& image_size, const ros::Time& stamp) const;
  std::vector<ParallelCandidate> detectSegments(const cv::Mat& gray, const cv::Mat& depth_m, const cv::Rect& roi,
                                                const cv::Mat& color_mask, bool use_tracking_prior) const;
  std::vector<ParallelCandidate> detectSegmentsOnce(const cv::Mat& gray, const cv::Mat& depth_m, const cv::Rect& roi,
                                                    const cv::Mat& line_mask, const cv::Mat& color_mask,
                                                    bool use_tracking_prior) const;
  std::vector<cv::Vec4f> detectLines(const cv::Mat& gray, const cv::Mat& mask) const;

  std::vector<cv::Point2d> sampleSegmentPoints(const cv::Point2d& p0, const cv::Point2d& p1, int samples) const;
  std::pair<double, double> lineDepthStats(const cv::Point2d& p0, const cv::Point2d& p1, const cv::Mat& depth_m,
                                           int samples = 24) const;
  double maskSupportRatio(const cv::Point2d& p0, const cv::Point2d& p1, const cv::Mat& mask, int samples = 24) const;
  ParallelCandidate buildParallelCandidate(const EdgeCandidate& first, const EdgeCandidate& second,
                                           const cv::Mat& color_mask) const;

  bool isAcceptedDepth(double depth) const;
  bool sampleDepth(const cv::Mat& depth_m, int u, int v, double* depth) const;
  void segmentBandPointsTo3d(const ParallelCandidate& candidate, const cv::Mat& depth_m,
                             std::vector<cv::Point2d>* points_2d, std::vector<Eigen::Vector3d>* points_3d,
                             double* band_valid_ratio) const;
  void candidateCurvePointsTo3d(const ParallelCandidate& candidate, const cv::Mat& depth_m,
                                std::vector<cv::Point2d>* points_2d, std::vector<Eigen::Vector3d>* points_3d,
                                double* valid_ratio) const;

  TrackFit fitLineRansac(const std::vector<Eigen::Vector3d>& points) const;
  TrackFit finalizeLineFit(const std::vector<Eigen::Vector3d>& inlier_points, int point_count) const;
  TrackFit curveTrackFromPoints(const std::vector<Eigen::Vector3d>& curve_points, const TrackFit* fit_template) const;
  TrackFit applyCurveTrack(const TrackFit& fit, const std::vector<Eigen::Vector3d>& curve_points,
                           double curve_valid_ratio) const;
  TrackFit applyCenterlineStateEstimator(const TrackFit& observed_fit,
                                         const std::vector<Eigen::Vector3d>& point_cloud_observation,
                                         const std::string& frame_id, const ros::Time& stamp);
  TrackFit applyTrackedLength(const TrackFit& fit) const;
  TrackFit applyTrackedEndpoints(const TrackFit& fit) const;
  bool estimateSupportInterval(const TrackFit& fit, const std::vector<Eigen::Vector3d>& support_points,
                               const TrackFit* previous_state, TrackFit* fit_with_support) const;
  std::vector<double> buildCenterlineObservationConfidence(const TrackFit& fit, int node_count) const;
  cv::Mat buildCandidateObservationMask(const cv::Size& image_size, const ParallelCandidate& candidate,
                                        const std::vector<cv::Point2d>& curve_points_2d, const cv::Mat& color_mask) const;
  bool extractCablePointCloudFromLatestCloud(const cv::Mat& observation_mask, const std::string& target_frame,
                                             const ros::Time& stamp,
                                             std::vector<Eigen::Vector3d>* cable_points) const;
  bool lookupAnchorPoint(const std::string& target_frame, const ros::Time& stamp, Eigen::Vector3d* anchor_point) const;
  double computeEndpointConfidence(const TrackFit& fit, const std::vector<double>& projections, double boundary,
                                   bool is_low_endpoint, const TrackFit* previous_state) const;
  void updateTrackedLengthState(const TrackFit& fit);
  void updateTrackedEndpointState(const TrackFit& fit, const std::string& frame_id, const ros::Time& stamp);

  bool optimizeCurve(const ParallelCandidate& candidate, const TrackFit& fit_camera,
                     const std::vector<Eigen::Vector3d>& points_camera, const TrackFit* previous_state,
                     TrackFit* optimized_fit) const;

  bool transformPoints(const std::vector<Eigen::Vector3d>& points, const std::string& source_frame,
                       const std::string& target_frame, const ros::Time& stamp,
                       std::vector<Eigen::Vector3d>* transformed) const;
  bool transformFitToFrame(const TrackFit& fit, const std::string& source_frame, const std::string& target_frame,
                           const ros::Time& stamp, TrackFit* transformed) const;

  bool passesTemporalGate(const TrackFit& fit, const ros::Time& stamp) const;
  double scoreCandidate(const ParallelCandidate& candidate, const TrackFit& fit, const ros::Time& stamp,
                        bool use_tracking_prior) const;
  TrackFit smoothTrack(const TrackFit& fit, const ros::Time& stamp) const;
  void updateTrack(const TrackFit& track, const Eigen::Vector3d& target, const std::string& frame_id,
                   const ros::Time& stamp, const TrackFit* camera_track, const std::string& camera_frame_id);
  const TrackFit* getPreviousCameraTrack(const std::string& frame_id, const ros::Time& stamp) const;
  bool hasRecentTrack(const ros::Time& stamp) const;
  void handleMissingTrack(cv::Mat* debug_image, const std_msgs::Header& header, const std::string& status_text);

  void drawCandidateSegments(cv::Mat* image, const std::vector<ParallelCandidate>& candidates) const;
  void drawDebugSegment(cv::Mat* image, const cv::Point2d& p0, const cv::Point2d& p1,
                        const std::vector<cv::Point2d>& points_2d) const;
  void publishMarker(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& target,
                     const std::string& frame_id, const ros::Time& stamp,
                     const std::vector<Eigen::Vector3d>& curve_points) const;
  void publishTarget(const Eigen::Vector3d& point_xyz, const std::string& frame_id, const ros::Time& stamp) const;
  void publishDebugImage(const cv::Mat& image, const std_msgs::Header& header);
  void publishStatus(const std::string& text) const;

  static Eigen::Vector2d toEigen(const cv::Point2d& point);
  static cv::Point2d toCv(const Eigen::Vector2d& point);
  static Eigen::Vector3d normalize(const Eigen::Vector3d& vector);
  static Eigen::Vector2d normalize(const Eigen::Vector2d& vector);
  static double polylineLength(const std::vector<Eigen::Vector3d>& points);
  static std::vector<Eigen::Vector3d> resamplePolyline(const std::vector<Eigen::Vector3d>& points, int sample_count);
  static std::vector<cv::Point2d> resamplePolyline(const std::vector<cv::Point2d>& points, int sample_count);
  static std::vector<Eigen::Vector3d> smoothPolyline(const std::vector<Eigen::Vector3d>& points, int window_size);
  static std::vector<Eigen::Vector3d> alignCurveDirection(const std::vector<Eigen::Vector3d>& points,
                                                          const std::vector<Eigen::Vector3d>& reference_points);
  static Eigen::Vector3d pointAtRatioOnPolyline(const std::vector<Eigen::Vector3d>& points, double ratio);
  static std::pair<double, double> projectInterval(const cv::Point2d& p0, const cv::Point2d& p1,
                                                   const cv::Point2d& origin, const Eigen::Vector2d& direction);
  static cv::Point2d pointOnLineAtProjection(const cv::Point2d& base_point, const Eigen::Vector2d& line_direction,
                                             const cv::Point2d& origin, const Eigen::Vector2d& projection_direction,
                                             double target_projection);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  image_geometry::PinholeCameraModel camera_model_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> color_sub_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo>> info_sub_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  ros::Subscriber point_cloud_sub_;

  ros::Publisher marker_pub_;
  ros::Publisher target_pub_;
  ros::Publisher debug_pub_;
  ros::Publisher status_pub_;
  ros::Publisher moveit_target_pub_;

  cv::Ptr<cv::LineSegmentDetector> lsd_detector_;
  bool use_lsd_;
  mutable std::mt19937 rng_;

  TrackerConfig config_;

  bool has_last_segment_;
  cv::Point2d last_segment_p0_;
  cv::Point2d last_segment_p1_;
  int consecutive_local_misses_;
  bool has_last_track_;
  bool has_last_camera_track_;
  Eigen::Vector3d last_center_;
  Eigen::Vector3d last_axis_;
  double last_length_;
  Eigen::Vector3d last_p0_;
  Eigen::Vector3d last_p1_;
  Eigen::Vector3d last_target_;
  std::string last_frame_id_;
  ros::Time last_track_stamp_;
  std::vector<Eigen::Vector3d> last_curve_points_;
  TrackFit last_camera_track_storage_;
  std::string last_camera_frame_id_;
  ros::Time last_debug_publish_stamp_;
  sensor_msgs::PointCloud2ConstPtr latest_point_cloud_msg_;
  bool has_length_track_;
  double tracked_length_estimate_;
  double tracked_length_confidence_;
  std::deque<double> recent_length_measurements_;
  bool has_endpoint_track_;
  Eigen::Vector3d tracked_endpoint_low_;
  Eigen::Vector3d tracked_endpoint_high_;
  double tracked_endpoint_confidence_low_;
  double tracked_endpoint_confidence_high_;
  std::string tracked_endpoint_frame_id_;
  bool has_centerline_state_;
  std::vector<Eigen::Vector3d> tracked_centerline_nodes_;
  std::vector<Eigen::Vector3d> tracked_centerline_velocities_;
  std::vector<double> tracked_centerline_confidence_;
  std::string tracked_centerline_frame_id_;
};

}  // namespace gp11_perception
