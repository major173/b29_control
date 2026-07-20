#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Gp11NativeConfig {
  int control_point_count;
  int curve_sample_count;
  int max_iterations;
  double point_sigma_m;
  double curve_point_sigma_m;
  double curve_image_sigma_px;
  double curve_smooth_sigma_m;
  double curve_length_sigma_m;
  double temporal_curve_sigma_m;
  double control_point_bound_m;
  double min_depth_m;
  double point_assign_distance_m;
  double support_cover_sigma_m;
} Gp11NativeConfig;

typedef struct Gp11NativeResult {
  int success;
  int iterations;
  double optimizer_cost;
  double center[3];
  double axis[3];
  double p0[3];
  double p1[3];
  double length;
  double radius;
  double curve_residual;
} Gp11NativeResult;

int gp11_optimize_curve(
    const Gp11NativeConfig* config,
    const double* observed_curve_3d,
    int observed_curve_count,
    const double* observed_curve_2d,
    int observed_curve_2d_count,
    const double* point_cloud_3d,
    int point_cloud_count,
    const double* previous_control_points,
    int previous_control_count,
    double fx,
    double fy,
    double cx,
    double cy,
    double reference_length,
    const double* support_frame_center,
    const double* support_frame_axis,
    double support_interval_low,
    double support_interval_high,
    double support_conf_low,
    double support_conf_high,
    double estimated_radius,
    double* output_control_points,
    double* output_curve_points,
    Gp11NativeResult* result);

#ifdef __cplusplus
}
#endif
