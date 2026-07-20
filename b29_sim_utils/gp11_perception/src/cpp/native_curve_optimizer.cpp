#include "gp11_perception/native_curve_optimizer.h"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace {

using MatrixX3 = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;
using MatrixX2 = Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor>;
using Vector3 = Eigen::Vector3d;
using Vector2 = Eigen::Vector2d;

double ClampValue(double value, double low, double high) {
  return std::max(low, std::min(value, high));
}

double SafeNorm(const Vector3& vector) {
  return std::max(vector.norm(), 1e-9);
}

Vector3 Normalize(const Vector3& vector) {
  return vector / SafeNorm(vector);
}

double PolylineLength3(const MatrixX3& points) {
  if (points.rows() < 2) {
    return 0.0;
  }
  double length = 0.0;
  for (int index = 0; index < points.rows() - 1; ++index) {
    length += (points.row(index + 1) - points.row(index)).norm();
  }
  return length;
}

double PolylineLength2(const MatrixX2& points) {
  if (points.rows() < 2) {
    return 0.0;
  }
  double length = 0.0;
  for (int index = 0; index < points.rows() - 1; ++index) {
    length += (points.row(index + 1) - points.row(index)).norm();
  }
  return length;
}

MatrixX3 RepeatPoint3(const Vector3& point, int count) {
  MatrixX3 result(count, 3);
  for (int index = 0; index < count; ++index) {
    result.row(index) = point.transpose();
  }
  return result;
}

MatrixX2 RepeatPoint2(const Vector2& point, int count) {
  MatrixX2 result(count, 2);
  for (int index = 0; index < count; ++index) {
    result.row(index) = point.transpose();
  }
  return result;
}

MatrixX3 ResamplePolyline3(const MatrixX3& points, int sample_count) {
  sample_count = std::max(2, sample_count);
  if (points.rows() == 0) {
    return MatrixX3(0, 3);
  }
  if (points.rows() == 1) {
    return RepeatPoint3(points.row(0), sample_count);
  }

  std::vector<double> cumulative(points.rows(), 0.0);
  for (int index = 1; index < points.rows(); ++index) {
    cumulative[index] = cumulative[index - 1] + (points.row(index) - points.row(index - 1)).norm();
  }
  const double total_length = cumulative.back();
  if (total_length < 1e-9) {
    return RepeatPoint3(points.row(0), sample_count);
  }

  MatrixX3 result(sample_count, 3);
  for (int sample_index = 0; sample_index < sample_count; ++sample_index) {
    const double target = (static_cast<double>(sample_index) / static_cast<double>(sample_count - 1)) * total_length;
    auto upper = std::lower_bound(cumulative.begin(), cumulative.end(), target);
    int segment_index = static_cast<int>(std::distance(cumulative.begin(), upper));
    segment_index = std::max(1, std::min(segment_index, static_cast<int>(points.rows()) - 1));
    const double segment_start = cumulative[segment_index - 1];
    const double segment_end = cumulative[segment_index];
    const double denom = std::max(segment_end - segment_start, 1e-9);
    const double ratio = (target - segment_start) / denom;
    result.row(sample_index) =
        (1.0 - ratio) * points.row(segment_index - 1) + ratio * points.row(segment_index);
  }
  return result;
}

MatrixX2 ResamplePolyline2(const MatrixX2& points, int sample_count) {
  sample_count = std::max(2, sample_count);
  if (points.rows() == 0) {
    return MatrixX2(0, 2);
  }
  if (points.rows() == 1) {
    return RepeatPoint2(points.row(0), sample_count);
  }

  std::vector<double> cumulative(points.rows(), 0.0);
  for (int index = 1; index < points.rows(); ++index) {
    cumulative[index] = cumulative[index - 1] + (points.row(index) - points.row(index - 1)).norm();
  }
  const double total_length = cumulative.back();
  if (total_length < 1e-9) {
    return RepeatPoint2(points.row(0), sample_count);
  }

  MatrixX2 result(sample_count, 2);
  for (int sample_index = 0; sample_index < sample_count; ++sample_index) {
    const double target = (static_cast<double>(sample_index) / static_cast<double>(sample_count - 1)) * total_length;
    auto upper = std::lower_bound(cumulative.begin(), cumulative.end(), target);
    int segment_index = static_cast<int>(std::distance(cumulative.begin(), upper));
    segment_index = std::max(1, std::min(segment_index, static_cast<int>(points.rows()) - 1));
    const double segment_start = cumulative[segment_index - 1];
    const double segment_end = cumulative[segment_index];
    const double denom = std::max(segment_end - segment_start, 1e-9);
    const double ratio = (target - segment_start) / denom;
    result.row(sample_index) =
        (1.0 - ratio) * points.row(segment_index - 1) + ratio * points.row(segment_index);
  }
  return result;
}

MatrixX3 AlignDirection3(const MatrixX3& points, const MatrixX3& reference) {
  if (points.rows() < 2 || reference.rows() < 2) {
    return points;
  }
  const double same_cost =
      (points.row(0) - reference.row(0)).norm() + (points.row(points.rows() - 1) - reference.row(reference.rows() - 1)).norm();
  const double reverse_cost =
      (points.row(0) - reference.row(reference.rows() - 1)).norm() +
      (points.row(points.rows() - 1) - reference.row(0)).norm();
  if (reverse_cost >= same_cost) {
    return points;
  }
  MatrixX3 reversed(points.rows(), 3);
  for (int index = 0; index < points.rows(); ++index) {
    reversed.row(index) = points.row(points.rows() - 1 - index);
  }
  return reversed;
}

MatrixX2 AlignDirection2(const MatrixX2& points, const MatrixX2& reference) {
  if (points.rows() < 2 || reference.rows() < 2) {
    return points;
  }
  const double same_cost =
      (points.row(0) - reference.row(0)).norm() + (points.row(points.rows() - 1) - reference.row(reference.rows() - 1)).norm();
  const double reverse_cost =
      (points.row(0) - reference.row(reference.rows() - 1)).norm() +
      (points.row(points.rows() - 1) - reference.row(0)).norm();
  if (reverse_cost >= same_cost) {
    return points;
  }
  MatrixX2 reversed(points.rows(), 2);
  for (int index = 0; index < points.rows(); ++index) {
    reversed.row(index) = points.row(points.rows() - 1 - index);
  }
  return reversed;
}

void ProjectCurve(const MatrixX3& curve, double fx, double fy, double cx, double cy, MatrixX2* projected, std::vector<char>* valid) {
  projected->resize(curve.rows(), 2);
  valid->assign(curve.rows(), 0);
  for (int index = 0; index < curve.rows(); ++index) {
    const double z = curve(index, 2);
    if (z <= 1e-6) {
      (*projected)(index, 0) = 0.0;
      (*projected)(index, 1) = 0.0;
      continue;
    }
    (*projected)(index, 0) = fx * curve(index, 0) / z + cx;
    (*projected)(index, 1) = fy * curve(index, 1) / z + cy;
    (*valid)[index] = 1;
  }
}

MatrixX3 BackProjectImageCurve(
    const MatrixX3& curve,
    const MatrixX2& observed_curve,
    double fx,
    double fy,
    double cx,
    double cy,
    double min_depth_m) {
  MatrixX3 result(curve.rows(), 3);
  for (int index = 0; index < curve.rows(); ++index) {
    const double z = std::max(curve(index, 2), min_depth_m);
    const double x = (observed_curve(index, 0) - cx) * z / std::max(fx, 1e-9);
    const double y = (observed_curve(index, 1) - cy) * z / std::max(fy, 1e-9);
    result.row(index) << x, y, z;
  }
  return result;
}

Vector3 SamplePointOnPolyline(const MatrixX3& points, double ratio) {
  if (points.rows() == 0) {
    return Vector3::Zero();
  }
  if (points.rows() == 1) {
    return points.row(0);
  }
  const double clamped_ratio = ClampValue(ratio, 0.0, 1.0);
  std::vector<double> cumulative(points.rows(), 0.0);
  for (int index = 1; index < points.rows(); ++index) {
    cumulative[index] = cumulative[index - 1] + (points.row(index) - points.row(index - 1)).norm();
  }
  const double total_length = cumulative.back();
  if (total_length < 1e-9) {
    return points.row(0);
  }
  const double target = clamped_ratio * total_length;
  auto upper = std::lower_bound(cumulative.begin(), cumulative.end(), target);
  int segment_index = static_cast<int>(std::distance(cumulative.begin(), upper));
  segment_index = std::max(1, std::min(segment_index, static_cast<int>(points.rows()) - 1));
  const double segment_start = cumulative[segment_index - 1];
  const double segment_end = cumulative[segment_index];
  const double denom = std::max(segment_end - segment_start, 1e-9);
  const double local_ratio = (target - segment_start) / denom;
  return (1.0 - local_ratio) * points.row(segment_index - 1) + local_ratio * points.row(segment_index);
}

void ApplySmoothness(MatrixX3* control_points, double smoothness_sigma) {
  if (control_points->rows() < 3) {
    return;
  }
  const double alpha = ClampValue(0.18 * (0.03 / std::max(smoothness_sigma, 1e-6)), 0.05, 0.45);
  MatrixX3 updated = *control_points;
  for (int index = 1; index < control_points->rows() - 1; ++index) {
    const Vector3 midpoint = 0.5 * (control_points->row(index - 1) + control_points->row(index + 1));
    updated.row(index) = (1.0 - alpha) * control_points->row(index) + alpha * midpoint.transpose();
  }
  *control_points = updated;
}

void ClampControlPoints(MatrixX3* control_points, const MatrixX3& initial_control_points, double bound_m, double min_depth_m) {
  for (int row = 0; row < control_points->rows(); ++row) {
    for (int col = 0; col < 3; ++col) {
      (*control_points)(row, col) = ClampValue(
          (*control_points)(row, col),
          initial_control_points(row, col) - bound_m,
          initial_control_points(row, col) + bound_m);
    }
    (*control_points)(row, 2) = std::max((*control_points)(row, 2), min_depth_m);
  }
}

void PullCurveFromPointCloud(
    const MatrixX3& current_curve,
    const MatrixX3& point_cloud,
    double assign_distance_m,
    MatrixX3* average_points,
    std::vector<int>* counts) {
  average_points->setZero(current_curve.rows(), 3);
  counts->assign(current_curve.rows(), 0);
  if (point_cloud.rows() == 0) {
    return;
  }
  const double max_distance_sq = assign_distance_m * assign_distance_m;
  for (int point_index = 0; point_index < point_cloud.rows(); ++point_index) {
    double best_distance_sq = std::numeric_limits<double>::infinity();
    int best_index = -1;
    for (int sample_index = 0; sample_index < current_curve.rows(); ++sample_index) {
      const double distance_sq = (point_cloud.row(point_index) - current_curve.row(sample_index)).squaredNorm();
      if (distance_sq < best_distance_sq) {
        best_distance_sq = distance_sq;
        best_index = sample_index;
      }
    }
    if (best_index < 0 || best_distance_sq > max_distance_sq) {
      continue;
    }
    average_points->row(best_index) += point_cloud.row(point_index);
    (*counts)[best_index] += 1;
  }
  for (int sample_index = 0; sample_index < average_points->rows(); ++sample_index) {
    if ((*counts)[sample_index] > 0) {
      average_points->row(sample_index) /= static_cast<double>((*counts)[sample_index]);
    }
  }
}

double EstimateWorldSigmaFromPixels(double image_sigma_px, double z, double fx, double fy, double min_depth_m) {
  const double depth = std::max(z, min_depth_m);
  const double f_avg = 0.5 * (std::max(fx, 1e-9) + std::max(fy, 1e-9));
  return std::max(depth * image_sigma_px / std::max(f_avg, 1e-9), 1e-5);
}

double MeanCurveDistance(const MatrixX3& lhs, const MatrixX3& rhs) {
  if (lhs.rows() == 0 || rhs.rows() == 0 || lhs.rows() != rhs.rows()) {
    return 0.0;
  }
  double total = 0.0;
  for (int index = 0; index < lhs.rows(); ++index) {
    total += (lhs.row(index) - rhs.row(index)).norm();
  }
  return total / static_cast<double>(lhs.rows());
}

void ProjectSupportInterval(const MatrixX3& curve_points,
                            const Vector3& support_center,
                            const Vector3& support_axis,
                            double* low,
                            double* high,
                            int* low_endpoint_index,
                            int* high_endpoint_index) {
  *low = 0.0;
  *high = 0.0;
  if (curve_points.rows() <= 0) {
    if (low_endpoint_index != nullptr) {
      *low_endpoint_index = 0;
    }
    if (high_endpoint_index != nullptr) {
      *high_endpoint_index = 0;
    }
    return;
  }

  const Vector3 axis = Normalize(support_axis);
  double min_projection = std::numeric_limits<double>::infinity();
  double max_projection = -std::numeric_limits<double>::infinity();
  for (int index = 0; index < curve_points.rows(); ++index) {
    const double projection = axis.dot(curve_points.row(index).transpose() - support_center);
    min_projection = std::min(min_projection, projection);
    max_projection = std::max(max_projection, projection);
  }
  *low = min_projection;
  *high = max_projection;

  if (low_endpoint_index != nullptr || high_endpoint_index != nullptr) {
    const double first_projection = axis.dot(curve_points.row(0).transpose() - support_center);
    const double last_projection = axis.dot(curve_points.row(curve_points.rows() - 1).transpose() - support_center);
    if (first_projection <= last_projection) {
      if (low_endpoint_index != nullptr) {
        *low_endpoint_index = 0;
      }
      if (high_endpoint_index != nullptr) {
        *high_endpoint_index = curve_points.rows() - 1;
      }
    } else {
      if (low_endpoint_index != nullptr) {
        *low_endpoint_index = curve_points.rows() - 1;
      }
      if (high_endpoint_index != nullptr) {
        *high_endpoint_index = 0;
      }
    }
  }
}

void ApplySupportCoveragePull(MatrixX3* control_points,
                              const Vector3& support_center,
                              const Vector3& support_axis,
                              double support_low,
                              double support_high,
                              double support_conf_low,
                              double support_conf_high) {
  if (control_points == nullptr || control_points->rows() < 2) {
    return;
  }

  double model_low = 0.0;
  double model_high = 0.0;
  int low_endpoint_index = 0;
  int high_endpoint_index = control_points->rows() - 1;
  ProjectSupportInterval(*control_points, support_center, support_axis, &model_low, &model_high, &low_endpoint_index,
                         &high_endpoint_index);

  const Vector3 axis = Normalize(support_axis);
  const double low_deficit = std::max(0.0, model_low - support_low);
  const double high_deficit = std::max(0.0, support_high - model_high);

  auto apply_side_pull = [&](int endpoint_index, double delta) {
    if (std::abs(delta) < 1e-9) {
      return;
    }
    (*control_points).row(endpoint_index) += delta * axis.transpose();
    const int neighbor_index = endpoint_index == 0 ? 1 : endpoint_index - 1;
    if (neighbor_index >= 0 && neighbor_index < control_points->rows()) {
      (*control_points).row(neighbor_index) += 0.45 * delta * axis.transpose();
    }
  };

  apply_side_pull(low_endpoint_index, -0.85 * ClampValue(support_conf_low, 0.0, 1.0) * low_deficit);
  apply_side_pull(high_endpoint_index, 0.85 * ClampValue(support_conf_high, 0.0, 1.0) * high_deficit);
}

double ComputeOptimizerCost(
    const Gp11NativeConfig& config,
    const MatrixX3& curve_points,
    const MatrixX3& observed_curve_3d,
    const MatrixX2& observed_curve_2d,
    const MatrixX3& point_cloud,
    const MatrixX3& control_points,
    const MatrixX3& previous_control_points,
    double fx,
    double fy,
    double cx,
    double cy,
    double reference_length,
    const Vector3* support_center,
    const Vector3* support_axis,
    double support_interval_low,
    double support_interval_high,
    double support_conf_low,
    double support_conf_high) {
  double cost = 0.0;
  if (observed_curve_3d.rows() == curve_points.rows()) {
    cost += (curve_points - observed_curve_3d).array().square().sum() /
            std::max(config.curve_point_sigma_m * config.curve_point_sigma_m, 1e-9);
  }
  if (observed_curve_2d.rows() == curve_points.rows()) {
    MatrixX2 projected;
    std::vector<char> valid;
    ProjectCurve(curve_points, fx, fy, cx, cy, &projected, &valid);
    for (int index = 0; index < projected.rows(); ++index) {
      if (!valid[index]) {
        cost += 64.0;
        continue;
      }
      cost += (projected.row(index) - observed_curve_2d.row(index)).squaredNorm() /
              std::max(config.curve_image_sigma_px * config.curve_image_sigma_px, 1e-9);
    }
  }
  if (point_cloud.rows() > 0) {
    MatrixX3 averages;
    std::vector<int> counts;
    PullCurveFromPointCloud(curve_points, point_cloud, config.point_assign_distance_m, &averages, &counts);
    const double point_sigma_sq = std::max(config.point_sigma_m * config.point_sigma_m, 1e-9);
    for (int index = 0; index < averages.rows(); ++index) {
      if (counts[index] <= 0) {
        continue;
      }
      cost += (curve_points.row(index) - averages.row(index)).squaredNorm() / point_sigma_sq;
    }
  }
  if (control_points.rows() >= 3) {
    for (int index = 1; index < control_points.rows() - 1; ++index) {
      const Vector3 second = control_points.row(index - 1) - 2.0 * control_points.row(index) + control_points.row(index + 1);
      cost += second.squaredNorm() /
              std::max(config.curve_smooth_sigma_m * config.curve_smooth_sigma_m, 1e-9);
    }
  }
  if (previous_control_points.rows() == control_points.rows()) {
    cost += (control_points - previous_control_points).array().square().sum() /
            std::max(config.temporal_curve_sigma_m * config.temporal_curve_sigma_m, 1e-9);
  }
  if (reference_length > 1e-9) {
    const double length_error = PolylineLength3(curve_points) - reference_length;
    cost += (length_error * length_error) /
            std::max(config.curve_length_sigma_m * config.curve_length_sigma_m, 1e-9);
  }
  if (support_center != nullptr && support_axis != nullptr && support_interval_high > support_interval_low) {
    double model_low = 0.0;
    double model_high = 0.0;
    ProjectSupportInterval(curve_points, *support_center, *support_axis, &model_low, &model_high, nullptr, nullptr);
    const double sigma_sq = std::max(config.support_cover_sigma_m * config.support_cover_sigma_m, 1e-9);
    const double low_error = std::max(0.0, model_low - support_interval_low);
    const double high_error = std::max(0.0, support_interval_high - model_high);
    cost += (ClampValue(support_conf_low, 0.0, 1.0) * low_error * low_error +
             ClampValue(support_conf_high, 0.0, 1.0) * high_error * high_error) /
            sigma_sq;
  }
  return cost;
}

void StoreFitResult(const MatrixX3& curve_points, double radius, Gp11NativeResult* result) {
  result->length = PolylineLength3(curve_points);
  const Vector3 p0 = curve_points.row(0);
  const Vector3 p1 = curve_points.row(curve_points.rows() - 1);
  const Vector3 chord = p1 - p0;
  Vector3 axis;
  if (chord.norm() < 1e-9) {
    const Vector3 center = curve_points.colwise().mean();
    Eigen::MatrixXd centered = curve_points.rowwise() - center.transpose();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(centered, Eigen::ComputeThinV);
    axis = Normalize(svd.matrixV().col(0));
  } else {
    axis = Normalize(chord);
  }
  const Vector3 center = SamplePointOnPolyline(curve_points, 0.5);

  double curve_residual = 0.0;
  for (int index = 0; index < curve_points.rows(); ++index) {
    curve_residual += (axis.cross(curve_points.row(index).transpose() - p0)).norm();
  }
  curve_residual /= static_cast<double>(std::max<Eigen::Index>(curve_points.rows(), 1));

  for (int axis_index = 0; axis_index < 3; ++axis_index) {
    result->center[axis_index] = center(axis_index);
    result->axis[axis_index] = axis(axis_index);
    result->p0[axis_index] = p0(axis_index);
    result->p1[axis_index] = p1(axis_index);
  }
  result->radius = radius;
  result->curve_residual = curve_residual;
}

}  // namespace

extern "C" int gp11_optimize_curve(
    const Gp11NativeConfig* config_ptr,
    const double* observed_curve_3d_ptr,
    int observed_curve_count,
    const double* observed_curve_2d_ptr,
    int observed_curve_2d_count,
    const double* point_cloud_3d_ptr,
    int point_cloud_count,
    const double* previous_control_points_ptr,
    int previous_control_count,
    double fx,
    double fy,
    double cx,
    double cy,
    double reference_length,
    const double* support_frame_center_ptr,
    const double* support_frame_axis_ptr,
    double support_interval_low,
    double support_interval_high,
    double support_conf_low,
    double support_conf_high,
    double estimated_radius,
    double* output_control_points_ptr,
    double* output_curve_points_ptr,
    Gp11NativeResult* result) {
  if (config_ptr == nullptr || observed_curve_3d_ptr == nullptr || output_control_points_ptr == nullptr ||
      output_curve_points_ptr == nullptr || result == nullptr || observed_curve_count < 2) {
    return 0;
  }

  const Gp11NativeConfig config = *config_ptr;
  const int control_point_count = std::max(3, config.control_point_count);
  const int curve_sample_count = std::max(control_point_count + 2, config.curve_sample_count);
  const int max_iterations = std::max(1, config.max_iterations);

  const Eigen::Map<const MatrixX3> observed_curve_map(observed_curve_3d_ptr, observed_curve_count, 3);
  const MatrixX3 observed_curve_3d = ResamplePolyline3(observed_curve_map, curve_sample_count);

  MatrixX2 observed_curve_2d;
  if (observed_curve_2d_ptr != nullptr && observed_curve_2d_count >= 2) {
    const Eigen::Map<const MatrixX2> observed_curve_2d_map(observed_curve_2d_ptr, observed_curve_2d_count, 2);
    observed_curve_2d = ResamplePolyline2(observed_curve_2d_map, curve_sample_count);
  }

  MatrixX3 point_cloud;
  if (point_cloud_3d_ptr != nullptr && point_cloud_count > 0) {
    const Eigen::Map<const MatrixX3> point_cloud_map(point_cloud_3d_ptr, point_cloud_count, 3);
    point_cloud = point_cloud_map;
  }

  MatrixX3 previous_control_points;
  if (previous_control_points_ptr != nullptr && previous_control_count >= 2) {
    const Eigen::Map<const MatrixX3> previous_control_map(previous_control_points_ptr, previous_control_count, 3);
    previous_control_points = AlignDirection3(ResamplePolyline3(previous_control_map, control_point_count), observed_curve_3d);
  }

  Vector3 support_center = Vector3::Zero();
  Vector3 support_axis = Vector3::UnitX();
  const bool has_support_interval =
      support_frame_center_ptr != nullptr && support_frame_axis_ptr != nullptr && support_interval_high > support_interval_low;
  if (has_support_interval) {
    support_center = Vector3(support_frame_center_ptr[0], support_frame_center_ptr[1], support_frame_center_ptr[2]);
    support_axis = Normalize(Vector3(support_frame_axis_ptr[0], support_frame_axis_ptr[1], support_frame_axis_ptr[2]));
  }

  MatrixX3 current_control_points = ResamplePolyline3(observed_curve_3d, control_point_count);
  const MatrixX3 initial_control_points = current_control_points;
  if (previous_control_points.rows() == current_control_points.rows()) {
    current_control_points = 0.80 * current_control_points + 0.20 * previous_control_points;
  }

  const double point_weight = 1.0 / std::max(config.point_sigma_m * config.point_sigma_m, 1e-9);
  const double curve_weight = 1.0 / std::max(config.curve_point_sigma_m * config.curve_point_sigma_m, 1e-9);
  const double temporal_weight = 1.0 / std::max(config.temporal_curve_sigma_m * config.temporal_curve_sigma_m, 1e-9);

  double previous_cost = std::numeric_limits<double>::infinity();
  int performed_iterations = 0;
  for (int iteration = 0; iteration < max_iterations; ++iteration) {
    MatrixX3 current_curve = ResamplePolyline3(current_control_points, curve_sample_count);
    current_curve = AlignDirection3(current_curve, observed_curve_3d);

    MatrixX3 accum = MatrixX3::Zero(curve_sample_count, 3);
    Eigen::VectorXd weights = Eigen::VectorXd::Constant(curve_sample_count, 1e-9);

    accum += curve_weight * observed_curve_3d;
    weights.array() += curve_weight;

    if (observed_curve_2d.rows() == curve_sample_count) {
      MatrixX2 projected;
      std::vector<char> valid;
      ProjectCurve(current_curve, fx, fy, cx, cy, &projected, &valid);
      const MatrixX2 aligned_image_curve = AlignDirection2(observed_curve_2d, projected);
      const MatrixX3 pseudo_curve = BackProjectImageCurve(
          current_curve,
          aligned_image_curve,
          fx,
          fy,
          cx,
          cy,
          config.min_depth_m);
      for (int sample_index = 0; sample_index < curve_sample_count; ++sample_index) {
        const double image_sigma_m = EstimateWorldSigmaFromPixels(
            config.curve_image_sigma_px,
            current_curve(sample_index, 2),
            fx,
            fy,
            config.min_depth_m);
        const double image_weight = 1.0 / std::max(image_sigma_m * image_sigma_m, 1e-9);
        accum.row(sample_index) += image_weight * pseudo_curve.row(sample_index);
        weights(sample_index) += image_weight;
      }
    }

    if (previous_control_points.rows() == control_point_count) {
      const MatrixX3 previous_curve = AlignDirection3(
          ResamplePolyline3(previous_control_points, curve_sample_count),
          current_curve);
      accum += temporal_weight * previous_curve;
      weights.array() += temporal_weight;
    }

    MatrixX3 point_cloud_targets;
    std::vector<int> point_counts;
    PullCurveFromPointCloud(
        current_curve,
        point_cloud,
        std::max(config.point_assign_distance_m, 2.5 * config.point_sigma_m),
        &point_cloud_targets,
        &point_counts);
    for (int sample_index = 0; sample_index < curve_sample_count; ++sample_index) {
      if (point_counts[sample_index] <= 0) {
        continue;
      }
      const double scaled_weight =
          point_weight * ClampValue(static_cast<double>(point_counts[sample_index]) / 4.0, 0.25, 1.0);
      accum.row(sample_index) += scaled_weight * point_cloud_targets.row(sample_index);
      weights(sample_index) += scaled_weight;
    }

    MatrixX3 updated_curve(curve_sample_count, 3);
    for (int sample_index = 0; sample_index < curve_sample_count; ++sample_index) {
      updated_curve.row(sample_index) = accum.row(sample_index) / std::max(weights(sample_index), 1e-9);
    }

    current_control_points = ResamplePolyline3(updated_curve, control_point_count);
    ApplySmoothness(&current_control_points, config.curve_smooth_sigma_m);

    if (reference_length > 1e-9) {
      MatrixX3 length_curve = ResamplePolyline3(current_control_points, curve_sample_count);
      const double current_length = PolylineLength3(length_curve);
      if (current_length > 1e-9) {
        const double ratio = reference_length / current_length;
        const double scale = 1.0 + 0.35 * (ClampValue(ratio, 0.8, 1.2) - 1.0);
        const Vector3 midpoint = SamplePointOnPolyline(length_curve, 0.5);
        for (int point_index = 0; point_index < current_control_points.rows(); ++point_index) {
          const Vector3 centered = current_control_points.row(point_index).transpose() - midpoint;
          current_control_points.row(point_index) = (midpoint + scale * centered).transpose();
        }
      }
    }

    ClampControlPoints(
        &current_control_points,
        initial_control_points,
        config.control_point_bound_m,
        config.min_depth_m);

    const MatrixX3 cost_curve = ResamplePolyline3(current_control_points, curve_sample_count);
    const double current_cost = ComputeOptimizerCost(
        config,
        cost_curve,
        observed_curve_3d,
        observed_curve_2d,
        point_cloud,
        current_control_points,
        previous_control_points,
        fx,
        fy,
        cx,
        cy,
        reference_length,
        has_support_interval ? &support_center : nullptr,
        has_support_interval ? &support_axis : nullptr,
        support_interval_low,
        support_interval_high,
        support_conf_low,
        support_conf_high);

    performed_iterations = iteration + 1;
    if (std::abs(previous_cost - current_cost) < 1e-5) {
      previous_cost = current_cost;
      break;
    }
    previous_cost = current_cost;
  }

  const MatrixX3 output_control_points = current_control_points;
  const MatrixX3 output_curve_points = ResamplePolyline3(output_control_points, curve_sample_count);

  for (int index = 0; index < output_control_points.rows(); ++index) {
    output_control_points_ptr[3 * index + 0] = output_control_points(index, 0);
    output_control_points_ptr[3 * index + 1] = output_control_points(index, 1);
    output_control_points_ptr[3 * index + 2] = output_control_points(index, 2);
  }
  for (int index = 0; index < output_curve_points.rows(); ++index) {
    output_curve_points_ptr[3 * index + 0] = output_curve_points(index, 0);
    output_curve_points_ptr[3 * index + 1] = output_curve_points(index, 1);
    output_curve_points_ptr[3 * index + 2] = output_curve_points(index, 2);
  }

  result->success = 1;
  result->iterations = performed_iterations;
  result->optimizer_cost = previous_cost;
  StoreFitResult(output_curve_points, estimated_radius, result);
  return 1;
}
