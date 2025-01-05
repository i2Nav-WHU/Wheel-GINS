#pragma once

#include <math.h>

#include <Eigen/Geometry>
#include <vector>

const double D2R = (M_PI / 180.0);
const double R2D = (180.0 / M_PI);

const double zupt_angular_velocity_threshold = 0.1;
const double zupt_special_force_threshold = 0.4;
const double zupt_velocity_threshold = 0.06;

const int IMU_RATE = 200;

static constexpr double NormG = 9.782940329221166;

const int wheelimu_leverarm_dim = 2;
const int wheel_radius_scale_dim = 1;
const int wheelimu_mounting_angle_dim = 2;

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

struct IMU {
  double timestamp;
  double dt;

  Vector3d angular_velocity;
  Vector3d acceleration;
};

struct GNSS {
  double timestamp;

  Vector3d blh;
  Vector3d std;
};

struct GroundTruth {
  double timestamp;
  Vector3d blh;
  Vector3d vel;
  Vector3d euler;
};
struct Attitude {
  Eigen::Quaterniond qbn;
  Eigen::Matrix3d cbn;
  Eigen::Vector3d euler;
};

struct PVA {
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  Attitude att;
};

struct ImuError {
  Eigen::Vector3d gyrbias;
  Eigen::Vector3d accbias;
  Eigen::Vector3d gyrscale;
  Eigen::Vector3d accscale;
};

struct NavState {
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  Eigen::Vector3d euler;

  ImuError imuerror;
};

struct AugmentedState {
  Eigen::Vector3d wheelimu_leverarm = Eigen::Vector3d::Zero();
  double wheel_radius_scale = 0.0;
  Eigen::Vector3d wheelimu_mounting_angle = Eigen::Vector3d::Zero();
};

struct ImuNoise {
  Eigen::Vector3d gyr_arw;
  Eigen::Vector3d acc_vrw;
  Eigen::Vector3d gyrbias_std;
  Eigen::Vector3d accbias_std;
  Eigen::Vector3d gyrscale_std;
  Eigen::Vector3d accscale_std;
  double corr_time;
};

struct Parameters {
  // initial state and state standard deviation
  NavState initstate;
  NavState initstate_std;

  // imu noise parameters
  ImuNoise imunoise;

  // install parameters

  Eigen::Vector3d odo_measurement_std = Eigen::Vector3d::Zero();

  double odo_update_interval = 0.0;
  double wheelradius = 0.0;

  double starttime = 0.0;
  double endtime = 0.0;
  double initAlignmentTime = 0.0;

  double gnss_update_interval = 1.0;

  Eigen::Vector3d gnssLeverarm = Eigen::Vector3d::Zero();

  // options
  bool if_estimate_wheelimu_leverarm = false;
  bool if_estimate_wheel_radius_scale = false;
  bool if_estimate_wheelimu_mounting_angle = false;

  bool if_fuse_gnss = false;
  bool if_set_gnss_outage = false;
  bool if_use_zupt = false;
  bool if_use_angular_velocity_update = false;

  double wheelimu_leverarm_std = 0.0;
  double wheel_radius_scale_std = 0.0;
  double mounting_angle_std = 0.0;

  double wheelimu_leverarm_noise_std = 0.0;
  double wheel_radius_scale_noise_std = 0.0;
  double mounting_angle_noise_std = 0.0;

  double zupt_noise_std = 0.0;
  double zihr_noise_std = 0.0;
  double angular_velocity_update_noise_std = 0.0;

  int gnss_outage_interval = 0;

  std::vector<std::pair<double, double>> gnss_outage;
};
