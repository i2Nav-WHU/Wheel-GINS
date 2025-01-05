#pragma once

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <iomanip>
#include <iostream>

#include "common/rotation.h"
#include "common/types.h"
#include "fileio/filesaver.h"

#ifdef _WIN32
#include <windows.h>
#define MKDIR(path) CreateDirectoryA(path.c_str(), NULL)
#else
#include <sys/stat.h>
#include <unistd.h>
#define MKDIR(path) mkdir(path.c_str(), S_IRWXU)
#endif

inline bool loadConfig(YAML::Node &config, YAML::Node &config_default,
                       Parameters &paras) {
  std::vector<double> initposstd_vec, initvelstd_vec, initattstd_vec;

  try {
    initposstd_vec = config_default["initposstd"].as<std::vector<double>>();
    initvelstd_vec = config_default["initvelstd"].as<std::vector<double>>();
    initattstd_vec = config_default["initattstd"].as<std::vector<double>>();
  } catch (YAML::Exception &exception) {
    std::cout << "Failed when loading configuration. Please check initial std "
                 "of position, velocity, and attitude!"
              << std::endl;
    return false;
  }
  for (int i = 0; i < 3; i++) {
    paras.initstate_std.pos[i] = initposstd_vec[i];
    paras.initstate_std.vel[i] = initvelstd_vec[i];
    paras.initstate_std.euler[i] = initattstd_vec[i] * D2R;
  }

  double arw, vrw, gbstd, abstd, gsstd, asstd;

  try {
    arw = config_default["imunoise"]["arw"].as<double>();
    vrw = config_default["imunoise"]["vrw"].as<double>();
    gbstd = config_default["imunoise"]["gbstd"].as<double>();
    abstd = config_default["imunoise"]["abstd"].as<double>();
    gsstd = config_default["imunoise"]["gsstd"].as<double>();
    asstd = config_default["imunoise"]["asstd"].as<double>();
    paras.imunoise.corr_time =
        config_default["imunoise"]["corrtime"].as<double>();
  } catch (YAML::Exception &exception) {
    std::cout << "Failed when loading configuration. Please check IMU noise!"
              << std::endl;
    return false;
  }
  for (int i = 0; i < 3; i++) {
    paras.imunoise.gyr_arw[i] = arw * (D2R / 60.0);
    paras.imunoise.acc_vrw[i] = vrw / 60.0;
    paras.imunoise.gyrbias_std[i] = gbstd * (D2R / 3600.0);
    paras.imunoise.accbias_std[i] = abstd * 1e-5;
    paras.imunoise.gyrscale_std[i] = gsstd * 1e-6;
    paras.imunoise.accscale_std[i] = asstd * 1e-6;

    paras.initstate_std.imuerror.gyrbias[i] = gbstd * (D2R / 3600.0);
    paras.initstate_std.imuerror.accbias[i] = abstd * 1e-5;
    paras.initstate_std.imuerror.gyrscale[i] = gsstd * 1e-6;
    paras.initstate_std.imuerror.accscale[i] = asstd * 1e-6;
  }

  paras.imunoise.corr_time *= 3600;

  std::vector<double> odo_measurement_std;

  double odo_update_interval, wheelradius;

  odo_measurement_std = config["ODO_std"].as<std::vector<double>>();

  paras.odo_measurement_std = Eigen::Vector3d(odo_measurement_std.data());

  paras.odo_update_interval = config_default["ODO_dt"].as<double>();
  paras.wheelradius = config["Wheel_Radius"].as<double>();

  paras.starttime = config["starttime"].as<double>();
  paras.endtime = config["endtime"].as<double>();
  paras.initAlignmentTime = config_default["initAlignmentTime"].as<double>();

  // Wheel-GINS parameters

  paras.if_fuse_gnss = config_default["GNSSfusion"]["if_fuse_gnss"].as<bool>();
  paras.if_set_gnss_outage =
      config_default["GNSSfusion"]["if_set_gnss_outage"].as<bool>();

  paras.gnss_update_interval =
      1 / config_default["GNSSfusion"]["frequency"].as<int>();
  paras.initstate.vel.setZero();
  paras.initstate.euler.setZero();
  paras.initstate.euler(2) =
      config["GNSSfusion"]["initheading"].as<double>() * D2R;

  std::vector<double> gnssleverarm, initBLH, initstatestd, GNSSoutage_epoch;

  gnssleverarm = config["GNSSfusion"]["GNSSLA"].as<std::vector<double>>();
  initBLH = config["GNSSfusion"]["initBLH"].as<std::vector<double>>();

  paras.gnssLeverarm = Eigen::Vector3d(gnssleverarm.data());
  paras.initstate.pos = Eigen::Vector3d(initBLH.data());
  paras.initstate.pos(0) *= D2R;
  paras.initstate.pos(1) *= D2R;

  GNSSoutage_epoch =
      config["GNSSfusion"]["GNSSoutage_epoch"].as<std::vector<double>>();

  paras.gnss_outage_interval =
      config["GNSSfusion"]["GNSSoutage_interval"].as<double>();  // seconds

  for (auto &it : GNSSoutage_epoch) {
    paras.gnss_outage.push_back(
        std::make_pair(it, it + paras.gnss_outage_interval));
  }

  // Options
  paras.if_estimate_wheelimu_leverarm =
      config_default["Options"]["if_estimate_wheelimu_leverarm"].as<bool>();
  paras.if_estimate_wheel_radius_scale =
      config_default["Options"]["if_estimate_wheel_radius_scale"].as<bool>();

  paras.if_estimate_wheelimu_mounting_angle =
      config_default["Options"]["if_estimate_wheelimu_mounting_angle"]
          .as<bool>();

  paras.if_use_zupt = config_default["Options"]["if_use_zupt"].as<bool>();

  paras.if_use_angular_velocity_update =
      config_default["Options"]["if_use_angular_velocity_update"].as<bool>();

  if (paras.if_use_zupt) {
    paras.zupt_noise_std =
        config_default["Options"]["zupt_noise_std"].as<double>();
    paras.zihr_noise_std =
        config_default["Options"]["zihr_noise_std"].as<double>() * D2R;
  }

  if (paras.if_estimate_wheelimu_leverarm) {
    paras.wheelimu_leverarm_std =
        config_default["Options"]["wheelimu_leverarm_std"].as<double>();
    paras.wheelimu_leverarm_noise_std =
        config_default["Options"]["wheelimu_leverarm_noise"].as<double>();
  }

  if (paras.if_estimate_wheel_radius_scale) {
    paras.wheel_radius_scale_std =
        config_default["Options"]["wheel_radius_scale_std"].as<double>();
    paras.wheel_radius_scale_noise_std =
        config_default["Options"]["wheel_radius_scale_noise"].as<double>();
  }

  if (paras.if_estimate_wheelimu_mounting_angle) {
    paras.mounting_angle_std =
        config_default["Options"]["wheelimu_mounting_angle_std"].as<double>() *
        D2R;
    paras.mounting_angle_noise_std =
        config_default["Options"]["wheelimu_mounting_angle_noise"]
            .as<double>() *
        D2R;
  }

  if (paras.if_use_angular_velocity_update) {
    paras.angular_velocity_update_noise_std =
        config_default["Options"]["angular_velocity_update_noise_std"]
            .as<double>() *
        D2R;
  }
  return true;
}

inline bool getFiles(YAML::Node &config, std::string &imupath,
                     std::string &outputpath, std::string &gnsspath) {
  try {
    imupath = config["imupath"].as<std::string>();
    outputpath = config["outputpath"].as<std::string>();
    gnsspath = config["gnsspath"].as<std::string>();

    return true;
  } catch (YAML::Exception &exception) {
    std::cout << "Failed when loading configuration: " << exception.what()
              << std::endl;
    return false;
  }
}

inline void createOutputDir(std::string &outputpath) {
#ifdef _WIN32
  if (GetFileAttributesA(outputpath.c_str()) == INVALID_FILE_ATTRIBUTES)
#else
  if (access(outputpath.c_str(), F_OK))
#endif
  {
    MKDIR(std::string("output"));
    MKDIR(outputpath);
  }
}

inline void writeNavResult(double time, NavState &navstate,
                           AugmentedState &augmentedState, FileSaver &navfile,
                           FileSaver &imuerrfile, FileSaver &augstatefile) {
  std::vector<double> result;
  double heading = navstate.euler[2] - M_PI / 2;
  heading = Rotation::heading(heading);  // heading of the vehicle

  result.clear();
  result.push_back(time);
  result.push_back(navstate.pos[0] * R2D);
  result.push_back(navstate.pos[1] * R2D);
  result.push_back(navstate.pos[2]);
  result.push_back(navstate.vel[0]);
  result.push_back(navstate.vel[1]);
  result.push_back(navstate.vel[2]);
  result.push_back(navstate.euler[0] * R2D);
  result.push_back(navstate.euler[1] * R2D);

  result.push_back(heading * R2D);
  navfile.dump(result);

  auto imuerr = navstate.imuerror;
  result.clear();
  result.push_back(time);
  result.push_back(imuerr.gyrbias[0] * R2D * 3600);
  result.push_back(imuerr.gyrbias[1] * R2D * 3600);
  result.push_back(imuerr.gyrbias[2] * R2D * 3600);
  result.push_back(imuerr.accbias[0] * 1e5);
  result.push_back(imuerr.accbias[1] * 1e5);
  result.push_back(imuerr.accbias[2] * 1e5);
  result.push_back(imuerr.gyrscale[0] * 1e6);
  result.push_back(imuerr.gyrscale[1] * 1e6);
  result.push_back(imuerr.gyrscale[2] * 1e6);
  result.push_back(imuerr.accscale[0] * 1e6);
  result.push_back(imuerr.accscale[1] * 1e6);
  result.push_back(imuerr.accscale[2] * 1e6);
  imuerrfile.dump(result);

  result.clear();
  result.push_back(time);
  result.push_back(augmentedState.wheelimu_leverarm[1]);
  result.push_back(augmentedState.wheelimu_leverarm[2]);
  result.push_back(augmentedState.wheel_radius_scale);
  result.push_back(augmentedState.wheelimu_mounting_angle[1] * R2D);
  result.push_back(augmentedState.wheelimu_mounting_angle[2] * R2D);
  augstatefile.dump(result);
}

inline void writeSTD(double time, Eigen::MatrixXd &cov, FileSaver &stdfile) {
  std::vector<double> result;

  result.clear();
  result.push_back(time);

  for (int i = 0; i < 6; i++) {
    result.push_back(sqrt(cov(i, i)));
  }
  for (int i = 6; i < 9; i++) {
    result.push_back(sqrt(cov(i, i)) * R2D);
  }

  for (int i = 9; i < 12; i++) {
    result.push_back(sqrt(cov(i, i)) * R2D * 3600);
  }
  for (int i = 12; i < 15; i++) {
    result.push_back(sqrt(cov(i, i)) * 1e5);
  }
  for (int i = 15; i < 21; i++) {
    result.push_back(sqrt(cov(i, i)) * 1e6);
  }
  if (cov.rows() > 21) {
    for (int i = 21; i < cov.rows(); i++) {
      result.push_back(sqrt(cov(i, i)));
    }
  }

  stdfile.dump(result);
}