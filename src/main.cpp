#include <absl/time/clock.h>

#include <Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <iostream>

#include "fileio/gnssfileloader.h"
#include "fileio/imufileloader.h"
#include "wheelgins/loadanddump.h"
#include "wheelgins/wheelgins.h"

void initStaticAlignment(IMU &imu_cur, GNSS &gnss, ImuFileLoader &imufile,
                         GnssFileLoader &gnssfile, WheelGINS &wheelgins,
                         Parameters &paras);

int main(int argc, char *argv[]) {
  YAML::Node config, config_default;

  try {
    config = YAML::LoadFile(argv[1]);
    config_default =
        YAML::LoadFile("/home/yibin/code/Wheel-GINS/config/default.yaml");
  } catch (YAML::Exception &exception) {
    std::cout << "Failed to read configuration file: " << exception.what()
              << std::endl;
    return -1;
  }

  Parameters paras;

  loadConfig(config, config_default, paras);

  std::string imupath, outputpath, gnsspath, groundtruthpath;
  getFiles(config, imupath, outputpath, gnsspath);

  createOutputDir(outputpath);

  GnssFileLoader gnssfile(gnsspath);
  ImuFileLoader imufile(imupath);
  WheelGINS wheelgins(paras);
  std::string suffix;
  if (paras.if_set_gnss_outage && paras.if_fuse_gnss) {
    suffix = "_outage_" + std::to_string(paras.gnss_outage_interval);
  }
  FileSaver navfile(outputpath + "/traj" + suffix + ".txt");
  FileSaver imuerrfile(outputpath + "/imuerror.txt");
  FileSaver stdfile(outputpath + "/std.txt");
  FileSaver augstatefile(outputpath + "/augmentedstate.txt");

  IMU imu_cur;
  GNSS gnss;

  initStaticAlignment(imu_cur, gnss, imufile, gnssfile, wheelgins, paras);

  double timestamp;
  NavState navstate;
  AugmentedState augstate;
  Eigen::MatrixXd cov;

  int percent = 0, lastpercent = 0;
  double interval = paras.endtime - paras.starttime;

  auto ts = absl::Now();

  while (imu_cur.timestamp < paras.endtime && !imufile.isEof()) {
    if (paras.if_fuse_gnss && gnss.timestamp < imu_cur.timestamp &&
        !gnssfile.isEof()) {
      gnss = gnssfile.next();
      wheelgins.addGNSSData(gnss);
    }

    imu_cur = imufile.next();

    wheelgins.addImuData(imu_cur);

    wheelgins.newImuProcess();

    timestamp = wheelgins.timestamp();
    navstate = wheelgins.getNavState();
    augstate = wheelgins.getAugmentedState();
    cov = wheelgins.getCovariance();

    writeNavResult(timestamp, navstate, augstate, navfile, imuerrfile,
                   augstatefile);
    writeSTD(timestamp, cov, stdfile);

    percent = int((imu_cur.timestamp - paras.starttime) / interval * 100);
    if (percent - lastpercent >= 1) {
      std::cout << "Processing: " << std::setw(3) << percent << "%\r"
                << std::flush;
      lastpercent = percent;
    }
  }

  imufile.close();
  gnssfile.close();
  navfile.close();
  imuerrfile.close();
  augstatefile.close();
  stdfile.close();

  auto te = absl::Now();

  std::cout << std::endl << std::endl << "Wheel-INS process finished! ";
  std::cout << "From " << paras.starttime << " s to " << paras.endtime
            << " s, total " << interval << " s! Used "
            << absl::ToDoubleSeconds(te - ts) << " s in total" << std::endl;

  return 0;
}

void initStaticAlignment(IMU &imu_cur, GNSS &gnss, ImuFileLoader &imufile,
                         GnssFileLoader &gnssfile, WheelGINS &wheelgins,
                         Parameters &paras) {
  do {
    imu_cur = imufile.next();
  } while (imu_cur.timestamp < paras.starttime);

  // Initial static alignment
  int k = 0;
  Vector3d init_gyro_mean = Vector3d::Zero();
  Vector3d init_acc_mean = Vector3d::Zero();
  while (imu_cur.timestamp < paras.starttime + paras.initAlignmentTime) {
    imu_cur = imufile.next();
    init_gyro_mean += imu_cur.angular_velocity;
    init_acc_mean += imu_cur.acceleration;
    k++;
  }

  init_gyro_mean /= k;
  init_acc_mean /= k;
  double init_roll = atan2(-init_acc_mean[1], -init_acc_mean[2]);
  double init_pitch =
      atan2(init_acc_mean[0], sqrt(init_acc_mean[1] * init_acc_mean[1] +
                                   init_acc_mean[2] * init_acc_mean[2]));
  wheelgins.setInitGyroBias(init_gyro_mean);
  wheelgins.setInitAttitude(init_roll, init_pitch);

  wheelgins.addImuData(imu_cur);

  if (paras.if_fuse_gnss) {
    do {
      gnss = gnssfile.next();
    } while (gnss.timestamp <= paras.starttime);
    wheelgins.addGNSSData(gnss);
  }
}
