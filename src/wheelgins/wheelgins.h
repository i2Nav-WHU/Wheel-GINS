#pragma once

#include <Eigen/Dense>
#include <iomanip>
#include <iostream>
#include <queue>
#include <vector>

#include "common/rotation.h"
#include "common/types.h"
#include "insmech.h"

class WheelGINS {
 public:
  explicit WheelGINS(Parameters &paras);

  ~WheelGINS() = default;

  inline void addImuData(IMU &imu) {
    // Manually add mounting angle error
    Vector3d mounting_angle = Vector3d::Zero();

    mounting_angle[1] = 0 * D2R;
    mounting_angle[2] = 0 * D2R;

    Matrix3d mounting_angle_mat = Rotation::euler2matrix(mounting_angle);
    imu.angular_velocity = mounting_angle_mat * imu.angular_velocity;
    imu.acceleration = mounting_angle_mat * imu.acceleration;

    imupre_ = imucur_;
    imucur_ = imu;

    INSMech::imuCompensate(imucur_, imuerror_);

    if (imuBuff_.size() < imuBuffsize) {
      imuBuff_.push_back(imucur_);
    } else {
      imuBuff_.pop_front();
      imuBuff_.push_back(imucur_);
    }
  }

  inline void addGNSSData(const GNSS &gnss) { gnssdata_ = gnss; }

  inline void setInitGyroBias(const Vector3d &gyro_bias) {
    imuerror_.gyrbias = gyro_bias;
  }
  inline void setInitAttitude(const double &roll, const double &pitch) {
    pvacur_.att.euler(0) = roll;
    pvacur_.att.euler(1) = pitch;
    pvacur_.att.cbn = Rotation::euler2matrix(pvacur_.att.euler);
    pvacur_.att.qbn = Rotation::euler2quaternion(pvacur_.att.euler);

    pvapre_ = pvacur_;
  }

  void newImuProcess();

  inline double timestamp() const { return imucur_.timestamp; }

  NavState getNavState();

  inline AugmentedState getAugmentedState() { return augstate_; }

  inline Eigen::MatrixXd getCovariance() { return Cov_; }

 private:
  void initialization(const NavState &initstate, const NavState &initstate_std);

  void velocityUpdate();

  void GNSSUpdate();

  inline void ZUPT();
  inline void odo_nhcUpdate();
  inline void angularVelUpdate();

  inline Matrix3d computeVehicleRotation();

  inline void getWheelVelocity();

  bool inGNSSOutage(double &timestamp);

  inline void detectZUPT();

  void insPropagation(IMU &imupre, IMU &imucur);

  void stateFeedback();

  inline void checkCov() {
    for (int i = 0; i < RANK; i++) {
      if (Cov_(i, i) < 0) {
        std::cout << "Covariance is negative at " << std::setprecision(10)
                  << imucur_.timestamp << " !" << std::endl;
        std::exit(EXIT_FAILURE);
      }
    }
  }

  void EKFPredict(Eigen::MatrixXd &Phi, Eigen::MatrixXd &Qd);

  void EKFUpdate(Eigen::MatrixXd &dz, Eigen::MatrixXd &H, Eigen::MatrixXd &R);

 private:
  int RANK = 21;
  int NOISERANK = 18;

  Parameters paras_;

  double last_velocity_update_t;
  double last_gnss_update_t;
  // double last_ZUPT_update_t;
  // double last_ZIHR_update_t;

  double wheelVelocity_;

  double zihr_preheading_ = 0.0;

  GNSS gnssdata_;

  IMU imupre_;
  IMU imucur_;

  AugmentedState augstate_;

  Matrix3d C_bv_ = Matrix3d::Identity();
  Matrix3d C_wheelv = Matrix3d::Identity();

  // Eigen::Vector3d wheelimu_mounting_angle_ = Eigen::Vector3d::Zero();

  PVA pvacur_;
  PVA pvapre_;
  ImuError imuerror_;

  Eigen::MatrixXd Cov_;
  Eigen::MatrixXd Qc_;
  Eigen::MatrixXd delta_x_;

  std::deque<IMU> imuBuff_;

  bool measument_updated_ = false;

  const int imuBuffsize = 0.2 * IMU_RATE;

  std::vector<double> x_, y_, z_;

  bool if_ZUPT_available_ = false;
  bool if_ZIHR_available_ = false;

  int ZIHR_num_ = 0;

  enum StateID {
    P_ID = 0,
    V_ID = 3,
    PHI_ID = 6,
    BG_ID = 9,
    BA_ID = 12,
    SG_ID = 15,
    SA_ID = 18,
    Wheelimu_leverarm_ID = 21,
    Wheel_radius_scale_ID = 23,
    Mounting_angle_ID = 24
  };
  enum NoiseID {
    VRW_ID = 0,
    ARW_ID = 3,
    BGSTD_ID = 6,
    BASTD_ID = 9,
    SGSTD_ID = 12,
    SASTD_ID = 15,
    Wheelimu_leverarm_std_ID = 18,
    Wheel_radius_scale_std_ID = 20,
    Mounting_angle_std_ID = 21
  };
};
