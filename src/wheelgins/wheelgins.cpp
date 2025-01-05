#include "wheelgins.h"

#include <math.h>

#include "common/earth.h"
#include "common/rotation.h"
#include "insmech.h"


namespace Eigen {
using Vector6d = Eigen::Matrix<double, 6, 1>;
}

auto create_covariance_block = [](const auto &v1) {
  return v1.cwiseAbs2().asDiagonal();
};

auto square = [](auto x) { return x * x; };

WheelGINS::WheelGINS(Parameters &paras) : paras_(paras) {
  if (paras_.if_estimate_wheelimu_leverarm) {
    RANK += wheelimu_leverarm_dim;
    NOISERANK += wheelimu_leverarm_dim;
    if (paras_.if_estimate_wheel_radius_scale) {
      RANK += wheel_radius_scale_dim;
      NOISERANK += wheel_radius_scale_dim;
      if (paras_.if_estimate_wheelimu_mounting_angle) {
        RANK += wheelimu_mounting_angle_dim;
        NOISERANK += wheelimu_mounting_angle_dim;
      }
    }
  }

  Cov_.resize(RANK, RANK);
  Qc_.resize(NOISERANK, NOISERANK);
  delta_x_.resize(RANK, 1);
  Cov_.setZero();
  Qc_.setZero();
  delta_x_.setZero();

  auto imunoise = paras_.imunoise;
  Qc_.block(ARW_ID, ARW_ID, 3, 3) = create_covariance_block(imunoise.gyr_arw);
  Qc_.block(VRW_ID, VRW_ID, 3, 3) = create_covariance_block(imunoise.acc_vrw);
  Qc_.block(BGSTD_ID, BGSTD_ID, 3, 3) =
      2 / imunoise.corr_time * create_covariance_block(imunoise.gyrbias_std);
  Qc_.block(BASTD_ID, BASTD_ID, 3, 3) =
      2 / imunoise.corr_time * create_covariance_block(imunoise.accbias_std);
  Qc_.block(SGSTD_ID, SGSTD_ID, 3, 3) =
      2 / imunoise.corr_time * create_covariance_block(imunoise.gyrscale_std);
  Qc_.block(SASTD_ID, SASTD_ID, 3, 3) =
      2 / imunoise.corr_time * create_covariance_block(imunoise.accscale_std);

  if (paras_.if_estimate_wheelimu_leverarm) {
    Qc_.block(Wheelimu_leverarm_std_ID, Wheelimu_leverarm_std_ID,
              wheelimu_leverarm_dim, wheelimu_leverarm_dim) =
        Eigen::Matrix2d::Identity() *
        square(paras_.wheelimu_leverarm_noise_std);
    if (paras_.if_estimate_wheel_radius_scale) {
      Qc_(Wheel_radius_scale_std_ID, Wheel_radius_scale_std_ID) =
          square(paras_.wheel_radius_scale_noise_std);
      if (paras_.if_estimate_wheelimu_mounting_angle) {
        Qc_.block(Mounting_angle_std_ID, Mounting_angle_std_ID,
                  wheelimu_mounting_angle_dim, wheelimu_mounting_angle_dim) =
            Eigen::Matrix2d::Identity() *
            square(paras_.mounting_angle_noise_std);
      }
    }
  }

  initialization(paras_.initstate, paras_.initstate_std);

  double processStarttime = paras_.starttime + paras_.initAlignmentTime;
  last_velocity_update_t = processStarttime;
  last_gnss_update_t = processStarttime;
}

void WheelGINS::initialization(const NavState &initstate,
                               const NavState &initstate_std) {
  pvacur_.pos = initstate.pos;
  pvacur_.vel = initstate.vel;

  pvacur_.att.euler[2] = initstate.euler[2];
  pvacur_.att.cbn = Rotation::euler2matrix(pvacur_.att.euler);
  pvacur_.att.qbn = Rotation::euler2quaternion(pvacur_.att.euler);

  imuerror_ = initstate.imuerror;

  pvapre_ = pvacur_;

  ImuError imuerror_std = initstate_std.imuerror;
  Cov_.block(P_ID, P_ID, 3, 3) = create_covariance_block(initstate_std.pos);
  Cov_.block(V_ID, V_ID, 3, 3) = create_covariance_block(initstate_std.vel);
  Cov_.block(PHI_ID, PHI_ID, 3, 3) =
      create_covariance_block(initstate_std.euler);
  Cov_.block(BG_ID, BG_ID, 3, 3) =
      create_covariance_block(imuerror_std.gyrbias);
  Cov_.block(BA_ID, BA_ID, 3, 3) =
      create_covariance_block(imuerror_std.accbias);
  Cov_.block(SG_ID, SG_ID, 3, 3) =
      create_covariance_block(imuerror_std.gyrscale);
  Cov_.block(SA_ID, SA_ID, 3, 3) =
      create_covariance_block(imuerror_std.accscale);

  if (paras_.if_estimate_wheelimu_leverarm) {
    Cov_.block(Wheelimu_leverarm_ID, Wheelimu_leverarm_ID,
               wheelimu_leverarm_dim, wheelimu_leverarm_dim) =
        Eigen::Matrix2d::Identity() * square(paras_.wheelimu_leverarm_std);

    if (paras_.if_estimate_wheel_radius_scale) {
      Cov_(Wheel_radius_scale_ID, Wheel_radius_scale_ID) =
          square(paras_.wheel_radius_scale_std);

      if (paras_.if_estimate_wheelimu_mounting_angle) {
        Cov_.block(Mounting_angle_ID, Mounting_angle_ID,
                   wheelimu_mounting_angle_dim, wheelimu_mounting_angle_dim) =
            Eigen::Matrix2d::Identity() * square(paras_.mounting_angle_std);
      }
    }
  }
}

void WheelGINS::newImuProcess() {
  insPropagation(imupre_, imucur_);
  velocityUpdate();
  if (paras_.if_fuse_gnss) {
    GNSSUpdate();
  }

  stateFeedback();

  checkCov();

  if (ZIHR_num_ >= 2 && last_velocity_update_t == imucur_.timestamp &&
      !if_ZIHR_available_) {
    if_ZIHR_available_ = true;
    zihr_preheading_ = pvacur_.att.euler[2];
  }

  pvapre_ = pvacur_;
  imupre_ = imucur_;
}

void WheelGINS::insPropagation(IMU &imupre, IMU &imucur) {
  INSMech::insMech(pvapre_, pvacur_, imupre, imucur);

  Eigen::MatrixXd Phi, F, Qd, G;

  Phi.resizeLike(Cov_);
  F.resizeLike(Cov_);
  Qd.resizeLike(Cov_);
  G.resize(RANK, NOISERANK);
  Phi.setIdentity();
  F.setZero();
  Qd.setZero();
  G.setZero();

  Eigen::Vector3d midvel, midpos;
  midvel = (pvacur_.vel + pvapre_.vel) / 2;
  midpos = pvapre_.pos + Earth::DRi(pvapre_.pos) * midvel * imucur.dt / 2;

  Eigen::Vector2d rmrn;
  Eigen::Vector3d wie_n, wen_n;
  rmrn = Earth::meridianPrimeVerticalRadius(midpos[0]);
  wie_n << WGS84_WIE * cos(midpos[0]), 0, -WGS84_WIE * sin(midpos[0]);
  wen_n << midvel[1] / (rmrn[1] + midpos[2]),
      -midvel[0] / (rmrn[0] + midpos[2]),
      -midvel[1] * tan(midpos[0]) / (rmrn[1] + midpos[2]);
  Eigen::Vector3d accel, omega;
  double rmh, rnh;
  rmh = rmrn[0] + pvapre_.pos[2];
  rnh = rmrn[1] + pvapre_.pos[2];
  accel = imucur.acceleration;
  omega = imucur.angular_velocity;
  double gravity = Earth::gravity(pvapre_.pos);

  Eigen::Matrix3d temp;

  temp.setZero();
  temp(0, 0) = -pvapre_.vel[2] / rmh;
  temp(0, 2) = pvapre_.vel[0] / rmh;
  temp(1, 0) = pvapre_.vel[1] * tan(pvapre_.pos[0]) / rnh;
  temp(1, 1) = -(pvapre_.vel[2] + pvapre_.vel[0] * tan(pvapre_.pos[0])) / rnh;
  temp(1, 2) = pvapre_.vel[1] / rnh;

  F.block(P_ID, P_ID, 3, 3) = temp;
  F.block(P_ID, V_ID, 3, 3) = Eigen::Matrix3d::Identity();

  // velocity error
  temp.setZero();
  temp(0, 0) = -2 * pvapre_.vel[1] * WGS84_WIE * cos(pvapre_.pos[0]) / rmh -
               pow(pvapre_.vel[1], 2) / rmh / rnh / pow(cos(pvapre_.pos[0]), 2);
  temp(0, 2) = pvapre_.vel[0] * pvapre_.vel[2] / rmh / rmh -
               pow(pvapre_.vel[1], 2) * tan(pvapre_.pos[0]) / rnh / rnh;
  temp(1, 0) =
      2 * WGS84_WIE *
          (pvapre_.vel[0] * cos(pvapre_.pos[0]) -
           pvapre_.vel[2] * sin(pvapre_.pos[0])) /
          rmh +
      pvapre_.vel[0] * pvapre_.vel[1] / rmh / rnh / pow(cos(pvapre_.pos[0]), 2);
  temp(1, 2) = (pvapre_.vel[1] * pvapre_.vel[2] +
                pvapre_.vel[0] * pvapre_.vel[1] * tan(pvapre_.pos[0])) /
               rnh / rnh;
  temp(2, 0) = 2 * WGS84_WIE * pvapre_.vel[1] * sin(pvapre_.pos[0]) / rmh;
  temp(2, 2) = -pow(pvapre_.vel[1], 2) / rnh / rnh -
               pow(pvapre_.vel[0], 2) / rmh / rmh +
               2 * gravity / (sqrt(rmrn[0] * rmrn[1]) + pvapre_.pos[2]);
  F.block(V_ID, P_ID, 3, 3) = temp;
  temp.setZero();
  temp(0, 0) = pvapre_.vel[2] / rmh;
  temp(0, 1) = -2 * (WGS84_WIE * sin(pvapre_.pos[0]) +
                     pvapre_.vel[1] * tan(pvapre_.pos[0]) / rnh);
  temp(0, 2) = pvapre_.vel[0] / rmh;
  temp(1, 0) = 2 * WGS84_WIE * sin(pvapre_.pos[0]) +
               pvapre_.vel[1] * tan(pvapre_.pos[0]) / rnh;
  temp(1, 1) = (pvapre_.vel[2] + pvapre_.vel[0] * tan(pvapre_.pos[0])) / rnh;
  temp(1, 2) = 2 * WGS84_WIE * cos(pvapre_.pos[0]) + pvapre_.vel[1] / rnh;
  temp(2, 0) = -2 * pvapre_.vel[0] / rmh;
  temp(2, 1) = -2 * (WGS84_WIE * cos(pvapre_.pos(0)) + pvapre_.vel[1] / rnh);
  F.block(V_ID, V_ID, 3, 3) = temp;
  F.block(V_ID, PHI_ID, 3, 3) =
      Rotation::skewSymmetric(pvapre_.att.cbn * accel);
  F.block(V_ID, BA_ID, 3, 3) = pvapre_.att.cbn;
  F.block(V_ID, SA_ID, 3, 3) = pvapre_.att.cbn * (accel.asDiagonal());

  // attitude error
  temp.setZero();
  temp(0, 0) = -WGS84_WIE * sin(pvapre_.pos[0]) / rmh;
  temp(0, 2) = pvapre_.vel[1] / rnh / rnh;
  temp(1, 2) = -pvapre_.vel[0] / rmh / rmh;
  temp(2, 0) = -WGS84_WIE * cos(pvapre_.pos[0]) / rmh -
               pvapre_.vel[1] / rmh / rnh / pow(cos(pvapre_.pos[0]), 2);
  temp(2, 2) = -pvapre_.vel[1] * tan(pvapre_.pos[0]) / rnh / rnh;
  F.block(PHI_ID, P_ID, 3, 3) = temp;
  temp.setZero();
  temp(0, 1) = 1 / rnh;
  temp(1, 0) = -1 / rmh;
  temp(2, 1) = -tan(pvapre_.pos[0]) / rnh;
  F.block(PHI_ID, V_ID, 3, 3) = temp;
  F.block(PHI_ID, PHI_ID, 3, 3) = -Rotation::skewSymmetric(wie_n + wen_n);
  F.block(PHI_ID, BG_ID, 3, 3) = -pvapre_.att.cbn;
  F.block(PHI_ID, SG_ID, 3, 3) = -pvapre_.att.cbn * (omega.asDiagonal());

  // imu bias error and scale error, modeled as the first-order Gauss-Markov
  // process
  F.block(BG_ID, BG_ID, 3, 3) =
      -1 / paras_.imunoise.corr_time * Eigen::Matrix3d::Identity();
  F.block(BA_ID, BA_ID, 3, 3) =
      -1 / paras_.imunoise.corr_time * Eigen::Matrix3d::Identity();
  F.block(SG_ID, SG_ID, 3, 3) =
      -1 / paras_.imunoise.corr_time * Eigen::Matrix3d::Identity();
  F.block(SA_ID, SA_ID, 3, 3) =
      -1 / paras_.imunoise.corr_time * Eigen::Matrix3d::Identity();

  G.block(V_ID, VRW_ID, 3, 3) = pvapre_.att.cbn;
  G.block(PHI_ID, ARW_ID, 3, 3) = pvapre_.att.cbn;
  G.block(BG_ID, BGSTD_ID, 3, 3) = Eigen::Matrix3d::Identity();
  G.block(BA_ID, BASTD_ID, 3, 3) = Eigen::Matrix3d::Identity();
  G.block(SG_ID, SGSTD_ID, 3, 3) = Eigen::Matrix3d::Identity();
  G.block(SA_ID, SASTD_ID, 3, 3) = Eigen::Matrix3d::Identity();

  if (paras_.if_estimate_wheelimu_leverarm) {
    G.block(Wheelimu_leverarm_ID, Wheelimu_leverarm_std_ID, 2, 2) =
        Eigen::Matrix2d::Identity();
    if (paras_.if_estimate_wheel_radius_scale) {
      G(Wheel_radius_scale_ID, Wheel_radius_scale_std_ID) = 1.0;
      if (paras_.if_estimate_wheelimu_mounting_angle) {
        G.block(Mounting_angle_ID, Mounting_angle_std_ID, 2, 2) =
            Eigen::Matrix2d::Identity();
      }
    }
  }

  Phi.setIdentity();
  Phi = Phi + F * imucur.dt;

  Qd = G * Qc_ * G.transpose();
  Qd = (Phi * Qd * Phi.transpose() + Qd) * imucur.dt / 2;

  EKFPredict(Phi, Qd);
}
void WheelGINS::odo_nhcUpdate() {
  Matrix3d C_nv = computeVehicleRotation().transpose();

  Vector3d c_bv_euler = Rotation::matrix2euler(C_bv_);
  Vector3d velocity_vframe = C_nv * pvacur_.vel;
  Matrix3d angularVelocity_skew =
      Rotation::skewSymmetric(imucur_.angular_velocity);
  Matrix3d leverarm_skew = Rotation::skewSymmetric(augstate_.wheelimu_leverarm);
  Matrix3d velocitySkew_nframe = C_nv * Rotation::skewSymmetric(pvacur_.vel);
  Vector3d velocity_leverarm =
      C_bv_ * angularVelocity_skew * augstate_.wheelimu_leverarm;

  Eigen::MatrixXd Hv;
  Hv.resize(3, RANK);
  Hv.setZero();

  Hv.block<3, 3>(0, V_ID) = C_nv;
  // Hv.block<3, 3>(0, PHI_ID) =
  //     C_nv * Rotation::skewSymmetric(pvacur_.att.cbn * angularVelocity_skew *
  //                                    augstate_.wheelimu_leverarm);
  Hv.block<3, 3>(0, BG_ID) = -C_bv_ * leverarm_skew;
  Hv.block<3, 3>(0, SG_ID) =
      -C_bv_ * leverarm_skew * imucur_.angular_velocity.asDiagonal();

  Hv.block<3, 1>(0, 8) = -velocitySkew_nframe.block<3, 1>(0, 2);

  if (paras_.if_estimate_wheelimu_leverarm) {
    Hv.block<3, 2>(0, Wheelimu_leverarm_ID) =
        (C_bv_ * angularVelocity_skew).block<3, 2>(0, 1);
    if (paras_.if_estimate_wheel_radius_scale) {
      Hv(0, Wheel_radius_scale_ID) = -wheelVelocity_;
      if (paras_.if_estimate_wheelimu_mounting_angle) {
        Hv.block<3, 2>(0, Mounting_angle_ID) =
            Rotation::skewSymmetric(velocity_leverarm).block<3, 2>(0, 1) +
            (C_wheelv *
             Rotation::skewSymmetric(
                 Rotation::euler2matrix(augstate_.wheelimu_mounting_angle) *
                 pvacur_.att.cbn.transpose() * pvacur_.vel))
                .block<3, 2>(0, 1);

        // Hv(0, Mounting_angle_ID) +=
        //     (sin(augstate_.wheelimu_mounting_angle[1]) *
        //          cos(augstate_.wheelimu_mounting_angle[2]) *
        //          imucur_.angular_velocity[0] -
        //      cos(augstate_.wheelimu_mounting_angle[1]) *
        //          cos(augstate_.wheelimu_mounting_angle[2]) *
        //          imucur_.angular_velocity[2]) *
        //     paras_.wheelradius;

        // Hv(0, Mounting_angle_ID + 1) +=
        //     (cos(augstate_.wheelimu_mounting_angle[1]) *
        //          sin(augstate_.wheelimu_mounting_angle[2]) *
        //          imucur_.angular_velocity[0] +
        //      cos(augstate_.wheelimu_mounting_angle[2]) *
        //          imucur_.angular_velocity[1] +
        //      cos(augstate_.wheelimu_mounting_angle[1]) *
        //          cos(augstate_.wheelimu_mounting_angle[2]) *
        //          imucur_.angular_velocity[2]) *
        //     paras_.wheelradius;
      }
    }
  }

  Eigen::MatrixXd Zv = velocity_vframe + velocity_leverarm;
  Zv(0) -= wheelVelocity_;

  Eigen::MatrixXd Rv =
      paras_.odo_measurement_std.cwiseProduct(paras_.odo_measurement_std)
          .asDiagonal();


  EKFUpdate(Zv, Hv, Rv);
}

void WheelGINS::angularVelUpdate() {
  Matrix3d C_b_wheel =
      Rotation::euler2matrix(augstate_.wheelimu_mounting_angle);
  Vector3d angularVelocity_wheel = C_b_wheel * imucur_.angular_velocity;

  Eigen::MatrixXd H_angVel;
  H_angVel.resize(2, RANK);
  H_angVel.setZero();

  H_angVel.block<2, 3>(0, BG_ID) = C_b_wheel.block<2, 3>(1, 0);
  H_angVel.block<2, 3>(0, SG_ID) =
      (C_b_wheel * imucur_.angular_velocity.asDiagonal()).block<2, 3>(1, 0);
  if (paras_.if_estimate_wheelimu_mounting_angle) {
    H_angVel.block<2, 2>(0, Mounting_angle_ID) =
        (Rotation::skewSymmetric(angularVelocity_wheel)).block<2, 2>(1, 1);
  }

  Eigen::MatrixXd Z_angVel = angularVelocity_wheel.tail(2);
  Eigen::MatrixXd R_angVel = Eigen::MatrixXd::Identity(2, 2) *
                             square(paras_.angular_velocity_update_noise_std);

  EKFUpdate(Z_angVel, H_angVel, R_angVel);
}

void WheelGINS::velocityUpdate() {
  if (imuBuff_.size() < imuBuffsize) return;

  if ((imucur_.timestamp - last_velocity_update_t) < paras_.odo_update_interval)
    return;

  getWheelVelocity();

  if (if_ZUPT_available_) {
    ZUPT();
  } else {
    odo_nhcUpdate();
    if (paras_.if_use_angular_velocity_update) {
      angularVelUpdate();
    }
  }

  last_velocity_update_t = imucur_.timestamp;
}

void WheelGINS::ZUPT() {
  Eigen::MatrixXd Z_zupt = pvacur_.vel;
  Eigen::MatrixXd H_zupt;
  H_zupt.resize(3, RANK);
  H_zupt.setZero();

  H_zupt.block<3, 3>(0, V_ID) = Eigen::Matrix3d::Identity();
  Eigen::MatrixXd R_zupt = Matrix3d::Identity() * square(paras_.zupt_noise_std);
  EKFUpdate(Z_zupt, H_zupt, R_zupt);

  if (if_ZIHR_available_) {
    double delta_heading = pvacur_.att.euler[2] - zihr_preheading_;
    delta_heading = Rotation::heading(delta_heading);

    Eigen::MatrixXd R_zihr =
        Eigen::MatrixXd::Identity(1, 1) * square(paras_.zihr_noise_std);

    Eigen::MatrixXd H_zihr = Eigen::MatrixXd::Zero(1, RANK);
    double tmp = pvacur_.att.cbn(0, 0) * pvacur_.att.cbn(0, 0) +
                 pvacur_.att.cbn(1, 0) * pvacur_.att.cbn(1, 0);
    H_zihr(0, PHI_ID) = pvacur_.att.cbn(0, 0) * pvacur_.att.cbn(2, 0) / tmp;
    H_zihr(0, PHI_ID + 1) = pvacur_.att.cbn(1, 0) * pvacur_.att.cbn(2, 0) / tmp;
    H_zihr(0, PHI_ID + 2) = -1.0;

    Eigen::MatrixXd Z_zihr = Eigen::MatrixXd::Identity(1, 1) * delta_heading;

    EKFUpdate(Z_zihr, H_zihr, R_zihr);
  }

  last_velocity_update_t = imucur_.timestamp;
}
bool WheelGINS::inGNSSOutage(double &timestamp) {
  if (paras_.if_set_gnss_outage) {
    for (auto &it : paras_.gnss_outage) {
      if (it.first < timestamp && timestamp < it.second) {
        return true;
      }
    }
  }
  return false;
}
void WheelGINS::GNSSUpdate() {
  if ((gnssdata_.timestamp - last_gnss_update_t) <
          paras_.gnss_update_interval ||
      inGNSSOutage(gnssdata_.timestamp))
    return;
  if (abs(gnssdata_.timestamp - imucur_.timestamp) > (1.0 / IMU_RATE)) return;

  Matrix3d C_vn = computeVehicleRotation();

  Eigen::Vector3d antenna_pos;
  Eigen::Matrix3d Dr, Dr_inv;
  Dr_inv = Earth::DRi(pvacur_.pos);
  Dr = Earth::DR(pvacur_.pos);
  Vector3d leverarm_vector =
      Dr_inv * (pvacur_.att.cbn * augstate_.wheelimu_leverarm +
                C_vn * paras_.gnssLeverarm);
  antenna_pos = pvacur_.pos + leverarm_vector;

  Eigen::MatrixXd dz;
  dz = Dr * (antenna_pos - gnssdata_.blh);

  Eigen::MatrixXd H_gnsspos;
  H_gnsspos.resize(3, Cov_.rows());
  H_gnsspos.setZero();
  H_gnsspos.block(0, P_ID, 3, 3) = Eigen::Matrix3d::Identity();
  H_gnsspos.block(0, PHI_ID, 3, 3) =
      Rotation::skewSymmetric(pvacur_.att.cbn * augstate_.wheelimu_leverarm +
                              C_vn * paras_.gnssLeverarm);


  if (paras_.if_estimate_wheelimu_leverarm) {
    H_gnsspos.block(0, Wheelimu_leverarm_ID, 3, 2) =
        pvacur_.att.cbn.block<3, 2>(0, 1);
  }

  if (paras_.if_estimate_wheelimu_mounting_angle) {
    H_gnsspos.block(0, Mounting_angle_ID, 3, 2) =
        (pvacur_.att.cbn *
         Rotation::euler2matrix(augstate_.wheelimu_mounting_angle) *
         Rotation::skewSymmetric(C_wheelv.transpose() * paras_.gnssLeverarm))
            .block(0, 1, 3, 2);
  }
  Eigen::MatrixXd R_gnsspos;
  R_gnsspos = create_covariance_block(gnssdata_.std);

  EKFUpdate(dz, H_gnsspos, R_gnsspos);

  last_gnss_update_t = gnssdata_.timestamp;


}

void WheelGINS::getWheelVelocity() {
  double gyro_x_mean = 0.0;

  Matrix3d mounting_angle_Mat =
      Rotation::euler2matrix(augstate_.wheelimu_mounting_angle);

  int i = 0;
  for (auto it = imuBuff_.begin(); it != imuBuff_.end(); ++it) {
    Vector3d gyro = mounting_angle_Mat * it->angular_velocity;
    gyro_x_mean += gyro[0];
  }
  gyro_x_mean /= imuBuffsize;

  wheelVelocity_ =
      -paras_.wheelradius * (gyro_x_mean) / (1 + augstate_.wheel_radius_scale);

  if (paras_.if_use_zupt) detectZUPT();
}

void WheelGINS::detectZUPT() {
  Eigen::Vector6d maxIMUdata, minIMUdata;
  maxIMUdata.setZero();
  minIMUdata.setZero();

  maxIMUdata.array() -= 999;
  minIMUdata.array() += 999;

  for (auto it = imuBuff_.begin(); it != imuBuff_.end(); ++it) {
    maxIMUdata.head(3) = maxIMUdata.head(3).cwiseMax(it->angular_velocity);
    minIMUdata.head(3) = minIMUdata.head(3).cwiseMin(it->angular_velocity);
    maxIMUdata.tail(3) = maxIMUdata.tail(3).cwiseMax(it->acceleration);
    minIMUdata.tail(3) = minIMUdata.tail(3).cwiseMin(it->acceleration);
  }

  double maxmin_angular_velocity =
      (maxIMUdata.head(3) - minIMUdata.head(3)).cwiseAbs().maxCoeff();
  double maxmin_acceleration =
      (maxIMUdata.tail(3) - minIMUdata.tail(3)).cwiseAbs().maxCoeff();

  if (maxmin_acceleration < zupt_special_force_threshold &&
      abs(wheelVelocity_) < zupt_velocity_threshold) {
    if_ZUPT_available_ = true;
    if (maxmin_angular_velocity < zupt_angular_velocity_threshold) {
      ZIHR_num_++;
    } else {
      ZIHR_num_ = 0;
    }
  } else {
    ZIHR_num_ = 0;
    if_ZUPT_available_ = false;
    if_ZIHR_available_ = false;
  }
}

Matrix3d WheelGINS::computeVehicleRotation() {
  Vector3d vehicle_euler = Vector3d::Zero();
  Matrix3d C_wheel_n =
      pvacur_.att.cbn *
      Rotation::euler2matrix(augstate_.wheelimu_mounting_angle).transpose();

  // horizontal motion assumption
  vehicle_euler[2] = Rotation::matrix2euler(C_wheel_n)[2] - M_PI / 2.0;

  Matrix3d C_vn = Rotation::euler2matrix(vehicle_euler);

  Vector3d wheel_v_euler = Vector3d::Zero();
  wheel_v_euler[0] = pvacur_.att.euler[0];
  wheel_v_euler[2] = M_PI / 2.0;

  C_wheelv = Rotation::euler2matrix(wheel_v_euler);

  C_bv_ = C_wheelv * Rotation::euler2matrix(augstate_.wheelimu_mounting_angle);


  return C_vn;
}

void WheelGINS::stateFeedback() {
  if (measument_updated_ == false) return;
  Eigen::Vector3d delta_r = delta_x_.block(P_ID, 0, 3, 1);
  Eigen::Matrix3d Dr_inv = Earth::DRi(pvacur_.pos);
  pvacur_.pos -= Dr_inv * delta_r;

  pvacur_.vel -= delta_x_.block(V_ID, 0, 3, 1);

  Vector3d delta_att = delta_x_.block(PHI_ID, 0, 3, 1);
  Eigen::Quaterniond qpn = Rotation::rotvec2quaternion(delta_att);
  pvacur_.att.qbn = qpn * pvacur_.att.qbn;
  pvacur_.att.cbn = Rotation::quaternion2matrix(pvacur_.att.qbn);
  pvacur_.att.euler = Rotation::matrix2euler(pvacur_.att.cbn);

  imuerror_.gyrbias += delta_x_.block(BG_ID, 0, 3, 1);
  imuerror_.accbias += delta_x_.block(BA_ID, 0, 3, 1);
  imuerror_.gyrscale += delta_x_.block(SG_ID, 0, 3, 1);
  imuerror_.accscale += delta_x_.block(SA_ID, 0, 3, 1);

  if (paras_.if_estimate_wheelimu_leverarm) {
    augstate_.wheelimu_leverarm.tail(2) -=
        delta_x_.block(Wheelimu_leverarm_ID, 0, 2, 1);
    if (paras_.if_estimate_wheel_radius_scale) {
      augstate_.wheel_radius_scale += delta_x_(Wheel_radius_scale_ID, 0);

      if (paras_.if_estimate_wheelimu_mounting_angle) {
        Vector3d delta_mounting_angle_3d = Vector3d::Zero();

        delta_mounting_angle_3d.tail(2) =
            delta_x_.block(Mounting_angle_ID, 0, 2, 1);
        Matrix3d mounting_angle_Mat =
            Rotation::euler2matrix(augstate_.wheelimu_mounting_angle);

        mounting_angle_Mat =
            (Eigen::Matrix3d::Identity() +
             Rotation::skewSymmetric(delta_mounting_angle_3d)) *
            mounting_angle_Mat;
        augstate_.wheelimu_mounting_angle.tail(2) =
            Rotation::matrix2euler(mounting_angle_Mat).tail(2);
      }
    }
  }

  delta_x_.setZero();
  measument_updated_ = false;
}

NavState WheelGINS::getNavState() {
  NavState state;

  state.pos = pvacur_.pos;
  state.vel = pvacur_.vel;

  Matrix3d mounting_angle_Mat =
      Rotation::euler2matrix(augstate_.wheelimu_mounting_angle);

  state.euler =
      Rotation::matrix2euler(pvacur_.att.cbn * mounting_angle_Mat.transpose());
  state.imuerror = imuerror_;

  return state;
}

void WheelGINS::EKFPredict(Eigen::MatrixXd &Phi, Eigen::MatrixXd &Qd) {
  assert(Phi.rows() == Cov_.rows());
  assert(Qd.rows() == Cov_.rows());

  Cov_ = Phi * Cov_ * Phi.transpose() + Qd;
  delta_x_ = Phi * delta_x_;
}

void WheelGINS::EKFUpdate(Eigen::MatrixXd &dz, Eigen::MatrixXd &H,
                          Eigen::MatrixXd &R) {
  assert(H.cols() == Cov_.rows());
  assert(dz.rows() == H.rows());
  assert(dz.rows() == R.rows());
  assert(dz.cols() == 1);

  auto HPHTR = H * Cov_ * H.transpose() + R;
  Eigen::MatrixXd K = Cov_ * H.transpose() * HPHTR.inverse();

  Eigen::MatrixXd I;
  I.resizeLike(Cov_);
  I.setIdentity();
  I = I - K * H;

  delta_x_ = delta_x_ + K * (dz - H * delta_x_);
  Cov_ = I * Cov_ * I.transpose() + K * R * K.transpose();

  measument_updated_ = true;
}