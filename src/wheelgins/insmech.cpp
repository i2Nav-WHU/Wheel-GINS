#include "insmech.h"

#include "common/earth.h"
#include "common/rotation.h"

void INSMech::insMech(const PVA &pvapre, PVA &pvacur, const IMU &imupre,
                      const IMU &imucur) {
  velUpdate(pvapre, pvacur, imupre, imucur);
  posUpdate(pvapre, pvacur, imupre, imucur);
  attUpdate(pvapre, pvacur, imupre, imucur);
}

void INSMech::velUpdate(const PVA &pvapre, PVA &pvacur, const IMU &imupre,
                        const IMU &imucur) {
  Eigen::Vector3d d_vfb, d_vfn, d_vgn, gl, midvel, midpos;
  Eigen::Vector3d temp1, temp2, temp3;
  Eigen::Matrix3d cnn, I33 = Eigen::Matrix3d::Identity();
  Eigen::Quaterniond qne, qee, qnn, qbb, q1, q2;

  Eigen::Vector3d imucur_dvel = imucur.acceleration * imucur.dt;
  Eigen::Vector3d imucur_dtheta = imucur.angular_velocity * imucur.dt;
  Eigen::Vector3d imupre_dvel = imupre.acceleration * imupre.dt;
  Eigen::Vector3d imupre_dtheta = imupre.angular_velocity * imupre.dt;

  Eigen::Vector2d rmrn = Earth::meridianPrimeVerticalRadius(pvapre.pos(0));
  Eigen::Vector3d wie_n, wen_n;
  wie_n << WGS84_WIE * cos(pvapre.pos[0]), 0, -WGS84_WIE * sin(pvapre.pos[0]);
  wen_n << pvapre.vel[1] / (rmrn[1] + pvapre.pos[2]),
      -pvapre.vel[0] / (rmrn[0] + pvapre.pos[2]),
      -pvapre.vel[1] * tan(pvapre.pos[0]) / (rmrn[1] + pvapre.pos[2]);
  double gravity = Earth::gravity(pvapre.pos);

  temp1 = imucur_dtheta.cross(imucur_dvel) / 2;
  temp2 = imupre_dtheta.cross(imucur_dvel) / 12;
  temp3 = imupre_dvel.cross(imucur_dtheta) / 12;

  d_vfb = imucur_dvel + temp1 + temp2 + temp3;

  temp1 = (wie_n + wen_n) * imucur.dt / 2;
  cnn = I33 - Rotation::skewSymmetric(temp1);
  d_vfn = cnn * pvapre.att.cbn * d_vfb;

  gl << 0, 0, gravity;
  d_vgn = (gl - (2 * wie_n + wen_n).cross(pvapre.vel)) * imucur.dt;

  midvel = pvapre.vel + (d_vfn + d_vgn) / 2;

  qnn = Rotation::rotvec2quaternion(temp1);
  temp2 << 0, 0, -WGS84_WIE * imucur.dt / 2;
  qee = Rotation::rotvec2quaternion(temp2);
  qne = Earth::qne(pvapre.pos);
  qne = qee * qne * qnn;
  midpos[2] = pvapre.pos[2] - midvel[2] * imucur.dt / 2;
  midpos = Earth::blh(qne, midpos[2]);

  rmrn = Earth::meridianPrimeVerticalRadius(midpos[0]);
  wie_n << WGS84_WIE * cos(midpos[0]), 0, -WGS84_WIE * sin(midpos[0]);
  wen_n << midvel[1] / (rmrn[1] + midpos[2]),
      -midvel[0] / (rmrn[0] + midpos[2]),
      -midvel[1] * tan(midpos[0]) / (rmrn[1] + midpos[2]);

  temp3 = (wie_n + wen_n) * imucur.dt / 2;
  cnn = I33 - Rotation::skewSymmetric(temp3);
  d_vfn = cnn * pvapre.att.cbn * d_vfb;

  gl << 0, 0, Earth::gravity(midpos);
  d_vgn = (gl - (2 * wie_n + wen_n).cross(midvel)) * imucur.dt;

  pvacur.vel = pvapre.vel + d_vfn + d_vgn;
}

void INSMech::posUpdate(const PVA &pvapre, PVA &pvacur, const IMU &imupre,
                        const IMU &imucur) {
  Eigen::Vector3d temp1, temp2, midvel, midpos;
  Eigen::Quaterniond qne, qee, qnn;

  midvel = (pvacur.vel + pvapre.vel) / 2;
  midpos = pvapre.pos + Earth::DRi(pvapre.pos) * midvel * imucur.dt / 2;

  Eigen::Vector2d rmrn;
  Eigen::Vector3d wie_n, wen_n;
  rmrn = Earth::meridianPrimeVerticalRadius(midpos[0]);
  wie_n << WGS84_WIE * cos(midpos[0]), 0, -WGS84_WIE * sin(midpos[0]);
  wen_n << midvel[1] / (rmrn[1] + midpos[2]),
      -midvel[0] / (rmrn[0] + midpos[2]),
      -midvel[1] * tan(midpos[0]) / (rmrn[1] + midpos[2]);

  temp1 = (wie_n + wen_n) * imucur.dt;
  qnn = Rotation::rotvec2quaternion(temp1);
  temp2 << 0, 0, -WGS84_WIE * imucur.dt;
  qee = Rotation::rotvec2quaternion(temp2);

  qne = Earth::qne(pvapre.pos);
  qne = qee * qne * qnn;
  pvacur.pos[2] = pvapre.pos[2] - midvel[2] * imucur.dt;
  pvacur.pos = Earth::blh(qne, pvacur.pos[2]);
}

void INSMech::attUpdate(const PVA &pvapre, PVA &pvacur, const IMU &imupre,
                        const IMU &imucur) {
  Eigen::Quaterniond qne_pre, qne_cur, qne_mid, qnn, qbb;
  Eigen::Vector3d temp1, midpos, midvel;

  Eigen::Vector3d imucur_dtheta = imucur.angular_velocity * imucur.dt;
  Eigen::Vector3d imupre_dtheta = imupre.angular_velocity * imupre.dt;

  midvel = (pvapre.vel + pvacur.vel) / 2;
  qne_pre = Earth::qne(pvapre.pos);
  qne_cur = Earth::qne(pvacur.pos);
  temp1 = Rotation::quaternion2vector(qne_cur.inverse() * qne_pre);
  qne_mid = qne_pre * Rotation::rotvec2quaternion(temp1 / 2).inverse();
  midpos[2] = (pvacur.pos[2] + pvapre.pos[2]) / 2;
  midpos = Earth::blh(qne_mid, midpos[2]);

  Eigen::Vector2d rmrn;
  Eigen::Vector3d wie_n, wen_n;
  rmrn = Earth::meridianPrimeVerticalRadius(midpos[0]);
  wie_n << WGS84_WIE * cos(midpos[0]), 0, -WGS84_WIE * sin(midpos[0]);
  wen_n << midvel[1] / (rmrn[1] + midpos[2]),
      -midvel[0] / (rmrn[0] + midpos[2]),
      -midvel[1] * tan(midpos[0]) / (rmrn[1] + midpos[2]);

  temp1 = -(wie_n + wen_n) * imucur.dt;
  qnn = Rotation::rotvec2quaternion(temp1);

  temp1 = imucur_dtheta + imupre_dtheta.cross(imucur_dtheta) / 12;
  qbb = Rotation::rotvec2quaternion(temp1);

  pvacur.att.qbn = qnn * pvapre.att.qbn * qbb;
  pvacur.att.cbn = Rotation::quaternion2matrix(pvacur.att.qbn);
  pvacur.att.euler = Rotation::matrix2euler(pvacur.att.cbn);
}

void INSMech::imuCompensate(IMU &imu, ImuError &imuerror) {
  imu.angular_velocity -= imuerror.gyrbias;
  imu.acceleration -= imuerror.accbias;

  Eigen::Vector3d gyrscale, accscale;
  gyrscale = Eigen::Vector3d::Ones() + imuerror.gyrscale;
  accscale = Eigen::Vector3d::Ones() + imuerror.accscale;
  imu.angular_velocity =
      imu.angular_velocity.cwiseProduct(gyrscale.cwiseInverse());
  imu.acceleration = imu.acceleration.cwiseProduct(accscale.cwiseInverse());
}
