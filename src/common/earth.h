/*
 * OB_GINS: An Optimization-Based GNSS/INS Integrated Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Hailiang Tang, Liqiang Wang
 *    Contact : thl@whu.edu.cn, wlq@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <Eigen/Geometry>

#include "types.h"

const double WGS84_WIE = 7.2921151467E-5;
const double WGS84_F = 0.0033528106647474805;
const double WGS84_RA = 6378137.0000000000;
const double WGS84_RB = 6356752.3142451793;
const double WGS84_GM0 = 398600441800000.00;
const double WGS84_E1 = 0.0066943799901413156;
const double WGS84_E2 = 0.0067394967422764341;

class Earth {
 public:
  static double gravity(const Vector3d &blh) {
    double sin2 = sin(blh[0]);
    sin2 *= sin2;

    return 9.7803267715 *
               (1 + 0.0052790414 * sin2 + 0.0000232718 * sin2 * sin2) +
           blh[2] * (0.0000000043977311 * sin2 - 0.0000030876910891) +
           0.0000000000007211 * blh[2] * blh[2];
  }

  static Eigen::Vector2d meridianPrimeVerticalRadius(double lat) {
    double tmp, sqrttmp;

    tmp = sin(lat);
    tmp *= tmp;
    tmp = 1 - WGS84_E1 * tmp;
    sqrttmp = sqrt(tmp);

    return {WGS84_RA * (1 - WGS84_E1) / (sqrttmp * tmp), WGS84_RA / sqrttmp};
  }

  static double RN(double lat) {
    double sinlat = sin(lat);
    return WGS84_RA / sqrt(1.0 - WGS84_E1 * sinlat * sinlat);
  }

  static Matrix3d cne(const Vector3d &blh) {
    double coslon, sinlon, coslat, sinlat;

    sinlat = sin(blh[0]);
    sinlon = sin(blh[1]);
    coslat = cos(blh[0]);
    coslon = cos(blh[1]);

    Matrix3d dcm;
    dcm(0, 0) = -sinlat * coslon;
    dcm(0, 1) = -sinlon;
    dcm(0, 2) = -coslat * coslon;

    dcm(1, 0) = -sinlat * sinlon;
    dcm(1, 1) = coslon;
    dcm(1, 2) = -coslat * sinlon;

    dcm(2, 0) = coslat;
    dcm(2, 1) = 0;
    dcm(2, 2) = -sinlat;

    return dcm;
  }

  static Quaterniond qne(const Vector3d &blh) {
    Quaterniond quat;

    double coslon, sinlon, coslat, sinlat;

    coslon = cos(blh[1] * 0.5);
    sinlon = sin(blh[1] * 0.5);
    coslat = cos(-M_PI * 0.25 - blh[0] * 0.5);
    sinlat = sin(-M_PI * 0.25 - blh[0] * 0.5);

    quat.w() = coslat * coslon;
    quat.x() = -sinlat * sinlon;
    quat.y() = sinlat * coslon;
    quat.z() = coslat * sinlon;

    return quat;
  }

  static Vector3d blh(const Quaterniond &qne, double height) {
    return {-2 * atan(qne.y() / qne.w()) - M_PI * 0.5,
            2 * atan2(qne.z(), qne.w()), height};
  }

  static Vector3d blh2ecef(const Vector3d &blh) {
    double coslat, sinlat, coslon, sinlon;
    double rnh, rn;

    coslat = cos(blh[0]);
    sinlat = sin(blh[0]);
    coslon = cos(blh[1]);
    sinlon = sin(blh[1]);

    rn = RN(blh[0]);
    rnh = rn + blh[2];

    return {rnh * coslat * coslon, rnh * coslat * sinlon,
            (rnh - rn * WGS84_E1) * sinlat};
  }

  static Vector3d ecef2blh(const Vector3d &ecef) {
    double p = sqrt(ecef[0] * ecef[0] + ecef[1] * ecef[1]);
    double rn;
    double lat, lon;
    double h = 0, h2;

    lat = atan(ecef[2] / (p * (1.0 - WGS84_E1)));
    lon = 2.0 * atan2(ecef[1], ecef[0] + p);

    do {
      h2 = h;
      rn = RN(lat);
      h = p / cos(lat) - rn;
      lat = atan(ecef[2] / (p * (1.0 - WGS84_E1 * rn / (rn + h))));
    } while (fabs(h - h2) > 1.0e-4);

    return {lat, lon, h};
  }

  static Matrix3d DRi(const Vector3d &blh) {
    Matrix3d dri = Matrix3d::Zero();

    Eigen::Vector2d rmn = meridianPrimeVerticalRadius(blh[0]);

    dri(0, 0) = 1.0 / (rmn[0] + blh[2]);
    dri(1, 1) = 1.0 / ((rmn[1] + blh[2]) * cos(blh[0]));
    dri(2, 2) = -1;
    return dri;
  }

  static Matrix3d DR(const Vector3d &blh) {
    Matrix3d dr = Matrix3d::Zero();

    Eigen::Vector2d rmn = meridianPrimeVerticalRadius(blh[0]);

    dr(0, 0) = rmn[0] + blh[2];
    dr(1, 1) = (rmn[1] + blh[2]) * cos(blh[0]);
    dr(2, 2) = -1;
    return dr;
  }

  static Vector3d local2global(const Vector3d &originBLH,
                               const Vector3d &local) {
    Vector3d ecef0 = blh2ecef(originBLH);
    Matrix3d cn0e = cne(originBLH);

    Vector3d ecef1 = ecef0 + cn0e * local;
    Vector3d blh1 = ecef2blh(ecef1);

    return blh1;
  }

  static Vector3d global2local(const Vector3d &originBLH,
                               const Vector3d &globalBLH) {
    Vector3d ecef0 = blh2ecef(originBLH);
    Matrix3d cn0e = cne(originBLH);

    Vector3d ecef1 = blh2ecef(globalBLH);

    return cn0e.transpose() * (ecef1 - ecef0);
  }

  static Vector3d iewe() { return {0, 0, WGS84_WIE}; }

  static Vector3d iewn(double lat) {
    return {WGS84_WIE * cos(lat), 0, -WGS84_WIE * sin(lat)};
  }

  static Vector3d iewn(const Vector3d &origin, const Vector3d &local) {
    Vector3d global = local2global(origin, local);

    return iewn(global[0]);
  }

  static Vector3d enwn(const Eigen::Vector2d &rmn, const Vector3d &blh,
                       const Vector3d &vel) {
    return {vel[1] / (rmn[1] + blh[2]), -vel[0] / (rmn[0] + blh[2]),
            -vel[1] * tan(blh[0]) / (rmn[1] + blh[2])};
  }

  static Vector3d enwn(const Vector3d &origin, const Vector3d &local,
                       const Vector3d &vel) {
    Vector3d global = local2global(origin, local);
    Eigen::Vector2d rmn = meridianPrimeVerticalRadius(global[0]);

    return enwn(rmn, global, vel);
  }
};