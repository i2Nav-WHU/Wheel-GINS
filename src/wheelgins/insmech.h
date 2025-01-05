#pragma once

#include "common/types.h"

class INSMech {
 private:
  static void posUpdate(const PVA &pvapre, PVA &pvacur, const IMU &imupre,
                        const IMU &imucur);
  static void velUpdate(const PVA &pvapre, PVA &pvacur, const IMU &imupre,
                        const IMU &imucur);
  static void attUpdate(const PVA &pvapre, PVA &pvacur, const IMU &imupre,
                        const IMU &imucur);

 public:
  static void insMech(const PVA &pvapre, PVA &pvacur, const IMU &imupre,
                      const IMU &imucur);

  static void imuCompensate(IMU &imu, ImuError &imuerror);
};