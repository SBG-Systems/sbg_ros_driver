#ifndef ELLIPSEMSG_HEADER
#define ELLIPSEMSG_HEADER

#include "ros/ros.h"

#include <sbgEComLib.h>
#include <sbgEComIds.h>
#include <sbgErrorCodes.h>

#include "sbg_driver/SbgStatus.h"
#include "sbg_driver/SbgUtcTime.h"
#include "sbg_driver/SbgImuData.h"
#include "sbg_driver/SbgEkfEuler.h"
#include "sbg_driver/SbgEkfQuat.h"
#include "sbg_driver/SbgEkfNav.h"
#include "sbg_driver/SbgShipMotion.h"
#include "sbg_driver/SbgMag.h"
#include "sbg_driver/SbgMagCalib.h"
#include "sbg_driver/SbgGpsVel.h"
#include "sbg_driver/SbgGpsPos.h"
#include "sbg_driver/SbgGpsHdt.h"
#include "sbg_driver/SbgGpsRaw.h"
#include "sbg_driver/SbgOdoVel.h"
#include "sbg_driver/SbgEvent.h"
#include "sbg_driver/SbgPressure.h"

// SBG_ECOM_LOG_STATUS
void read_ecom_log_status(sbg_driver::SbgStatus &msg, const SbgBinaryLogData *pLogData);

// SBG_ECOM_LOG_UTC_TIME
void read_ecom_log_utc_time(sbg_driver::SbgUtcTime &msg, const SbgBinaryLogData *pLogData);

// SBG_ECOM_LOG_IMU_DATA
void read_ecom_log_imu_data(sbg_driver::SbgImuData &msg, const SbgBinaryLogData *pLogData);

// SBG_ECOM_LOG_EKF_EULER
void read_ecom_log_ekf_euler(sbg_driver::SbgEkfEuler &msg, const SbgBinaryLogData *pLogData);

// SBG_ECOM_LOG_EKF_QUAT
void read_ecom_log_ekf_quat(sbg_driver::SbgEkfQuat &msg, const SbgBinaryLogData *pLogData);

// SBG_ECOM_LOG_EKF_NAV
void read_ecom_log_ekf_nav(sbg_driver::SbgEkfNav &msg, const SbgBinaryLogData *pLogData);

// SBG_ECOM_LOG_SHIP_MOTION_0
void read_ecom_log_ship_motion(sbg_driver::SbgShipMotion &msg, const SbgBinaryLogData *pLogData);

// SBG_ECOM_LOG_MAG
void read_ecom_log_mag(sbg_driver::SbgMag &msg, const SbgBinaryLogData *pLogData);

// SBG_ECOM_LOG_MAG_CALIB
void read_ecom_log_mag_calib(sbg_driver::SbgMagCalib &msg, const SbgBinaryLogData *pLogData);

// SBG_ECOM_LOG_GPS1_VEL
void read_ecom_log_gps_vel(sbg_driver::SbgGpsVel &msg, const SbgBinaryLogData *pLogData);

// SBG_ECOM_LOG_GPS1_POS
void read_ecom_log_gps_pos(sbg_driver::SbgGpsPos &msg, const SbgBinaryLogData *pLogData);

// SBG_ECOM_LOG_GPS1_HDT
void read_ecom_log_gps_hdt(sbg_driver::SbgGpsHdt &msg, const SbgBinaryLogData *pLogData);

// SBG_ECOM_LOG_GPS1_RAW
void read_ecom_log_gps_raw(sbg_driver::SbgGpsRaw &msg, const SbgBinaryLogData *pLogData);

// SBG_ECOM_LOG_ODO_VEL
void read_ecom_log_odo_vel(sbg_driver::SbgOdoVel &msg, const SbgBinaryLogData *pLogData);

// SBG_ECOM_LOG_EVENT_A
void read_ecom_log_event(sbg_driver::SbgEvent &msg, const SbgBinaryLogData *pLogData);

// SBG_ECOM_LOG_PRESSURE
void read_ecom_log_pressure(sbg_driver::SbgPressure &msg, const SbgBinaryLogData *pLogData);

// Fill the SbgEkfStatus with val
void read_ekf_solution_status(sbg_driver::SbgEkfStatus &msg, const uint32 &val);

// Fill the SbgMagStatus with val
void read_mag_status(sbg_driver::SbgMagStatus &msg, const uint16 &val);

// Fill the SbgGpsVelStatus with val
void read_gps_vel_status(sbg_driver::SbgGpsVelStatus &msg, const uint32 &val);

// Fill the SbgGpsPosStatus with val
void read_gps_pos_status(sbg_driver::SbgGpsPosStatus &msg, const uint32 &val);

// Read Info from the device
void read_get_info(SbgEComHandle *comHandle);

#endif
