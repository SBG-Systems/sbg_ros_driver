#include "ellipse_msg.h"

void read_ecom_log_status(sbg_driver::SbgStatus &msg, const SbgBinaryLogData *pLogData){
	msg.header.stamp = ros::Time::now();
	msg.time_stamp = pLogData->statusData.timeStamp;

	msg.status_general.main_power = (pLogData->statusData.generalStatus & (1 << 0)) >> 0;
	msg.status_general.imu_power = (pLogData->statusData.generalStatus & (1 << 1)) >> 1;
	msg.status_general.gps_power = (pLogData->statusData.generalStatus & (1 << 2)) >> 2;
	msg.status_general.settings = (pLogData->statusData.generalStatus & (1 << 3)) >> 3;
	msg.status_general.temperature = (pLogData->statusData.generalStatus & (1 << 4)) >> 4;

	msg.status_com.port_a = (pLogData->statusData.comStatus & (1 << 0)) >> 0;
	msg.status_com.port_b = (pLogData->statusData.comStatus & (1 << 1)) >> 1;
	msg.status_com.port_c = (pLogData->statusData.comStatus & (1 << 2)) >> 2;
	msg.status_com.port_d = (pLogData->statusData.comStatus & (1 << 3)) >> 3;
	msg.status_com.port_e = (pLogData->statusData.comStatus & (1 << 4)) >> 4;
	msg.status_com.port_a_rx = (pLogData->statusData.comStatus & (1 << 5)) >> 5;
	msg.status_com.port_a_tx = (pLogData->statusData.comStatus & (1 << 6)) >> 6;
	msg.status_com.port_b_rx = (pLogData->statusData.comStatus & (1 << 7)) >> 7;
	msg.status_com.port_b_tx = (pLogData->statusData.comStatus & (1 << 8)) >> 8;
	msg.status_com.port_c_rx = (pLogData->statusData.comStatus & (1 << 9)) >> 9;
	msg.status_com.port_c_tx = (pLogData->statusData.comStatus & (1 << 10)) >> 10;
	msg.status_com.port_d_rx = (pLogData->statusData.comStatus & (1 << 11)) >> 11;
	msg.status_com.port_d_tx = (pLogData->statusData.comStatus & (1 << 12)) >> 12;
	msg.status_com.port_e_rx = (pLogData->statusData.comStatus & (1 << 13)) >> 13;
	msg.status_com.port_e_tx = (pLogData->statusData.comStatus & (1 << 14)) >> 14;
	msg.status_com.can_rx = (pLogData->statusData.comStatus & (1 << 25)) >> 25;
	msg.status_com.can_tx = (pLogData->statusData.comStatus & (1 << 26)) >> 26;
	msg.status_com.can_status = (pLogData->statusData.comStatus & (0b111 << 27)) >> 27;

	msg.status_aiding.gps1_pos_recv = (pLogData->statusData.aidingStatus & (1 << 0)) >> 0;
	msg.status_aiding.gps1_vel_recv = (pLogData->statusData.aidingStatus & (1 << 1)) >> 1;
	msg.status_aiding.gps1_hdt_recv = (pLogData->statusData.aidingStatus & (1 << 2)) >> 2;
	msg.status_aiding.gps1_utc_recv = (pLogData->statusData.aidingStatus & (1 << 3)) >> 3;
	msg.status_aiding.mag_recv = (pLogData->statusData.aidingStatus & (1 << 8)) >> 8;
	msg.status_aiding.odo_recv = (pLogData->statusData.aidingStatus & (1 << 9)) >> 9;
	msg.status_aiding.dvl_recv = (pLogData->statusData.aidingStatus & (1 << 10)) >> 10;
}

void read_ecom_log_utc_time(sbg_driver::SbgUtcTime &msg, const SbgBinaryLogData *pLogData){
	msg.header.stamp = ros::Time::now();
	msg.time_stamp = pLogData->utcData.timeStamp;

	msg.clock_status.clock_stable = (pLogData->utcData.status & (1 << 0)) >> 0;
	msg.clock_status.clock_status = (pLogData->utcData.status & (0b1111 << 1)) >> 1;
	msg.clock_status.clock_utc_sync = (pLogData->utcData.status & (1 << 5)) >> 5;
	msg.clock_status.clock_utc_status = (pLogData->utcData.status & (0b1111 << 6)) >> 6;
	msg.year = pLogData->utcData.year;
	msg.month = pLogData->utcData.month;
	msg.day = pLogData->utcData.day;
	msg.hour = pLogData->utcData.hour;
	msg.min = pLogData->utcData.minute;
	msg.sec = pLogData->utcData.second;
	msg.nanosec = pLogData->utcData.nanoSecond;
	msg.gps_tow = pLogData->utcData.gpsTimeOfWeek;
}

void read_ecom_log_imu_data(sbg_driver::SbgImuData &msg, const SbgBinaryLogData *pLogData){
	msg.header.stamp = ros::Time::now();
	msg.time_stamp = pLogData->imuData.timeStamp;

	msg.imu_status.imu_com = (pLogData->imuData.status & (1 << 0)) >> 0;
	msg.imu_status.imu_status = (pLogData->imuData.status & (1 << 1)) >> 1;
	msg.imu_status.imu_accel_x = (pLogData->imuData.status & (1 << 2)) >> 2;
	msg.imu_status.imu_accel_y = (pLogData->imuData.status & (1 << 3)) >> 3;
	msg.imu_status.imu_accel_z = (pLogData->imuData.status & (1 << 4)) >> 4;
	msg.imu_status.imu_gyro_x = (pLogData->imuData.status & (1 << 5)) >> 5;
	msg.imu_status.imu_gyro_y = (pLogData->imuData.status & (1 << 6)) >> 6;
	msg.imu_status.imu_gyro_z = (pLogData->imuData.status & (1 << 7)) >> 7;
	msg.imu_status.imu_accels_in_range = (pLogData->imuData.status & (1 << 8)) >> 8;
	msg.imu_status.imu_gyros_in_range = (pLogData->imuData.status & (1 << 9)) >> 9;

	msg.accel.x = pLogData->imuData.accelerometers[0];
	msg.accel.y = pLogData->imuData.accelerometers[1];
	msg.accel.z = pLogData->imuData.accelerometers[2];
	msg.gyro.x = pLogData->imuData.gyroscopes[0];
	msg.gyro.y = pLogData->imuData.gyroscopes[1];
	msg.gyro.z = pLogData->imuData.gyroscopes[2];
	msg.temp = pLogData->imuData.temperature;
	msg.delta_vel.x = pLogData->imuData.deltaVelocity[0];
	msg.delta_vel.y = pLogData->imuData.deltaVelocity[1];
	msg.delta_vel.z = pLogData->imuData.deltaVelocity[2];
	msg.delta_angle.x = pLogData->imuData.deltaAngle[0];
	msg.delta_angle.y = pLogData->imuData.deltaAngle[1];
	msg.delta_angle.z = pLogData->imuData.deltaAngle[2];
}

void read_ekf_solution_status(sbg_driver::SbgEkfStatus &msg, const uint32 &val){
	msg.solution_mode = val & (0b1111);
	msg.attitude_valid = (val & (1 << 4)) >> 4;
	msg.heading_valid = (val & (1 << 5)) >> 5;
	msg.velocity_valid = (val & (1 << 6)) >> 6;
	msg.position_valid = (val & (1 << 7)) >> 7;
	msg.vert_ref_used = (val & (1 << 8)) >> 8;
	msg.mag_ref_used = (val & (1 << 9)) >> 9;
	msg.gps1_vel_used = (val & (1 << 10)) >> 10;
	msg.gps1_pos_used = (val & (1 << 11)) >> 11;
	msg.gps1_course_used = (val & (1 << 12)) >> 12;
	msg.gps1_hdt_used = (val & (1 << 13)) >> 13;
	msg.gps2_vel_used = (val & (1 << 14)) >> 14;
	msg.gps2_pos_used = (val & (1 << 15)) >> 15;
	msg.gps2_course_used = (val & (1 << 16)) >> 16;
	msg.gps2_hdt_used = (val & (1 << 17)) >> 17;
	msg.odo_used = (val & (1 << 18)) >> 18;
}

void read_ecom_log_ekf_euler(sbg_driver::SbgEkfEuler &msg, const SbgBinaryLogData *pLogData){
	msg.header.stamp = ros::Time::now();
	msg.time_stamp = pLogData->ekfEulerData.timeStamp;
	msg.angle.x = pLogData->ekfEulerData.euler[0];
	msg.angle.y = pLogData->ekfEulerData.euler[1];
	msg.angle.z = pLogData->ekfEulerData.euler[2];
	msg.accuracy.x = pLogData->ekfEulerData.eulerStdDev[0];
	msg.accuracy.y = pLogData->ekfEulerData.eulerStdDev[1];
	msg.accuracy.z = pLogData->ekfEulerData.eulerStdDev[2];
	read_ekf_solution_status(msg.status, pLogData->ekfEulerData.status);
}

void read_ecom_log_ekf_quat(sbg_driver::SbgEkfQuat &msg, const SbgBinaryLogData *pLogData){
	msg.header.stamp = ros::Time::now();
	msg.header.stamp = ros::Time::now();
	msg.time_stamp = pLogData->ekfQuatData.timeStamp;
	msg.quaternion.x = pLogData->ekfQuatData.quaternion[1];
	msg.quaternion.y = pLogData->ekfQuatData.quaternion[2];
	msg.quaternion.z = pLogData->ekfQuatData.quaternion[3];
	msg.quaternion.w = pLogData->ekfQuatData.quaternion[0];
	msg.accuracy.x = pLogData->ekfQuatData.eulerStdDev[0];
	msg.accuracy.y = pLogData->ekfQuatData.eulerStdDev[1];
	msg.accuracy.z = pLogData->ekfQuatData.eulerStdDev[2];
	read_ekf_solution_status(msg.status, pLogData->ekfQuatData.status);
}

void read_ecom_log_ekf_nav(sbg_driver::SbgEkfNav &msg, const SbgBinaryLogData *pLogData){
	msg.header.stamp = ros::Time::now();

	msg.time_stamp = pLogData->ekfNavData.timeStamp;
	msg.velocity.x = pLogData->ekfNavData.velocity[0];
	msg.velocity.y = pLogData->ekfNavData.velocity[1];
	msg.velocity.z = pLogData->ekfNavData.velocity[2];
	msg.velocity_accuracy.x = pLogData->ekfNavData.velocityStdDev[0];
	msg.velocity_accuracy.y = pLogData->ekfNavData.velocityStdDev[1];
	msg.velocity_accuracy.z = pLogData->ekfNavData.velocityStdDev[2];
	msg.position.x = pLogData->ekfNavData.position[0];
	msg.position.y = pLogData->ekfNavData.position[1];
	msg.position.z = pLogData->ekfNavData.position[2];
	msg.undulation = pLogData->ekfNavData.undulation;

	read_ekf_solution_status(msg.status, pLogData->ekfNavData.status);
}

void read_ecom_log_ship_motion(sbg_driver::SbgShipMotion &msg, const SbgBinaryLogData *pLogData){
	msg.header.stamp = ros::Time::now();
	msg.time_stamp = pLogData->shipMotionData.timeStamp;
	msg.heave_period = pLogData->shipMotionData.mainHeavePeriod;
	msg.ship_motion.x = pLogData->shipMotionData.shipMotion[0];
	msg.ship_motion.y = pLogData->shipMotionData.shipMotion[1];
	msg.ship_motion.z = pLogData->shipMotionData.shipMotion[2];
	msg.acceleration.x = pLogData->shipMotionData.shipAccel[0];
	msg.acceleration.y = pLogData->shipMotionData.shipAccel[1];
	msg.acceleration.z = pLogData->shipMotionData.shipAccel[2];
	msg.velocity.x = pLogData->shipMotionData.shipVel[0];
	msg.velocity.y = pLogData->shipMotionData.shipVel[1];
	msg.velocity.z = pLogData->shipMotionData.shipVel[2];

	msg.status.heave_valid = (pLogData->shipMotionData.status & (1 << 0)) >> 0;
	msg.status.heave_vel_aided = (pLogData->shipMotionData.status & (1 << 1)) >> 1;
	msg.status.period_available = (pLogData->shipMotionData.status & (1 << 2)) >> 2;
	msg.status.period_valid = (pLogData->shipMotionData.status & (1 << 3)) >> 3;
}

void read_ecom_log_mag(sbg_driver::SbgMag &msg, const SbgBinaryLogData *pLogData){
	msg.header.stamp = ros::Time::now();

}

void read_ecom_log_mag_calib(sbg_driver::SbgMagCalib &msg, const SbgBinaryLogData *pLogData){
	msg.header.stamp = ros::Time::now();

}

void read_ecom_log_gps_vel(sbg_driver::SbgGpsVel &msg, const SbgBinaryLogData *pLogData){
	msg.header.stamp = ros::Time::now();

}

void read_ecom_log_gps_pos(sbg_driver::SbgGpsPos &msg, const SbgBinaryLogData *pLogData){
	msg.header.stamp = ros::Time::now();

}

void read_ecom_log_gps_hdt(sbg_driver::SbgGpsHdt &msg, const SbgBinaryLogData *pLogData){
	msg.header.stamp = ros::Time::now();

}

void read_ecom_log_gps_raw(sbg_driver::SbgGpsRaw &msg, const SbgBinaryLogData *pLogData){
	msg.header.stamp = ros::Time::now();

}

void read_ecom_log_odo_vel(sbg_driver::SbgOdoVel &msg, const SbgBinaryLogData *pLogData){
	msg.header.stamp = ros::Time::now();

}

void read_ecom_log_event(sbg_driver::SbgEvent &msg, const SbgBinaryLogData *pLogData){
	msg.header.stamp = ros::Time::now();

}

void read_ecom_log_pressure(sbg_driver::SbgPressure &msg, const SbgBinaryLogData *pLogData){
	msg.header.stamp = ros::Time::now();

}
