/****************************************************************************
 *
 *   Copyright (C) 2014 Aarav Unmanned Systems Pvt. Ltd. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name AUS nor the names of its developers may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file sensor_pub.cpp
 *
 * @author Siddharth Bharat Purohit <sidbpurohit@gmail.com>
 */

#include "sensor_pub.hpp"
#include <systemlib/err.h>

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_time.h>
#include <drivers/drv_sensor.h>
#include <lib/conversion/rotation.h>

UavcanSensorPub *UavcanSensorPub::_instance;
int32_t UavcanSensorPub::_mag_primary_id = 0;
UavcanSensorPub::UavcanSensorPub(uavcan::INode &node) :
	_node(node),
	_uavcan_pub_pressure(node),
	_uavcan_pub_temperature(node),
	_uavcan_pub_mag(node),
	_uavcan_pub_imu(node),
	_uavcan_pub_rcin(node),
	_uavcan_pub_rf(node),
	_orb_timer(node)
{
	_uavcan_pub_pressure.setPriority(UAVCAN_SENSOR_TRANSFER_PRIORITY);
	_uavcan_pub_temperature.setPriority(UAVCAN_SENSOR_TRANSFER_PRIORITY);
	_uavcan_pub_mag.setPriority(UAVCAN_SENSOR_TRANSFER_PRIORITY);
	_uavcan_pub_imu.setPriority(UAVCAN_SENSOR_TRANSFER_PRIORITY);
	_uavcan_pub_rcin.setPriority(UAVCAN_SENSOR_TRANSFER_PRIORITY);
	_uavcan_pub_rf.setPriority(UAVCAN_SENSOR_TRANSFER_PRIORITY);
}

UavcanSensorPub::~UavcanSensorPub()
{

}

int UavcanSensorPub::update()
{
	px4_pollfd_struct_t fds;
	//bool gps_updated = false;
	bool baro_updated = false;
	bool mag_updated = false;
	bool rcin_updated = false;
	bool rf_updated = false;

	_baro_sub = orb_subscribe(ORB_ID(sensor_baro));

	uint8_t group_count = orb_group_count(ORB_ID(sensor_mag));
	for (unsigned i = 0; i < group_count; i++) {
		_mag_sub[i] = orb_subscribe_multi(ORB_ID(sensor_mag), i);
		int32_t priority;
		orb_priority(_mag_sub[i], &priority);
		_mag_priority[i] = (uint8_t)priority;
	}
	_gyro_sub = orb_subscribe(ORB_ID(sensor_gyro));
	_accel_sub = orb_subscribe(ORB_ID(sensor_accel));
	_rcin_sub = orb_subscribe(ORB_ID(input_rc));
	_rf_sub = orb_subscribe(ORB_ID(distance_sensor));

	fds.fd = _gyro_sub;
	fds.events = POLLIN;
	sensor_baro_s baro = {};
	sensor_gyro_s gyro = {};
	sensor_accel_s accel = {};
	sensor_mag_s mag = {};
	input_rc_s rcin = {};
	distance_sensor_s rf = {};

	while (!_task_should_exit) {
		int ret = px4_poll(&fds, 1, 1000);

		if (ret < 0) {
			// Poll error, sleep and try again
			usleep(10000);
			continue;

		} else if (ret == 0) {
			// Poll timeout or no new data, do nothing
			continue;
		}

		if (!(fds.revents & POLLIN)) {
			// no new data
			continue;
		}

		/*
		 * Rate limiting - we don't want to congest the bus
		 */
		const auto timestamp = _node.getMonotonicTime();

		if ((timestamp - _prev_sensor_pub).toUSec() < (1000000 / MAX_RATE_HZ)) {
			usleep(100);
			continue;
		}

		_prev_sensor_pub = timestamp;

		orb_copy(ORB_ID(sensor_gyro), _gyro_sub, &gyro);
		orb_copy(ORB_ID(sensor_accel), _accel_sub, &accel);

		uavcan::equipment::ahrs::RawIMU imu_msg;
		imu_msg.gyro_raw[0] = gyro.x;
		imu_msg.gyro_raw[1] = gyro.y;
		imu_msg.gyro_raw[2] = gyro.z;
		imu_msg.gyro_integral[0] = gyro.x_integral;
		imu_msg.gyro_integral[1] = gyro.y_integral;
		imu_msg.gyro_integral[2] = gyro.z_integral;
		imu_msg.gyro_integral_dt = gyro.integral_dt;

		imu_msg.accel_raw[0] = accel.x;
		imu_msg.accel_raw[1] = accel.y;
		imu_msg.accel_raw[2] = accel.z;
		imu_msg.accel_integral[0] = accel.x_integral;
		imu_msg.accel_integral[1] = accel.y_integral;
		imu_msg.accel_integral[2] = accel.z_integral;
		imu_msg.accel_integral_dt = accel.integral_dt;
		(void)_uavcan_pub_imu.broadcast(imu_msg);

		for(uint8_t i = 0; i < group_count; i++) {
			mag_updated = false;
			orb_check(_mag_sub[i], &mag_updated);
			//printf("MAG DATA: %d %d %.6f %.6f %.6f\n", i, _mag_priority[i], (double)mag.x, (double)mag.y, (double)mag.z);
			if (mag_updated && (_mag_priority[i] == ORB_PRIO_MAX)) {
				orb_copy(ORB_ID(sensor_mag), _mag_sub[i], &mag);
					uavcan::equipment::ahrs::MagneticFieldStrength mag_msg;
					rotate_3f(ROTATION_ROLL_180, mag.x, mag.y, mag.z);
					mag_msg.magnetic_field_ga[0] = mag.x;
					mag_msg.magnetic_field_ga[1] = mag.y;
					mag_msg.magnetic_field_ga[2] = mag.z;
					(void)_uavcan_pub_mag.broadcast(mag_msg);
				}
		}

		orb_check(_baro_sub, &baro_updated);

		if (baro_updated) {
			orb_copy(ORB_ID(sensor_baro), _baro_sub, &baro);
			uavcan::equipment::air_data::StaticTemperature temp_msg;
			uavcan::equipment::air_data::StaticPressure press_msg;
			temp_msg.static_temperature = baro.temperature;
			press_msg.static_pressure = baro.pressure * 100.0f;
			(void)_uavcan_pub_pressure.broadcast(press_msg);
			(void)_uavcan_pub_temperature.broadcast(temp_msg);
		}

		orb_check(_rcin_sub, &rcin_updated);

		if (rcin_updated) {
			orb_copy(ORB_ID(input_rc), _rcin_sub, &rcin);
			uavcan::equipment::rc::Radioin rcin_msg;
			for(int i = 0; i < rcin.channel_count; i++) {
				rcin_msg.channel.push_back(static_cast<int>(rcin.values[i]));
			}
			(void)_uavcan_pub_rcin.broadcast(rcin_msg);
		}

		orb_check(_rf_sub, &rf_updated);

		if (rf_updated) {
			orb_copy(ORB_ID(distance_sensor), _rf_sub, &rf);
			uavcan::equipment::range_sensor::Measurement rf_msg;
			rf_msg.range = rf.current_distance;
			(void)_uavcan_pub_rf.broadcast(rf_msg);
		}
	}

	exit(0);
}


int UavcanSensorPub::start(uavcan::INode &node)
{
	/*
	 * Node init
	 */
	UavcanSensorPub::_instance = new UavcanSensorPub(node);

	if (_instance == nullptr) {
		warnx("Out of memory");
		return -1;
	}


	/* start the task */
	static auto run_trampoline = [](int, char *[]) {return UavcanSensorPub::_instance->update();};
	UavcanSensorPub::_instance->_control_task = px4_task_spawn_cmd("sensors_pub",
			SCHED_DEFAULT,
			SCHED_PRIORITY_MAX - 15,
			6000,
			(px4_main_t)run_trampoline,
			nullptr);

	if (UavcanSensorPub::_instance->_control_task < 0) {
		PX4_WARN("task start failed");
		return -errno;
	}

	param_get(param_find("CAL_MAG0_ID"), &_mag_primary_id);
	return OK;
}