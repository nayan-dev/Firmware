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
 * @file sensor_pub.hpp
 *
 * @author Siddharth Bharat Purohit <sidbpurohit@gmail.com>
 */

#pragma once

#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/distance_sensor.h>
#include <uavcan/uavcan.hpp>
#include <uavcan/equipment/air_data/StaticPressure.hpp>
#include <uavcan/equipment/air_data/StaticTemperature.hpp>
#include <uavcan/equipment/ahrs/RawIMU.hpp>
#include <uavcan/equipment/ahrs/MagneticFieldStrength.hpp>
#include <uavcan/equipment/rc/Radioin.hpp>
#include <uavcan/equipment/range_sensor/Measurement.hpp>


#define UAVCAN_SENSOR_TRANSFER_PRIORITY 6
#define MAX_RATE_HZ 1000
#define SENSOR_COUNT_MAX                3

class UavcanSensorPub
{
public:
	UavcanSensorPub(uavcan::INode &node);
	~UavcanSensorPub();

	int init();

	static int start(uavcan::INode &node);
	static UavcanSensorPub *instance() { return _instance; }
private:

	typedef uavcan::MethodBinder<UavcanSensorPub *, void (UavcanSensorPub::*)(const uavcan::TimerEvent &)>
	TimerCbBinder;
	int	_control_task = -1;		// task handle for task
	int	_baro_sub = -1;
	int _mag_sub[SENSOR_COUNT_MAX];
	int _gyro_sub = -1;
	int _accel_sub = -1;
	int _rcin_sub = -1;
	int _rf_sub = -1;
	int _sens_sub  = -1;
	uint8_t _mag_priority[SENSOR_COUNT_MAX];

	bool _task_should_exit = false;
	static int32_t _mag_primary_id;
	int update();
	static UavcanSensorPub	*_instance;			///< singleton pointer
	/*
	 * libuavcan related things
	 */
	uavcan::MonotonicTime							_prev_sensor_pub;   ///< rate limiting
	uavcan::INode									&_node;
	uavcan::Publisher<uavcan::equipment::air_data::StaticPressure>			_uavcan_pub_pressure;
	uavcan::Publisher<uavcan::equipment::air_data::StaticTemperature>			_uavcan_pub_temperature;
	uavcan::Publisher<uavcan::equipment::ahrs::MagneticFieldStrength>			_uavcan_pub_mag;
	uavcan::Publisher<uavcan::equipment::ahrs::RawIMU>			_uavcan_pub_imu;
	uavcan::Publisher<uavcan::equipment::rc::Radioin>			_uavcan_pub_rcin;
	uavcan::Publisher<uavcan::equipment::range_sensor::Measurement>			_uavcan_pub_rf;
	uavcan::TimerEventForwarder<TimerCbBinder>				_orb_timer;

};
