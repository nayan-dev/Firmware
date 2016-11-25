/****************************************************************************
 *
 *   Copyright (c) 2016 Aarav Unmanned Systems Pvt. Ltd. All rights reserved.
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
 * @author Siddharth Bharat Purohit <sidbpurohit@gmail.com>
 */

#include "imu.hpp"

const char *const UavcanIMUBridge::NAME = "accel";
const char *const UavcanGyro::NAME = "gyro";

UavcanIMUBridge::UavcanIMUBridge(uavcan::INode &node) :
	UavcanCDevSensorBridgeBase("uavcan_accel", "/dev/uavcan/accel", ACCEL_BASE_DEVICE_PATH, ORB_ID(sensor_accel)),
	_sub_imu(node)
{
	_device_id.devid_s.devtype = DRV_ACC_DEVTYPE_UAVCAN;
	_gyro = new UavcanGyro(node, this);
	_accel_scale.x_scale = 1.0F;
	_accel_scale.y_scale = 1.0F;
	_accel_scale.z_scale = 1.0F;
}

UavcanGyro::UavcanGyro(uavcan::INode &node, UavcanIMUBridge *parent) :
	UavcanCDevSensorBridgeBase("uavcan_gyro", "/dev/uavcan/gyro", GYRO_BASE_DEVICE_PATH, ORB_ID(sensor_gyro)),
	_parent(parent)
{
	_device_id.devid_s.devtype = DRV_GYR_DEVTYPE_UAVCAN;
	_gyro_scale.x_scale = 1.0F;
	_gyro_scale.y_scale = 1.0F;
	_gyro_scale.z_scale = 1.0F;
}

int UavcanIMUBridge::init()
{
	int res = device::CDev::init();

	if (res < 0) {
		return res;
	}

	//initialise gyro device
	res = _gyro->init();

	if (res < 0) {
		return res;
	}

	res = _sub_imu.start(IMUCbBinder(this, &UavcanIMUBridge::imu_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

int UavcanGyro::init()
{
	int res = device::CDev::init();

	if (res < 0) {
		return res;
	}

	return 0;
}

ssize_t UavcanIMUBridge::read(struct file *filp, char *buffer, size_t buflen)
{
	static uint64_t last_read = 0;
	struct accel_report *accel_buf = reinterpret_cast<struct accel_report *>(buffer);

	/* buffer must be large enough */
	unsigned count = buflen / sizeof(struct accel_report);

	if (count < 1) {
		return -ENOSPC;
	}

	if (last_read < _accel_report.timestamp) {
		/* copy report */
		lock();
		*accel_buf = _accel_report;
		last_read = _accel_report.timestamp;
		unlock();
		return sizeof(struct accel_report);

	} else {
		/* no new data available, warn caller */
		return -EAGAIN;
	}
}

ssize_t UavcanGyro::read(struct file *filp, char *buffer, size_t buflen)
{
	static uint64_t last_read = 0;
	struct gyro_report *gyro_buf = reinterpret_cast<struct gyro_report *>(buffer);

	/* buffer must be large enough */
	unsigned count = buflen / sizeof(struct gyro_report);

	if (count < 1) {
		return -ENOSPC;
	}

	if (last_read < _gyro_report.timestamp) {
		/* copy report */
		lock();
		*gyro_buf = _gyro_report;
		last_read = _gyro_report.timestamp;
		unlock();
		return sizeof(struct gyro_report);

	} else {
		/* no new data available, warn caller */
		return -EAGAIN;
	}
}

int UavcanIMUBridge::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case SENSORIOCSQUEUEDEPTH: {
			return OK;			// Pretend that this stuff is supported to keep APM happy
		}

	case ACCELIOCSSCALE: {
			std::memcpy(&_accel_scale, reinterpret_cast<const void *>(arg), sizeof(_accel_scale));
			return 0;
		}

	case ACCELIOCGSCALE: {
			std::memcpy(reinterpret_cast<void *>(arg), &_accel_scale, sizeof(_accel_scale));
			return 0;
		}

	case ACCELIOCSELFTEST: {
			return 0;           // Nothing to do
		}

	case ACCELIOCGEXTERNAL: {
			return 1;           // declare it external rise it's priority and to allow for correct orientation compensation
		}

	case ACCELIOCSSAMPLERATE: {
			return 0;           // Pretend that this stuff is supported to keep the sensor app happy
		}

	case ACCELIOCGSAMPLERATE:
	case ACCELIOCSRANGE:
	case ACCELIOCGRANGE:
	case ACCELIOCSLOWPASS:
	case ACCELIOCGLOWPASS: {
			return -EINVAL;
		}

	default: {
			return CDev::ioctl(filp, cmd, arg);
		}
	}
}

int UavcanGyro::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case SENSORIOCSQUEUEDEPTH: {
			return OK;			// Pretend that this stuff is supported to keep APM happy
		}

	case GYROIOCSSCALE: {
			std::memcpy(&_gyro_scale, reinterpret_cast<const void *>(arg), sizeof(_gyro_scale));
			return 0;
		}

	case GYROIOCGSCALE: {
			std::memcpy(reinterpret_cast<void *>(arg), &_gyro_scale, sizeof(_gyro_scale));
			return 0;
		}

	case GYROIOCSELFTEST: {
			return 0;           // Nothing to do
		}

	case GYROIOCGEXTERNAL: {
			return 1;           // declare it external rise it's priority and to allow for correct orientation compensation
		}

	case GYROIOCSSAMPLERATE: {
			return 0;           // Pretend that this stuff is supported to keep the sensor app happy
		}

	case GYROIOCGSAMPLERATE:
	case GYROIOCSRANGE:
	case GYROIOCGRANGE:
	case GYROIOCSLOWPASS:
	case GYROIOCGLOWPASS: {
			return -EINVAL;
		}

	default: {
			return CDev::ioctl(filp, cmd, arg);
		}
	}
}

void UavcanIMUBridge::imu_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::ahrs::RawIMU>
				 &msg)
{
	lock();

	/*
	 * FIXME HACK
	 * This code used to rely on msg.getMonotonicTimestamp().toUSec() instead of HRT.
	 * It stopped working when the time sync feature has been introduced, because it caused libuavcan
	 * to use an independent time source (based on hardware TIM5) instead of HRT.
	 * The proper solution is to be developed.
	 */

	_accel_report.timestamp = hrt_absolute_time();
	_accel_report.integral_dt = msg.accel_integral_dt;

	_accel_report.x_integral = msg.accel_integral[0];
	_accel_report.y_integral = msg.accel_integral[1];
	_accel_report.z_integral = msg.accel_integral[2];

	_accel_report.x = msg.accel_raw[0];
	_accel_report.y = msg.accel_raw[1];
	_accel_report.z = msg.accel_raw[2];


	_gyro->_gyro_report.timestamp = hrt_absolute_time();
	_gyro->_gyro_report.integral_dt = msg.gyro_integral_dt;


	_gyro->_gyro_report.x_integral = msg.gyro_integral[0];
	_gyro->_gyro_report.y_integral = msg.gyro_integral[1];
	_gyro->_gyro_report.z_integral = msg.gyro_integral[2];

	_gyro->_gyro_report.x = msg.gyro_raw[0];
	_gyro->_gyro_report.y = msg.gyro_raw[1];
	_gyro->_gyro_report.z = msg.gyro_raw[2];


	unlock();

	publish(msg.getSrcNodeID().get(), &_accel_report);
	_gyro->publish_gyro(msg);
}

void UavcanGyro::publish_gyro(const uavcan::ReceivedDataStructure<uavcan::equipment::ahrs::RawIMU>
			      &msg)
{
	publish(msg.getSrcNodeID().get(), &_gyro_report);
}
