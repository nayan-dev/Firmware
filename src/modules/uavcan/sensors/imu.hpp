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

#pragma once

#include "sensor_bridge.hpp"
#include <drivers/drv_hrt.h>
#include <systemlib/err.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_accel.h>

#include <uavcan/equipment/ahrs/RawIMU.hpp>
class UavcanGyro;

class UavcanIMUBridge : public UavcanCDevSensorBridgeBase
{
public:
	static const char *const NAME;

	UavcanIMUBridge(uavcan::INode &node);

	const char *get_name() const override { return NAME; }

	int init() override;
	friend class UavcanGyro;

private:
	UavcanGyro *_gyro;
	ssize_t	read(struct file *filp, char *buffer, size_t buflen);
	int ioctl(struct file *filp, int cmd, unsigned long arg) override;

	void imu_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::ahrs::RawIMU> &msg);

	typedef uavcan::MethodBinder < UavcanIMUBridge *,
		void (UavcanIMUBridge::*)
		(const uavcan::ReceivedDataStructure<uavcan::equipment::ahrs::RawIMU> &) >
		IMUCbBinder;

	uavcan::Subscriber<uavcan::equipment::ahrs::RawIMU, IMUCbBinder> _sub_imu;
	struct accel_calibration_s _accel_scale = {};

	accel_report _accel_report =  {};
};


class UavcanGyro : public UavcanCDevSensorBridgeBase
{
public:
	static const char *const NAME;

	UavcanGyro(uavcan::INode &node, UavcanIMUBridge *parent);

	const char *get_name() const override { return NAME; }

	int init() override;
	friend class UavcanIMUBridge;

private:
	UavcanIMUBridge *_parent;
	ssize_t	read(struct file *filp, char *buffer, size_t buflen);
	int ioctl(struct file *filp, int cmd, unsigned long arg) override;
	void publish_gyro(const uavcan::ReceivedDataStructure<uavcan::equipment::ahrs::RawIMU> &msg);

	struct gyro_calibration_s _gyro_scale = {};
	gyro_report _gyro_report =  {};
};