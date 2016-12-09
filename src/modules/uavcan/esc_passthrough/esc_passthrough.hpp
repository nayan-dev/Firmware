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
 * @file esc_passthrough.hpp
 *
 * @author Siddharth Bharat Purohit <sidbpurohit@gmail.com>
 */

#pragma once


#include <uavcan/uavcan.hpp>
#include <drivers/device/device.h>
#include <uavcan/equipment/esc/OBCRawActuator.hpp>

#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_controls.h>

#include <drivers/drv_orb_dev.h>


#define UAVCAN_SENSOR_TRANSFER_PRIORITY 6
#define MAX_RATE_HZ 1000

class UavcanESCPassthrough : public device::CDev
{
public:
	UavcanESCPassthrough(uavcan::INode &node) :
		device::CDev("uavcan_esc_out", "/dev/uavcan/esc_out"),
		_control_mode_sub(orb_subscribe(ORB_ID(vehicle_control_mode))),
		_actuator_controls_pub(nullptr),
		_offboard_control_mode_pub(nullptr),
		_sub_esc(node)
	{
		_device_id.devid_s.bus_type = DeviceBusType_UAVCAN;
		_device_id.devid_s.bus = 0;
	}


	static int start(uavcan::INode &node);

	static UavcanESCPassthrough *instance() { return _instance; }

private:
	int	_control_task = -1;		// task handle for task
	struct vehicle_control_mode_s _control_mode;
	int 	_control_mode_sub;
	orb_advert_t _actuator_controls_pub;
	orb_advert_t _offboard_control_mode_pub;
	int32_t _pwm_disarmed;
	static int _sp_man_sub;
	static struct manual_control_setpoint_s sp_man;		///< the current manual control setpoint
	static int _pwm_fd;
	bool _task_should_exit = false;
	static UavcanESCPassthrough *_instance;
	void esc_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::OBCRawActuator> &msg);
	int failsafe_loop();
	typedef uavcan::MethodBinder < UavcanESCPassthrough *,
		void (UavcanESCPassthrough::*)
		(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::OBCRawActuator> &) >
		ESCCbBinder;

	uavcan::Subscriber<uavcan::equipment::esc::OBCRawActuator, ESCCbBinder> _sub_esc;
};
