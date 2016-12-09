/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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
 * @file esc.cpp
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include "esc.hpp"
#include <systemlib/err.h>


#define MOTOR_BIT(x) (1<<(x))

UavcanEscController::UavcanEscController(uavcan::INode &node) :
	_node(node),
	_uavcan_pub_raw_cmd(node),
	_uavcan_sub_status(node),
	_orb_timer(node)
{
	_uavcan_pub_raw_cmd.setPriority(UAVCAN_COMMAND_TRANSFER_PRIORITY);

	if (_perfcnt_invalid_input == nullptr) {
		errx(1, "uavcan: couldn't allocate _perfcnt_invalid_input");
	}

	if (_perfcnt_scaling_error == nullptr) {
		errx(1, "uavcan: couldn't allocate _perfcnt_scaling_error");
	}
}

UavcanEscController::~UavcanEscController()
{
	perf_free(_perfcnt_invalid_input);
	perf_free(_perfcnt_scaling_error);
}

int UavcanEscController::init()
{
	// ESC status subscription
	int res = _uavcan_sub_status.start(StatusCbBinder(this, &UavcanEscController::esc_status_sub_cb));

	if (res < 0) {
		warnx("ESC status sub failed %i", res);
		return res;
	}

	param_get(param_find("PWM_MIN"),&_pwm_min);

	param_get(param_find("PWM_MAX"),&_pwm_max);

	// ESC status will be relayed from UAVCAN bus into ORB at this rate
	_orb_timer.setCallback(TimerCbBinder(this, &UavcanEscController::orb_pub_timer_cb));
	_orb_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000 / ESC_STATUS_UPDATE_RATE_HZ));

	return res;
}

void UavcanEscController::update_outputs(float *outputs, unsigned num_outputs)
{
	if ((outputs == nullptr) ||
	    (num_outputs > uavcan::equipment::esc::RawCommand::FieldTypes::cmd::MaxSize) ||
	    (num_outputs > esc_status_s::CONNECTED_ESC_MAX)) {
		perf_count(_perfcnt_invalid_input);
		return;
	}

	/*
	 * Rate limiting - we don't want to congest the bus
	 */
	const auto timestamp = _node.getMonotonicTime();

	if ((timestamp - _prev_cmd_pub).toUSec() < (1000000 / MAX_RATE_HZ)) {
		return;
	}

	_prev_cmd_pub = timestamp;

	/*
	 * Fill the command message
	 * If unarmed, we publish an empty message anyway
	 */
	uavcan::equipment::esc::OBCRawActuator msg;

	for (unsigned i = 0; i < num_outputs; i++) {
		if (_armed_mask & MOTOR_BIT(i)) {
			msg.act_ctrl.push_back(static_cast<int>(outputs[i]));
		} else {
			msg.act_ctrl.push_back(static_cast<unsigned>(0));
		}
	}
//	uavcan::equipment::esc::RawCommand msg;
//
//	for (unsigned i = 0; i < num_outputs; i++) {
//		if (_armed_mask & MOTOR_BIT(i)) {
//
//			float control_value = outputs[i];
//
//			/* check for invalid / disabled channels */
//			if (!isfinite(control_value)) {
//				control_value = 0;
//				continue;
//			}
//
//			int scaled = control_value * (_pwm_max - _pwm_min) / 2 + (_pwm_max + _pwm_min) / 2;
//
//			/* last line of defense against invalid inputs */
//			if (scaled < _pwm_min) {
//				scaled = _pwm_min;
//
//			} else if (scaled > _pwm_max) {
//				scaled = _pwm_max;
//			}
//
//			msg.cmd.push_back(static_cast<int>(scaled));
//
//			_esc_status.esc[i].esc_setpoint_raw = abs(static_cast<int>(scaled));
//
//		} else {
//			msg.cmd.push_back(static_cast<unsigned>(0));
//		}
//	}
//
	/*
	 * Publish the command message to the bus
	 * Note that for a quadrotor it takes one CAN frame
	 */
	(void)_uavcan_pub_raw_cmd.broadcast(msg);
}

void UavcanEscController::arm_all_escs(bool arm)
{
	if (arm) {
		_armed_mask = -1;

	} else {
		_armed_mask = 0;
	}
}

void UavcanEscController::arm_single_esc(int num, bool arm)
{
	if (arm) {
		_armed_mask = MOTOR_BIT(num);

	} else {
		_armed_mask = 0;
	}
}

void UavcanEscController::esc_status_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::Status> &msg)
{
	if (msg.esc_index < esc_status_s::CONNECTED_ESC_MAX) {
		_esc_status.esc_count = uavcan::max<int>(_esc_status.esc_count, msg.esc_index + 1);
		_esc_status.timestamp = msg.getMonotonicTimestamp().toUSec();

		auto &ref = _esc_status.esc[msg.esc_index];

		ref.esc_address = msg.getSrcNodeID().get();

		ref.esc_voltage     = msg.voltage;
		ref.esc_current     = msg.current;
		ref.esc_temperature = msg.temperature;
		ref.esc_setpoint    = msg.power_rating_pct;
		ref.esc_rpm         = msg.rpm;
		ref.esc_errorcount  = msg.error_count;
	}
}

void UavcanEscController::orb_pub_timer_cb(const uavcan::TimerEvent &)
{
	_esc_status.counter += 1;
	_esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_CAN;

	if (_esc_status_pub != nullptr) {
		(void)orb_publish(ORB_ID(esc_status), _esc_status_pub, &_esc_status);

	} else {
		_esc_status_pub = orb_advertise(ORB_ID(esc_status), &_esc_status);
	}
}
