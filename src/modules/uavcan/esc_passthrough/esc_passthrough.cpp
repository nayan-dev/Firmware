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
 * @file esc_passthrough.cpp
 *
 * @author Siddharth Bharat Purohit <sidbpurohit@gmail.com>
 */
#include <px4_defines.h>
#include <drivers/drv_hrt.h>
#include "esc_passthrough.hpp"
#include <drivers/drv_pwm_output.h>
#include <uORB/topics/manual_control_setpoint.h>

UavcanESCPassthrough *UavcanESCPassthrough::_instance = nullptr;
int UavcanESCPassthrough::_sp_man_sub = -1;
struct manual_control_setpoint_s UavcanESCPassthrough::sp_man = {};		///< the current manual control setpoint
int UavcanESCPassthrough::_pwm_fd = -1;


void UavcanESCPassthrough::esc_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::OBCRawActuator>
	 &msg)
{

	struct offboard_control_mode_s offboard_control_mode = {};

	struct actuator_controls_s actuator_controls = {};

	bool values_finite =
		PX4_ISFINITE(msg.act_ctrl[0]) &&
		PX4_ISFINITE(msg.act_ctrl[1]) &&
		PX4_ISFINITE(msg.act_ctrl[2]) &&
		PX4_ISFINITE(msg.act_ctrl[3]) &&
		PX4_ISFINITE(msg.act_ctrl[4]) &&
		PX4_ISFINITE(msg.act_ctrl[5]) &&
		PX4_ISFINITE(msg.act_ctrl[6]) &&
		PX4_ISFINITE(msg.act_ctrl[7]);

	if (values_finite) {

		/* ignore all since we are setting raw actuators here */
		offboard_control_mode.ignore_thrust             = true;
		offboard_control_mode.ignore_attitude           = true;
		offboard_control_mode.ignore_bodyrate           = true;
		offboard_control_mode.ignore_position           = true;
		offboard_control_mode.ignore_velocity           = true;
		offboard_control_mode.ignore_acceleration_force = true;

		offboard_control_mode.timestamp = hrt_absolute_time();

		if (_offboard_control_mode_pub == nullptr) {
			_offboard_control_mode_pub = orb_advertise(ORB_ID(offboard_control_mode), &offboard_control_mode);

		} else {
			orb_publish(ORB_ID(offboard_control_mode), _offboard_control_mode_pub, &offboard_control_mode);
		}


		/* If we are in offboard control mode, publish the actuator controls */
		bool updated;
		orb_check(_control_mode_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
		}
		static uint64_t last_time;
		if ((hrt_absolute_time() - last_time) > 1000000) {
			printf("Act: %.6f %.6f %.6f %.6f\n", (double)msg.act_ctrl[0],(double)msg.act_ctrl[1],(double)msg.act_ctrl[2],(double)msg.act_ctrl[3]);
			last_time = hrt_absolute_time();
		}
		if (_control_mode.flag_control_offboard_enabled) {

			actuator_controls.timestamp = hrt_absolute_time();

			/* Set duty cycles for the servos in actuator_controls_0 */
			for (size_t i = 0; i < 8; i++) {
				actuator_controls.control[i] = msg.act_ctrl[i];
			}

			if (_actuator_controls_pub == nullptr) {
				_actuator_controls_pub = orb_advertise(ORB_ID(actuator_controls_0), &actuator_controls);

			} else {
				orb_publish(ORB_ID(actuator_controls_0), _actuator_controls_pub, &actuator_controls);
			}
		}
	}

	//int ret = OK;
	//if(_sp_man_sub == -1) {
	//	_sp_man_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	//	memset(&sp_man, 0, sizeof(sp_man));
	//}
	//if(_pwm_fd < 0) {
	//	_pwm_fd = ::open(PWM_OUTPUT0_DEVICE_PATH, 0);
	//}
	//
	//if (_pwm_fd < 0) {
	//	PX4_ERR("can't open %s err: %s", PWM_OUTPUT0_DEVICE_PATH, strerror(errno));
	//	return;
	//}
	//
	//if (msg.cmd.size() < 4) {
	//	// Without valid argument, set all channels to PWM_DISARMED
	//	for (unsigned i = 0; i < PWM_OUTPUT_MAX_CHANNELS; i++) {
	//		printf("Fault Out: %d\n", _pwm_disarmed);
	//		ret = ::ioctl(_pwm_fd, PWM_SERVO_SET(i), _pwm_disarmed);
	//
	//		if (ret != OK) {
	//			PX4_ERR("PWM_SERVO_SET(%d) %s %d", i, strerror(errno), ret);
	//		}
	//	}
	//
	//} else {
	//	bool updated = false;
	//
	//	orb_check(_sp_man_sub, &updated);
	//
	//	if (updated) {
	//		orb_copy(ORB_ID(manual_control_setpoint), _sp_man_sub, &sp_man);
	//	}
	//	//printf("%d %d %d %d\n", msg.cmd[0],msg.cmd[1],msg.cmd[2],msg.cmd[3]);
	//	for (unsigned i = 0; i < msg.cmd.size(); i++) {
	//		if (!isnan(msg.cmd[i])) {
	//			float pwm = msg.cmd[i];
	//			if(pwm > _pwm_disarmed && sp_man.kill_switch != manual_control_setpoint_s::SWITCH_POS_ON) {
	//				uint16_t pwm_out = msg.cmd[i];
	//				//printf("%d Armed Out: %d %d\n", i, pwm_out, msg.cmd[i]);
	//				ret = ::ioctl(_pwm_fd, PWM_SERVO_SET(i), pwm_out);
	//			} else {
	//				//printf("%d Disarmed Out: %d\n", i, _pwm_disarmed);
	//				ret = ::ioctl(_pwm_fd, PWM_SERVO_SET(i), _pwm_disarmed);
	//			}
	//
	//			if (ret != OK) {
	//				PX4_ERR("PWM_SERVO_SET(%d) %s %d", i, strerror(errno), ret);
	//			}
	//		}
	//	}
	//}
}

int UavcanESCPassthrough::failsafe_loop()
{

	int fs_man_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	struct manual_control_setpoint_s fs_man = {};
	px4_pollfd_struct_t fds;
	fds.fd = fs_man_sub;
	fds.events = POLLIN;

	while(!_task_should_exit) {
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
		orb_copy(ORB_ID(manual_control_setpoint), fs_man_sub, &fs_man);
		if(fs_man.kill_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
			for(uint16_t i = 0; i < 6; i++) {
				ret = up_pwm_servo_set( i, _pwm_disarmed);
			}
		}
	}
	return 0;
}

int UavcanESCPassthrough::start(uavcan::INode &node)
{
	/*
	 * Node init
	 */
	UavcanESCPassthrough::_instance = new UavcanESCPassthrough(node);

	if (_instance == nullptr) {
		warnx("Out of memory");
		return -1;
	}


	/* start the task */
//	_pwm_fd = ::open(PWM_OUTPUT0_DEVICE_PATH, 0);
//
//	if (_pwm_fd < 0) {
//		PX4_ERR("can't open %s err: %s", PWM_OUTPUT0_DEVICE_PATH, strerror(errno));
//		return _instance->_pwm_fd;
//	}
//
	param_get(param_find("PWM_DISARMED"), &_instance->_pwm_disarmed);
//
//	for (unsigned i = 0; i < 6; i++) {
//		int ret = ::ioctl(_instance->_pwm_fd, PWM_SERVO_SET(i), _instance->_pwm_disarmed);
//		if (ret != OK) {
//			PX4_ERR("PWM_SERVO_SET(%d) %s %d", i, strerror(errno), ret);
//		}
//		printf("PWM FD CB %d\n", _instance->_pwm_fd);
//	}

	/* start the failsafe task */
	//static auto run_trampoline = [](int, char *[]) {return UavcanESCPassthrough::_instance->failsafe_loop();};
	//UavcanESCPassthrough::_instance->_control_task = px4_task_spawn_cmd("failsafe_loop",
	//		SCHED_DEFAULT,
	//		SCHED_PRIORITY_MAX - 15,
	//		2000,
	//		(px4_main_t)run_trampoline,
	//		nullptr);
	//if (UavcanESCPassthrough::_instance->_control_task < 0) {
	//	PX4_WARN("task start failed");
	//	return -errno;
	//}
	int res = _instance->_sub_esc.start(ESCCbBinder(_instance, &UavcanESCPassthrough::esc_sub_cb));

	if (res < 0) {
		return res;
	}

	return OK;
}