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

#pragma once

#include <stdint.h>

#include "sensor_bridge.hpp"
#include <px4_tasks.h>
#include <px4_getopt.h>
#include <px4_posix.h>
#include <errno.h>
#include <string.h>
#include <termios.h>

#include <uavcan/uavcan.hpp>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_hrt.h>
#include <uavcan/equipment/rc/Radioin.hpp>


class UavcanRCinBridge : public UavcanCDevSensorBridgeBase
{
public:
	static const char *const NAME;

	UavcanRCinBridge(uavcan::INode &node);

	const char *get_name() const override { return NAME; }

	int init() override;

private:
	ssize_t	read(struct file *filp, char *buffer, size_t buflen);
	int ioctl(struct file *filp, int cmd, unsigned long arg) override;

	void rcin_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::rc::Radioin> &msg);

	typedef uavcan::MethodBinder < UavcanRCinBridge *,
		void (UavcanRCinBridge::*)
		(const uavcan::ReceivedDataStructure<uavcan::equipment::rc::Radioin> &) >
		RCinCbBinder;

	uavcan::Subscriber<uavcan::equipment::rc::Radioin, RCinCbBinder> _sub_rcin;
	rc_input_values _report =  {};
};
