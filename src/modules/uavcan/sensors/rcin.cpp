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

#include "rcin.hpp"

const char *const UavcanRCinBridge::NAME = "rcin";

UavcanRCinBridge::UavcanRCinBridge(uavcan::INode &node) :
	UavcanCDevSensorBridgeBase("uavcan_rcin", "/dev/uavcan/rcin", RC_INPUT0_DEVICE_PATH, ORB_ID(input_rc)),
	_sub_rcin(node)
{

}

int UavcanRCinBridge::init()
{
	int res = device::CDev::init();

	if (res < 0) {
		return res;
	}

	res = _sub_rcin.start(RCinCbBinder(this, &UavcanRCinBridge::rcin_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

ssize_t UavcanRCinBridge::read(struct file *filp, char *buffer, size_t buflen)
{
	static uint64_t last_read = 0;
	struct rc_input_values *rcin_buf = reinterpret_cast<struct rc_input_values *>(buffer);

	/* buffer must be large enough */
	unsigned count = buflen / sizeof(struct rc_input_values);

	if (count < 1) {
		return -ENOSPC;
	}

	if (last_read < _report.timestamp) {
		/* copy report */
		lock();
		*rcin_buf = _report;
		last_read = _report.timestamp;
		unlock();
		return sizeof(struct rc_input_values);

	} else {
		/* no new data available, warn caller */
		return -EAGAIN;
	}
}


int UavcanRCinBridge::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	return OK;	//Not Implemented
}

void UavcanRCinBridge::rcin_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::rc::Radioin>
	 &msg)
{
	lock();
	_report.timestamp = hrt_absolute_time();

	_report.timestamp_last_signal = _report.timestamp;

	_report.channel_count = msg.channel.size();

	_report.rc_failsafe = false;

	_report.rc_lost = false;

	_report.rc_lost_frame_count = 0;

	_report.rc_total_frame_count = 1;

	_report.rc_ppm_frame_length = 0;

	_report.input_source = input_rc_s::RC_INPUT_SOURCE_UAVCAN;

	_report.rssi = RC_INPUT_RSSI_MAX;

	for(int i = 0; i < msg.channel.size(); i++) {
		_report.values[i] = msg.channel[i];
	}
	unlock();

	publish(msg.getSrcNodeID().get(), &_report);
}