/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file mavlink_timesync.cpp
 * Mavlink timesync implementation.
 *
 * @author Mohammed Kabir <mhkabir98@gmail.com>
 */

#include <stdio.h>

#include <uORB/topics/timesync.h>

#include "mavlink_timesync.h"
#include "mavlink_main.h"

MavlinkTimesync::MavlinkTimesync(Mavlink *mavlink) :
	_timesync_status_pub(nullptr),
	_time_offset_avg_alpha(0.8),
	_time_offset(0),
	_mavlink(mavlink)
{
}
MavlinkTimesync::~MavlinkTimesync()
{
	if (_uavcan_parameter_value_sub >= 0) {
		orb_unsubscribe(_uavcan_parameter_value_sub);
	}

	if (_uavcan_parameter_request_pub) {
		orb_unadvertise(_uavcan_parameter_request_pub);
	}
}

unsigned
MavlinkTimesync::get_size()
{
	return MAVLINK_MSG_ID_PARAM_VALUE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
}

void
MavlinkTimesync::handle_message(const mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_TIMESYNC: {

			mavlink_timesync_t tsync = {};
			mavlink_msg_timesync_decode(msg, &tsync);

			struct time_offset_s tsync_offset = {};

			uint64_t now_ns = hrt_absolute_time() * 1000LL ;

			if (tsync.tc1 == 0) {

				mavlink_timesync_t rsync; // return timestamped sync message

				rsync.tc1 = now_ns;
				rsync.ts1 = tsync.ts1;

				mavlink_msg_timesync_send_struct(_mavlink->get_channel(), &rsync);

				return;

			} else if (tsync.tc1 > 0) {

				int64_t offset_ns = (int64_t)(tsync.ts1 + now_ns - tsync.tc1 * 2) / 2 ;
				int64_t dt = _time_offset - offset_ns;

				if (dt > 10000000LL || dt < -10000000LL) { // 10 millisecond skew
					_time_offset = offset_ns;

					// Provide a warning only if not syncing initially
					if (_time_offset != 0) {
						PX4_ERR("[timesync] Hard setting offset.");
					}

				} else {
					smooth_time_offset(offset_ns);
				}
			}

			tsync_offset.offset_ns = _time_offset ;

			if (_time_offset_pub == nullptr) {
				_time_offset_pub = orb_advertise(ORB_ID(time_offset), &tsync_offset);

			} else {
				orb_publish(ORB_ID(time_offset), _time_offset_pub, &tsync_offset);
			}

			break;
		}

	case MAVLINK_MSG_ID_SYSTEM_TIME: {
			


			break;
		}

	default:
		break;
	}
}


int
MavlinkTimesync::send_param(param_t param, int component_id)
{
	if (param == PARAM_INVALID) {
		return 1;
	}

	/* no free TX buf to send this param */
	if (_mavlink->get_free_tx_buf() < MAVLINK_MSG_ID_PARAM_VALUE_LEN) {
		return 1;
	}

	mavlink_param_value_t msg;

	/*
	 * get param value, since MAVLink encodes float and int params in the same
	 * space during transmission, copy param onto float val_buf
	 */
	if (param_get(param, &msg.param_value) != OK) {
		return 2;
	}

	msg.param_count = param_count_used();
	msg.param_index = param_get_used_index(param);

	/*
	 * coverity[buffer_size_warning : FALSE]
	 *
	 * The MAVLink spec does not require the string to be NUL-terminated if it
	 * has length 16. In this case the receiving end needs to terminate it
	 * when copying it.
	 */
	strncpy(msg.param_id, param_name(param), MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);

	/* query parameter type */
	param_type_t type = param_type(param);

	/*
	 * Map onboard parameter type to MAVLink type,
	 * endianess matches (both little endian)
	 */
	if (type == PARAM_TYPE_INT32) {
		msg.param_type = MAVLINK_TYPE_INT32_T;

	} else if (type == PARAM_TYPE_FLOAT) {
		msg.param_type = MAVLINK_TYPE_FLOAT;

	} else {
		msg.param_type = MAVLINK_TYPE_FLOAT;
	}

	/* default component ID */
	if (component_id < 0) {
		mavlink_msg_param_value_send_struct(_mavlink->get_channel(), &msg);

	} else {
		// Re-pack the message with a different component ID
		mavlink_message_t mavlink_packet;
		mavlink_msg_param_value_encode_chan(mavlink_system.sysid, component_id, _mavlink->get_channel(), &mavlink_packet, &msg);
		_mavlink_resend_uart(_mavlink->get_channel(), &mavlink_packet);
	}

	return 0;
}
