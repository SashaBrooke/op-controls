#include <string.h>

#include "stream.h"
#include "packet.h"

#include "pb_encode.h"

size_t encode_axis_information(packet_t *packet, opcomms_extcomm_Axis axis,
	float angle_deg, opcomms_extcomm_MotorDirection motor_dir, uint32_t motor_pwm) {

	if (!packet) return 0;

	opcomms_extcomm_ExtcommPacket pkt = opcomms_extcomm_ExtcommPacket_init_zero;
	pkt.header.schema_version = opcomms_extcomm_SchemaVersion_SCHEMA_V1;
	pkt.header.pkt_id = STREAM_PACKET_ID_IGNORED;

	// Fill AxisInfo
	pkt.which_payload = opcomms_extcomm_ExtcommPacket_stream_tag;
	pkt.payload.stream.which_data = opcomms_extcomm_Stream_axis_info_tag;
	pkt.payload.stream.data.axis_info.axis = axis;
	pkt.payload.stream.data.axis_info.angle_deg = angle_deg;
	pkt.payload.stream.data.axis_info.motor_dir = motor_dir;
	pkt.payload.stream.data.axis_info.motor_pwm = motor_pwm;

	pb_ostream_t stream = pb_ostream_from_buffer(packet->data, sizeof(packet->data));
	bool ok = pb_encode(&stream, opcomms_extcomm_ExtcommPacket_fields, &pkt);
	if (!ok || stream.bytes_written > 0xFF) {
		packet->len = 0;
		return 0;
	}
	packet->len = (uint8_t)stream.bytes_written;
	return packet->len;
}
