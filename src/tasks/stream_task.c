#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "pico/stdlib.h"

#include "pb_encode.h"
#include "extcomm.pb.h"

#include "tasks/stream_task.h"
#include "gimbal/gimbal.h"
#include "task_context.h"
#include "hardware_configuration.h"
#include "comms/packet.h"

void stream_task(void *pvParameters) {
    task_context_t *ctx = (task_context_t *)pvParameters;
    gimbal_t *gimbal = ctx->gimbal;

    TickType_t last = xTaskGetTickCount();
    
    for (;;) {
        gpio_put(TEST_PIN, 1);
        
        // Create AxisInfo stream packet
        opcomms_extcomm_ExtcommPacket extcomm_packet = opcomms_extcomm_ExtcommPacket_init_zero;
        
        extcomm_packet.has_header = true;
        extcomm_packet.header.schema_version = opcomms_extcomm_SchemaVersion_SCHEMA_V1;
        
        extcomm_packet.which_payload = opcomms_extcomm_ExtcommPacket_stream_tag;
        extcomm_packet.payload.stream.which_data = opcomms_extcomm_Stream_axis_info_tag;
        
        // Populate with pan axis data from gimbal state
        opcomms_extcomm_AxisInfo *axis_info = &extcomm_packet.payload.stream.data.axis_info;
        axis_info->axis = opcomms_extcomm_Axis_AXIS_PAN;
        axis_info->angle_deg = gimbal->state.pan_position;
        axis_info->motor_dir = (gimbal->state.pan_motor_dir == 0) ?
            opcomms_extcomm_MotorDirection_MOTOR_DIR_CW :
            opcomms_extcomm_MotorDirection_MOTOR_DIR_CCW;
        
        // Package into packet
        packet_t packet = {0};

        packet.data[0] = PACKET_MARKER; // Add 0xAA sync marker at the start

        pb_ostream_t stream = pb_ostream_from_buffer(&packet.data[PACKET_MARKER_SIZE],
                                                    MAX_PACKET_SIZE - PACKET_MARKER_SIZE);

        if (pb_encode_delimited(&stream, opcomms_extcomm_ExtcommPacket_fields, &extcomm_packet)) {
            packet.len = stream.bytes_written + PACKET_MARKER_SIZE;

            // Send packet to TX queue
            xQueueSend(ctx->txQueue, &packet, 0);
        }

        gpio_put(TEST_PIN, 0);

        vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
    }
}
