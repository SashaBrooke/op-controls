#include "comms/command.h"
#include "sensors/as5600.h"
#include "gimbal/gimbal_configuration.h"
#include "software_configuration.h"

static void handle_stream_cmd(const opcomms_intcomm_StreamCommand *sc,
                              gimbal_t *gimbal,
                              opcomms_intcomm_Ack *ack) {
    if (sc->has_stream_rate_hz) {
        if (sc->stream_rate_hz < GIMBAL_MIN_STREAM_RATE_HZ ||
            sc->stream_rate_hz > GIMBAL_MAX_STREAM_RATE_HZ) {
            ack->code = opcomms_intcomm_AckCode_STATUS_INVALID_DATA;
            return;
        }
        gimbal->state.stream_rate = (uint16_t)sc->stream_rate_hz;
    }
    if (sc->has_global_streaming) gimbal->state.streaming        = sc->global_streaming;
    if (sc->has_pan_pos_stream)   gimbal->state.pan_pos_stream   = sc->pan_pos_stream;
    if (sc->has_pan_pid_stream)   gimbal->state.pan_pid_stream   = sc->pan_pid_stream;
    if (sc->has_pan_motor_stream) gimbal->state.pan_motor_stream = sc->pan_motor_stream;
}

static void handle_system_action(opcomms_intcomm_SystemAction action,
                                 gimbal_t *gimbal,
                                 opcomms_intcomm_Ack *ack) {
    switch (action) {
        case opcomms_intcomm_SystemAction_ACTION_GIMBAL_FREE:
            gimbal->state.gimbal_mode = GIMBAL_MODE_FREE;
            break;

        case opcomms_intcomm_SystemAction_ACTION_GIMBAL_ARM: {
            if (gimbal->state.pan_lower_limit == GIMBAL_DEFAULT_UNSET_ROM ||
                gimbal->state.pan_upper_limit == GIMBAL_DEFAULT_UNSET_ROM) {
                ack->code = opcomms_intcomm_AckCode_STATUS_UNSET_LIMITS;
                return;
            }
            uint16_t raw = as5600_get_raw_angle((as5600_t *)&gimbal->engine.pan_encoder);
            bool inLimits = (gimbal->state.pan_lower_limit < gimbal->state.pan_upper_limit)
                ? (raw >= gimbal->state.pan_lower_limit && raw <= gimbal->state.pan_upper_limit)
                : (raw >= gimbal->state.pan_lower_limit || raw <= gimbal->state.pan_upper_limit);

            if (!inLimits) {
                ack->code = opcomms_intcomm_AckCode_STATUS_OUT_OF_LIMITS;
                return;
            }
            gimbal->state.pan_position_setpoint = (float)raw;
            gimbal->state.gimbal_mode = GIMBAL_MODE_ARMED;
            break;
        }

        case opcomms_intcomm_SystemAction_ACTION_CONFIG_SAVE:
            if (gimbal->state.gimbal_mode != GIMBAL_MODE_FREE) {
                ack->code = opcomms_intcomm_AckCode_STATUS_OPERATION_FAILED;
                return;
            }
            gimbal->config.pan_position_controller = gimbal->engine.pan_position_controller;
            save_gimbal_configuration(&gimbal->config);
            gimbal->state.saved_configuration = true;
            break;

        case opcomms_intcomm_SystemAction_ACTION_CONFIG_READ: {
            ack->which_response_data = opcomms_intcomm_Ack_config_data_tag;
            opcomms_intcomm_GimbalConfigData *cfg = &ack->response_data.config_data;
            volatile pid_controller_t *saved_pid = &gimbal->config.pan_position_controller;

            cfg->has_pan_pid = true;
            cfg->pan_pid.has_kp          = true; cfg->pan_pid.kp          = saved_pid->Kp;
            cfg->pan_pid.has_ki          = true; cfg->pan_pid.ki          = saved_pid->Ki;
            cfg->pan_pid.has_kd          = true; cfg->pan_pid.kd          = saved_pid->Kd;
            cfg->pan_pid.has_tau         = true; cfg->pan_pid.tau         = saved_pid->tau;
            cfg->pan_pid.has_out_lim_min = true; cfg->pan_pid.out_lim_min = saved_pid->outLimMin;
            cfg->pan_pid.has_out_lim_max = true; cfg->pan_pid.out_lim_max = saved_pid->outLimMax;
            cfg->pan_pid.has_int_lim_min = true; cfg->pan_pid.int_lim_min = saved_pid->intLimMin;
            cfg->pan_pid.has_int_lim_max = true; cfg->pan_pid.int_lim_max = saved_pid->intLimMax;
            break;
        }

        case opcomms_intcomm_SystemAction_ACTION_GIMBAL_READ: {
            ack->which_response_data = opcomms_intcomm_Ack_state_data_tag;
            opcomms_intcomm_GimbalStateData *st = &ack->response_data.state_data;

            st->mode = (gimbal->state.gimbal_mode == GIMBAL_MODE_ARMED)
                ? opcomms_intcomm_GimbalMode_GIMBAL_MODE_ARMED
                : opcomms_intcomm_GimbalMode_GIMBAL_MODE_FREE;
            st->saved_configuration    = gimbal->state.saved_configuration;
            st->pan_setpoint           = gimbal->state.pan_position_setpoint;
            st->pan_current_angle      = gimbal->state.pan_position;
            st->has_pan_limits         = true;
            st->pan_limits.lower_limit = gimbal->state.pan_lower_limit;
            st->pan_limits.upper_limit = gimbal->state.pan_upper_limit;
            st->pan_limits.is_set      = (gimbal->state.pan_lower_limit != GIMBAL_DEFAULT_UNSET_ROM);
            st->streaming_global       = gimbal->state.streaming;
            st->stream_rate_hz         = gimbal->state.stream_rate;
            break;
        }

        default:
            ack->code = opcomms_intcomm_AckCode_STATUS_INVALID_DATA;
            break;
    }
}

static void handle_setpoint_cmd(const opcomms_intcomm_SetpointCommand *sp,
                                gimbal_t *gimbal,
                                opcomms_intcomm_Ack *ack) {
    if (sp->axis != opcomms_intcomm_Axis_AXIS_PAN ||
        sp->setpoint < AS5600_RAW_ANGLE_MIN ||
        sp->setpoint > AS5600_RAW_ANGLE_MAX) {
        ack->code = opcomms_intcomm_AckCode_STATUS_INVALID_DATA;
        return;
    }

    bool outOfLimits = (gimbal->state.pan_lower_limit < gimbal->state.pan_upper_limit)
        ? (sp->setpoint < gimbal->state.pan_lower_limit || sp->setpoint > gimbal->state.pan_upper_limit)
        : (sp->setpoint > gimbal->state.pan_upper_limit && sp->setpoint < gimbal->state.pan_lower_limit);

    if (outOfLimits) {
        ack->code = opcomms_intcomm_AckCode_STATUS_OUT_OF_LIMITS;
        return;
    }
    gimbal->state.pan_position_setpoint = sp->setpoint;
}

static void handle_pid_set_cmd(const opcomms_intcomm_PidSetCommand *pc,
                               gimbal_t *gimbal,
                               opcomms_intcomm_Ack *ack) {
    if (pc->axis != opcomms_intcomm_Axis_AXIS_PAN || !pc->has_params) {
        ack->code = opcomms_intcomm_AckCode_STATUS_INVALID_DATA;
        return;
    }

    const opcomms_intcomm_PidParams *p = &pc->params;
    volatile pid_controller_t *ctrl = &gimbal->engine.pan_position_controller;

    if ((p->has_kp && p->kp < 0.0f) || (p->has_ki && p->ki < 0.0f) ||
        (p->has_kd && p->kd < 0.0f) || (p->has_tau && p->tau < 0.0f)) {
        ack->code = opcomms_intcomm_AckCode_STATUS_INVALID_DATA;
        return;
    }

    float nextOutMin = p->has_out_lim_min ? p->out_lim_min : ctrl->outLimMin;
    float nextOutMax = p->has_out_lim_max ? p->out_lim_max : ctrl->outLimMax;
    float nextIntMin = p->has_int_lim_min ? p->int_lim_min : ctrl->intLimMin;
    float nextIntMax = p->has_int_lim_max ? p->int_lim_max : ctrl->intLimMax;

    if (nextOutMin > nextOutMax || nextIntMin > nextIntMax) {
        ack->code = opcomms_intcomm_AckCode_STATUS_INVALID_DATA;
        return;
    }

    if (p->has_kp)          ctrl->Kp        = p->kp;
    if (p->has_ki)          ctrl->Ki        = p->ki;
    if (p->has_kd)          ctrl->Kd        = p->kd;
    if (p->has_tau)         ctrl->tau       = p->tau;
    if (p->has_out_lim_min) ctrl->outLimMin = p->out_lim_min;
    if (p->has_out_lim_max) ctrl->outLimMax = p->out_lim_max;
    if (p->has_int_lim_min) ctrl->intLimMin = p->int_lim_min;
    if (p->has_int_lim_max) ctrl->intLimMax = p->int_lim_max;

    gimbal->state.saved_configuration = false;
}

static void handle_pid_revert_cmd(const opcomms_intcomm_PidRevertCommand *rev,
                                  gimbal_t *gimbal,
                                  opcomms_intcomm_Ack *ack) {
    if (rev->axis != opcomms_intcomm_Axis_AXIS_PAN) {
        ack->code = opcomms_intcomm_AckCode_STATUS_INVALID_DATA;
        return;
    }
    gimbal->engine.pan_position_controller = gimbal->config.pan_position_controller;
}

void command_dispatch(const opcomms_intcomm_Command *cmd,
                      gimbal_t *gimbal,
                      opcomms_intcomm_Ack *ack_out) {
    *ack_out = (opcomms_intcomm_Ack)opcomms_intcomm_Ack_init_zero;
    ack_out->packet_id = cmd->packet_id;
    ack_out->code      = opcomms_intcomm_AckCode_STATUS_OK;

    switch (cmd->which_cmd) {
        case opcomms_intcomm_Command_stream_command_tag:
            handle_stream_cmd(&cmd->cmd.stream_command, gimbal, ack_out);
            break;
        case opcomms_intcomm_Command_system_action_tag:
            handle_system_action(cmd->cmd.system_action, gimbal, ack_out);
            break;
        case opcomms_intcomm_Command_setpoint_command_tag:
            handle_setpoint_cmd(&cmd->cmd.setpoint_command, gimbal, ack_out);
            break;
        case opcomms_intcomm_Command_pid_set_command_tag:
            handle_pid_set_cmd(&cmd->cmd.pid_set_command, gimbal, ack_out);
            break;
        case opcomms_intcomm_Command_pid_revert_command_tag:
            handle_pid_revert_cmd(&cmd->cmd.pid_revert_command, gimbal, ack_out);
            break;
        default:
            ack_out->code = opcomms_intcomm_AckCode_STATUS_INVALID_DATA;
            break;
    }
}
