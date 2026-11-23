#ifndef COMMS_COMMAND_H
#define COMMS_COMMAND_H

#include "intcomm.pb.h"
#include "gimbal/gimbal.h"

void command_dispatch(const opcomms_intcomm_Command *cmd,
                      gimbal_t *gimbal,
                      opcomms_intcomm_Ack *ack_out);

#endif // COMMS_COMMAND_H
