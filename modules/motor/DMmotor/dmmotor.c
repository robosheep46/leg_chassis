#include "dmmotor.h"
#include "bsp_can.h"
#include "cmsis_os2.h"
#include "memory.h"
#include "general_def.h"
#include "motor_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_dwt.h"
#include <stddef.h>
#include <stdint.h>
#include "stdio.h"

static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];
static TaskHandle_t dm_task_handle;

static uint8_t enable_flag[4] = {0};

static CANInstance dm_sender_assignment[4] = {
    [0] = {.can_handle = &hfdcan2, .txconf.Identifier = 0x01, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = 0x08, .tx_buff = {0}},
    [1] = {.can_handle = &hfdcan2, .txconf.Identifier = 0x02, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = 0x08, .tx_buff = {0}},
    [2] = {.can_handle = &hfdcan1, .txconf.Identifier = 0x03, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = 0x08, .tx_buff = {0}},
    [3] = {.can_handle = &hfdcan1, .txconf.Identifier = 0x04, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = 0x08, .tx_buff = {0}},
};

static void group_motor(DMMotorInstance *motor, CAN_Init_Config_s *config) {
    if (motor->motor_can_instace->tx_id == 0x01) {
        motor->sender_group = 0;
        motor->motor_can_instace->txconf.Identifier = 0x01;
        enable_flag[0] = 1;
        dm_sender_assignment[0].tx_id = motor->motor_can_instace->tx_id;
    } else if (motor->motor_can_instace->tx_id == 0x02) {
        motor->sender_group = 1;
        motor->motor_can_instace->txconf.Identifier = 0x02;
        enable_flag[1] = 1;
        dm_sender_assignment[1].tx_id = motor->motor_can_instace->tx_id;
    } else if (motor->motor_can_instace->tx_id == 0x03) {
        motor->sender_group = 2;
        motor->motor_can_instace->txconf.Identifier = 0x03;
        enable_flag[2] = 1;
        dm_sender_assignment[2].tx_id = motor->motor_can_instace->tx_id;
    } else if (motor->motor_can_instace->tx_id == 0x04) {
        motor->sender_group = 3;
        motor->motor_can_instace->txconf.Identifier = 0x04;
        enable_flag[3] = 1;
        dm_sender_assignment[3].tx_id = motor->motor_can_instace->tx_id;
    }
}

static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

static float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static void set_dm_motor_mode(dm_motor_mit_mode_e cmd, DMMotorInstance *motor) {
    memset(motor->motor_can_instace->tx_buff, 0xff, 7);
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd;
    can_transmit(motor->motor_can_instace, 1);
}

void dmmotor_set_control_mode(DMMotorInstance *motor, dm_motor_control_mode_e mode) {
    if ((mode == MIT_MODE && motor->mit_mode_flag && motor->switch_state == SWITCH_IDLE) ||
        (mode == POSITION_MODE && motor->position_mode_flag && motor->switch_state == SWITCH_IDLE))
        return;
    if (motor->switch_state != SWITCH_IDLE && motor->switch_target_mode == mode)
        return;

    motor->switch_state = SWITCH_SEND_CMD;
    motor->switch_target_mode = mode;
    motor->send_count = 0;
    motor->switch_retry_cnt = 0;
}

static void dmmotor_decode(CANInstance *motor_can) {
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &(motor->measure);

    DaemonReload(motor->motor_daemon);
    measure->id = (rxbuff[0]) & 0x0F;
    measure->state = (rxbuff[0]) >> 4;

    if (measure->state == 0x01) {
        uint16_t tmp;
        measure->last_position = measure->position;
        measure->last_torque = measure->torque;
        tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
        measure->position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);
        tmp = (uint16_t)((rxbuff[3] << 4) | (rxbuff[4] >> 4));
        measure->velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);
        tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
        measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);
        measure->T_Mos = (float)rxbuff[6];
        measure->T_Rotor = (float)rxbuff[7];

        if (measure->position - measure->last_position > 12)
            measure->total_round--;
        else if (measure->position - measure->last_position < -12)
            measure->total_round++;

        measure->angle_single_round = ECD_ANGLE_COEF_DM * (float)measure->position;
        measure->total_angle = measure->total_round * 360 + measure->angle_single_round;

        if (motor->motor_type == DM8009) {
            measure->real_total_angle = measure->total_angle * 4;
            float temp_angle = measure->angle_single_round * 4;
            measure->real_total_round = (int32_t)(temp_angle / 360.0f);
            measure->real_angle_single_round = temp_angle - measure->real_total_round * 360.0f;
        } else if (motor->motor_type == DM4310) {
            measure->real_total_angle = measure->total_angle * 3;
            float temp_angle = measure->angle_single_round * 3;
            measure->real_total_round = (int32_t)(temp_angle / 360.0f);
            measure->real_angle_single_round = temp_angle - measure->real_total_round * 360.0f;
        }

        if (measure->real_angle_single_round > 180.0f) {
            measure->real_angle_single_round -= 360.0f;
            measure->real_total_round++;
        } else if (measure->real_angle_single_round < -180.0f) {
            measure->real_angle_single_round += 360.0f;
            measure->real_total_round--;
        }
    } else {
        if (rxbuff[2] == 0x33) {
            uint8_t reg_id = rxbuff[3];
            uint32_t value = rxbuff[4] | (rxbuff[5] << 8) | (rxbuff[6] << 16) | (rxbuff[7] << 24);
            if (reg_id == 0x0A) {
                motor->read_control_mode = value;
                if (motor->switch_state == SWITCH_WAIT_CONFIRM && value == motor->switch_target_mode) {
                    if (motor->switch_target_mode == MIT_MODE) {
                        motor->mit_mode_flag = 1;
                        motor->position_mode_flag = 0;
                    } else {
                        motor->position_mode_flag = 1;
                        motor->mit_mode_flag = 0;
                    }
                    motor->switch_state = SWITCH_IDLE;
                }
            }
        }
    }
}

static void dmmotor_lost_callback(void *motor_ptr) {
    DMMotorInstance *motor = (DMMotorInstance *)motor_ptr;
    set_dm_motor_mode(DM_CMD_MOTOR_MODE, motor);
    motor->init_flag = 0;
    motor->init_state = INIT_IDLE;
    motor->cali_state = CALI_IDLE;
    memset(&(motor->measure), 0, sizeof(motor->measure));
}

static void dm_motor_other_error_callback(void *motor_ptr) {
    DMMotorInstance *motor = (DMMotorInstance *)motor_ptr;
    motor->other_error_flag = 1;
}

void dmmotor_cali_encoder(DMMotorInstance *motor) {
    if (motor->cali_state != CALI_IDLE) return;
    motor->cali_state = CALI_ENABLE_1;
    motor->cali_timestamp = xTaskGetTickCount();
}

DMMotorInstance *dmmotor_init(Motor_Init_Config_s *config) {
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    memset(motor, 0, sizeof(DMMotorInstance));

    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    motor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    motor->motor_type = config->motor_type;
    motor->motor_can_instace->tx_id = config->can_init_config.tx_id;
    motor->motor_can_instace->rx_id = config->can_init_config.rx_id;
    config->can_init_config.can_module_callback = dmmotor_decode;
    config->can_init_config.id = motor;
    motor->motor_can_instace = can_register(&config->can_init_config);
    group_motor(motor, &config->can_init_config);

    Daemon_Init_Config_s conf = {
        .owner_callback = dmmotor_lost_callback,
        .other_modules_error_callback = dm_motor_other_error_callback,
        .owner_id = motor,
        .reload_count = 50,
    };
    motor->motor_daemon = DaemonRegister(&conf);
    motor->control_mode = MIT_MODE;
    motor->init_flag = 0;
    motor->init_state = INIT_IDLE;
    motor->switch_state = SWITCH_IDLE;
    motor->cali_state = CALI_IDLE;

    dm_motor_instance[idx++] = motor;
    return motor;
}

void dmmotor_set_position(DMMotorInstance *motor, float position, float speed) {
    motor->motor_controller.angle_PID.Ref = position;
    motor->motor_controller.speed_PID.Ref = speed;
}

void dmmotor_set_torque(DMMotorInstance *motor, float Torque) {
    if (Torque >= 18)
        motor->motor_controller.current_PID.Ref = 18;
    else if (Torque <= -18)
        motor->motor_controller.current_PID.Ref = -18;
    else
        motor->motor_controller.current_PID.Ref = Torque;
}

void dmmotor_enable(DMMotorInstance *motor) {
    motor->stop_flag = MOTOR_ENALBED;
}

void dmmotor_stop(DMMotorInstance *motor) {
    motor->motor_controller.current_PID.Ref = 0;
    motor->stop_flag = MOTOR_STOP;
}

static void dmmotor_mit_mode(DMMotorInstance *motor) {
    motor->motor_can_instace->txconf.Identifier = motor->motor_can_instace->tx_id;
    uint16_t torque_uint;
    if (motor->other_error_flag || motor->stop_flag == MOTOR_STOP)
        torque_uint = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);
    else
        torque_uint = float_to_uint(motor->motor_controller.current_PID.Ref, DM_T_MIN, DM_T_MAX, 12);

    uint8_t *buf = motor->motor_can_instace->tx_buff;
    buf[0] = 0; buf[1] = 0; buf[2] = 0; buf[3] = 0; buf[4] = 0; buf[5] = 0;
    buf[6] = (torque_uint >> 8) & 0x0F;
    buf[7] = torque_uint & 0xFF;
    if (motor->init_flag == 1)
        can_transmit(motor->motor_can_instace, 1);
}

static void dmmotor_position_mode(DMMotorInstance *motor) {
    motor->motor_can_instace->txconf.Identifier = motor->motor_can_instace->tx_id + 0x100;
    float p_des = motor->motor_controller.angle_PID.Ref;
    float v_des = motor->motor_controller.speed_PID.Ref;
    if (motor->stop_flag == MOTOR_STOP)
        v_des = 0.0f;

    uint8_t *buf = motor->motor_can_instace->tx_buff;
    memcpy(buf,      &p_des, 4);
    memcpy(buf + 4,  &v_des, 4);
    can_transmit(motor->motor_can_instace, 1);
}

static void dmmotor_read_register(DMMotorInstance *motor, uint8_t reg) {
    uint8_t can_id_l = motor->motor_can_instace->tx_id & 0xFF;
    uint8_t can_id_h = (motor->motor_can_instace->tx_id >> 8) & 0x07;
    uint8_t data[8] = {can_id_l, can_id_h, 0x33, reg, 0,0,0,0};
    motor->motor_can_instace->txconf.Identifier = 0x7FF;
    memcpy(motor->motor_can_instace->tx_buff, data, 8);
    can_transmit(motor->motor_can_instace, 1);
}

void DMMotorTask(void *argument) {
    for (;;) {
        for (int i = 0; i < DM_MOTOR_CNT; i++) {
            DMMotorInstance *motor = dm_motor_instance[i];
            if (!motor) continue;

            // ----- 1. 校准状态机（最高优先级）-----
            if (motor->cali_state != CALI_IDLE) {
                switch (motor->cali_state) {
                    case CALI_ENABLE_1:
                        set_dm_motor_mode(DM_CMD_MOTOR_MODE, motor);
                        motor->cali_state = CALI_WAIT_1;
                        motor->cali_timestamp = xTaskGetTickCount();
                        break;
                    case CALI_WAIT_1:
                        if ((xTaskGetTickCount() - motor->cali_timestamp) >= pdMS_TO_TICKS(100))
                            motor->cali_state = CALI_SEND_CMD;
                        break;
                    case CALI_SEND_CMD:
                        set_dm_motor_mode(DM_CMD_ZERO_POSITION, motor);
                        motor->cali_state = CALI_WAIT_2;
                        motor->cali_timestamp = xTaskGetTickCount();
                        break;
                    case CALI_WAIT_2:
                        if ((xTaskGetTickCount() - motor->cali_timestamp) >= pdMS_TO_TICKS(100))
                            motor->cali_state = CALI_ENABLE_2;
                        break;
                    case CALI_ENABLE_2:
                        set_dm_motor_mode(DM_CMD_MOTOR_MODE, motor);
                        motor->cali_state = CALI_IDLE;
                        break;
                }
                continue;  // 校准期间跳过其他状态
            }

            // ----- 2. 初始化状态机 -----
            if (motor->init_flag == 0) {
                if (motor->init_state == INIT_IDLE) {
                    set_dm_motor_mode(DM_CMD_MOTOR_MODE, motor);
                    motor->init_state = INIT_SEND_CMD;
                    motor->init_timestamp = xTaskGetTickCount();
                } else if (motor->init_state == INIT_SEND_CMD) {
                    if ((xTaskGetTickCount() - motor->init_timestamp) >= pdMS_TO_TICKS(100)) {
                        motor->init_flag = 1;
                        motor->init_state = INIT_IDLE;
                    }
                }
                continue;
            }

            // ----- 3. 模式切换状态机 -----
            switch (motor->switch_state) {
                case SWITCH_IDLE:
                    if ((motor->control_mode == MIT_MODE && !motor->mit_mode_flag) ||
                        (motor->control_mode == POSITION_MODE && !motor->position_mode_flag)) {
                        dmmotor_set_control_mode(motor, motor->control_mode);
                    }
                    break;

                case SWITCH_SEND_CMD:
                    if (motor->send_count < 3) {
                        uint8_t can_id_l = motor->motor_can_instace->tx_id & 0xFF;
                        uint8_t can_id_h = (motor->motor_can_instace->tx_id >> 8) & 0x07;
                        uint8_t data[8] = {can_id_l, can_id_h, 0x55, 0x0A, motor->switch_target_mode, 0, 0, 0};
                        if (motor->switch_target_mode == POSITION_MODE) data[3] = 10;
                        motor->motor_can_instace->txconf.Identifier = 0x7FF;
                        memcpy(motor->motor_can_instace->tx_buff, data, 8);
                        can_transmit(motor->motor_can_instace, 1);

                        motor->send_count++;
                        if (motor->send_count < 3) {
                            motor->switch_state = SWITCH_WAIT_SEND;
                            motor->switch_timestamp = xTaskGetTickCount();
                        } else {
                            motor->switch_state = SWITCH_READ_REG;
                            motor->switch_timestamp = xTaskGetTickCount();
                        }
                    }
                    break;

                case SWITCH_WAIT_SEND:
                    if ((xTaskGetTickCount() - motor->switch_timestamp) >= pdMS_TO_TICKS(10)) {
                        motor->switch_state = SWITCH_SEND_CMD;
                    }
                    break;

                case SWITCH_READ_REG:
                    dmmotor_read_register(motor, 0x0A);
                    motor->switch_state = SWITCH_WAIT_CONFIRM;
                    motor->switch_timestamp = xTaskGetTickCount();
                    break;

                case SWITCH_WAIT_CONFIRM:
                    if ((xTaskGetTickCount() - motor->switch_timestamp) >= pdMS_TO_TICKS(200)) {
                        if (++motor->switch_retry_cnt < 3) {
                            motor->send_count = 0;
                            motor->switch_state = SWITCH_SEND_CMD;
                        } else {
                            motor->switch_state = SWITCH_IDLE;
                        }
                    }
                    break;
            }
        }

        // ----- 4. 所有电机就绪时才发送控制指令 -----
        uint8_t all_ready = 1;
        for (int i = 0; i < DM_MOTOR_CNT; i++) {
            DMMotorInstance *motor = dm_motor_instance[i];
            if (!motor) continue;
            if (motor->init_flag == 0 || motor->switch_state != SWITCH_IDLE || motor->cali_state != CALI_IDLE) {
                all_ready = 0;
                break;
            }
        }

        if (all_ready) {
            for (int i = 0; i < DM_MOTOR_CNT; i++) {
                DMMotorInstance *motor = dm_motor_instance[i];
                if (motor->control_mode == MIT_MODE)
                    dmmotor_mit_mode(motor);
                else if (motor->control_mode == POSITION_MODE)
                    dmmotor_position_mode(motor);
            }
        }

        osDelay(1);
    }
}

const osThreadAttr_t yaw_4310_task_attributes = {
    .name = "dm_task",
    .stack_size = 1024 * 2,
    .priority = (osPriority_t)osPriorityNormal,
};

void dmmotor_task_init(void) {
    dm_task_handle = osThreadNew(DMMotorTask, NULL, &yaw_4310_task_attributes);
}