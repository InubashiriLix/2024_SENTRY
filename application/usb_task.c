/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       usb_task.c/h
  * @brief      usb outputs the error message.usb����������?1?7
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "usb_task.h"

#include "cmsis_os.h"
#include "remote_control.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"
#include "Can_receive.h"
#include "crc8_crc16.h"
#include "detect_task.h"
#include "voltage_task.h"
#include "vision.h"

extern vision_control_t vision_control;

static void usb_printf(const char *fmt, ...);
static void get_INS_quat_data(uint8_t array);
uint8_t usb_txbuf[256];
vision_t vision_data;
extern USBD_HandleTypeDef hUsbDeviceFS;
int32_t usb_count = 0;
static void get_buller_speed(uint8_t array);
static void get_HP_data(uint8_t array);
void vision_init();
static void vision_data_update();
static void get_vision_data(void);
static void get_yaw_data(uint8_t array);
static void get_pitch_data(uint8_t array);

vision_rx_t vision_rx;
MotionRx_t motion_rx;
ProjectileTx_slow_t projectile_tx_slow;
ProjectileTx_fast_t projectile_tx_fast;

// border line of two sides
const uint8_t borderline_side = 100;

extern RC_ctrl_t rc_ctrl;
void usb_task_(void const *argument)
{

    MX_USB_DEVICE_Init();
    vision_init();
    osDelay(1000);

    while (1) {
        vision_data_update();
        get_vision_data();
        vTaskDelay(10);
    }
}

void usb_task(void const *argument)
{
    MX_USB_DEVICE_Init();
    projectile_tx_struct_init();
    osDelay(1000);

    while (1) {
        for (int i = 0; i < 9; i++) {
            projectile_tx_fast_update();
            send_projectile_tx_fast();
            vTaskDelay(10);
        }
        projectile_tx_slow_update();
        send_projectile_tx_slow();
        vTaskDelay(10);
    }
}

uint32_t system_time;
uint16_t a = 20;
static void get_vision_data(void)
{
    usb_txbuf[0] = vision_data.SOF;
    get_INS_quat_data(1);
    usb_txbuf[17] = vision_control.vision_mode;
    usb_txbuf[18] = 0xFF;
    if (vision_data.robot_state_v->robot_id > 100) {
        usb_txbuf[19] = 0;
    } else
        usb_txbuf[19] = 1;
    get_buller_speed(20);
    get_HP_data(24);
    uint32_t system_time = xTaskGetTickCount();
    usb_txbuf[39]        = system_time >> 24;
    usb_txbuf[38]        = system_time >> 16;
    usb_txbuf[37]        = system_time >> 8;
    usb_txbuf[36]        = system_time;
    get_yaw_data(40);
    get_pitch_data(44);
    append_CRC8_check_sum(usb_txbuf, sizeof(usb_txbuf));
    CDC_Transmit_FS(usb_txbuf, sizeof(usb_txbuf));
}

unsigned long size_tx_slow = sizeof(ProjectileTx_slow_t);
unsigned long size_tx_fast = sizeof(ProjectileTx_fast_t);
static void send_projectile_tx_slow()
{
    memcpy(usb_txbuf, &projectile_tx_slow, size_tx_slow);
    append_CRC8_check_sum(usb_txbuf, size_tx_slow);
    CDC_Transmit_FS(usb_txbuf, size_tx_slow);
}

static void send_projectile_tx_fast()
{
    memcpy(usb_txbuf, &projectile_tx_fast, size_tx_fast);
    append_CRC8_check_sum(usb_txbuf, size_tx_fast);
    CDC_Transmit_FS(usb_txbuf, size_tx_fast);
}

void vision_init()
{
    vision_data.INS_quat_vision = get_INS_quat_point();
    vision_data.robot_state_v   = get_robot_status_point();
    vision_data.vision_gimbal   = get_gimbal_point();
    vision_data.SOF             = 0XA5;
    vision_data.vision_mode     = CLASSIC;
}

void projectile_tx_struct_init()
{
    projectile_tx_fast_update();
    projectile_tx_slow_update();
}

static void get_INS_quat_data(uint8_t array)
{
    union refree_4_byte_t INS_data[4];
    INS_data[0].f = *vision_data.INS_quat_vision;
    INS_data[1].f = *(vision_data.INS_quat_vision + 1);
    INS_data[2].f = *(vision_data.INS_quat_vision + 2);
    INS_data[3].f = *(vision_data.INS_quat_vision + 3);

    usb_txbuf[array + 1] = INS_data[0].buf[1];
    usb_txbuf[array + 2] = INS_data[0].buf[2];
    usb_txbuf[array + 3] = INS_data[0].buf[3];

    usb_txbuf[array + 4] = INS_data[1].buf[0];
    usb_txbuf[array + 5] = INS_data[1].buf[1];
    usb_txbuf[array + 6] = INS_data[1].buf[2];
    usb_txbuf[array + 7] = INS_data[1].buf[3];

    usb_txbuf[array + 8]  = INS_data[2].buf[0];
    usb_txbuf[array + 9]  = INS_data[2].buf[1];
    usb_txbuf[array + 10] = INS_data[2].buf[2];
    usb_txbuf[array + 11] = INS_data[2].buf[3];

    usb_txbuf[array + 12] = INS_data[3].buf[0];
    usb_txbuf[array + 13] = INS_data[3].buf[1];
    usb_txbuf[array + 14] = INS_data[3].buf[2];
    usb_txbuf[array + 15] = INS_data[3].buf[3];
}

static void get_pitch_data(uint8_t array)
{
    union refree_4_byte_t pitch_data;
    pitch_data.f = vision_data.vision_gimbal->gimbal_pitch_motor.absolute_angle;

    usb_txbuf[array]     = pitch_data.buf[0];
    usb_txbuf[array + 1] = pitch_data.buf[1];
    usb_txbuf[array + 2] = pitch_data.buf[2];
    usb_txbuf[array + 3] = pitch_data.buf[3];
}

static void get_yaw_data(uint8_t array)
{
    union refree_4_byte_t yaw_data;
    yaw_data.f = vision_data.vision_gimbal->gimbal_yaw_motor.absolute_angle;

    usb_txbuf[array]     = yaw_data.buf[0];
    usb_txbuf[array + 1] = yaw_data.buf[1];
    usb_txbuf[array + 2] = yaw_data.buf[2];
    usb_txbuf[array + 3] = yaw_data.buf[3];
}
static void vision_data_update()
{
    vision_data.bullet_speed = get_bullet_speed();
}

static void projectile_tx_slow_update()
{
    projectile_tx_slow.header = 0xA5;

    projectile_tx_slow.timestamp = xTaskGetTickCount();

    projectile_tx_slow.vision_mode = CLASSIC;

    projectile_tx_slow.shoot_remote = rc_ctrl.mouse.press_r; // ????, ??????
    // FIXME: what are the one things above?

    // due to the id of robot is
    //  1????????
    //  2????????
    //  3/4/5?????????????ID 3~5???
    //  6????????
    //  7????????
    //  8?????
    //  9?????
    //  10??????
    //  11?????
    //  101????????
    //  102????????
    //  103/104/105?????????????ID 3~5???
    //  106????????
    //  107????????
    //  108?????
    //  109?????
    //  110??????
    //  111?????
    // ??????
    projectile_tx_slow.current_side_color = get_robot_id() == 0 ? 0 : get_robot_id() < 100 ? 1
                                                                                           : 2;
    // 0 -> unknown  1 -> red  2 -> blue
    if (projectile_tx_slow.current_side_color == 2) { // current side is blue
        projectile_tx_slow.enemy_1_robot_HP = get_game_robot_HP_point()->red_1_robot_HP;
        projectile_tx_slow.enemy_2_robot_HP = get_game_robot_HP_point()->red_2_robot_HP;
        projectile_tx_slow.enemy_3_robot_HP = get_game_robot_HP_point()->red_3_robot_HP;
        projectile_tx_slow.enemy_4_robot_HP = get_game_robot_HP_point()->red_4_robot_HP;
        projectile_tx_slow.enemy_5_robot_HP = get_game_robot_HP_point()->red_5_robot_HP;
        projectile_tx_slow.enemy_7_robot_HP = get_game_robot_HP_point()->red_7_robot_HP;
        projectile_tx_slow.enemy_outpost_HP = get_game_robot_HP_point()->red_outpost_HP;
        projectile_tx_slow.enemy_base_HP    = get_game_robot_HP_point()->red_base_HP;
    } else if (projectile_tx_slow.current_side_color == 1) { // current side is red
        projectile_tx_slow.enemy_1_robot_HP = get_game_robot_HP_point()->blue_1_robot_HP;
        projectile_tx_slow.enemy_2_robot_HP = get_game_robot_HP_point()->blue_2_robot_HP;
        projectile_tx_slow.enemy_3_robot_HP = get_game_robot_HP_point()->blue_3_robot_HP;
        projectile_tx_slow.enemy_4_robot_HP = get_game_robot_HP_point()->blue_4_robot_HP;
        projectile_tx_slow.enemy_5_robot_HP = get_game_robot_HP_point()->blue_5_robot_HP;
        projectile_tx_slow.enemy_7_robot_HP = get_game_robot_HP_point()->blue_7_robot_HP;
        projectile_tx_slow.enemy_outpost_HP = get_game_robot_HP_point()->blue_outpost_HP;
    }

    projectile_tx_slow.field_events = get_field_event_point()->event_data;

    projectile_tx_slow.game_type         = get_game_state_point()->game_type;
    projectile_tx_slow.game_progress     = get_game_state_point()->game_progress;
    projectile_tx_slow.state_remain_time = get_game_state_point()->stage_remain_time;
    projectile_tx_slow.sync_time_stamp   = get_game_state_point()->SyncTimeStamp;

    projectile_tx_slow.winner = get_game_result_point()->winner;

    // uint8_t *robot_buffs_ptr = (uint8_t*) get_game_robot_state_point();
    // FIXME: the robot buffers (robot_replenish_blood, shooter_cooling_acceleration,
    // robot_defense_bonus, robot_attack_bonus) are not defined in the referee,
    // but has been exists in the upper machine
    // temporary solution
    projectile_tx_slow.recovery_buff      = get_buff_musk_point()->recovery_buff;
    projectile_tx_slow.cooling_buff       = get_buff_musk_point()->cooling_buff;
    projectile_tx_slow.defence_buff       = get_buff_musk_point()->defence_buff;
    projectile_tx_slow.vulnerability_buff = get_buff_musk_point()->vulnerability_buff;
    projectile_tx_slow.attack_buff        = get_buff_musk_point()->attack_buff;

    projectile_tx_slow.x     = get_game_robot_pos_point()->x;
    projectile_tx_slow.y     = get_game_robot_pos_point()->y;
    projectile_tx_slow.angle = get_game_robot_pos_point()->angle;

    projectile_tx_slow.robot_id                        = get_robot_status_point()->robot_id;
    projectile_tx_slow.robot_level                     = get_robot_status_point()->robot_level;
    projectile_tx_slow.current_HP                      = get_robot_status_point()->current_HP;
    projectile_tx_slow.maximum_HP                      = get_robot_status_point()->maximum_HP;
    projectile_tx_slow.shooter_barrel_cooling_value    = get_robot_status_point()->shooter_barrel_cooling_value;
    projectile_tx_slow.shooter_barrel_heat_limit       = get_robot_status_point()->shooter_barrel_heat_limit;
    projectile_tx_slow.chassis_power_limit             = get_robot_status_point()->chassis_power_limit;
    projectile_tx_slow.power_management_gimbal_output  = get_robot_status_point()->power_management_gimbal_output;
    projectile_tx_slow.power_management_chassis_output = get_robot_status_point()->power_management_chassis_output;
    projectile_tx_slow.power_management_shooter_output = get_robot_status_point()->power_management_shooter_output;

    // // FIXME: the client command is not defined in the referee, but has been exists in the upper machine
    // // NOTE: temporarily set the command_target_position to (0, 0, 0)
    projectile_tx_slow.command_target_position[0] = 0;
    projectile_tx_slow.command_target_position[1] = 0;
    projectile_tx_slow.command_target_position[2] = 0;

    // // FIXME: IS there any need for the keyboard_key_pressed? We are dealing with SENTRY
    projectile_tx_slow.keyboard_key_pressed = get_remote_control_point()->key.v;

    // // FIXME: where is the command_target_robot_id?
    projectile_tx_slow.command_target_robot_id = 0;

    // // FIXME: where is the receive_target_position?
    projectile_tx_slow.receive_target_position[0] = 0;
    projectile_tx_slow.receive_target_position[1] = 0;

    // // FIXME: where is the receive_target_robot_id?
    projectile_tx_slow.receive_target_robot_id = 0;
}

static void projectile_tx_fast_update(void)
{

    projectile_tx_fast.header = 0xF8;

    const float *q_ = get_INS_quat_point();
    for (int i = 0; i < 4; i++) {
        projectile_tx_fast.q[i] = *(q_ + i * sizeof(float));
    }

    projectile_tx_fast.yaw   = get_gimbal_point()->gimbal_yaw_motor.absolute_angle;
    projectile_tx_fast.pitch = get_gimbal_point()->gimbal_pitch_motor.absolute_angle;

    projectile_tx_fast.lever_mode = rc_ctrl.rc.s[0];

    projectile_tx_fast.bullet_speed = get_bullet_speed();
}

static void get_HP_data(uint8_t array)
{
    for (int i = 0; i <= 11; i++) {
        usb_txbuf[array + 1] = 0;
    }
}
static void get_buller_speed(uint8_t array)
{
    union refree_4_byte_t INS_data;
    INS_data.f = vision_data.bullet_speed;

    usb_txbuf[array]     = INS_data.buf[0];
    usb_txbuf[array + 1] = INS_data.buf[1];
    usb_txbuf[array + 2] = INS_data.buf[2];
    usb_txbuf[array + 3] = INS_data.buf[3];
}

static void usb_printf(const char *fmt, ...)
{
    static uint8_t usb_buf[256];
    static va_list ap;
    uint16_t len = 0;

    va_start(ap, fmt);
    len = vsnprintf((char *)usb_buf, sizeof(usb_buf), fmt, ap);
    if (len > sizeof(usb_buf)) {
        len = sizeof(usb_buf); // ��?���
    }
    va_end(ap);

    CDC_Transmit_FS(usb_buf, len);
}

// ==============================  for PARITY CHECK  ==============================
// ====== ODD ======
// ???
uint8_t calculate_parity(const uint8_t *data, size_t length)
{
    uint8_t parity = 0;
    for (size_t i = 0; i < length; i++) {
        parity ^= data[i]; // ???????
    }
    return parity & 1; // ??????0 ??????1 ??????
}

uint8_t vertify_parity(const uint8_t *data, size_t length)
{
    // ?????????????????????
    uint8_t calculated_parity_code = calculate_parity(data, length - 2);

    // ???????????????
    uint8_t stored_parity_code = data[length - 1];

    // ???????
    if (calculated_parity_code == stored_parity_code) {
        return 1; // ????
    } else {
        return 0; // ????
    }
}