// *
//   ****************************(C) COPYRIGHT 2019 DJI****************************
//   * @file       usb_task.c/h
//   * @brief      no action.
//   * @note
//   * @history
//   *  Version    Date            Author          Modification
//   *  V1.0.0     Dec-26-2018     RM              1. done
//   *
//   @verbatim
//   ==============================================================================

//   ==============================================================================
//   @endverbatim
//   ****************************(C) COPYRIGHT 2019 DJI****************************
//   */
#ifndef USB_TASK_H
#define USB_TASK_H
#include "struct_typedef.h"
#include "chassis_task.h"
#include "chassis_behaviour.h"

#include "cmsis_os.h"

#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"
#include "chassis_power_control.h"
#include "referee.h"
#include "gimbal_task.h"


typedef enum {
    CLASSIC = 0,
    WIND    = 1,
} vision_mode_e;
// typedef __packed struct
// {
//     uint8_t SOF;
//     uint8_t target_found; // it is renamed as the autofire in the motion_rx struct
//     fp32 pitch_angle;
//     fp32 yaw_angle;
//     uint8_t checksum;
// } vision_rx_t;

typedef struct
{
    uint8_t SOF;
    const fp32 *INS_quat_vision;
    vision_mode_e vision_mode;
    gimbal_control_t *vision_gimbal;
    uint8_t shoot_remote;
    uint8_t enery_color;
    ext_game_robot_state_t *robot_state_v;
    fp32 bullet_speed;
} vision_t;

#define PROJECTILE_TX_SLOW_HEADER 0xA5
#define PROJECTILE_TX_FAST_HEADER 0xF8

typedef struct ProjectileTx_slow {
    uint8_t header;

    // TODO: WHAT IS THIS?
    uint32_t timestamp;

    // NOTE: the mode can be set as any value you want
    uint8_t vision_mode; // default -> CLASSIC 0, (WIND -> 1)

    // the gurgement part (coming from c borad)
    uint8_t shoot_remote;
    // TODO: WTF is the one things above

    // the current side color (which team we are)
    // 0 -> unknown
    // 1 -> red
    // 2 -> blue
    uint8_t current_side_color;
    // the enemt_hp
    uint16_t enemy_1_robot_HP;
    uint16_t enemy_2_robot_HP;
    uint16_t enemy_3_robot_HP;
    uint16_t enemy_4_robot_HP;
    uint16_t enemy_5_robot_HP;
    uint16_t enemy_7_robot_HP;
    uint16_t enemy_outpost_HP;
    uint16_t enemy_base_HP;

    // field events
    uint32_t field_events;

    // game status
    uint8_t game_type;
    uint8_t game_progress;
    uint16_t state_remain_time;
    uint64_t sync_time_stamp;

    // game result
    uint8_t winner;

    // robot buffs
    uint8_t recovery_buff;
    uint8_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;

    // robot position
    float x;
    float y;
    float angle;

    // robot status
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;

    // ext_power_heat_data_t
    uint16_t chassis_voltage;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_1_barrel_heat;
    uint16_t shooter_17mm_2_barrel_heat;

    // client command
    float command_target_position[3];
    // float target_position_x;
    // float target_position_y;
    // float target_position_z;
    uint16_t keyboard_key_pressed;
    uint8_t command_target_robot_id;

    // client receive
    float receive_target_position[2];
    // float target_position_x;
    // float target_position_y;
    uint8_t receive_target_robot_id;

    uint8_t checksum;
} ProjectileTx_slow_t;

typedef struct __attribute__((packed)) {
    uint8_t header; // 0xF8
    float q[4];

    float yaw;
    float pitch;

    uint8_t lever_mode;
    // #define RC_SW_UP                ((uint16_t)1)
    // #define RC_SW_MID               ((uint16_t)3)
    // #define RC_SW_DOWN              ((uint16_t)2)

    float bullet_speed;

    uint8_t checksum;
} ProjectileTx_fast_t;

#define MOTION_RX_HEADER 0x5A
typedef struct MotionRx {
    // the type of the message
    uint8_t header;       // 第 0-3 位
    float yaw_angle;            // 第 4-7 位
    float pitch_angle;          // 第 4-11 位
    float pitch_actual;        // 第 8-15 位
    uint8_t raw1[4];      // 第 12-19 位
    float yaw_actual;        // 第 12-23 位
    uint8_t raw2[4];      // 第 19～27位
    float linear_x;       // 第 24～31 位
    float linear_y;       // 第 28～35 位
    float angular_z;      // 第 32～39 位
    uint8_t target_found;     // 第 41 位 the autofire is to tell the robot to fire
    uint8_t placeholder2; // 第 42 位
    uint8_t placeholder3; // 第 43 位
    uint8_t checksum;     // 第 44 位 (校验位)
} MotionRx_t;
// extern size_t MotionRxSize = sizeof(MotionRx_t);
extern MotionRx_t motion_rx;

void usb_task_(void const *argument);
static void projectile_tx_struct_init(void);
static void projectile_tx_slow_update(void);
static void projectile_tx_fast_update(void);
static void send_projectile_tx_slow(void);
static void send_projectile_tx_fast(void);

uint8_t calculate_parity(const uint8_t *data, size_t length);
uint8_t vertify_parity(const uint8_t *data, size_t length);

union refree_4_byte_t {
    float f;
    unsigned char buf[4];
};
extern void usb_task(void const *argument);

#endif
