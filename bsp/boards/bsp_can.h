#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "struct_typedef.h"
#include "mytype.h"
#define FILTER_BUF_LEN 5

/*鎺ユ敹鍒扮殑浜戝彴鐢垫満鐨勫弬鏁扮粨鏋勪綋*/
typedef struct
{
    int16_t speed_rpm;
    int16_t real_current;
    int16_t given_current;
    uint8_t hall;
    uint16_t angle;       //abs angle range:[0,8191]
    uint16_t last_angle;  //abs angle range:[0,8191]
    uint16_t offset_angle;
    int32_t round_cnt;
    int32_t total_angle;
    u8 buf_idx;
    u16 angle_buf[FILTER_BUF_LEN];
    u16 fited_angle;
    u32 msg_cnt;
} moto_measure_t;

extern void can_filter_init(void);
/* Extern  ------------------------------------------------------------------*/
extern moto_measure_t moto_chassis[];
extern moto_measure_t moto_yaw, moto_pit, moto_poke, moto_info;
extern float real_current_from_judgesys;  //unit :mA
extern float dynamic_limit_current;       //unit :mA,;    //from judge_sys
extern float ZGyroModuleAngle, yaw_zgyro_angle;

#endif
