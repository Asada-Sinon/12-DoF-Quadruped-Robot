#ifndef __J60_H
#define __J60_H

#include "main.h"
#include "can.h"

#define CAN_QUEUE_SIZE 32  // 每个CAN总线的队列大小

/* J60 CAN命令ID定义 */
// 命令索引值 (Bit5~Bit10)
#define J60_CMD_MOTOR_DISABLE         (0x01 << 5)  // 电机失能
#define J60_CMD_MOTOR_ENABLE          (0x02 << 5)  // 电机使能
#define J60_CMD_MOTOR_CONTROL         (0x04 << 5)  // 电机控制
#define J60_CMD_GET_CONFIG            (0x18 << 5)  // 获取配置
#define J60_CMD_SET_CAN_TIMEOUT       (0x09 << 5)  // 设置CAN超时
#define J60_CMD_SET_BANDWIDTH         (0x0A << 5)  // 设置电流环带宽
#define J60_CMD_SAVE_CONFIG           (0x10 << 5)  // 保存配置
#define J60_CMD_GET_STATUSWORD        (0x17 << 5)  // 获取状态字
#define J60_CMD_ERROR_RESET           (0x11 << 5)  // 错误复位

// 收发标志位 (Bit4)
#define J60_FLAG_SEND                 (0 << 4)     // 发送标志
#define J60_FLAG_RECV                 (1 << 4)     // 接收标志

typedef struct {
    uint8_t custom_id;
    uint32_t cmd_id;
    uint8_t data[8];
    uint8_t length;
    uint32_t timestamp;  // 用于超时检测
} CAN_Queue_Item;

typedef struct {
    CAN_Queue_Item items[CAN_QUEUE_SIZE];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} CAN_Queue;

/* CAN总线索引定义 */
typedef enum {
    CAN_BUS_1 = 0,
    CAN_BUS_2 = 1,
    CAN_BUS_3 = 2,
    CAN_BUS_NONE = 0xFF,
    CAN_BUS_COUNT = 3
} CAN_Bus_Index;

/* 电机配置结构体 */
typedef struct {
    uint8_t can_bus;      // CAN总线索引 (0=CAN1, 1=CAN2, 2=CAN3)
    uint8_t local_id;     // 电机在CAN总线上的原始ID (0-15)
    uint8_t is_used;      // 是否使用此电机
} J60_Motor_Config;

/* J60电机参数结构体 */
typedef struct {
    uint8_t custom_id;    // 自定义电机ID (0-15)
    uint8_t can_bus;      // CAN总线索引 (0=CAN1, 1=CAN2, 2=CAN3)
    uint8_t local_id;     // 电机在CAN总线上的原始ID (0-15)
    float position;       // 位置 (rad)
    float velocity;       // 速度 (rad/s)
    float torque;         // 力矩 (Nm)
    float temperature;    // 温度 (°C)
    uint8_t temp_type;    // 温度类型 (0=mosfet温度, 1=电机温度)
    uint8_t error_code;   // 错误码
    float kp;             // 位置环增益
    float kd;             // 速度环增益
} J60_Motor;

/* 函数声明 */
HAL_StatusTypeDef J60_Init(void);
HAL_StatusTypeDef J60_ConfigMotor(uint8_t custom_id, uint8_t can_bus, uint8_t local_id);
HAL_StatusTypeDef J60_ConfigMotors(J60_Motor_Config* configs, uint8_t num_motors);
J60_Motor* J60_GetMotor(uint8_t custom_id);

HAL_StatusTypeDef J60_EnableMotor(uint8_t custom_id);
HAL_StatusTypeDef J60_DisableMotor(uint8_t custom_id);
HAL_StatusTypeDef J60_MotorControl(uint8_t custom_id, float position, float velocity, float kp, float kd, float torque);

void J60_ProcessReceivedData(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *pRxHeader, uint8_t *data);
void J60_ProcessCanQueues(void);
#endif /* __J60_H */
