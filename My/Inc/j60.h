#ifndef __J60_H
#define __J60_H

#include "main.h"
#include "can.h"

#define CAN_QUEUE_SIZE 32  // ÿ��CAN���ߵĶ��д�С

/* J60 CAN����ID���� */
// ��������ֵ (Bit5~Bit10)
#define J60_CMD_MOTOR_DISABLE         (0x01 << 5)  // ���ʧ��
#define J60_CMD_MOTOR_ENABLE          (0x02 << 5)  // ���ʹ��
#define J60_CMD_MOTOR_CONTROL         (0x04 << 5)  // �������
#define J60_CMD_GET_CONFIG            (0x18 << 5)  // ��ȡ����
#define J60_CMD_SET_CAN_TIMEOUT       (0x09 << 5)  // ����CAN��ʱ
#define J60_CMD_SET_BANDWIDTH         (0x0A << 5)  // ���õ���������
#define J60_CMD_SAVE_CONFIG           (0x10 << 5)  // ��������
#define J60_CMD_GET_STATUSWORD        (0x17 << 5)  // ��ȡ״̬��
#define J60_CMD_ERROR_RESET           (0x11 << 5)  // ����λ

// �շ���־λ (Bit4)
#define J60_FLAG_SEND                 (0 << 4)     // ���ͱ�־
#define J60_FLAG_RECV                 (1 << 4)     // ���ձ�־

typedef struct {
    uint8_t custom_id;
    uint32_t cmd_id;
    uint8_t data[8];
    uint8_t length;
    uint32_t timestamp;  // ���ڳ�ʱ���
} CAN_Queue_Item;

typedef struct {
    CAN_Queue_Item items[CAN_QUEUE_SIZE];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} CAN_Queue;

/* CAN������������ */
typedef enum {
    CAN_BUS_1 = 0,
    CAN_BUS_2 = 1,
    CAN_BUS_3 = 2,
    CAN_BUS_NONE = 0xFF,
    CAN_BUS_COUNT = 3
} CAN_Bus_Index;

/* ������ýṹ�� */
typedef struct {
    uint8_t can_bus;      // CAN�������� (0=CAN1, 1=CAN2, 2=CAN3)
    uint8_t local_id;     // �����CAN�����ϵ�ԭʼID (0-15)
    uint8_t is_used;      // �Ƿ�ʹ�ô˵��
} J60_Motor_Config;

/* J60��������ṹ�� */
typedef struct {
    uint8_t custom_id;    // �Զ�����ID (0-15)
    uint8_t can_bus;      // CAN�������� (0=CAN1, 1=CAN2, 2=CAN3)
    uint8_t local_id;     // �����CAN�����ϵ�ԭʼID (0-15)
    float position;       // λ�� (rad)
    float velocity;       // �ٶ� (rad/s)
    float torque;         // ���� (Nm)
    float temperature;    // �¶� (��C)
    uint8_t temp_type;    // �¶����� (0=mosfet�¶�, 1=����¶�)
    uint8_t error_code;   // ������
    float kp;             // λ�û�����
    float kd;             // �ٶȻ�����
} J60_Motor;

/* �������� */
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
