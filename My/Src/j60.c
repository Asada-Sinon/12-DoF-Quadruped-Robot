#include "j60.h"
#include <string.h>
#include "can.h"

/* ȫ�ֱ��� */
J60_Motor j60_motors[16];  // ֧��16�����������0-15
/* CAN������� */
CAN_Queue can_queues[CAN_BUS_COUNT];
/* CAN������� */
CAN_HandleTypeDef* can_instances[CAN_BUS_COUNT] = {
    &hcan1,  // CAN1
    &hcan2,  // CAN2
    &hcan3   // CAN3
};

/* ��ԭʼID���Զ���ID��ӳ��� */
uint8_t can_local_to_custom_map[CAN_BUS_COUNT][16] = {0};

// ��ʼ������
static void CAN_Queue_Init(void)
{
    memset(can_queues, 0, sizeof(can_queues));
}

/**
 * @brief  ��ʼ��J60���CAN����
 * @retval HAL״̬
 */
HAL_StatusTypeDef J60_Init(void)
{
    CAN_FilterTypeDef filter;
    
    // ��ʼ����������ӳ���
    memset(j60_motors, 0, sizeof(j60_motors));
    memset(can_local_to_custom_map, 0xFF, sizeof(can_local_to_custom_map)); // ��ʼ��Ϊ0xFF��ʾδӳ��
    // ��ʼ��CAN��Ϣ����
    CAN_Queue_Init();
    // �����е����ʼ��Ϊδʹ��״̬
    for (uint8_t i = 0; i < 16; i++) {
        j60_motors[i].can_bus = CAN_BUS_NONE;
    }
    
    // ����ͨ������
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterActivation = ENABLE;
    
    // CAN1����������
    filter.SlaveStartFilterBank = 14;
    filter.FilterBank = 0;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &filter);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    
    // CAN2����������
    filter.FilterBank = 14;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    HAL_CAN_ConfigFilter(&hcan2, &filter);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    
    // CAN3����������
    filter.FilterBank = 0;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    HAL_CAN_ConfigFilter(&hcan3, &filter);   
    HAL_CAN_Start(&hcan3);
    HAL_CAN_ActivateNotification(&hcan3, CAN_IT_RX_FIFO0_MSG_PENDING);
    
    return HAL_OK;
}

/**
 * @brief  ���õ������ӳ���ϵ
 * @param  custom_id: �Զ�����ID (0-15)
 * @param  can_bus: CAN�������� (0-2)
 * @param  local_id: �����CAN�����ϵ�ԭʼID (0-15)
 * @retval HAL״̬
 */
HAL_StatusTypeDef J60_ConfigMotor(uint8_t custom_id, uint8_t can_bus, uint8_t local_id)
{
    // ��������Ч��
    if (custom_id > 15 || can_bus >= CAN_BUS_COUNT || local_id > 15) {
        return HAL_ERROR;
    }
    
    // �����ӳ��
    uint8_t old_map = can_local_to_custom_map[can_bus][local_id];
    if (old_map != 0xFF && old_map != custom_id) {
        j60_motors[old_map].can_bus = CAN_BUS_NONE;
    }
    
    // ������Զ���ID�ľ�����
    if (j60_motors[custom_id].can_bus != CAN_BUS_NONE) {
        can_local_to_custom_map[j60_motors[custom_id].can_bus][j60_motors[custom_id].local_id] = 0xFF;
    }
    
    // ������ӳ��
    j60_motors[custom_id].custom_id = custom_id;
    j60_motors[custom_id].can_bus = can_bus;
    j60_motors[custom_id].local_id = local_id;
    can_local_to_custom_map[can_bus][local_id] = custom_id;
    
    return HAL_OK;
}

/**
 * @brief  �������õ��ӳ���ϵ
 * @param  configs: �����������
 * @param  num_motors: �������
 * @retval HAL״̬
 */
HAL_StatusTypeDef J60_ConfigMotors(J60_Motor_Config* configs, uint8_t num_motors)
{
    if (num_motors > 16) {
        return HAL_ERROR;
    }
    
    for (uint8_t i = 0; i < num_motors; i++) {
        if (configs[i].is_used) {
            if (J60_ConfigMotor(i, configs[i].can_bus, configs[i].local_id) != HAL_OK) {
                return HAL_ERROR;
            }
        }
    }
    
    return HAL_OK;
}

/**
 * @brief  ����J60�����CAN ID
 * @param  motor_id: ���ID (0-15)
 * @param  cmd_id: ����ID
 * @param  is_rx: �Ƿ�Ϊ����֡(0:����֡, 1:����֡)
 * @retval ������ɵ�CAN ID
 */
static uint32_t J60_MakeCanId(uint8_t motor_id, uint32_t cmd_id, uint8_t is_rx)
{
    uint32_t can_id = 0;
    
    // �ؽ�ID (Bit0-Bit3)
    can_id |= (motor_id & 0x0F);
    
    // �շ���־λ (Bit4)
    if (is_rx) {
        can_id |= J60_FLAG_RECV;
    } else {
        can_id |= J60_FLAG_SEND;
    }
    
    // �������� (Bit5-Bit10)
    can_id |= cmd_id;
    
    return can_id;
}

/**
 * @brief  ����CAN��Ϣ��J60���
 * @param  custom_id: �Զ�����ID (0-15)
 * @param  cmd_id: ����ID
 * @param  data: ����ָ��
 * @param  length: ���ݳ���
 * @retval HAL״̬
 */
static HAL_StatusTypeDef J60_SendCanMessage_NonBlocking(uint8_t custom_id, uint32_t cmd_id, uint8_t *data, uint8_t length)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    uint8_t tx_data[8] = {0};
    
    // ������
    if (custom_id > 15 || length > 8) {
        return HAL_ERROR;
    }
    
    // ��ȡ�������
    J60_Motor *motor = &j60_motors[custom_id];
    if (motor->can_bus == CAN_BUS_NONE) {
        return HAL_ERROR; // ���δ����
    }
    
    // ����Ƿ��п�������
    if (HAL_CAN_GetTxMailboxesFreeLevel(can_instances[motor->can_bus]) == 0) {
        return HAL_BUSY; // ������������������æ״̬
    }
    
    // ���÷���֡ͷ
    txHeader.StdId = J60_MakeCanId(motor->local_id, cmd_id, 0);
    txHeader.ExtId = 0;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = length;
    
    // ��������
    if (data != NULL && length > 0) {
        memcpy(tx_data, data, length);
    }
    
    // ������Ϣ
    return HAL_CAN_AddTxMessage(can_instances[motor->can_bus], &txHeader, tx_data, &txMailbox);
}

// �����Ϣ������
static HAL_StatusTypeDef CAN_Queue_Add(uint8_t can_bus, uint8_t custom_id, uint32_t cmd_id, uint8_t *data, uint8_t length)
{
    if (can_bus >= CAN_BUS_COUNT) {
        return HAL_ERROR;
    }
    
    CAN_Queue *queue = &can_queues[can_bus];
    
    if (queue->count >= CAN_QUEUE_SIZE) {
        return HAL_BUSY;  // ��������
    }
    
    CAN_Queue_Item *item = &queue->items[queue->tail];
    item->custom_id = custom_id;
    item->cmd_id = cmd_id;
    item->length = length;
    item->timestamp = HAL_GetTick();
    
    if (data && length > 0) {
        memcpy(item->data, data, length);
    }
    
    queue->tail = (queue->tail + 1) % CAN_QUEUE_SIZE;
    queue->count++;
    
    return HAL_OK;
}

// J60_SendCanMessage����ʹ�÷�����ģʽ
static HAL_StatusTypeDef J60_SendCanMessage(uint8_t custom_id, uint32_t cmd_id, uint8_t *data, uint8_t length)
{
    // �������
    if (custom_id > 15 || length > 8) {
        return HAL_ERROR;
    }
    
    // ��ȡ�������
    J60_Motor *motor = &j60_motors[custom_id];
    if (motor->can_bus == CAN_BUS_NONE) {
        return HAL_ERROR; // ���δ����
    }
    
    // ����ֱ�ӷ���
    HAL_StatusTypeDef result = J60_SendCanMessage_NonBlocking(custom_id, cmd_id, data, length);
    
    // �������������������
    if (result == HAL_BUSY) {
        return CAN_Queue_Add(motor->can_bus, custom_id, cmd_id, data, length);
    }
    
    return result;
}

// ����CAN�����е���Ϣ
void J60_ProcessCanQueues(void)
{
    uint32_t current_time = HAL_GetTick();
    
    // ����ÿ��CAN���ߵĶ���
    for (uint8_t can_bus = 0; can_bus < CAN_BUS_COUNT; can_bus++) {
        CAN_Queue *queue = &can_queues[can_bus];
        
        // �������Ϊ�գ�����
        if (queue->count == 0) {
            continue;
        }
        
        // ����Ƿ��п�������
        if (HAL_CAN_GetTxMailboxesFreeLevel(can_instances[can_bus]) == 0) {
            continue;  // ���������������ȴ��´δ���
        }
        
        // ȡ������ͷ������Ϣ
        CAN_Queue_Item *item = &queue->items[queue->head];
        
        // �����Ϣ�Ƿ�ʱ��200ms��
        if (current_time - item->timestamp > 200) {
            // ��Ϣ��ʱ������
            queue->head = (queue->head + 1) % CAN_QUEUE_SIZE;
            queue->count--;
            continue;
        }
        
        // ���Է�����Ϣ
        HAL_StatusTypeDef result = J60_SendCanMessage_NonBlocking(item->custom_id, item->cmd_id, item->data, item->length);
        
        if (result == HAL_OK) {
            // ���ͳɹ����Ӷ������Ƴ�
            queue->head = (queue->head + 1) % CAN_QUEUE_SIZE;
            queue->count--;
        }
        // �������ʧ�ܣ������ڶ����У��´��ٳ���
    }
}

/**
 * @brief  ʹ��J60���
 * @param  custom_id: �Զ�����ID (0-15)
 * @retval HAL״̬
 */
HAL_StatusTypeDef J60_EnableMotor(uint8_t custom_id)
{
    return J60_SendCanMessage(custom_id, J60_CMD_MOTOR_ENABLE, NULL, 0);
}

/**
 * @brief  ʧ��J60���
 * @param  custom_id: �Զ�����ID (0-15)
 * @retval HAL״̬
 */
HAL_StatusTypeDef J60_DisableMotor(uint8_t custom_id)
{
    return J60_SendCanMessage(custom_id, J60_CMD_MOTOR_DISABLE, NULL, 0);
}

/**
 * @brief  ������Ƕ�ӳ�䵽16λ�޷�������
 * @param  position: �Ƕ�ֵ (-40rad��40rad)
 * @retval 16λ�޷������� (0-65535)
 */
uint16_t J60_MapPositionToUint16(float position)
{
    if (position < -40.0f) position = -40.0f;
    if (position > 40.0f) position = 40.0f;
    
    return (uint16_t)((position + 40.0f) * 65535.0f / 80.0f);
}

/**
 * @brief  ��������ٶ�ӳ�䵽14λ�޷�������
 * @param  velocity: ���ٶ�ֵ (-40rad/s��40rad/s)
 * @retval 14λ�޷������� (0-16383)
 */
uint16_t J60_MapVelocityToUint14(float velocity)
{
    if (velocity < -40.0f) velocity = -40.0f;
    if (velocity > 40.0f) velocity = 40.0f;
    
    return (uint16_t)((velocity + 40.0f) * 16383.0f / 80.0f);
}

/**
 * @brief  ���ն�ϵ��Kpӳ�䵽10λ�޷�������
 * @param  kp: �ն�ϵ�� (0-1023)
 * @retval 10λ�޷������� (0-1023)
 */
uint16_t J60_MapKpToUint10(float kp)
{
    if (kp < 0.0f) kp = 0.0f;
    if (kp > 1023.0f) kp = 1023.0f;
    
    return (uint16_t)kp;
}

/**
 * @brief  ������ϵ��Kdӳ�䵽8λ�޷�������
 * @param  kd: ����ϵ�� (0.0-51.0)
 * @retval 8λ�޷������� (0-255)
 */
uint8_t J60_MapKdToUint8(float kd)
{
    if (kd < 0.0f) kd = 0.0f;
    if (kd > 51.0f) kd = 51.0f;
    
    return (uint8_t)(kd * 255.0f / 51.0f);
}

/**
 * @brief  ������Ť��ӳ�䵽16λ�޷�������
 * @param  torque: Ť��ֵ (-40Nm��40Nm)
 * @retval 16λ�޷������� (0-65535)
 */
uint16_t J60_MapTorqueToUint16(float torque)
{
    if (torque < -40.0f) torque = -40.0f;
    if (torque > 40.0f) torque = 40.0f;
    
    return (uint16_t)((torque + 40.0f) * 65535.0f / 80.0f);
}

/**
 * @brief  ��20λ�޷�������ӳ�䵽����Ƕ�
 * @param  value: 20λ�޷������� (0-1048575)
 * @retval �Ƕ�ֵ (-40rad��40rad)
 */
float J60_MapUint20ToPosition(uint32_t value)
{
    if (value > 1048575) value = 1048575;
    
    return -40.0f + (float)value * 80.0f / 1048575.0f;
}

/**
 * @brief  ��20λ�޷�������ӳ�䵽������ٶ�
 * @param  value: 20λ�޷������� (0-1048575)
 * @retval ���ٶ�ֵ (-40rad/s��40rad/s)
 */
float J60_MapUint20ToVelocity(uint32_t value)
{
    if (value > 1048575) value = 1048575;
    
    return -40.0f + (float)value * 80.0f / 1048575.0f;
}

/**
 * @brief  ��16λ�޷�������ӳ�䵽����Ť��
 * @param  value: 16λ�޷������� (0-65535)
 * @retval Ť��ֵ (-40Nm��40Nm)
 */
float J60_MapUint16ToTorque(uint16_t value)
{
    return -40.0f + (float)value * 80.0f / 65535.0f;
}

/**
 * @brief  ��7λ�޷�������ӳ�䵽�����¶�
 * @param  value: 7λ�޷������� (0-127)
 * @retval �¶�ֵ (-20�浽200��)
 */
float J60_MapUint7ToTemperature(uint8_t value)
{
    if (value > 127) value = 127;
    
    return -20.0f + (float)value * 220.0f / 127.0f;
}

/**
 * @brief  �������ģʽ - �ۺϿ���λ�á��ٶȺ�����
 * @param  custom_id: �Զ�����ID (0-15)
 * @param  position: Ŀ��λ��(rad) ��Χ[-40,40]
 * @param  velocity: Ŀ���ٶ�(rad/s) ��Χ[-40,40]
 * @param  kp: �ն�ϵ�� ��Χ[0,1023]
 * @param  kd: ����ϵ�� ��Χ[0,51]
 * @param  torque: Ŀ������(Nm) ��Χ[-40,40]
 * @retval HAL״̬
 */
HAL_StatusTypeDef J60_MotorControl(uint8_t custom_id, float position, float velocity, float kp, float kd, float torque)
{
    uint8_t data[8] = {0};
    uint16_t pos_u16, vel_u14, kp_u10, torque_u16;
    uint8_t kd_u8;
    
    // �������
    if (custom_id > 15) {
        return HAL_ERROR;
    }
    
    // ӳ������ֵ��CANЭ��ֵ
    pos_u16 = J60_MapPositionToUint16(position);
    vel_u14 = J60_MapVelocityToUint14(velocity);
    kp_u10 = J60_MapKpToUint10(kp);
    kd_u8 = J60_MapKdToUint8(kd);
    torque_u16 = J60_MapTorqueToUint16(torque);
    
    // ����CAN����
    // Byte 0-1: Ŀ��Ƕ� (16λ)
    data[0] = (uint8_t)(pos_u16 & 0xFF);
    data[1] = (uint8_t)((pos_u16 >> 8) & 0xFF);
    
    // Byte 2-3: Ŀ����ٶ�(14λ)+Kp�ն�(10λ�ĵ�2λ)
    data[2] = (uint8_t)(vel_u14 & 0xFF);
    data[3] = (uint8_t)(((vel_u14 >> 8) & 0x3F) | ((kp_u10 & 0x03) << 6));
    
    // Byte 4: Kp�ն�(10λ�ĸ�8λ)
    data[4] = (uint8_t)((kp_u10 >> 2) & 0xFF);
    
    // Byte 5: Kd����(8λ)
    data[5] = kd_u8;
    
    // Byte 6-7: Ŀ��Ť��(16λ)
    data[6] = (uint8_t)(torque_u16 & 0xFF);
    data[7] = (uint8_t)((torque_u16 >> 8) & 0xFF);
    
    // ������Ʋ���������ṹ��
    j60_motors[custom_id].kp = kp;
    j60_motors[custom_id].kd = kd;
    
    // ��������
    return J60_SendCanMessage(custom_id, J60_CMD_MOTOR_CONTROL, data, 8);
}

/**
 * @brief  ������յ���CAN���� - ��Ҫ�޸�������MOTOR_CONTROL����ķ���
 * @param  hcan: CAN���ָ��
 * @param  pRxHeader: CAN����֡ͷָ��
 * @param  data: ���յ�������
 * @retval ��
 */
void J60_ProcessReceivedData(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *pRxHeader, uint8_t *data)
{
    uint32_t can_id = pRxHeader->StdId;
    uint8_t local_id = can_id & 0x0F;  // ʵ�ʵ��ID�ǵ�4λ
    uint8_t is_rx = (can_id & J60_FLAG_RECV) != 0;
    uint32_t cmd_id = can_id & ~(0x1F); // �����5λ��������������
    
    // ȷ���ǽ���֡
    if (!is_rx) {
        return;
    }

    // ȷ��CAN��������
    uint8_t can_bus_index;
    if (hcan == &hcan1) {
        can_bus_index = CAN_BUS_1;
    } else if (hcan == &hcan2) {
        can_bus_index = CAN_BUS_2;
    } else if (hcan == &hcan3) {
        can_bus_index = CAN_BUS_3;
    } else {
        return;
    }

    // �����Զ���ID��ע��ؽ�ID������+0x10��ƫ��(MOTOR_CONTROL����)
    uint8_t adjusted_local_id = local_id;
    if ((cmd_id == J60_CMD_MOTOR_CONTROL) && (local_id >= 0x10)) {
        // �����MOTOR_CONTROL���������ID��ƫ�ƣ�������ԭʼID
        adjusted_local_id = local_id - 0x10;
    }
    
    uint8_t custom_id = can_local_to_custom_map[can_bus_index][adjusted_local_id];
    if (custom_id > 15) {
        return; // δӳ���ID
    }
    
    // �������������
    switch (cmd_id) {
        case J60_CMD_MOTOR_CONTROL: {
            if (pRxHeader->DLC >= 8) {
                // ����20λλ��(3�ֽ�)
                uint32_t pos_uint20 = 
                    ((uint32_t)data[0]) | 
                    ((uint32_t)data[1] << 8) | 
                    (((uint32_t)data[2] & 0x0F) << 16);
                
                // ����20λ�ٶ�(2.5�ֽ�)
                uint32_t vel_uint20 = 
                    (((uint32_t)data[2] & 0xF0) >> 4) | 
                    ((uint32_t)data[3] << 4) | 
                    (((uint32_t)data[4] & 0xFF) << 12);
                
                // ����16λŤ��(2�ֽ�)
                uint16_t torque_uint16 = 
                    ((uint16_t)data[5]) | 
                    ((uint16_t)(data[6] & 0xFF) << 8);
                
                // �����¶ȱ�־λ���¶�ֵ
                uint8_t temp_type = (data[7] & 0x01);
                uint8_t temp_uint7 = (data[7] >> 1) & 0x7F;
                
                // ת��Ϊ����ֵ
                float position = J60_MapUint20ToPosition(pos_uint20);
                float velocity = J60_MapUint20ToVelocity(vel_uint20);
                float torque = J60_MapUint16ToTorque(torque_uint16);
                float temperature = J60_MapUint7ToTemperature(temp_uint7);
                
                // ���µ��״̬
                j60_motors[custom_id].position = position;
                j60_motors[custom_id].velocity = velocity;
                j60_motors[custom_id].torque = torque;
                j60_motors[custom_id].temperature = temperature;
                j60_motors[custom_id].temp_type = temp_type;
            }
            break;
        }
        
        // ������������Ĵ���...
        
        default:
            break;
    }
}
/**
 * @brief  CAN�����жϻص�����
 * @param  hcan: CAN���ָ��
 * @retval ��
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t data[8];
    
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, data) == HAL_OK) {
        J60_ProcessReceivedData(hcan, &rxHeader, data);
    }
}

/**
 * @brief  ��ȡ�������
 * @param  custom_id: �Զ�����ID (0-15)
 * @retval ��������ṹ��ָ�룬���ID��Ч����NULL
 */
J60_Motor* J60_GetMotor(uint8_t custom_id)
{
    if (custom_id > 15 || j60_motors[custom_id].can_bus == CAN_BUS_NONE) {
        return NULL;
    }
    
    return &j60_motors[custom_id];
}
