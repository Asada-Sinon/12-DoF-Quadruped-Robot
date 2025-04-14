#include "j60.h"
#include <string.h>
#include "can.h"

/* 全局变量 */
J60_Motor j60_motors[16];  // 支持16个电机，索引0-15
/* CAN缓冲队列 */
CAN_Queue can_queues[CAN_BUS_COUNT];
/* CAN句柄数组 */
CAN_HandleTypeDef* can_instances[CAN_BUS_COUNT] = {
    &hcan1,  // CAN1
    &hcan2,  // CAN2
    &hcan3   // CAN3
};

/* 从原始ID到自定义ID的映射表 */
uint8_t can_local_to_custom_map[CAN_BUS_COUNT][16] = {0};

// 初始化队列
static void CAN_Queue_Init(void)
{
    memset(can_queues, 0, sizeof(can_queues));
}

/**
 * @brief  初始化J60电机CAN总线
 * @retval HAL状态
 */
HAL_StatusTypeDef J60_Init(void)
{
    CAN_FilterTypeDef filter;
    
    // 初始化电机数组和映射表
    memset(j60_motors, 0, sizeof(j60_motors));
    memset(can_local_to_custom_map, 0xFF, sizeof(can_local_to_custom_map)); // 初始化为0xFF表示未映射
    // 初始化CAN消息队列
    CAN_Queue_Init();
    // 将所有电机初始化为未使用状态
    for (uint8_t i = 0; i < 16; i++) {
        j60_motors[i].can_bus = CAN_BUS_NONE;
    }
    
    // 基本通用配置
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterActivation = ENABLE;
    
    // CAN1过滤器配置
    filter.SlaveStartFilterBank = 14;
    filter.FilterBank = 0;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &filter);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    
    // CAN2过滤器配置
    filter.FilterBank = 14;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    HAL_CAN_ConfigFilter(&hcan2, &filter);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    
    // CAN3过滤器配置
    filter.FilterBank = 0;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    HAL_CAN_ConfigFilter(&hcan3, &filter);   
    HAL_CAN_Start(&hcan3);
    HAL_CAN_ActivateNotification(&hcan3, CAN_IT_RX_FIFO0_MSG_PENDING);
    
    return HAL_OK;
}

/**
 * @brief  配置单个电机映射关系
 * @param  custom_id: 自定义电机ID (0-15)
 * @param  can_bus: CAN总线索引 (0-2)
 * @param  local_id: 电机在CAN总线上的原始ID (0-15)
 * @retval HAL状态
 */
HAL_StatusTypeDef J60_ConfigMotor(uint8_t custom_id, uint8_t can_bus, uint8_t local_id)
{
    // 检查参数有效性
    if (custom_id > 15 || can_bus >= CAN_BUS_COUNT || local_id > 15) {
        return HAL_ERROR;
    }
    
    // 清除旧映射
    uint8_t old_map = can_local_to_custom_map[can_bus][local_id];
    if (old_map != 0xFF && old_map != custom_id) {
        j60_motors[old_map].can_bus = CAN_BUS_NONE;
    }
    
    // 清除此自定义ID的旧配置
    if (j60_motors[custom_id].can_bus != CAN_BUS_NONE) {
        can_local_to_custom_map[j60_motors[custom_id].can_bus][j60_motors[custom_id].local_id] = 0xFF;
    }
    
    // 设置新映射
    j60_motors[custom_id].custom_id = custom_id;
    j60_motors[custom_id].can_bus = can_bus;
    j60_motors[custom_id].local_id = local_id;
    can_local_to_custom_map[can_bus][local_id] = custom_id;
    
    return HAL_OK;
}

/**
 * @brief  批量配置电机映射关系
 * @param  configs: 电机配置数组
 * @param  num_motors: 电机数量
 * @retval HAL状态
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
 * @brief  构造J60电机的CAN ID
 * @param  motor_id: 电机ID (0-15)
 * @param  cmd_id: 命令ID
 * @param  is_rx: 是否为接收帧(0:发送帧, 1:接收帧)
 * @retval 构造完成的CAN ID
 */
static uint32_t J60_MakeCanId(uint8_t motor_id, uint32_t cmd_id, uint8_t is_rx)
{
    uint32_t can_id = 0;
    
    // 关节ID (Bit0-Bit3)
    can_id |= (motor_id & 0x0F);
    
    // 收发标志位 (Bit4)
    if (is_rx) {
        can_id |= J60_FLAG_RECV;
    } else {
        can_id |= J60_FLAG_SEND;
    }
    
    // 命令索引 (Bit5-Bit10)
    can_id |= cmd_id;
    
    return can_id;
}

/**
 * @brief  发送CAN消息到J60电机
 * @param  custom_id: 自定义电机ID (0-15)
 * @param  cmd_id: 命令ID
 * @param  data: 数据指针
 * @param  length: 数据长度
 * @retval HAL状态
 */
static HAL_StatusTypeDef J60_SendCanMessage_NonBlocking(uint8_t custom_id, uint32_t cmd_id, uint8_t *data, uint8_t length)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    uint8_t tx_data[8] = {0};
    
    // 检查参数
    if (custom_id > 15 || length > 8) {
        return HAL_ERROR;
    }
    
    // 获取电机配置
    J60_Motor *motor = &j60_motors[custom_id];
    if (motor->can_bus == CAN_BUS_NONE) {
        return HAL_ERROR; // 电机未配置
    }
    
    // 检查是否有空闲邮箱
    if (HAL_CAN_GetTxMailboxesFreeLevel(can_instances[motor->can_bus]) == 0) {
        return HAL_BUSY; // 所有邮箱已满，返回忙状态
    }
    
    // 配置发送帧头
    txHeader.StdId = J60_MakeCanId(motor->local_id, cmd_id, 0);
    txHeader.ExtId = 0;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = length;
    
    // 复制数据
    if (data != NULL && length > 0) {
        memcpy(tx_data, data, length);
    }
    
    // 发送消息
    return HAL_CAN_AddTxMessage(can_instances[motor->can_bus], &txHeader, tx_data, &txMailbox);
}

// 添加消息到队列
static HAL_StatusTypeDef CAN_Queue_Add(uint8_t can_bus, uint8_t custom_id, uint32_t cmd_id, uint8_t *data, uint8_t length)
{
    if (can_bus >= CAN_BUS_COUNT) {
        return HAL_ERROR;
    }
    
    CAN_Queue *queue = &can_queues[can_bus];
    
    if (queue->count >= CAN_QUEUE_SIZE) {
        return HAL_BUSY;  // 队列已满
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

// J60_SendCanMessage函数使用非阻塞模式
static HAL_StatusTypeDef J60_SendCanMessage(uint8_t custom_id, uint32_t cmd_id, uint8_t *data, uint8_t length)
{
    // 参数检查
    if (custom_id > 15 || length > 8) {
        return HAL_ERROR;
    }
    
    // 获取电机配置
    J60_Motor *motor = &j60_motors[custom_id];
    if (motor->can_bus == CAN_BUS_NONE) {
        return HAL_ERROR; // 电机未配置
    }
    
    // 尝试直接发送
    HAL_StatusTypeDef result = J60_SendCanMessage_NonBlocking(custom_id, cmd_id, data, length);
    
    // 如果邮箱满，则加入队列
    if (result == HAL_BUSY) {
        return CAN_Queue_Add(motor->can_bus, custom_id, cmd_id, data, length);
    }
    
    return result;
}

// 处理CAN队列中的消息
void J60_ProcessCanQueues(void)
{
    uint32_t current_time = HAL_GetTick();
    
    // 处理每个CAN总线的队列
    for (uint8_t can_bus = 0; can_bus < CAN_BUS_COUNT; can_bus++) {
        CAN_Queue *queue = &can_queues[can_bus];
        
        // 如果队列为空，跳过
        if (queue->count == 0) {
            continue;
        }
        
        // 检查是否有空闲邮箱
        if (HAL_CAN_GetTxMailboxesFreeLevel(can_instances[can_bus]) == 0) {
            continue;  // 所有邮箱已满，等待下次处理
        }
        
        // 取出队列头部的消息
        CAN_Queue_Item *item = &queue->items[queue->head];
        
        // 检查消息是否超时（200ms）
        if (current_time - item->timestamp > 200) {
            // 消息超时，丢弃
            queue->head = (queue->head + 1) % CAN_QUEUE_SIZE;
            queue->count--;
            continue;
        }
        
        // 尝试发送消息
        HAL_StatusTypeDef result = J60_SendCanMessage_NonBlocking(item->custom_id, item->cmd_id, item->data, item->length);
        
        if (result == HAL_OK) {
            // 发送成功，从队列中移除
            queue->head = (queue->head + 1) % CAN_QUEUE_SIZE;
            queue->count--;
        }
        // 如果发送失败，保留在队列中，下次再尝试
    }
}

/**
 * @brief  使能J60电机
 * @param  custom_id: 自定义电机ID (0-15)
 * @retval HAL状态
 */
HAL_StatusTypeDef J60_EnableMotor(uint8_t custom_id)
{
    return J60_SendCanMessage(custom_id, J60_CMD_MOTOR_ENABLE, NULL, 0);
}

/**
 * @brief  失能J60电机
 * @param  custom_id: 自定义电机ID (0-15)
 * @retval HAL状态
 */
HAL_StatusTypeDef J60_DisableMotor(uint8_t custom_id)
{
    return J60_SendCanMessage(custom_id, J60_CMD_MOTOR_DISABLE, NULL, 0);
}

/**
 * @brief  将物理角度映射到16位无符号整数
 * @param  position: 角度值 (-40rad到40rad)
 * @retval 16位无符号整数 (0-65535)
 */
uint16_t J60_MapPositionToUint16(float position)
{
    if (position < -40.0f) position = -40.0f;
    if (position > 40.0f) position = 40.0f;
    
    return (uint16_t)((position + 40.0f) * 65535.0f / 80.0f);
}

/**
 * @brief  将物理角速度映射到14位无符号整数
 * @param  velocity: 角速度值 (-40rad/s到40rad/s)
 * @retval 14位无符号整数 (0-16383)
 */
uint16_t J60_MapVelocityToUint14(float velocity)
{
    if (velocity < -40.0f) velocity = -40.0f;
    if (velocity > 40.0f) velocity = 40.0f;
    
    return (uint16_t)((velocity + 40.0f) * 16383.0f / 80.0f);
}

/**
 * @brief  将刚度系数Kp映射到10位无符号整数
 * @param  kp: 刚度系数 (0-1023)
 * @retval 10位无符号整数 (0-1023)
 */
uint16_t J60_MapKpToUint10(float kp)
{
    if (kp < 0.0f) kp = 0.0f;
    if (kp > 1023.0f) kp = 1023.0f;
    
    return (uint16_t)kp;
}

/**
 * @brief  将阻尼系数Kd映射到8位无符号整数
 * @param  kd: 阻尼系数 (0.0-51.0)
 * @retval 8位无符号整数 (0-255)
 */
uint8_t J60_MapKdToUint8(float kd)
{
    if (kd < 0.0f) kd = 0.0f;
    if (kd > 51.0f) kd = 51.0f;
    
    return (uint8_t)(kd * 255.0f / 51.0f);
}

/**
 * @brief  将物理扭矩映射到16位无符号整数
 * @param  torque: 扭矩值 (-40Nm到40Nm)
 * @retval 16位无符号整数 (0-65535)
 */
uint16_t J60_MapTorqueToUint16(float torque)
{
    if (torque < -40.0f) torque = -40.0f;
    if (torque > 40.0f) torque = 40.0f;
    
    return (uint16_t)((torque + 40.0f) * 65535.0f / 80.0f);
}

/**
 * @brief  将20位无符号整数映射到物理角度
 * @param  value: 20位无符号整数 (0-1048575)
 * @retval 角度值 (-40rad到40rad)
 */
float J60_MapUint20ToPosition(uint32_t value)
{
    if (value > 1048575) value = 1048575;
    
    return -40.0f + (float)value * 80.0f / 1048575.0f;
}

/**
 * @brief  将20位无符号整数映射到物理角速度
 * @param  value: 20位无符号整数 (0-1048575)
 * @retval 角速度值 (-40rad/s到40rad/s)
 */
float J60_MapUint20ToVelocity(uint32_t value)
{
    if (value > 1048575) value = 1048575;
    
    return -40.0f + (float)value * 80.0f / 1048575.0f;
}

/**
 * @brief  将16位无符号整数映射到物理扭矩
 * @param  value: 16位无符号整数 (0-65535)
 * @retval 扭矩值 (-40Nm到40Nm)
 */
float J60_MapUint16ToTorque(uint16_t value)
{
    return -40.0f + (float)value * 80.0f / 65535.0f;
}

/**
 * @brief  将7位无符号整数映射到物理温度
 * @param  value: 7位无符号整数 (0-127)
 * @retval 温度值 (-20℃到200℃)
 */
float J60_MapUint7ToTemperature(uint8_t value)
{
    if (value > 127) value = 127;
    
    return -20.0f + (float)value * 220.0f / 127.0f;
}

/**
 * @brief  电机控制模式 - 综合控制位置、速度和力矩
 * @param  custom_id: 自定义电机ID (0-15)
 * @param  position: 目标位置(rad) 范围[-40,40]
 * @param  velocity: 目标速度(rad/s) 范围[-40,40]
 * @param  kp: 刚度系数 范围[0,1023]
 * @param  kd: 阻尼系数 范围[0,51]
 * @param  torque: 目标力矩(Nm) 范围[-40,40]
 * @retval HAL状态
 */
HAL_StatusTypeDef J60_MotorControl(uint8_t custom_id, float position, float velocity, float kp, float kd, float torque)
{
    uint8_t data[8] = {0};
    uint16_t pos_u16, vel_u14, kp_u10, torque_u16;
    uint8_t kd_u8;
    
    // 参数检查
    if (custom_id > 15) {
        return HAL_ERROR;
    }
    
    // 映射物理值到CAN协议值
    pos_u16 = J60_MapPositionToUint16(position);
    vel_u14 = J60_MapVelocityToUint14(velocity);
    kp_u10 = J60_MapKpToUint10(kp);
    kd_u8 = J60_MapKdToUint8(kd);
    torque_u16 = J60_MapTorqueToUint16(torque);
    
    // 构造CAN数据
    // Byte 0-1: 目标角度 (16位)
    data[0] = (uint8_t)(pos_u16 & 0xFF);
    data[1] = (uint8_t)((pos_u16 >> 8) & 0xFF);
    
    // Byte 2-3: 目标角速度(14位)+Kp刚度(10位的低2位)
    data[2] = (uint8_t)(vel_u14 & 0xFF);
    data[3] = (uint8_t)(((vel_u14 >> 8) & 0x3F) | ((kp_u10 & 0x03) << 6));
    
    // Byte 4: Kp刚度(10位的高8位)
    data[4] = (uint8_t)((kp_u10 >> 2) & 0xFF);
    
    // Byte 5: Kd阻尼(8位)
    data[5] = kd_u8;
    
    // Byte 6-7: 目标扭矩(16位)
    data[6] = (uint8_t)(torque_u16 & 0xFF);
    data[7] = (uint8_t)((torque_u16 >> 8) & 0xFF);
    
    // 保存控制参数到电机结构体
    j60_motors[custom_id].kp = kp;
    j60_motors[custom_id].kd = kd;
    
    // 发送命令
    return J60_SendCanMessage(custom_id, J60_CMD_MOTOR_CONTROL, data, 8);
}

/**
 * @brief  处理接收到的CAN数据 - 需要修改来适配MOTOR_CONTROL命令的反馈
 * @param  hcan: CAN句柄指针
 * @param  pRxHeader: CAN接收帧头指针
 * @param  data: 接收到的数据
 * @retval 无
 */
void J60_ProcessReceivedData(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *pRxHeader, uint8_t *data)
{
    uint32_t can_id = pRxHeader->StdId;
    uint8_t local_id = can_id & 0x0F;  // 实际电机ID是低4位
    uint8_t is_rx = (can_id & J60_FLAG_RECV) != 0;
    uint32_t cmd_id = can_id & ~(0x1F); // 清除低5位，保留命令索引
    
    // 确保是接收帧
    if (!is_rx) {
        return;
    }

    // 确定CAN总线索引
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

    // 查找自定义ID，注意关节ID可能有+0x10的偏移(MOTOR_CONTROL命令)
    uint8_t adjusted_local_id = local_id;
    if ((cmd_id == J60_CMD_MOTOR_CONTROL) && (local_id >= 0x10)) {
        // 如果是MOTOR_CONTROL命令反馈，且ID有偏移，调整回原始ID
        adjusted_local_id = local_id - 0x10;
    }
    
    uint8_t custom_id = can_local_to_custom_map[can_bus_index][adjusted_local_id];
    if (custom_id > 15) {
        return; // 未映射的ID
    }
    
    // 根据命令处理数据
    switch (cmd_id) {
        case J60_CMD_MOTOR_CONTROL: {
            if (pRxHeader->DLC >= 8) {
                // 解析20位位置(3字节)
                uint32_t pos_uint20 = 
                    ((uint32_t)data[0]) | 
                    ((uint32_t)data[1] << 8) | 
                    (((uint32_t)data[2] & 0x0F) << 16);
                
                // 解析20位速度(2.5字节)
                uint32_t vel_uint20 = 
                    (((uint32_t)data[2] & 0xF0) >> 4) | 
                    ((uint32_t)data[3] << 4) | 
                    (((uint32_t)data[4] & 0xFF) << 12);
                
                // 解析16位扭矩(2字节)
                uint16_t torque_uint16 = 
                    ((uint16_t)data[5]) | 
                    ((uint16_t)(data[6] & 0xFF) << 8);
                
                // 解析温度标志位和温度值
                uint8_t temp_type = (data[7] & 0x01);
                uint8_t temp_uint7 = (data[7] >> 1) & 0x7F;
                
                // 转换为物理值
                float position = J60_MapUint20ToPosition(pos_uint20);
                float velocity = J60_MapUint20ToVelocity(vel_uint20);
                float torque = J60_MapUint16ToTorque(torque_uint16);
                float temperature = J60_MapUint7ToTemperature(temp_uint7);
                
                // 更新电机状态
                j60_motors[custom_id].position = position;
                j60_motors[custom_id].velocity = velocity;
                j60_motors[custom_id].torque = torque;
                j60_motors[custom_id].temperature = temperature;
                j60_motors[custom_id].temp_type = temp_type;
            }
            break;
        }
        
        // 保留其他命令的处理...
        
        default:
            break;
    }
}
/**
 * @brief  CAN接收中断回调函数
 * @param  hcan: CAN句柄指针
 * @retval 无
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
 * @brief  获取电机参数
 * @param  custom_id: 自定义电机ID (0-15)
 * @retval 电机参数结构体指针，如果ID无效返回NULL
 */
J60_Motor* J60_GetMotor(uint8_t custom_id)
{
    if (custom_id > 15 || j60_motors[custom_id].can_bus == CAN_BUS_NONE) {
        return NULL;
    }
    
    return &j60_motors[custom_id];
}
