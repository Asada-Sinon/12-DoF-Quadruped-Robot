#ifndef _FSM_H
#define _FSM_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 状态枚举 - 添加新状态只需在此枚举中添加
 */
typedef enum {
    STATE_INVALID = 0,  // 无效状态
    STATE_PASSIVE,      // 阻尼模式
    STATE_STAND,        // 站立模式
    STATE_TROT,         // 小跑步态
    
    // 测试状态
    STATE_TEST1,
    STATE_TEST2,
    STATE_TEST3,
    
    STATE_COUNT         // 状态总数（必须放在最后）
} StateName;

/**
 * @brief 函数指针类型定义
 */
typedef void (*StateHandler)(void);               // 无参无返回值的函数指针
typedef bool (*StateChecker)(StateName next);     // 状态转换检查函数

/**
 * @brief 状态机结构体定义
 */
typedef struct {
    StateHandler enter;          // 进入状态的处理函数
    StateHandler run;            // 状态运行时的处理函数
    StateHandler exit;           // 退出状态的处理函数
    StateChecker checkChange;  // 状态转换检查函数
} FsmState;

/**
 * @brief 初始化状态机
 * @param initial_state 初始状态
 */
void fsm_init(StateName initial_state);

/**
 * @brief 注册状态处理函数
 * @param state 状态枚举
 * @param enter 进入函数
 * @param run 运行函数
 * @param exit 退出函数
 * @param check 状态转换检查函数（可为NULL表示总是允许转换）
 * @param name 状态名称
 */
void fsm_register_state(StateName state, 
                        StateHandler enter, 
                        StateHandler run, 
                        StateHandler exit,
                        StateChecker check);

/**
 * @brief 更新状态机（在主循环中调用）
 */
void fsm_update(void);

/**
 * @brief 请求状态转换
 * @param next_state 目标状态
 * @return 是否成功请求转换
 */
bool fsm_change_to(StateName next_state);

/**
 * @brief 获取当前状态
 * @return 当前状态枚举
 */
StateName fsm_get_current_state(void);

/**
 * @brief 获取上一个状态
 * @return 上一个状态枚举
 */
StateName fsm_get_previous_state(void);

/**
 * @brief 获取下一个状态
 * @return 下一个状态枚举
 */
StateName fsm_get_next_state(void);

void dog_fsm_init(void);

#endif /* _FSM_H */
