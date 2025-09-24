#include "fsm.h"
#include <string.h>
#include <stdio.h>

#include "dog.h"

/* 定义默认的空处理函数 */
static void default_handler(void) {
    /* 什么也不做 */
}

/* 默认的状态转换检查函数 - 始终允许转换 */
static bool default_checker(StateName next) {
    return true;
}

/* 状态机内部上下文 */
struct {
    StateName current;        // 当前状态
    StateName previous;       // 上一个状态
    StateName next;        // 待转换的状态
    bool change_enable;  //  是否允许切换状态
    FsmState states[STATE_COUNT];  // 状态处理函数表
} fsm_context;

/* 状态机初始化 */
void fsm_init(StateName initial_state) {
    // 确保初始状态有效
    if (initial_state >= STATE_COUNT) {
        initial_state = STATE_INVALID;
    }
    
    // 初始化所有状态为默认处理函数
    for (int i = 0; i < STATE_COUNT; i++) {
        fsm_context.states[i].enter = default_handler;
        fsm_context.states[i].run = default_handler;
        fsm_context.states[i].exit = default_handler;
        fsm_context.states[i].checkChange = default_checker;
    }
    
    // 设置初始状态
    fsm_context.current = initial_state;
    fsm_context.previous = initial_state;
    fsm_context.next = initial_state;
    fsm_context.change_enable = false;
    
    // 执行初始状态的enter函数
    fsm_context.states[initial_state].enter();
}

/* 注册状态处理函数 */
void fsm_register_state(StateName state, 
                       StateHandler enter, 
                       StateHandler run, 
                       StateHandler exit,
                       StateChecker check) {
    // 验证状态有效性
    if (state >= STATE_COUNT) {
        return;
    }
    
    // 注册状态处理函数
    if (enter) fsm_context.states[state].enter = enter;
    if (run) fsm_context.states[state].run = run;
    if (exit) fsm_context.states[state].exit = exit;
    if (check) fsm_context.states[state].checkChange = check;
}

/* 更新状态机 */
void fsm_update(void) {
    // 检查是否有待处理的状态转换
    // 检查是否允许从当前状态转换到目标状态
    if (fsm_context.change_enable && fsm_context.current != fsm_context.next && fsm_context.states[fsm_context.current].checkChange(fsm_context.next)) {
        StateName next = fsm_context.next;
        
        // 执行当前状态的退出函数
        fsm_context.states[fsm_context.current].exit();
        
        // 更新状态
        fsm_context.previous = fsm_context.current;
        fsm_context.current = next;
        fsm_context.change_enable = false;
        
        // 执行新状态的进入函数
        fsm_context.states[fsm_context.current].enter();
    }
    
    // 执行当前状态的运行函数
    fsm_context.states[fsm_context.current].run();
}

/* 请求状态转换 */
bool fsm_change_to(StateName next_state) {
    // 检查目标状态是否有效
    if (next_state >= STATE_COUNT) {
        return false;
    }
    // 设置待转换状态
    fsm_context.next = next_state;
    fsm_context.change_enable = true; 
    return true;
}

/* 获取当前状态 */
StateName fsm_get_current_state(void) {
    return fsm_context.current;
}

/* 获取上一个状态 */
StateName fsm_get_previous_state(void) {
    return fsm_context.previous;
}

/* 获取下一个状态 */
StateName fsm_get_next_state(void) {
    return fsm_context.next;
}

/* --------------------------------初始化 ------------------------------- */
// 前向声明所有状态注册函数
void register_stand_state(void);
void register_trot_state(void);
void register_passive_state(void);

void dog_fsm_init(void) {
    // 先初始化状态机框架
    fsm_init(STATE_INVALID);
    // 注册各个状态
    register_stand_state();  
    register_trot_state();
    register_passive_state();
}
