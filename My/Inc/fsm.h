#ifndef _FSM_H
#define _FSM_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief ״̬ö�� - �����״ֻ̬���ڴ�ö�������
 */
typedef enum {
    STATE_INVALID = 0,  // ��Ч״̬
    STATE_PASSIVE,      // ����ģʽ
    STATE_STAND,        // վ��ģʽ
    STATE_TROT,         // С�ܲ�̬
    
    // ����״̬
    STATE_TEST1,
    STATE_TEST2,
    STATE_TEST3,
    
    STATE_COUNT         // ״̬����������������
} StateName;

/**
 * @brief ����ָ�����Ͷ���
 */
typedef void (*StateHandler)(void);               // �޲��޷���ֵ�ĺ���ָ��
typedef bool (*StateChecker)(StateName next);     // ״̬ת����麯��

/**
 * @brief ״̬���ṹ�嶨��
 */
typedef struct {
    StateHandler enter;          // ����״̬�Ĵ�����
    StateHandler run;            // ״̬����ʱ�Ĵ�����
    StateHandler exit;           // �˳�״̬�Ĵ�����
    StateChecker checkChange;  // ״̬ת����麯��
} FsmState;

/**
 * @brief ��ʼ��״̬��
 * @param initial_state ��ʼ״̬
 */
void fsm_init(StateName initial_state);

/**
 * @brief ע��״̬������
 * @param state ״̬ö��
 * @param enter ���뺯��
 * @param run ���к���
 * @param exit �˳�����
 * @param check ״̬ת����麯������ΪNULL��ʾ��������ת����
 * @param name ״̬����
 */
void fsm_register_state(StateName state, 
                        StateHandler enter, 
                        StateHandler run, 
                        StateHandler exit,
                        StateChecker check);

/**
 * @brief ����״̬��������ѭ���е��ã�
 */
void fsm_update(void);

/**
 * @brief ����״̬ת��
 * @param next_state Ŀ��״̬
 * @return �Ƿ�ɹ�����ת��
 */
bool fsm_change_to(StateName next_state);

/**
 * @brief ��ȡ��ǰ״̬
 * @return ��ǰ״̬ö��
 */
StateName fsm_get_current_state(void);

/**
 * @brief ��ȡ��һ��״̬
 * @return ��һ��״̬ö��
 */
StateName fsm_get_previous_state(void);

/**
 * @brief ��ȡ��һ��״̬
 * @return ��һ��״̬ö��
 */
StateName fsm_get_next_state(void);

void dog_fsm_init(void);

#endif /* _FSM_H */
