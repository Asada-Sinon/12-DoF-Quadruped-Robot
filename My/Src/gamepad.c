#include "gamepad.h"
#include "dog.h"
#include "fsm.h"
#include "stdlib.h"

int16_t _channels[16];
// ͨ������
#define LEFT_X_CH   3
#define LEFT_Y_CH   2
#define RIGHT_X_CH  0
#define RIGHT_Y_CH  1

#define SWITCH_CH1   4
#define SWITCH_CH2   5
#define SWITCH_CH3   6
#define SWITCH_CH4   7

#define CHANNEL_MIDDLE 992 

#define GAMEPAD_CONNECTED (_channels[0] || _channels[1] || _channels[2] || _channels[3] || _channels[4] || _channels[5] || _channels[6] || _channels[7] || _channels[8] || _channels[9] || _channels[10] || _channels[11] || _channels[12] || _channels[13] || _channels[14] || _channels[15])

// ����λ����ֵ����
#define SWITCH_UP_THRESHOLD      500
#define SWITCH_DOWN_THRESHOLD    1500

// ����״̬�жϺ�
#define SWITCH_UP(ch)          (_channels[ch] < SWITCH_UP_THRESHOLD)
#define SWITCH_DOWN(ch)        (_channels[ch] > SWITCH_DOWN_THRESHOLD)
#define SWITCH_MIDDLE(ch)      (_channels[ch] <= SWITCH_DOWN_THRESHOLD && _channels[ch] >= SWITCH_UP_THRESHOLD)


float vx = 0.0f;
float vy = 0.0f;
float w = 0.0f;

float vx_smooth = 0.0f;
float vy_smooth = 0.0f;
float w_smooth = 0.0f;

int dead_zone = 50;
int big_dead_zone = 200;
float v_inc = 0.005f;
float v_dead_zone = 0.05f;
float vx_scale = 0.005;
float vy_scale = 0.005;
float w_scale = 0.001;

// ���ĵ�������
#define COG_ADJUST_STEP 0.01f

// ��¼��һ��ҡ��λ��
static int16_t prev_right_x = CHANNEL_MIDDLE;
static int16_t prev_right_y = CHANNEL_MIDDLE;

void HT10A_process(uint8_t buffer[30])
{
	_channels[0] = (buffer[1] | ((buffer[2] & 0x07) << 8)) & 0x07FF;
    // ͨ��1: bits 11-21 (byte2��5λ + byte3��6λ)
    _channels[1] = ((buffer[2] >> 3) | (buffer[3] << 5)) & 0x07FF;   
    // ͨ��2: bits 22-32 (byte3��2λ + byte4ȫ8λ + byte5��1λ)
    _channels[2] = ((buffer[3] >> 6) | (buffer[4] << 2) | ((buffer[5] & 0x01) << 10)) & 0x07FF;  
    // ͨ��3: bits 33-43 (byte5��7λ + byte6��4λ)
    _channels[3] = ((buffer[5] >> 1) | (buffer[6] << 7)) & 0x07FF;  
    // ͨ��4: bits 44-54 (byte6��4λ + byte7��7λ)
    _channels[4] = ((buffer[6] >> 4) | (buffer[7] << 4)) & 0x07FF;    
    // ͨ��5: bits 55-65 (byte7��1λ + byte8ȫ8λ + byte9��2λ)
    _channels[5] = ((buffer[7] >> 7) | (buffer[8] << 1) | ((buffer[9] & 0x03) << 9)) & 0x07FF;    
    // ͨ��6: bits 66-76 (byte9��6λ + byte10��5λ)
    _channels[6] = ((buffer[9] >> 2) | (buffer[10] << 6)) & 0x07FF;
    // ͨ��7: bits 77-87 (byte10��3λ + byte11ȫ8λ)
    _channels[7] = ((buffer[10] >> 5) | (buffer[11] << 3)) & 0x07FF;    
    // ͨ��8: bits 88-98 (byte12��8λ + byte13��3λ)
    _channels[8] = (buffer[12] | ((buffer[13] & 0x07) << 8)) & 0x07FF;    
    // ͨ��9: bits 99-109 (byte13��5λ + byte14��6λ)
    _channels[9] = ((buffer[13] >> 3) | (buffer[14] << 5)) & 0x07FF;    
    // ͨ��10: bits 110-120 (byte14��2λ + byte15ȫ8λ + byte16��1λ)
    _channels[10] = ((buffer[14] >> 6) | (buffer[15] << 2) | ((buffer[16] & 0x01) << 10)) & 0x07FF;    
    // ͨ��11: bits 121-131 (byte16��7λ + byte17��4λ)
    _channels[11] = ((buffer[16] >> 1) | (buffer[17] << 7)) & 0x07FF;    
    // ͨ��12: bits 132-142 (byte17��4λ + byte18��7λ)
    _channels[12] = ((buffer[17] >> 4) | (buffer[18] << 4)) & 0x07FF;    
    // ͨ��13: bits 143-153 (byte18��1λ + byte19ȫ8λ + byte20��2λ)
    _channels[13] = ((buffer[18] >> 7) | (buffer[19] << 1) | ((buffer[20] & 0x03) << 9)) & 0x07FF;    
    // ͨ��14: bits 154-164 (byte20��6λ + byte21��5λ)
    _channels[14] = ((buffer[20] >> 2) | (buffer[21] << 6)) & 0x07FF;    
    // ͨ��15: bits 165-175 (byte21��3λ + byte22ȫ8λ)
    _channels[15] = ((buffer[21] >> 5) | (buffer[22] << 3)) & 0x07FF;
    
    if (_channels[LEFT_Y_CH] - CHANNEL_MIDDLE > dead_zone)
        vx = (_channels[LEFT_Y_CH] - CHANNEL_MIDDLE - dead_zone) * vx_scale;
    else if (_channels[LEFT_Y_CH] - CHANNEL_MIDDLE < -dead_zone)
        vx = (_channels[LEFT_Y_CH] - CHANNEL_MIDDLE + dead_zone) * vx_scale;
    else
        vx = 0;

    if (_channels[LEFT_X_CH] - CHANNEL_MIDDLE > dead_zone)
        vy = -(_channels[LEFT_X_CH] - CHANNEL_MIDDLE - dead_zone) * vy_scale;
    else if (_channels[LEFT_X_CH] - CHANNEL_MIDDLE < -dead_zone)
        vy = -(_channels[LEFT_X_CH] - CHANNEL_MIDDLE + dead_zone) * vy_scale;
    else
        vy = 0;
    
    if (_channels[RIGHT_X_CH] - CHANNEL_MIDDLE > dead_zone)
        w = -(_channels[RIGHT_X_CH] - CHANNEL_MIDDLE - dead_zone) * w_scale;
    else if (_channels[RIGHT_X_CH] - CHANNEL_MIDDLE < -dead_zone)
        w = -(_channels[RIGHT_X_CH] - CHANNEL_MIDDLE + dead_zone) * w_scale;
    else
        w = 0;
}

uint8_t start_gamepad_control = 0;
void gamepad_control_init()
{
    start_gamepad_control = 1;
}

// �ֱ����ƻ����ٶ�
void gamepad_control()
{
    if (!start_gamepad_control || !GAMEPAD_CONNECTED)
        return;
        
    // ������4���м�λ��ʱ����ǰ�����˵�����
    if (!SWITCH_UP(SWITCH_CH4)) {
        // ��ȡ��ǰ����λ��
        float *cog_forward_offset = get_dog_params()->posture.center_of_gravity.trot_cog_forward_offset;
        float *cog_backward_offset = get_dog_params()->posture.center_of_gravity.trot_cog_backward_offset;
        
        // �����ҡ��Y�ᣨǰ�󣩱仯����������X��������
        if (abs(_channels[RIGHT_Y_CH] - CHANNEL_MIDDLE) > big_dead_zone && 
            abs(prev_right_y - CHANNEL_MIDDLE) <= big_dead_zone) {
            // ����λ��ʼ�ƶ�ҡ�˵�˲��
            if (_channels[RIGHT_Y_CH] > CHANNEL_MIDDLE) {
                // ��ǰ��
                if (SWITCH_MIDDLE(SWITCH_CH4)) { // �������4���м�λ��
                    cog_forward_offset[X_IDX] += COG_ADJUST_STEP;
                }
                else if(SWITCH_DOWN(SWITCH_CH4)) { // �������4������λ��
                    cog_backward_offset[X_IDX] += COG_ADJUST_STEP;
                }
            }   
            else {
                // �����
                if (SWITCH_MIDDLE(SWITCH_CH4)) { // �������4���м�λ��
                    cog_forward_offset[X_IDX] -= COG_ADJUST_STEP;
                }
                else if(SWITCH_DOWN(SWITCH_CH4)) { // �������4������λ��
                    cog_backward_offset[X_IDX] -= COG_ADJUST_STEP;
                }
            }
        }
        
        // �����ҡ��X�ᣨ���ң��仯������Y��������
        if (abs(_channels[RIGHT_X_CH] - CHANNEL_MIDDLE) > big_dead_zone && 
            abs(prev_right_x - CHANNEL_MIDDLE) <= big_dead_zone) {
            // ����λ��ʼ�ƶ�ҡ�˵�˲��
            if (_channels[RIGHT_X_CH] > CHANNEL_MIDDLE) {
                // ������
                if (SWITCH_MIDDLE(SWITCH_CH4)) { // �������4���м�λ��
                    cog_forward_offset[Y_IDX] -= COG_ADJUST_STEP;
                }
                else if(SWITCH_DOWN(SWITCH_CH4)) { // �������4������λ��
                    cog_backward_offset[Y_IDX] -= COG_ADJUST_STEP;
                }
            } else {
                // ������
                if (SWITCH_MIDDLE(SWITCH_CH4)) { // �������4���м�λ��
                    cog_forward_offset[Y_IDX] += COG_ADJUST_STEP;
                }
                else if(SWITCH_DOWN(SWITCH_CH4)) { // �������4������λ��
                    cog_backward_offset[Y_IDX] += COG_ADJUST_STEP;
                }
            }
        }
        // ������һ��ҡ��λ��
        prev_right_x = _channels[RIGHT_X_CH];
        prev_right_y = _channels[RIGHT_Y_CH];
        w = 0;
    } else {
        // �������ĵ���ģʽʱ������ҡ��λ�ü�¼
        prev_right_x = CHANNEL_MIDDLE;
        prev_right_y = CHANNEL_MIDDLE;
    }
    
    // ��������ٶ�
    vx_smooth = smooth(vx_smooth, vx, v_inc);
    vy_smooth = smooth(vy_smooth, vy, v_inc);
    w_smooth = smooth(w_smooth, w, v_inc);
    dog_set_body_vel(vx_smooth, vy_smooth, w_smooth);
    
    if (SWITCH_DOWN(SWITCH_CH3)) {
        fsm_change_to(STATE_PASSIVE);
        return;
    }   
    
    if (fabs(vx_smooth) < v_dead_zone && 
        fabs(vy_smooth) < v_dead_zone && 
        fabs(w_smooth) < v_dead_zone && 
        SWITCH_UP(SWITCH_CH2)) {
        fsm_change_to(STATE_STAND);
    } else {
        fsm_change_to(STATE_TROT);
    } 
}
