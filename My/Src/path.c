#include "path.h"
#include "dog.h"
#include "math.h"
#include "vision.h"
#include "estimator.h"
#include "imu.h"
#include "fsm.h"

// ·���滮����
typedef struct {
    float start_x;    // ���x����
    float start_y;    // ���y����
    float target_x;   // Ŀ���x����
    float target_y;   // Ŀ���y����
    float current_x;  // ��ǰx����
    float current_y;  // ��ǰy����
    float angle;      // ·��������ϵ�Ƕ�
    float yaw;        // Ŀ��ƫ����
    float distance;   // ��Ŀ���ľ���
    uint8_t is_moving;// �Ƿ����˶�
    
    // �ٶȿ��Ʋ���
    float max_speed;      // ����ٶ� (m/s)
    float current_speed;  // ��ǰ�ٶ� (m/s)
    float accel;         // ���ٶ� (m/s^2)
    float decel;         // ���ٶ� (m/s^2)
    
    // ·�����ٲ���
    float cross_track_error;  // ����������
    float kp_cross;          // ����������ϵ��
    float kd_cross;          // �������΢��ϵ��
    float kp_yaw;        // ����������ϵ��
    float kd_yaw;        // �������΢��ϵ��

    float dt; // ��������
} PathPlan;

PathPlan path = {0};

// ��ʼ��·���滮��
void path_init(void) {
    // �����ٶȺͼ��ٶȲ���
    path.max_speed = 0.3f;    // ����ٶ�0.3m/s
    path.accel = 0.2f;        // ���ٶ�0.2m/s^2
    path.decel = 0.3f;        // ���ٶ�0.3m/s^2
    
    // ���ÿ��Ʋ���
    path.kp_cross = 0.03;     // �����������ϵ��
    path.kd_cross = 0.0f;
    path.kp_yaw = 0.015;
    path.kd_yaw = 0.001;
    path.dt = 0.002f;         // ��������2ms
}


// ����Ŀ��� 
// x����������ϵ��Ŀ���x����
// y����������ϵ��Ŀ���y����
// yaw����������ϵ��Ŀ���ƫ����
// keep_yaw���Ƿ񱣳�ƫ���ǣ�Ϊ1ʱ����ƫ����Ϊyaw��Ϊ0ʱ����ƫ���ǳ���������ϵ����
void path_set_target(float x, float y, float yaw, uint8_t keep_yaw) {
    // ��ʼ��·������
    path_init();
    // ��ȡ��ǰλ����Ϊ���
    path.start_x = vision_get_pos(0);
    path.start_y = vision_get_pos(1);
    
    // ����Ŀ���
    path.target_x = x;
    path.target_y = y;
    
    // ����·���Ƕ�
    path.angle = atan2f(y - path.start_y, x - path.start_x);
    
    // Ŀ��ƫ����
    path.yaw = yaw;

    // �����ܾ���
    path.distance = sqrtf(powf(x - path.start_x, 2) + powf(y - path.start_y, 2));
    
    // ��ʼ���ٶ�
    path.current_speed = 0.0f;
    
    // �����˶�
    path.is_moving = 1;
}

float accel_distance = 0;
float decel_distance = 0;
// ����·��״̬
void path_update(void) {
    if (!path.is_moving) {
        return;
    }
    
    // ��ȡ��ǰ��������������ϵ������
    path.current_x = vision_get_pos(0);
    path.current_y = vision_get_pos(1);

    // ��ȡ��ǰ����ƫ����
    float current_yaw = imu_get_data()->angle[2]; // ��λdeg
    float current_w = imu_get_data()->gyro[2]; // ��λdeg/s

    // ��������������ϵ������ת����������ϵ��
    float line_x = cosf(path.angle) * path.current_x + sinf(path.angle) * path.current_y;
    float line_y = -sinf(path.angle) * path.current_x + cosf(path.angle) * path.current_y;
    
    // ��������ϵ���ٶ�ת������������ϵ��
    float world_current_vx = cosf(current_yaw * DEGREE_TO_RADIAN) * est_get_body_vel(0) - sinf(current_yaw * DEGREE_TO_RADIAN) * est_get_body_vel(1);
    float world_current_vy = sinf(current_yaw * DEGREE_TO_RADIAN) * est_get_body_vel(0) + cosf(current_yaw * DEGREE_TO_RADIAN) * est_get_body_vel(1);

    // ��������ϵ���ٶ�ת����������ϵ��
    float line_vx = cosf(path.angle) * world_current_vx + sinf(path.angle) * world_current_vy;
    float line_vy = -sinf(path.angle) * world_current_vx + cosf(path.angle) * world_current_vy;

    // ���㵱ǰ��Ŀ���ľ���
    float current_distance = sqrtf(powf(path.target_x - path.current_x, 2) + 
                                 powf(path.target_y - path.current_y, 2));
    
    // �������������
    path.cross_track_error = line_y;
    
    // �������Ŀ��㸽����ֹͣ�˶�
    if (current_distance < 5.0f) { // 5cm��Χ
        path.is_moving = 0;
        path.current_speed = 0.0f;
        dog_set_body_vel(0, 0, 0);
        fsm_change_to(STATE_STAND);
        return;
    }
    
    // ������ٺͼ�������ľ���
    accel_distance = 100 * (path.max_speed * path.max_speed) / (2.0f * path.accel);
    decel_distance = 100 * (path.current_speed * path.current_speed) / (2.0f * path.decel);
    
    // �ٶȹ滮
    if (current_distance > (accel_distance + decel_distance)) {
        // ���ʣ�������ڼ��ٺͼ�������ľ��룬����򱣳����ٽ׶�
        if (path.current_speed < path.max_speed) {
            // ���ٽ׶�
            path.current_speed += path.accel * path.dt;
            // ���ٽ׶�
            if (path.current_speed > path.max_speed) {
                path.current_speed = path.max_speed;
            }
        }
    } else {
        // ���ٽ׶�
        path.current_speed -= path.decel * path.dt;
    }
    
    // ������С�ٶ�
    if (path.current_speed < 0.05f) {
        path.current_speed = 0.05f;
    }
    
    // ������ϵy������yaw��pd����
    float line_correction_y = path.kp_cross * (0 - path.cross_track_error) + path.kd_cross * (0 - line_vy);
    float correction_w = path.kp_yaw * (path.yaw - current_yaw) + path.kd_yaw * (0 - current_w);

    // ������ϵת��������ϵ
    float world_vx = cosf(path.angle) * path.current_speed - sinf(path.angle) * line_correction_y;
    float world_vy = sinf(path.angle) * path.current_speed + cosf(path.angle) * line_correction_y;

    // ��������ϵת��������ϵ
    float body_vx = cosf(current_yaw * DEGREE_TO_RADIAN) * world_vx + sinf(current_yaw * DEGREE_TO_RADIAN) * world_vy;
    float body_vy = -sinf(current_yaw * DEGREE_TO_RADIAN) * world_vx + cosf(current_yaw * DEGREE_TO_RADIAN) * world_vy;

    // �����˶��ٶ�
    dog_set_body_vel(
        body_vx,
        body_vy,
        correction_w
    );
}

// ����Ƿ񵽴�Ŀ���
uint8_t path_is_finished(void) {
    return !path.is_moving;
}

// ��ȡ��ǰ·��״̬
void path_get_state(float *speed, float *cross_error) {
    if (speed != NULL) {
        *speed = path.current_speed;
    }
    if (cross_error != NULL) {
        *cross_error = path.cross_track_error;
    }
}
