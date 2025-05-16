/************************************************************
 * ��������˿������˲���״̬������(18ά״̬����) - ����Cʵ��
 * ״̬����x: [pb_x, pb_y, pb_z, vb_x, vb_y, vb_z, 
 *            p0_x, p0_y, p0_z, p1_x, p1_y, p1_z, 
 *            p2_x, p2_y, p2_z, p3_x, p3_y, p3_z]
 * �������y: [psfB_0x, psfB_0y, psfB_0z, ..., psfB_3z,    // 12ά��������ϵ����˵�����λ��
 *            vsfB_0x, vsfB_0y, vsfB_0z, ..., vsfB_3z,     // 12ά��������ϵ����˵������ٶ�
 *            psz_0, psz_1, psz_2, psz_3]                  // 4ά��˸߶ȣ���ֹ�����ۣ���0
 * ����: IMU���ݡ���˽Ӵ����������ؽڱ���������
 * ���: ���ƵĻ���λ�á��ٶȡ����λ�ü������
 ************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "estimator.h"
#include "dog.h"  // ����dog.h��ʹ���Ȳ��˶�ѧ����
#include "matrix.h"
KalmanFilter kf;
ExtendedSensorData sensor_data;


// ��ʼ���������˲���
void init_kalman_filter(KalmanFilter* kf, float dt) {
    int i, j;
    
    // ����ʱ�䲽��
    kf->dt = dt;
    kf->prev_time = 0;
    
    // ��ʼ��״̬��������һʱ��״̬Ϊ0
    memset(kf->x, 0, sizeof(kf->x));
    memset(kf->prev_x, 0, sizeof(kf->prev_x));
    
    // ��ʼ���������Ϊ0
    memset(kf->y, 0, sizeof(kf->y));
    
    // ��ʼ��״̬Э�������Ϊ�Խ��� * ��ʼ��ȷ����
    memset(kf->P, 0, sizeof(kf->P));
    for (i = 0; i < STATE_DIM; i++) {
        // ����λ�����ٶȵĳ�ʼ��ȷ���Ƚϴ�
        if (i < 6) {
            kf->P[i][i] = 1.0f;
        } else {
            // ���λ�õĳ�ʼ��ȷ���Ƚ�С(��������˳�ʼվ����̬��֪)
            kf->P[i][i] = 0.1f;
        }
    }
    
    // ��ʼ��״̬ת�ƾ��� F (��ɢʱ��ϵͳ)
    memset(kf->F, 0, sizeof(kf->F));
    for (i = 0; i < STATE_DIM; i++) {
        kf->F[i][i] = 1.0f;  // ��λ���󲿷�
    }
    // λ�����ٶȵĹ�ϵ: pb(k+1) = pb(k) + vb(k)*dt
    kf->F[0][3] = kf->dt;  // pb_x��vb_x
    kf->F[1][4] = kf->dt;  // pb_y��vb_y
    kf->F[2][5] = kf->dt;  // pb_z��vb_z
    
    // ��ʼ������������� B
    memset(kf->B, 0, sizeof(kf->B));
    // ���ٶȶ��ٶȵ�Ӱ��: vb(k+1) = vb(k) + a(k)*dt
    kf->B[3][0] = kf->dt;  // vb_x��ax
    kf->B[4][1] = kf->dt;  // vb_y��ay
    kf->B[5][2] = kf->dt;  // vb_z��az
    
    // ��ʼ����������Э������� Q
    memset(kf->Q, 0, sizeof(kf->Q));
    // �����ʵ��Ĺ�������ֵ (����ϵͳ���Ե���)
    // ����λ������
    for (i = 0; i < 3; i++) 
        kf->Q[i][i] = 0.01f;
    // �����ٶ�����
    for (i = 3; i < 6; i++) 
        kf->Q[i][i] = 0.1f;
    // ���λ������(ȡ�����Ƿ�Ӵ�����)
    // ��update�����и��ݽӴ�״̬��̬����
    for (i = 6; i < STATE_DIM; i++) 
        kf->Q[i][i] = 0.05f;
    
    // ��ʼ���۲���� H �Ͳ�������Э������� R
    memset(kf->H, 0, sizeof(kf->H));
    memset(kf->R, 0, sizeof(kf->R));
    
    // ��ʼ��Ч��������
    kf->valid_measurements = 0;
}

// ���ݽӴ�״̬���¹۲�ģ��
void update_observation_model(KalmanFilter* kf, ExtendedSensorData* sensor_data) {
    int i, j;
    int meas_idx = 0;
    
    // ���ù۲����Ͳ�������Э����
    memset(kf->H, 0, sizeof(float) * MEAS_DIM * STATE_DIM);
    memset(kf->R, 0, sizeof(float) * MEAS_DIM * MEAS_DIM);
    
    // 1. IMU���ٶ����������ٶȹ���(��Ҫ������̬���㵽��������ϵ)
    // �ⲿ����Ҫ�Ӿֲ�����ϵת������������ϵ������򻯴���
    for (i = 0; i < 3; i++) {
        kf->H[meas_idx][i+3] = 1.0f;  // �ٶȹ۲�
        kf->R[meas_idx][meas_idx] = 0.1f;  // �ٶȲ�������
        meas_idx++;
    }
    
    // 2. ������˽Ӵ���Ϣ
    // ����Ӵ�����ʱ�������λ������������ϵ�б仯��С(���費��)
    for (i = 0; i < LEG_NUM; i++) {
        if (sensor_data->contact[i]) {
            // ����Ӵ����棬����λ�ò�����ӵ��۲�
            for (j = 0; j < 3; j++) {
                // ���λ����״̬�����е�����: 6+i*3+j
                kf->H[meas_idx][6+i*3+j] = 1.0f;
                // �Ӵ���λ������С����Ϊ�����ṩ��Լ��
                kf->R[meas_idx][meas_idx] = 0.01f;
                meas_idx++;
            }
        }
    }
    
    // 3. ʹ���˶�ѧģ����Ϊ����λ�õļ�Ӳ���
    // ����ÿ���Ӵ�������㣬���Ը��ݹؽڽǶȼ��������������λ��
    for (i = 0; i < LEG_NUM; i++) {
        if (sensor_data->contact[i]) {
            // ��ӻ���λ�������λ�õĹ�ϵԼ��
            for (j = 0; j < 3; j++) {
                // ����λ�� = ���λ�� - ����˵���������λ��(�������˶�ѧ�ṩ)
                kf->H[meas_idx][j] = 1.0f;  // ����λ��
                kf->H[meas_idx][6+i*3+j] = -1.0f;  // ���λ�õĸ�ֵ
                kf->R[meas_idx][meas_idx] = 0.05f;  // �˶�ѧԼ������
                meas_idx++;
            }
        }
    }
    
    // ��¼��Ч��������
    kf->valid_measurements = meas_idx;
}

// ׼����������
void prepare_measurement_vector(KalmanFilter* kf, ExtendedSensorData* sensor_data, float z[MEAS_DIM]) {
    int i, j;
    int meas_idx = 0;
    
    // 1. IMU���ٶȻ��ֵõ����ٶȹ���(�򻯴���)
    for (i = 0; i < 3; i++) {
        // ��������Ѿ����ֲ����ٶ�ת������������ϵ����������������
        // ʵ��Ӧ������Ҫʹ����Ԫ����ת�����������ת��
        z[meas_idx++] = kf->x[i+3] + sensor_data->acc[i] * kf->dt;
    }
    
    // 2. �Ӵ���˵�λ�ò���
    for (i = 0; i < LEG_NUM; i++) {
        if (sensor_data->contact[i]) {
            for (j = 0; j < 3; j++) {
                z[meas_idx++] = sensor_data->foot_pos[i*3+j];
            }
        }
    }
    
    // 3. ����λ�õļ�Ӳ���(ͨ���˶�ѧ)
    float relative_pos[3];
    for (i = 0; i < LEG_NUM; i++) {
        if (sensor_data->contact[i]) {
            // �������˵���������λ��(�������˶�ѧ�ṩ)
            // ����ʹ�����е��˶�ѧ��������ӹؽڽǶȵ����λ�õ�ӳ��
            float joint_pos[3];
            float foot_to_body[3];
            
            // �ӹؽ���������ȡ��ǰ�ȵĹؽڽǶ�
            memcpy(joint_pos, &sensor_data->joint_angles[i*3], 3 * sizeof(float));
            
            // �������������Źؽڵ�λ��
            leg_forward_kinematics(i, joint_pos, foot_to_body);
            
            // �������˵���������λ��(�����Źؽ�λ�ڻ�������ϵԭ��)
            relative_pos[0] = -foot_to_body[0];
            relative_pos[1] = -foot_to_body[1];
            relative_pos[2] = -foot_to_body[2];
            
            // ����λ�� = ���λ�� - ���λ��
            for (j = 0; j < 3; j++) {
                z[meas_idx++] = sensor_data->foot_pos[i*3+j] - relative_pos[j];
            }
        }
    }
}

// ������������Э�����������Ƿ�Ӵ�����
void adjust_process_noise(KalmanFilter* kf, ExtendedSensorData* sensor_data) {
    int i, j;
    
    // �������λ�õĹ�������
    for (i = 0; i < LEG_NUM; i++) {
        // �Ӵ���������λ�ñ仯��С(����С)�����յ����λ�ñ仯��(������)
        float foot_noise = sensor_data->contact[i] ? 0.001f : 0.1f;
        for (j = 0; j < 3; j++) {
            kf->Q[6+i*3+j][6+i*3+j] = foot_noise;
        }
    }
}

// �������˲���Ԥ�ⲽ��
void kf_predict(KalmanFilter* kf, float u[CTRL_DIM]) {
    int i, j, k;
    float temp_x[MAX_MATRIX_DIM] = {0};
    float temp_matrix[MAX_MATRIX_DIM][MAX_MATRIX_DIM] = {0};
    float Fx[MAX_MATRIX_DIM] = {0};
    float Bu[MAX_MATRIX_DIM] = {0};
    float FP[MAX_MATRIX_DIM][MAX_MATRIX_DIM] = {0};
    float F_transpose[MAX_MATRIX_DIM][MAX_MATRIX_DIM] = {0};
    
    // ������һʱ��״̬(���ڼ����ٶ�)
    memcpy(kf->prev_x, kf->x, sizeof(kf->x));
    kf->prev_time += kf->dt;
    
    // 1. Ԥ��״̬: x(k|k-1) = F * x(k-1|k-1) + B * u(k)
    
    // ���� Fx
    mat_mult_vec_ptr(STATE_DIM, STATE_DIM, &kf->F[0][0], kf->x, Fx);
    
    // ����п�������u������ Bu
    if (u != NULL) {
        mat_mult_vec_ptr(STATE_DIM, CTRL_DIM, &kf->B[0][0], u, Bu);
        
        // ���� x = Fx + Bu
        vec_add_ptr(STATE_DIM, Fx, Bu, kf->x);
    } else {
        // ���û�п������룬ֱ��ʹ�� x = Fx
        memcpy(kf->x, Fx, sizeof(Fx));
    }
    
    // 2. Ԥ�����Э����: P(k|k-1) = F * P(k-1|k-1) * F^T + Q
    
    // ���� FP = F * P
    mat_mult_ptr(STATE_DIM, STATE_DIM, STATE_DIM, &kf->F[0][0], &kf->P[0][0], &FP[0][0]);
    
    // ���� F ��ת��
    mat_transpose_ptr(STATE_DIM, STATE_DIM, &kf->F[0][0], &F_transpose[0][0]);
    
    // ���� FPFT = FP * F^T
    mat_mult_ptr(STATE_DIM, STATE_DIM, STATE_DIM, &FP[0][0], &F_transpose[0][0], &temp_matrix[0][0]);
    
    // ���� P = FPFT + Q
    mat_add_ptr(STATE_DIM, STATE_DIM, &temp_matrix[0][0], &kf->Q[0][0], &kf->P[0][0]);
}

// �������˲������²���
void kf_update(KalmanFilter* kf, float z[MEAS_DIM]) {
    int i, j, k;
    int valid_meas = kf->valid_measurements;
    
    if (valid_meas <= 0) return; // û����Ч��������������
    
    // ��ʱ����
    float Hx[MAX_VALID_MEAS] = {0};
    float innovation[MAX_VALID_MEAS] = {0};
    float HP[MAX_VALID_MEAS][MAX_MATRIX_DIM] = {0};
    float H_transpose[MAX_MATRIX_DIM][MAX_VALID_MEAS] = {0};
    float HPHT[MAX_VALID_MEAS][MAX_VALID_MEAS] = {0};
    float S[MAX_VALID_MEAS][MAX_VALID_MEAS] = {0};
    float S_inv[MAX_VALID_MEAS][MAX_VALID_MEAS] = {0};
    float PHT[MAX_MATRIX_DIM][MAX_VALID_MEAS] = {0};
    float K[MAX_MATRIX_DIM][MAX_VALID_MEAS] = {0};
    float Ky[MAX_MATRIX_DIM] = {0};
    float KH[MAX_MATRIX_DIM][MAX_MATRIX_DIM] = {0};
    float IKH[MAX_MATRIX_DIM][MAX_MATRIX_DIM] = {0};
    float temp_P[MAX_MATRIX_DIM][MAX_MATRIX_DIM] = {0};
    
    // ��ʱ�����;��� (Ϊ����Ӧ�仯����Ч��������)
    float H_valid[MAX_VALID_MEAS][MAX_MATRIX_DIM] = {0};
    float R_valid[MAX_VALID_MEAS][MAX_VALID_MEAS] = {0};
    float z_valid[MAX_VALID_MEAS] = {0};
    
    // ��ȡ��Ч��H, R��z
    for (i = 0; i < valid_meas; i++) {
        z_valid[i] = z[i];
        for (j = 0; j < valid_meas; j++) {
            R_valid[i][j] = kf->R[i][j];
        }
        for (j = 0; j < STATE_DIM; j++) {
            H_valid[i][j] = kf->H[i][j];
        }
    }
    
    // 0. ��������в�: innovation = z - H*x
    
    // ���� H * x
    mat_mult_vec_ptr(valid_meas, STATE_DIM, &H_valid[0][0], kf->x, Hx);
    
    // ���� innovation = z - Hx
    vec_sub_ptr(valid_meas, z_valid, Hx, innovation);
    
    // 1. ����в�Э����: S = H * P * H^T + R
    
    // ���� HP = H * P
    mat_mult_ptr(valid_meas, STATE_DIM, STATE_DIM, &H_valid[0][0], &kf->P[0][0], &HP[0][0]);
    
    // ���� H ��ת��
    mat_transpose_ptr(valid_meas, STATE_DIM, &H_valid[0][0], &H_transpose[0][0]);
    
    // ���� HPHT = HP * H^T
    mat_mult_ptr(valid_meas, STATE_DIM, valid_meas, &HP[0][0], &H_transpose[0][0], &HPHT[0][0]);
    
    // ���� S = HPHT + R
    mat_add_ptr(valid_meas, valid_meas, &HPHT[0][0], &R_valid[0][0], &S[0][0]);
    
    // 2. ���㿨��������: K = P * H^T * S^(-1)
    
    // ���� S �������
    if (mat_inverse_ptr(valid_meas, &S[0][0], &S_inv[0][0]) != MAT_SUCCESS) {
        printf("����: ��������ʧ��\n");
        return;
    }
    
    // ���� P * H^T
    mat_mult_ptr(STATE_DIM, STATE_DIM, valid_meas, &kf->P[0][0], &H_transpose[0][0], &PHT[0][0]);
    
    // ���� K = PHT * S_inv
    mat_mult_ptr(STATE_DIM, valid_meas, valid_meas, &PHT[0][0], &S_inv[0][0], &K[0][0]);
    
    // 3. ����״̬����: x(k|k) = x(k|k-1) + K * innovation
    
    // ���� K * innovation
    mat_mult_vec_ptr(STATE_DIM, valid_meas, &K[0][0], innovation, Ky);
    
    // ����״̬����: x = x + Ky
    vec_add_ptr(STATE_DIM, kf->x, Ky, kf->x);
    
    // 4. �������Э����: P(k|k) = (I - K*H) * P(k|k-1)
    
    // ���� KH = K * H
    mat_mult_ptr(STATE_DIM, valid_meas, STATE_DIM, &K[0][0], &H_valid[0][0], &KH[0][0]);
    
    // ���� I - KH
    for (i = 0; i < STATE_DIM; i++) {
        for (j = 0; j < STATE_DIM; j++) {
            IKH[i][j] = (i == j ? 1.0f : 0.0f) - KH[i][j];
        }
    }
    
    // ���� (I - KH) * P
    mat_mult_ptr(STATE_DIM, STATE_DIM, STATE_DIM, &IKH[0][0], &kf->P[0][0], &temp_P[0][0]);
        
    // �������Э����
    memcpy(kf->P, temp_P, sizeof(temp_P));
}

// ����28ά�������
void compute_output_vector(KalmanFilter* kf) {
    int i, j, idx = 0;
    
    // ����������λ������ psfB (12ά)
    for (i = 0; i < LEG_NUM; i++) {
        for (j = 0; j < 3; j++) {
            // ���λ�ü�ȥ����λ��
            kf->y[idx++] = kf->x[6 + i*3 + j] - kf->x[j];
        }
    }
    
    // �����������ٶ����� vsfB (12ά)
    for (i = 0; i < LEG_NUM; i++) {
        for (j = 0; j < 3; j++) {
            // �������λ�ñ仯�ʼ�ȥ�����ٶ�
            // (��ǰ���λ�� - ��һʱ�����λ��)/dt - �����ٶ�
            float foot_vel = (kf->x[6 + i*3 + j] - kf->prev_x[6 + i*3 + j]) / kf->dt;
            kf->y[12 + idx++] = foot_vel - kf->x[3 + j];
        }
    }
    
    // ������˸߶� psz (4ά)
    for (i = 0; i < LEG_NUM; i++) {
        // ���z������Ǹ߶�
        kf->y[24 + i] = kf->x[6 + i*3 + 2];
    }
}


// ��ȡ28ά�������
void kf_get_output_vector(KalmanFilter* kf, float output[OUTPUT_DIM]) {
    memcpy(output, kf->y, OUTPUT_DIM * sizeof(float));
}

// ��ȡ������λ������
void kf_get_foot_relative_positions(KalmanFilter* kf, float psfB[LEG_NUM][3]) {
    for (int i = 0; i < LEG_NUM; i++) {
        memcpy(psfB[i], &kf->y[i*3], 3 * sizeof(float));
    }
}

// ��ȡ�������ٶ�����
void kf_get_foot_relative_velocities(KalmanFilter* kf, float vsfB[LEG_NUM][3]) {
    for (int i = 0; i < LEG_NUM; i++) {
        memcpy(vsfB[i], &kf->y[12 + i*3], 3 * sizeof(float));
    }
}

/**
 * ����Ԫ������ZYX��ת����
 * ����: ��Ԫ�� q[4] = [w, x, y, z]
 * ���: 3x3��ת���� R��ʹ��ZYX��ת˳��(��yaw����pitch�����roll)
 */
void quaternion_to_rotation_matrix_zyx(const float q[4], float R[3][3]) {
    float w = q[0];
    float x = q[1];
    float y = q[2];
    float z = q[3];
    
    // ��һ����Ԫ����ȷ����λ����
    float norm = sqrtf(w*w + x*x + y*y + z*z);
    if (norm < 1e-6f) {
        // �����Ԫ���ӽ��㣬���ص�λ����
        R[0][0] = 1.0f; R[0][1] = 0.0f; R[0][2] = 0.0f;
        R[1][0] = 0.0f; R[1][1] = 1.0f; R[1][2] = 0.0f;
        R[2][0] = 0.0f; R[2][1] = 0.0f; R[2][2] = 1.0f;
        return;
    }
    
    w /= norm;
    x /= norm;
    y /= norm;
    z /= norm;
    
    // ������Ԫ����ƽ����
    float w2 = w*w;
    float x2 = x*x;
    float y2 = y*y;
    float z2 = z*z;
    
    // ����˻���
    float xy = x*y;
    float xz = x*z;
    float yz = y*z;
    float wx = w*x;
    float wy = w*y;
    float wz = w*z;
    
    // ������ת����
    R[0][0] = w2 + x2 - y2 - z2;  // 1 - 2*(y2 + z2)
    R[0][1] = 2.0f * (xy - wz);
    R[0][2] = 2.0f * (xz + wy);
    
    R[1][0] = 2.0f * (xy + wz);
    R[1][1] = w2 - x2 + y2 - z2;  // 1 - 2*(x2 + z2)
    R[1][2] = 2.0f * (yz - wx);
    
    R[2][0] = 2.0f * (xz - wy);
    R[2][1] = 2.0f * (yz + wx);
    R[2][2] = w2 - x2 - y2 + z2;  // 1 - 2*(x2 + y2)
}


/**
 * �����ٶȴӻ�������ϵת������������ϵ
 * ʹ��IMU�ṩ����Ԫ��
 */
void body_to_world_acc(float R[3][3], const float acc_body[3], float acc_world[3]) {
    // Ӧ����ת�任
    mat_mult_vec_ptr(3, 3, &R[0][0], acc_body, acc_world);  // ʹ��ָ��汾
    // �����������ٶ� (������������ϵZ������)
    acc_world[2] += 9.81f;
}

//������������ϵ���������ڻ����λ��
void world_foot_to_body_pos(uint8_t leg_id, const float R[3][3], float PsfBi[3]) {
    float PbfBi[3];
    leg_get_current_foot_pos_body(leg_id, PbfBi);
    mat_mult_vec_ptr(3, 3, &R[0][0], PbfBi, PsfBi);  // ���&R[0][0]�Ի�ȡָ��
}

// ������������ϵ���������ڻ�����ٶ�
void world_foot_to_body_vel(uint8_t leg_id, const float R[3][3], float VsfBi[3]) {
    float PbfBi[3];
    float WbxPbfBi[3];
    float VbfBi[3];
    float Wb_add_Vb[3];
    float joint_pos[3];
    float joint_vel[3];
    leg_get_current_foot_pos_body(leg_id, PbfBi);
    vec3_cross(sensor_data.gyro, PbfBi, WbxPbfBi);  // ʹ��vec3_cross���vec_cross
    leg_get_current_joint_pos(leg_id, joint_pos);
    leg_get_current_joint_vel(leg_id, joint_vel);
    leg_forward_kinematics_vel(leg_id, joint_pos, joint_vel, VbfBi);
    vec_add_ptr(3, WbxPbfBi, VbfBi, Wb_add_Vb);
    mat_mult_vec_ptr(3, 3, &R[0][0], Wb_add_Vb, VsfBi);  // ���&R[0][0]�Ի�ȡָ��
}

// �������˲�������������
void process_quadruped_state_estimation() {
    // 1. ���ݽӴ�״̬������������
    adjust_process_noise(&kf, &sensor_data);
    
    // 2. ���¹۲�ģ��
    update_observation_model(&kf, &sensor_data);
    
    // 3. ׼����������(IMU���ٶ�����������ϵ��)
    float u[CTRL_DIM];
    // �����ٶȴӻ�������ϵת������������ϵ 
    // ʹ����Ԫ����ת������򻯴���
//    transform_acceleration_to_world_frame(sensor_data.quaternion, sensor_data.acc, u);
    // 4. Ԥ�ⲽ��
    kf_predict(&kf, u);
    
    // 5. ׼����������
    float z[MEAS_DIM];
    prepare_measurement_vector(&kf, &sensor_data, z);
    
    // 6. ���²���
    kf_update(&kf, z);
    
    // 7. �����������
    compute_output_vector(&kf);
}

float dt[3] = { 0.002, 0.002, 0.002 }; // ��ɢ������ʱ�䣬��������s
    float I3[3][3] = {
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1}
    };
    float _I3[3][3] = {
            {-1, 0, 0},
            {0, -1, 0},
            {0, 0, -1}
    };
    float I3dt[3][3] = {
        {0.002, 0, 0},
        {0, 0.002, 0},
        {0, 0, 0.002}
    };
    float I12[12][12] = {
        {1,0,0,0,0,0,0,0,0,0,0,0},
        {0,1,0,0,0,0,0,0,0,0,0,0},
        {0,0,1,0,0,0,0,0,0,0,0,0},
        {0,0,0,1,0,0,0,0,0,0,0,0},
        {0,0,0,0,1,0,0,0,0,0,0,0},
        {0,0,0,0,0,1,0,0,0,0,0,0},
        {0,0,0,0,0,0,1,0,0,0,0,0},
        {0,0,0,0,0,0,0,1,0,0,0,0},
        {0,0,0,0,0,0,0,0,1,0,0,0},
        {0,0,0,0,0,0,0,0,0,1,0,0},
        {0,0,0,0,0,0,0,0,0,0,1,0},
        {0,0,0,0,0,0,0,0,0,0,0,1}
    };
    float I18[18][18] = {
        {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1}
    };

void estimation_init() {
    // ������ɢ��״̬ת�ƾ���F = I + dt*A
    //[I3 I3dt 0]
    //[    I3   ]
    //[      I12] 18x18
    memset(kf.F, 0, sizeof(kf.F));
    mat_add_block_ptr(0, 0, 3, 3, 18, &kf.F[0][0], &I3[0][0], &kf.F[0][0]);
    mat_add_block_ptr(0, 3, 3, 3, 18, &kf.F[0][0], &I3dt[0][0], &kf.F[0][0]);
    mat_add_block_ptr(3, 3, 3, 3, 18, &kf.F[0][0], &I3[0][0], &kf.F[0][0]);
    mat_add_block_ptr(6, 6, 12, 12, 18, &kf.F[0][0], &I12[0][0], &kf.F[0][0]);
    // ������ɢ�����ƾ���B = dt*B
    memset(kf.B, 0, sizeof(kf.B));
    mat_add_block_ptr(0, 3, 3, 3, 18, &kf.B[0][0], &I3dt[0][0], &kf.B[0][0]);
    // ������ɢ���۲����H = C
    // [-I3 0 I3 0 0 0]
    // [-I3 0 0 I3 0 0]
    // [-I3 0 0 0 I3 0]
    // [-I3 0 0 0 0 I3]
    // [0 -I3 0 0 0 0]
    // [0 -I3 0 0 0 0]
    // [0 -I3 0 0 0 0]
    // [0 -I3 0 0 0 0]
    // [0 0 [0 0 1] 0 0 0]
    // [0 0 0 [0 0 1] 0 0]
    // [0 0 0 0 [0 0 1] 0]
    // [0 0 0 0 0 [0 0 1]] 28x18
    memset(kf.H, 0, sizeof(kf.H));
    mat_add_block_ptr(0, 0, 3, 3, 18, &kf.H[0][0], &_I3[0][0], &kf.H[0][0]);
    mat_add_block_ptr(3, 0, 3, 3, 18, &kf.H[0][0], &_I3[0][0], &kf.H[0][0]);
    mat_add_block_ptr(6, 0, 3, 3, 18, &kf.H[0][0], &_I3[0][0], &kf.H[0][0]);
    mat_add_block_ptr(9, 0, 3, 3, 18, &kf.H[0][0], &_I3[0][0], &kf.H[0][0]);
    mat_add_block_ptr(12, 3, 3, 3, 18, &kf.H[0][0], &_I3[0][0], &kf.H[0][0]);
    mat_add_block_ptr(15, 3, 3, 3, 18, &kf.H[0][0], &_I3[0][0], &kf.H[0][0]);
    mat_add_block_ptr(18, 3, 3, 3, 18, &kf.H[0][0], &_I3[0][0], &kf.H[0][0]);
    mat_add_block_ptr(21, 3, 3, 3, 18, &kf.H[0][0], &_I3[0][0], &kf.H[0][0]);
    mat_add_block_ptr(0, 6, 12, 12, 18, &kf.H[0][0], &I12[0][0], &kf.H[0][0]);
    kf.H[24][8] = 1;
    kf.H[25][11] = 1;
    kf.H[26][14] = 1;
    kf.H[27][17] = 1;

}

void estimation_run() {
    /* -----------�м����------------- */
    float Rot[3][3] = {0};
    float u[3] = {0};
    float xhat[STATE_DIM] = {0};
    float yhat[OUTPUT_DIM] = {0};
    float Ppri[STATE_DIM][STATE_DIM] = {0};
    float PpriT[STATE_DIM][STATE_DIM] = {0};
    float AP[STATE_DIM][STATE_DIM] = {0};
    float AT[STATE_DIM][STATE_DIM] = {0};
    float CT[STATE_DIM][OUTPUT_DIM] = {0};
    float CP[OUTPUT_DIM][STATE_DIM] = {0};
    float S[OUTPUT_DIM][OUTPUT_DIM] = {0};
    float ST[OUTPUT_DIM][OUTPUT_DIM] = {0};
    float SL[OUTPUT_DIM][OUTPUT_DIM] = {0};
    float SU[OUTPUT_DIM][OUTPUT_DIM] = {0};
    int LU_P[OUTPUT_DIM] = {0};
    float invSy[OUTPUT_DIM] = {0};
    float y_yhat[OUTPUT_DIM] = {0};
    float invSC[OUTPUT_DIM][STATE_DIM] = {0};
    float invSR[OUTPUT_DIM][OUTPUT_DIM] = {0};
    float invSTC[STATE_DIM][OUTPUT_DIM] = {0};
    float KC[STATE_DIM][STATE_DIM] = {0};
    float I_KC[STATE_DIM][STATE_DIM] = {0};
    float PCT[STATE_DIM][OUTPUT_DIM] = {0};
    float I_KCT[STATE_DIM][STATE_DIM] = {0};
    float I_KCPI_KCT[STATE_DIM][STATE_DIM] = {0};
    float KR[STATE_DIM][OUTPUT_DIM] = {0};
    float KRKT[STATE_DIM][STATE_DIM] = {0};
    /* -----------���ݴ���״̬����Э�������------------- */
    /* -----------Ԥ�ⲿ��------------- */
    // R
    quaternion_to_rotation_matrix_zyx(sensor_data.quaternion, Rot);
    // u = R * acc + g
    body_to_world_acc(Rot, sensor_data.acc, u);
    // xhat = A * x + B * u
    mat_mult_vec_ptr(STATE_DIM, STATE_DIM, &kf.F[0][0], kf.x, xhat);
    vec_add_ptr(STATE_DIM, xhat, u, xhat);
    // yhat = C * x
    mat_mult_vec_ptr(OUTPUT_DIM, STATE_DIM, &kf.H[0][0], kf.x, yhat);
    // Ppri = A * P * A^T + Q
    mat_mult_ptr(STATE_DIM, STATE_DIM, STATE_DIM, &kf.F[0][0], &kf.P[0][0], &AP[0][0]);
    mat_transpose_ptr(STATE_DIM, STATE_DIM, &kf.F[0][0], &AT[0][0]);
    mat_mult_ptr(STATE_DIM, STATE_DIM, STATE_DIM, &AP[0][0], &AT[0][0], &Ppri[0][0]);
    mat_add_ptr(STATE_DIM, STATE_DIM, &Ppri[0][0], &kf.Q[0][0], &Ppri[0][0]);

    /* -----------���²���------------- */
    // S = R + C * Ppri * C^T
    mat_mult_ptr(OUTPUT_DIM, STATE_DIM, STATE_DIM, &kf.H[0][0], &Ppri[0][0], &CP[0][0]);
    mat_transpose_ptr(OUTPUT_DIM, STATE_DIM, &kf.H[0][0], &CT[0][0]);
    mat_mult_ptr(OUTPUT_DIM, STATE_DIM, OUTPUT_DIM, &CP[0][0], &CT[0][0], &S[0][0]);
    mat_add_ptr(OUTPUT_DIM, OUTPUT_DIM, &S[0][0], &kf.R[0][0], &S[0][0]);
    // S^-1(y - yhat) 28x1
    vec_sub_ptr(OUTPUT_DIM, kf.y, yhat, y_yhat);
    mat_lu_decomp_ptr(OUTPUT_DIM, &S[0][0], &SL[0][0], &SU[0][0], LU_P);
    mat_solve_with_lu_ptr(OUTPUT_DIM, &SL[0][0], &SU[0][0], LU_P, 1, y_yhat, invSy);
    // S^-1 * C 28x18
    mat_solve_with_lu_ptr(OUTPUT_DIM, &SL[0][0], &SU[0][0], LU_P, STATE_DIM, &kf.H[0][0], &invSC[0][0]);
    // S^-1 * R 28x28
    mat_solve_with_lu_ptr(OUTPUT_DIM, &SL[0][0], &SU[0][0], LU_P, OUTPUT_DIM, &kf.R[0][0], &invSR[0][0]);
    // (S^T)^-1 * C 28x18
    mat_transpose_ptr(OUTPUT_DIM, OUTPUT_DIM, &S[0][0], &ST[0][0]);
    mat_solve_with_lu_ptr(OUTPUT_DIM, &SL[0][0], &SU[0][0], LU_P, STATE_DIM, &kf.H[0][0], &invSTC[0][0]);

    // I - Ppri * C^T * S^-1 * C 18x18
    mat_mult_ptr(STATE_DIM, STATE_DIM, OUTPUT_DIM, &Ppri[0][0], &CT[0][0], &PCT[0][0]);
    mat_mult_ptr(STATE_DIM, OUTPUT_DIM, STATE_DIM, &PCT[0][0], &invSC[0][0], &KC[0][0]);
    mat_sub_ptr(STATE_DIM, STATE_DIM, &I18[0][0], &KC[0][0], &I_KC[0][0]);
    // x = xhat + Ppri * C^T * S^-1 * (y - yhat) 18x1
    mat_mult_ptr(STATE_DIM, OUTPUT_DIM, 1, &PCT[0][0], &invSy[0], &kf.x[0]);
    vec_add_ptr(STATE_DIM, xhat, &kf.x[0], &kf.x[0]);
    // P = (I-KC) * Ppri * (I-KC)^T + Ppri * C^T * S^-1 * R * (S^T)^-1 * C * Ppri^T
    mat_mult_ptr(STATE_DIM, STATE_DIM, STATE_DIM, &I_KC[0][0], &Ppri[0][0], &I_KCPI_KCT[0][0]);
    mat_mult_ptr(STATE_DIM, STATE_DIM, STATE_DIM, &I_KCPI_KCT[0][0], &I_KCT[0][0], &I_KCPI_KCT[0][0]);
    mat_mult_ptr(STATE_DIM, OUTPUT_DIM, OUTPUT_DIM, &PCT[0][0], &invSR[0][0], &KR[0][0]);
    mat_mult_ptr(STATE_DIM, OUTPUT_DIM, STATE_DIM, &KR[0][0], &invSTC[0][0], &KRKT[0][0]);
    mat_transpose_ptr(STATE_DIM, STATE_DIM, &Ppri[0][0], &PpriT[0][0]);
    mat_mult_ptr(STATE_DIM, STATE_DIM, STATE_DIM, &KRKT[0][0], &PpriT[0][0], &KRKT[0][0]);
    mat_add_ptr(STATE_DIM, STATE_DIM, &I_KCPI_KCT[0][0], &KRKT[0][0], &kf.P[0][0]);
}