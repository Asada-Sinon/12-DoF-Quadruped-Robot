#include "matrix.h"
#include "math.h"
#include "stdio.h"
#include "stm32f767xx.h"
#include "arm_math.h"  // HALDSP��
// �Զ��������㺯��
/**
 * ����˷�: C = A * B (ʹ��HALDSP��)
 * @param[in] m A���������
 * @param[in] n A�����������B���������
 * @param[in] p B���������
 * @param[in] A �������A����СΪm��n�����д洢
 * @param[in] B �������B����СΪn��p�����д洢
 * @param[out] C �������C����СΪm��p�����д洢
 */
void mat_mult_ptr(int m, int n, int p, const float *A, const float *B, float *C) {
    arm_matrix_instance_f32 mat_A, mat_B, mat_C;
    
    // ��ʼ������ʵ��
    arm_mat_init_f32(&mat_A, m, n, (float *)A);
    arm_mat_init_f32(&mat_B, n, p, (float *)B);
    arm_mat_init_f32(&mat_C, m, p, C);
    
    // ִ�о���˷�
    arm_mat_mult_f32(&mat_A, &mat_B, &mat_C);
}

/**
 * �����������: y = A * x (ʹ��HALDSP��)
 * @param[in] m A���������
 * @param[in] n A�����������x������ά��
 * @param[in] A �������A����СΪm��n�����д洢
 * @param[in] x ��������x����СΪn
 * @param[out] y �������y����СΪm
 */
void mat_mult_vec_ptr(int m, int n, const float *A, const float *x, float *y) {
    arm_matrix_instance_f32 mat_A, vec_x, vec_y;
    
    // ��ʼ�����������ʵ��
    arm_mat_init_f32(&mat_A, m, n, (float *)A);
    arm_mat_init_f32(&vec_x, n, 1, (float *)x);
    arm_mat_init_f32(&vec_y, m, 1, y);
    
    // ִ�о��������˷�
    arm_mat_mult_f32(&mat_A, &vec_x, &vec_y);
}

/**
 * ����ӷ�: C = A + B (ʹ��HALDSP��)
 * @param[in] m ���������
 * @param[in] n ���������
 * @param[in] A �������A����СΪm��n�����д洢
 * @param[in] B �������B����СΪm��n�����д洢
 * @param[out] C �������C����СΪm��n�����д洢
 */
void mat_add_ptr(int m, int n, const float *A, const float *B, float *C) {
    arm_matrix_instance_f32 mat_A, mat_B, mat_C;
    
    // ��ʼ������ʵ��
    arm_mat_init_f32(&mat_A, m, n, (float *)A);
    arm_mat_init_f32(&mat_B, m, n, (float *)B);
    arm_mat_init_f32(&mat_C, m, n, C);
    
    // ִ�о���ӷ�
    arm_mat_add_f32(&mat_A, &mat_B, &mat_C);
}

/**
 * �������: C = A - B (ʹ��HALDSP��)
 * @param[in] m ���������
 * @param[in] n ���������
 * @param[in] A �������A����СΪm��n�����д洢
 * @param[in] B �������B����СΪm��n�����д洢
 * @param[out] C �������C����СΪm��n�����д洢
 */
void mat_sub_ptr(int m, int n, const float *A, const float *B, float *C) {
    arm_matrix_instance_f32 mat_A, mat_B, mat_C;
    
    // ��ʼ������ʵ��
    arm_mat_init_f32(&mat_A, m, n, (float *)A);
    arm_mat_init_f32(&mat_B, m, n, (float *)B);
    arm_mat_init_f32(&mat_C, m, n, C);
    
    // ִ�о������
    arm_mat_sub_f32(&mat_A, &mat_B, &mat_C);
}

/**
 * ����ת��: B = A^T (ʹ��HALDSP��)
 * @param[in] m A���������
 * @param[in] n A���������
 * @param[in] A ������󣬴�СΪm��n�����д洢
 * @param[out] B ���ת�þ��󣬴�СΪn��m�����д洢
 */
void mat_transpose_ptr(int m, int n, const float *A, float *B) {
    arm_matrix_instance_f32 mat_A, mat_B;
    
    // ��ʼ������ʵ��
    arm_mat_init_f32(&mat_A, m, n, (float *)A);
    arm_mat_init_f32(&mat_B, n, m, B);
    
    // ִ�о���ת��
    arm_mat_trans_f32(&mat_A, &mat_B);
}

/**
 * ��������: c = a - b (ʹ��HALDSP��)
 * @param[in] n ������ά��
 * @param[in] a ��������a����СΪn
 * @param[in] b ��������b����СΪn
 * @param[out] c �������c����СΪn
 */
void vec_sub_ptr(int n, const float *a, const float *b, float *c) {
    arm_sub_f32((float *)a, (float *)b, c, n);
}

/**
 * �����ӷ�: c = a + b (ʹ��HALDSP��)
 * @param[in] n ������ά��
 * @param[in] a ��������a����СΪn
 * @param[in] b ��������b����СΪn
 * @param[out] c �������c����СΪn
 */
void vec_add_ptr(int n, const float *a, const float *b, float *c) {
    arm_add_f32((float *)a, (float *)b, c, n);
}

/**
 * �����ӿ�ӷ�: C = A + B (ʹ��HALDSP��)
 * @param[in] row ��������ʼ��
 * @param[in] col ��������ʼ��
 * @param[in] m �ӿ������
 * @param[in] n �ӿ������
 * @param[in] ld ������������������
 * @param[in] A ����������Ĵ����
 * @param[in] B ���ӵ�С���󣬴�СС��A����
 * @param[out] C ������󣬴�С��A������ͬ
 * @example ��3x3����I3�ӵ�28x18����C�ĵ�6�е�6�е���9�е�9��(��ʼΪ0��0��)
 * mat_add_block_ptr(6, 6, 3, 3, 18, &C[0][0], &I3[0][0], &C[0][0]);
 */
void mat_add_block_ptr(int row, int col, int m, int n, int ld,
                          const float *A, 
                          const float *B,
                          float *C) {
    int i, j;
    float temp_A[MAX_MATRIX_DIM * MAX_MATRIX_DIM];
    float temp_B[MAX_MATRIX_DIM * MAX_MATRIX_DIM];
    float temp_C[MAX_MATRIX_DIM * MAX_MATRIX_DIM];
    
    // ��ȡ�ӿ�
    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {
            temp_A[i*n + j] = A[(row+i)*ld + (col+j)];
            temp_B[i*n + j] = B[i*n + j];
        }
    }
    
    // ʹ��HALDSP��ľ���ӷ�
    arm_matrix_instance_f32 mat_A, mat_B, mat_C;
    arm_mat_init_f32(&mat_A, m, n, temp_A);
    arm_mat_init_f32(&mat_B, m, n, temp_B);
    arm_mat_init_f32(&mat_C, m, n, temp_C);
    arm_mat_add_f32(&mat_A, &mat_B, &mat_C);
    
    // ��������ƻ�ԭ����
    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {
            C[(row+i)*ld + (col+j)] = temp_C[i*n + j];
        }
    }
}

/**
 * �����ӿ����: C(destRow:destRow+m-1, destCol:destCol+n-1) = A(srcRowA:srcRowA+m-1, srcColA:srcColA+n-1) - B(srcRowB:srcRowB+m-1, srcColB:srcColB+n-1)
 * @param[in] m �ӿ������
 * @param[in] n �ӿ������
 * @param[in] A �������A
 * @param[in] ldA A�������������򲽳���
 * @param[in] srcRowA A���ӿ���ʼ������
 * @param[in] srcColA A���ӿ���ʼ������
 * @param[in] B �������B
 * @param[in] ldB B�������������򲽳���
 * @param[in] srcRowB B���ӿ���ʼ������
 * @param[in] srcColB B���ӿ���ʼ������
 * @param[out] C �������C
 * @param[in] ldC C�������������򲽳���
 * @param[in] destRow C��Ŀ���ӿ���ʼ������
 * @param[in] destCol C��Ŀ���ӿ���ʼ������
 */
void mat_subblock_sub_ptr(int m, int n, 
                          const float *A, int ldA, int srcRowA, int srcColA, 
                          const float *B, int ldB, int srcRowB, int srcColB, 
                          float *C, int ldC, int destRow, int destCol) {
    int i, j;
    float temp_A[MAX_MATRIX_DIM * MAX_MATRIX_DIM];
    float temp_B[MAX_MATRIX_DIM * MAX_MATRIX_DIM];
    float temp_C[MAX_MATRIX_DIM * MAX_MATRIX_DIM];
    
    // ��ȡ�ӿ�
    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {
            temp_A[i*n + j] = A[(srcRowA+i)*ldA + (srcColA+j)];
            temp_B[i*n + j] = B[(srcRowB+i)*ldB + (srcColB+j)];
        }
    }
    
    // ʹ��HALDSP��ľ������
    arm_matrix_instance_f32 mat_A, mat_B, mat_C;
    arm_mat_init_f32(&mat_A, m, n, temp_A);
    arm_mat_init_f32(&mat_B, m, n, temp_B);
    arm_mat_init_f32(&mat_C, m, n, temp_C);
    arm_mat_sub_f32(&mat_A, &mat_B, &mat_C);
    
    // ��������ƻ�ԭ����
    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {
            C[(destRow+i)*ldC + (destCol+j)] = temp_C[i*n + j];
        }
    }
}

/**
 * �����ӿ鸴��: C(destRow:destRow+m-1, destCol:destCol+n-1) = A(srcRow:srcRow+m-1, srcCol:srcCol+n-1)
 * @param[in] m �ӿ������
 * @param[in] n �ӿ������
 * @param[in] A �������A
 * @param[in] ldA A�������������򲽳���
 * @param[in] srcRow A���ӿ���ʼ������
 * @param[in] srcCol A���ӿ���ʼ������
 * @param[out] C �������C
 * @param[in] ldC C�������������򲽳���
 * @param[in] destRow C��Ŀ���ӿ���ʼ������
 * @param[in] destCol C��Ŀ���ӿ���ʼ������
 */
void mat_subblock_copy_ptr(int m, int n, 
                           const float *A, int ldA, int srcRow, int srcCol, 
                           float *C, int ldC, int destRow, int destCol) {
    int i, j;
    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {
            C[(destRow+i)*ldC + (destCol+j)] = A[(srcRow+i)*ldA + (srcCol+j)];
        }
    }
}

/**
 * ��ά����ֱ�Ӳ��
 * @param[in] a ��������a������Ϊ3
 * @param[in] b ��������b������Ϊ3
 * @param[out] c �������c = a �� b������Ϊ3
 */
void vec3_cross(const float a[3], const float b[3], float c[3]) {
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}

/**
 * ��ά������� (ָ��汾) (ʹ��HALDSP��)
 * @param[in] a ��������a������Ϊ3
 * @param[in] b ��������b������Ϊ3
 * @param[out] c �������c = a �� b������Ϊ3
 */
void vec3_cross_ptr(const float *a, const float *b, float *c) {
    vec3_cross((float *)a, (float *)b, c);
}

/**
 * ��������LU�ֽ� (ʹ��HALDSP��)
 * A = P^(-1) * L * U������P���û�����(ͨ��P�����ʾ)
 * 
 * @param[in] n ����A��ά��(n��n)
 * @param[in] A �������A����СΪn��n�����д洢
 * @param[out] L ��������Ǿ���L���Խ���Ԫ��Ϊ1����СΪn��n�����д洢
 * @param[out] U ��������Ǿ���U����СΪn��n�����д洢
 * @param[out] P ����н�����¼���飬��СΪn
 * @return �ɹ�����MAT_SUCCESS��ʧ�ܷ���MAT_ERROR
 */
int mat_lu_decomp_ptr(int n, const float *A, float *L, float *U, int *P) {
    if (n > MAX_MATRIX_DIM) {
        return MAT_ERROR;
    }
    
    // HALDSP�ⲻֱ���ṩLU�ֽ⣬���ṩ��������
    // �˴���Ҫ�Զ���LU�ֽ⣬��ʹ��Cholesky�ֽ����
    // ʵ��ʵ��ȡ����HALDSP��汾�;�������
    
    // ����򵥸���ԭ�д���
    int i, j, k;
    float sum;
    int pivot_row;
    float pivot_val;
    int temp_idx;
    
    // ��ʼ��L��U����
    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            if (i == j) {
                L[i*n + j] = 1.0f;
            } else {
                L[i*n + j] = 0.0f;
            }
            U[i*n + j] = 0.0f;
        }
        P[i] = i;
    }
    
    // LU�ֽ��������Ԫ
    for (i = 0; i < n; i++) {
        // ������Ԫ
        pivot_row = i;
        pivot_val = 0.0f;
        for (j = i; j < n; j++) {
            if (fabs(A[P[j]*n + i]) > pivot_val) {
                pivot_val = fabs(A[P[j]*n + i]);
                pivot_row = j;
            }
        }
        
        // �������Ƿ����
        if (pivot_val < 1e-5f) {
            return MAT_ERROR;
        }
        
        // ����������
        if (pivot_row != i) {
            temp_idx = P[i];
            P[i] = P[pivot_row];
            P[pivot_row] = temp_idx;
        }
        
        // ����U�ĵ�i��
        for (j = i; j < n; j++) {
            sum = 0.0f;
            for (k = 0; k < i; k++) {
                sum += L[P[i]*n + k] * U[k*n + j];
            }
            U[i*n + j] = A[P[i]*n + j] - sum;
        }
        
        // ����L�ĵ�i��
        for (j = i+1; j < n; j++) {
            sum = 0.0f;
            for (k = 0; k < i; k++) {
                sum += L[P[j]*n + k] * U[k*n + i];
            }
            if (fabs(U[i*n + i]) < 1e-5f) {
                return MAT_ERROR;
            }
            L[P[j]*n + i] = (A[P[j]*n + i] - sum) / U[i*n + i];
        }
    }
    
    return MAT_SUCCESS;
}

/**
 * ʹ���Ѽ����LU�ֽ�������Է����� A * X = B���õ� X = A^-1 * B
 * ��������Ҫ���ʹ��ͬһ��A�Բ�ͬB��������������ظ�����LU�ֽ�
 * 
 * @param[in] n A�����ά�ȣ�n��n��
 * @param[in] L �Ѽ���������Ǿ��󣬴�СΪn��n�����д洢
 * @param[in] U �Ѽ���������Ǿ��󣬴�СΪn��n�����д洢
 * @param[in] P �н�����¼���飬��СΪn
 * @param[in] m B���������
 * @param[in] B �������B����СΪn��m�����д洢
 * @param[out] X �������X = A^-1 * B����СΪn��m�����д洢
 * @return �ɹ�����MAT_SUCCESS��ʧ�ܷ���MAT_ERROR
 */
int mat_solve_with_lu_ptr(int n, const float *L, const float *U, const int *P, 
                          int m, const float *B, float *X) {
    // HALDSP�ⲻֱ��֧�ִ˲�������Ҫ�Զ���ʵ��
    // ���ﱣ��ԭ��ʵ��...����ԭ������ͬ��
    int i, j, k;
    float y[MAX_MATRIX_DIM];
    float b[MAX_MATRIX_DIM];
    
    if (n > MAX_MATRIX_DIM) {
        return MAT_ERROR;
    }
    
    for (j = 0; j < m; j++) {
        for (i = 0; i < n; i++) {
            b[i] = B[i*m + j];
        }
        
        // ǰ���滻
        for (i = 0; i < n; i++) {
            y[i] = b[P[i]];
            for (k = 0; k < i; k++) {
                y[i] -= L[P[i]*n + k] * y[k];
            }
        }
        
        // �����滻
        for (i = n-1; i >= 0; i--) {
            X[i*m + j] = y[i];
            for (k = i+1; k < n; k++) {
                X[i*m + j] -= U[i*n + k] * X[k*m + j];
            }
            if (fabs(U[i*n + i]) < 1e-5f) {
                return MAT_ERROR;
            }
            X[i*m + j] /= U[i*n + i];
        }
    }
    
    return MAT_SUCCESS;
}

// �������� (ʹ��HALDSP��)
int mat_inverse_ptr(int n, const float *A, float *Ainv) {
    arm_matrix_instance_f32 mat_A, mat_Ainv;
    arm_status status;
    
    // ��ʼ������ʵ��
    arm_mat_init_f32(&mat_A, n, n, (float *)A);
    arm_mat_init_f32(&mat_Ainv, n, n, Ainv);
    
    // ִ�о�������
    status = arm_mat_inverse_f32(&mat_A, &mat_Ainv);
    
    if (status != ARM_MATH_SUCCESS) {
        return MAT_ERROR;
    }
    
    return MAT_SUCCESS;
}

// ר�Ŵ���3x3��������溯�� (ʹ��HALDSP��)
int mat_inverse_3x3_ptr(const float *A, float *Ainv) {
    return mat_inverse_ptr(3, A, Ainv);
}

/**
 * ʹ��LU�ֽ������󷽳� A * X = B���õ� X = A^-1 * B
 * �˷���������������ٳ˸���Ч
 * 
 * @param[in] n A�����ά�ȣ�n��n��
 * @param[in] A �������A����СΪn��n�����д洢
 * @param[in] m B���������
 * @param[in] B �������B����СΪn��m�����д洢
 * @param[out] X �������X = A^-1 * B����СΪn��m�����д洢
 * @return �ɹ�����MAT_SUCCESS��ʧ�ܷ���MAT_ERROR
 */
int mat_solve_ptr(int n, const float *A, int m, const float *B, float *X) {
    // ���ڵ�һ�Ҳ����������������ʹ��HALDSP�ľ�������Ȼ�����
    if (m == 1) {
        float Ainv[MAX_MATRIX_DIM * MAX_MATRIX_DIM];
        
        if (mat_inverse_ptr(n, A, Ainv) != MAT_SUCCESS) {
            return MAT_ERROR;
        }
        
        mat_mult_vec_ptr(n, n, Ainv, B, X);
        return MAT_SUCCESS;
    }
    
    // ���ڶ����Ҳ�������ʹ��LU�ֽⷽ��
    float L[MAX_MATRIX_DIM * MAX_MATRIX_DIM] = {0};
    float U[MAX_MATRIX_DIM * MAX_MATRIX_DIM] = {0};
    int P[MAX_MATRIX_DIM];
    
    if (mat_lu_decomp_ptr(n, A, L, U, P) != MAT_SUCCESS) {
        return MAT_ERROR;
    }
    
    return mat_solve_with_lu_ptr(n, L, U, P, m, B, X);
}

/**
 * ר�Ŵ���3x3���󷽳����Ŀ��ٺ��� A * X = B���õ� X = A^-1 * B
 * ʹ�ÿ���Ĭ����ֱ�����
 * 
 * @param[in] A �������A����СΪ3��3�����д洢
 * @param[in] m B���������
 * @param[in] B �������B����СΪ3��m�����д洢
 * @param[out] X �������X = A^-1 * B����СΪ3��m�����д洢
 * @return �ɹ�����MAT_SUCCESS��ʧ�ܷ���MAT_ERROR
 */
int mat_solve_3x3_ptr(const float *A, int m, const float *B, float *X) {
    return mat_solve_ptr(3, A, m, B, X);
}

