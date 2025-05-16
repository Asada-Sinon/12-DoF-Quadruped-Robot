#include "Matrix.h"
#include "math.h"
#include "stdio.h"
// �Զ��������㺯��
/**
 * ����˷�: C = A * B (ʹ��ָ����ʽ)
 * @param[in] m A���������
 * @param[in] n A�����������B���������
 * @param[in] p B���������
 * @param[in] A �������A����СΪm��n�����д洢
 * @param[in] B �������B����СΪn��p�����д洢
 * @param[out] C �������C����СΪm��p�����д洢
 */
void mat_mult_ptr(int m, int n, int p, const float *A, const float *B, float *C) {
    int i, j, k;
    for (i = 0; i < m; i++) {
        for (j = 0; j < p; j++) {
            C[i*p + j] = 0.0f;
            for (k = 0; k < n; k++) {
                C[i*p + j] += A[i*n + k] * B[k*p + j];
            }
        }
    }
}

/**
 * �����������: y = A * x (ʹ��ָ����ʽ)
 * @param[in] m A���������
 * @param[in] n A�����������x������ά��
 * @param[in] A �������A����СΪm��n�����д洢
 * @param[in] x ��������x����СΪn
 * @param[out] y �������y����СΪm
 */
void mat_mult_vec_ptr(int m, int n, const float *A, const float *x, float *y) {
    int i, j;
    for (i = 0; i < m; i++) {
        y[i] = 0.0f;
        for (j = 0; j < n; j++) {
            y[i] += A[i*n + j] * x[j];
        }
    }
}

/**
 * ����ӷ�: C = A + B (ʹ��ָ����ʽ)
 * @param[in] m ���������
 * @param[in] n ���������
 * @param[in] A �������A����СΪm��n�����д洢
 * @param[in] B �������B����СΪm��n�����д洢
 * @param[out] C �������C����СΪm��n�����д洢
 */
void mat_add_ptr(int m, int n, const float *A, const float *B, float *C) {
    int i, j, idx;
    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {
            idx = i*n + j;
            C[idx] = A[idx] + B[idx];
        }
    }
}

/**
 * �������: C = A - B (ʹ��ָ����ʽ)
 * @param[in] m ���������
 * @param[in] n ���������
 * @param[in] A �������A����СΪm��n�����д洢
 * @param[in] B �������B����СΪm��n�����д洢
 * @param[out] C �������C����СΪm��n�����д洢
 */
void mat_sub_ptr(int m, int n, const float *A, const float *B, float *C) {
    int i, j, idx;
    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {
            idx = i*n + j;
            C[idx] = A[idx] - B[idx];
        }
    }
}

/**
 * ����ת��: B = A^T (ʹ��ָ����ʽ)
 * @param[in] m A���������
 * @param[in] n A���������
 * @param[in] A ������󣬴�СΪm��n�����д洢
 * @param[out] B ���ת�þ��󣬴�СΪn��m�����д洢
 */
void mat_transpose_ptr(int m, int n, const float *A, float *B) {
    int i, j;
    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {
            B[j*m + i] = A[i*n + j];
        }
    }
}

/**
 * ��������: c = a - b (ʹ��ָ����ʽ)
 * @param[in] n ������ά��
 * @param[in] a ��������a����СΪn
 * @param[in] b ��������b����СΪn
 * @param[out] c �������c����СΪn
 */
void vec_sub_ptr(int n, const float *a, const float *b, float *c) {
    int i;
    for (i = 0; i < n; i++) {
        c[i] = a[i] - b[i];
    }
}

/**
 * �����ӷ�: c = a + b (ʹ��ָ����ʽ)
 * @param[in] n ������ά��
 * @param[in] a ��������a����СΪn
 * @param[in] b ��������b����СΪn
 * @param[out] c �������c����СΪn
 */
void vec_add_ptr(int n, const float *a, const float *b, float *c) {
    int i;
    for (i = 0; i < n; i++) {
        c[i] = a[i] + b[i];
    }
}

/**
 * �����ӿ�ӷ�: C = A + B
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
    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {
            C[(row + i) * ld + (col + j)] = 
                A[(row +i) * ld + (col + j)] + 
                B[(i) * n + (j)];
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
    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {
            C[(destRow + i) * ldC + (destCol + j)] = 
                A[(srcRowA + i) * ldA + (srcColA + j)] - 
                B[(srcRowB + i) * ldB + (srcColB + j)];
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
            C[(destRow + i) * ldC + (destCol + j)] = A[(srcRow + i) * ldA + (srcCol + j)];
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
 * ��ά������� (ָ��汾)
 * @param[in] a ��������a������Ϊ3
 * @param[in] b ��������b������Ϊ3
 * @param[out] c �������c = a �� b������Ϊ3
 */
void vec3_cross_ptr(const float *a, const float *b, float *c) {
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}

/**
 * ��������LU�ֽ� (ʹ�ò�����Ԫ��)
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
    int i, j, k;
    float sum;
    int pivot_row;
    float pivot_val;
    int temp_idx;
    
    if (n > MAX_MATRIX_DIM) {
        return MAT_ERROR; // ����ά�ȳ�������
    }
    
    // ��ʼ��L��U����
    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            if (i == j) {
                L[i*n + j] = 1.0f; // L�Խ��߳�ʼ��Ϊ1
            } else {
                L[i*n + j] = 0.0f;
            }
            U[i*n + j] = 0.0f;
        }
        P[i] = i; // ��ʼ���н�����ʹ����������
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
            return MAT_ERROR; // ����ӽ����죬������
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
        for (j = i+1; j < n; j++) { // ��i+1��ʼ����Ϊ�Խ���Ԫ������Ϊ1
            sum = 0.0f;
            for (k = 0; k < i; k++) {
                sum += L[P[j]*n + k] * U[k*n + i];
            }
            // ������Էǳ�С����
            if (fabs(U[i*n + i]) < 1e-5f) {
                return MAT_ERROR; // ��Ԫ̫С��������
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
    int i, j, k;
    float y[MAX_MATRIX_DIM];    // �м������
    float b[MAX_MATRIX_DIM];    // �Ҳ�����
    
    if (n > MAX_MATRIX_DIM) {
        return MAT_ERROR; // ����ά�ȳ�������
    }
    
    // ��ⷽ����A*X = B����B��ÿһ�зֱ����
    for (j = 0; j < m; j++) {
        // ���쵱ǰ�е��Ҳ�����
        for (i = 0; i < n; i++) {
            b[i] = B[i*m + j];
        }
        
        // ǰ���滻���Ly = b (�����н���)
        for (i = 0; i < n; i++) {
            y[i] = b[P[i]]; // Ӧ���н���
            for (k = 0; k < i; k++) {
                y[i] -= L[P[i]*n + k] * y[k];
            }
        }
        
        // �����滻���Ux = y
        for (i = n-1; i >= 0; i--) {
            X[i*m + j] = y[i];
            for (k = i+1; k < n; k++) {
                X[i*m + j] -= U[i*n + k] * X[k*m + j];
            }
            // �����ֵ�ȶ���
            if (fabs(U[i*n + i]) < 1e-5f) {
                return MAT_ERROR; // ��Ԫ̫С�����ܵ�����ֵ���ȶ�
            }
            X[i*m + j] /= U[i*n + i];
        }
    }
    
    return MAT_SUCCESS;
}

// ���������ָ��汾 (ʹ��LU�ֽⷨ)
// ����ֵ: 0�ɹ�, -1ʧ��
// A��Ainv�ǰ��д洢��һά������ʽ
int mat_inverse_ptr(int n, const float *A, float *Ainv) {
    int i, j, k;
    float L[MAX_MATRIX_DIM * MAX_MATRIX_DIM] = {0}; // �����Ǿ���
    float U[MAX_MATRIX_DIM * MAX_MATRIX_DIM] = {0}; // �����Ǿ���
    int P[MAX_MATRIX_DIM];    // �н�����¼(��������)
    float y[MAX_MATRIX_DIM];    // �м������
    float b[MAX_MATRIX_DIM];    // �Ҳ�����
    
    printf("A(ָ��汾):\n");
    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            printf("%f ", A[i*n + j]);
        }
        printf("\n");
    }
    
    if (n > MAX_MATRIX_DIM) {
        return MAT_ERROR; // ����ά�ȳ�������
    }
    
    // ʹ��LU�ֽ⺯������L��U��P
    if (mat_lu_decomp_ptr(n, A, L, U, P) != MAT_SUCCESS) {
        return MAT_ERROR; // LU�ֽ�ʧ�ܣ�������ܲ�����
    }
    
    // ͨ����ⷽ��AX=I����ȡ�����
    // ��ÿһ�зֱ����
    for (j = 0; j < n; j++) {
        // ���쵥λ����
        for (i = 0; i < n; i++) {
            b[i] = (i == j) ? 1.0f : 0.0f;
        }
        
        // ǰ���滻���Ly = b (�����н���)
        for (i = 0; i < n; i++) {
            y[i] = b[P[i]]; // Ӧ���н���
            for (k = 0; k < i; k++) {
                y[i] -= L[P[i]*n + k] * y[k];
            }
        }
        
        // �����滻���Ux = y
        for (i = n - 1; i >= 0; i--) {
            Ainv[i*n + j] = y[i];
            for (k = i + 1; k < n; k++) {
                Ainv[i*n + j] -= U[i*n + k] * Ainv[k*n + j];
            }
            // �ٴμ����ֵ�ȶ���
            if (fabs(U[i*n + i]) < 1e-5f) {
                return MAT_ERROR; // ��Ԫ̫С�����ܵ�����ֵ���ȶ�
            }
            Ainv[i*n + j] /= U[i*n + i];
        }
    }
    
    return MAT_SUCCESS;
}

// ר�Ŵ���3x3��������溯����ָ��汾
int mat_inverse_3x3_ptr(const float *A, float *Ainv) {
    // ��������ʽ
    float det = A[0] * (A[4] * A[8] - A[5] * A[7])
              - A[1] * (A[3] * A[8] - A[5] * A[6])
              + A[2] * (A[3] * A[7] - A[4] * A[6]);
    
    // �ж��Ƿ����
    if (fabs(det) < 1e-5f) {
        printf("�������죬����ʽ�ӽ���0: %f\n", det);
        return MAT_ERROR; // ���󲻿���
    }
    
    // �����������ת�ó�������ʽ
    float inv_det = 1.0f / det;
    
    Ainv[0] = (A[4] * A[8] - A[5] * A[7]) * inv_det;
    Ainv[1] = (A[2] * A[7] - A[1] * A[8]) * inv_det;
    Ainv[2] = (A[1] * A[5] - A[2] * A[4]) * inv_det;
    
    Ainv[3] = (A[5] * A[6] - A[3] * A[8]) * inv_det;
    Ainv[4] = (A[0] * A[8] - A[2] * A[6]) * inv_det;
    Ainv[5] = (A[2] * A[3] - A[0] * A[5]) * inv_det;
    
    Ainv[6] = (A[3] * A[7] - A[4] * A[6]) * inv_det;
    Ainv[7] = (A[1] * A[6] - A[0] * A[7]) * inv_det;
    Ainv[8] = (A[0] * A[4] - A[1] * A[3]) * inv_det;
    
    return MAT_SUCCESS;
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
    int i, j;
    float L[MAX_MATRIX_DIM * MAX_MATRIX_DIM] = {0}; // �����Ǿ���
    float U[MAX_MATRIX_DIM * MAX_MATRIX_DIM] = {0}; // �����Ǿ���
    int P[MAX_MATRIX_DIM];    // �н�����¼(��������)
    
    if (n > MAX_MATRIX_DIM) {
        return MAT_ERROR; // ����ά�ȳ�������
    }
    
    // ʹ��LU�ֽ⺯������L��U��P
    if (mat_lu_decomp_ptr(n, A, L, U, P) != MAT_SUCCESS) {
        return MAT_ERROR; // LU�ֽ�ʧ�ܣ�������ܲ�����
    }
    
    // ʹ���Ѽ����LU�ֽ���ⷽ����
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
    // �������A������ʽ
    float det = A[0] * (A[4] * A[8] - A[5] * A[7])
              - A[1] * (A[3] * A[8] - A[5] * A[6])
              + A[2] * (A[3] * A[7] - A[4] * A[6]);
    
    // �ж��Ƿ����
    if (fabs(det) < 1e-5f) {
        return MAT_ERROR; // ���󲻿���
    }
    
    // �����������Ԫ��
    float adj[9];
    adj[0] = A[4] * A[8] - A[5] * A[7];  // ����ʽM11
    adj[1] = A[2] * A[7] - A[1] * A[8];  // -M12
    adj[2] = A[1] * A[5] - A[2] * A[4];  // M13
    adj[3] = A[5] * A[6] - A[3] * A[8];  // -M21
    adj[4] = A[0] * A[8] - A[2] * A[6];  // M22
    adj[5] = A[2] * A[3] - A[0] * A[5];  // -M23
    adj[6] = A[3] * A[7] - A[4] * A[6];  // M31
    adj[7] = A[1] * A[6] - A[0] * A[7];  // -M32
    adj[8] = A[0] * A[4] - A[1] * A[3];  // M33
    
    // ������������B�ĳ˻����ٳ�������ʽ
    float inv_det = 1.0f / det;
    for (int j = 0; j < m; j++) {
        for (int i = 0; i < 3; i++) {
            X[i*m + j] = 0;
            for (int k = 0; k < 3; k++) {
                X[i*m + j] += adj[i*3 + k] * B[k*m + j];
            }
            X[i*m + j] *= inv_det;
        }
    }
    
    return MAT_SUCCESS;
}

