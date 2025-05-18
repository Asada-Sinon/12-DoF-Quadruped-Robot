#include "matrix.h"
#include "math.h"
#include "stdio.h"
#include "stm32f767xx.h"
#include "arm_math.h"  // HALDSP库
// 自定义矩阵计算函数
/**
 * 矩阵乘法: C = A * B (使用HALDSP库)
 * @param[in] m A矩阵的行数
 * @param[in] n A矩阵的列数和B矩阵的行数
 * @param[in] p B矩阵的列数
 * @param[in] A 输入矩阵A，大小为m×n，按行存储
 * @param[in] B 输入矩阵B，大小为n×p，按行存储
 * @param[out] C 输出矩阵C，大小为m×p，按行存储
 */
void mat_mult_ptr(int m, int n, int p, const float *A, const float *B, float *C) {
    arm_matrix_instance_f32 mat_A, mat_B, mat_C;
    
    // 初始化矩阵实例
    arm_mat_init_f32(&mat_A, m, n, (float *)A);
    arm_mat_init_f32(&mat_B, n, p, (float *)B);
    arm_mat_init_f32(&mat_C, m, p, C);
    
    // 执行矩阵乘法
    arm_mat_mult_f32(&mat_A, &mat_B, &mat_C);
}

/**
 * 矩阵乘以向量: y = A * x (使用HALDSP库)
 * @param[in] m A矩阵的行数
 * @param[in] n A矩阵的列数和x向量的维度
 * @param[in] A 输入矩阵A，大小为m×n，按行存储
 * @param[in] x 输入向量x，大小为n
 * @param[out] y 输出向量y，大小为m
 */
void mat_mult_vec_ptr(int m, int n, const float *A, const float *x, float *y) {
    arm_matrix_instance_f32 mat_A, vec_x, vec_y;
    
    // 初始化矩阵和向量实例
    arm_mat_init_f32(&mat_A, m, n, (float *)A);
    arm_mat_init_f32(&vec_x, n, 1, (float *)x);
    arm_mat_init_f32(&vec_y, m, 1, y);
    
    // 执行矩阵向量乘法
    arm_mat_mult_f32(&mat_A, &vec_x, &vec_y);
}

/**
 * 矩阵加法: C = A + B (使用HALDSP库)
 * @param[in] m 矩阵的行数
 * @param[in] n 矩阵的列数
 * @param[in] A 输入矩阵A，大小为m×n，按行存储
 * @param[in] B 输入矩阵B，大小为m×n，按行存储
 * @param[out] C 输出矩阵C，大小为m×n，按行存储
 */
void mat_add_ptr(int m, int n, const float *A, const float *B, float *C) {
    arm_matrix_instance_f32 mat_A, mat_B, mat_C;
    
    // 初始化矩阵实例
    arm_mat_init_f32(&mat_A, m, n, (float *)A);
    arm_mat_init_f32(&mat_B, m, n, (float *)B);
    arm_mat_init_f32(&mat_C, m, n, C);
    
    // 执行矩阵加法
    arm_mat_add_f32(&mat_A, &mat_B, &mat_C);
}

/**
 * 矩阵减法: C = A - B (使用HALDSP库)
 * @param[in] m 矩阵的行数
 * @param[in] n 矩阵的列数
 * @param[in] A 输入矩阵A，大小为m×n，按行存储
 * @param[in] B 输入矩阵B，大小为m×n，按行存储
 * @param[out] C 输出矩阵C，大小为m×n，按行存储
 */
void mat_sub_ptr(int m, int n, const float *A, const float *B, float *C) {
    arm_matrix_instance_f32 mat_A, mat_B, mat_C;
    
    // 初始化矩阵实例
    arm_mat_init_f32(&mat_A, m, n, (float *)A);
    arm_mat_init_f32(&mat_B, m, n, (float *)B);
    arm_mat_init_f32(&mat_C, m, n, C);
    
    // 执行矩阵减法
    arm_mat_sub_f32(&mat_A, &mat_B, &mat_C);
}

/**
 * 矩阵转置: B = A^T (使用HALDSP库)
 * @param[in] m A矩阵的行数
 * @param[in] n A矩阵的列数
 * @param[in] A 输入矩阵，大小为m×n，按行存储
 * @param[out] B 输出转置矩阵，大小为n×m，按行存储
 */
void mat_transpose_ptr(int m, int n, const float *A, float *B) {
    arm_matrix_instance_f32 mat_A, mat_B;
    
    // 初始化矩阵实例
    arm_mat_init_f32(&mat_A, m, n, (float *)A);
    arm_mat_init_f32(&mat_B, n, m, B);
    
    // 执行矩阵转置
    arm_mat_trans_f32(&mat_A, &mat_B);
}

/**
 * 向量减法: c = a - b (使用HALDSP库)
 * @param[in] n 向量的维度
 * @param[in] a 输入向量a，大小为n
 * @param[in] b 输入向量b，大小为n
 * @param[out] c 输出向量c，大小为n
 */
void vec_sub_ptr(int n, const float *a, const float *b, float *c) {
    arm_sub_f32((float *)a, (float *)b, c, n);
}

/**
 * 向量加法: c = a + b (使用HALDSP库)
 * @param[in] n 向量的维度
 * @param[in] a 输入向量a，大小为n
 * @param[in] b 输入向量b，大小为n
 * @param[out] c 输出向量c，大小为n
 */
void vec_add_ptr(int n, const float *a, const float *b, float *c) {
    arm_add_f32((float *)a, (float *)b, c, n);
}

/**
 * 矩阵子块加法: C = A + B (使用HALDSP库)
 * @param[in] row 矩阵块的起始行
 * @param[in] col 矩阵块的起始列
 * @param[in] m 子块的行数
 * @param[in] n 子块的列数
 * @param[in] ld 矩阵块所属矩阵的列数
 * @param[in] A 矩阵块所属的大矩阵
 * @param[in] B 被加的小矩阵，大小小于A矩阵
 * @param[out] C 输出矩阵，大小与A矩阵相同
 * @example 将3x3矩阵I3加到28x18矩阵C的第6行第6列到第9行第9列(起始为0行0列)
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
    
    // 提取子块
    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {
            temp_A[i*n + j] = A[(row+i)*ld + (col+j)];
            temp_B[i*n + j] = B[i*n + j];
        }
    }
    
    // 使用HALDSP库的矩阵加法
    arm_matrix_instance_f32 mat_A, mat_B, mat_C;
    arm_mat_init_f32(&mat_A, m, n, temp_A);
    arm_mat_init_f32(&mat_B, m, n, temp_B);
    arm_mat_init_f32(&mat_C, m, n, temp_C);
    arm_mat_add_f32(&mat_A, &mat_B, &mat_C);
    
    // 将结果复制回原矩阵
    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {
            C[(row+i)*ld + (col+j)] = temp_C[i*n + j];
        }
    }
}

/**
 * 矩阵子块减法: C(destRow:destRow+m-1, destCol:destCol+n-1) = A(srcRowA:srcRowA+m-1, srcColA:srcColA+n-1) - B(srcRowB:srcRowB+m-1, srcColB:srcColB+n-1)
 * @param[in] m 子块的行数
 * @param[in] n 子块的列数
 * @param[in] A 输入矩阵A
 * @param[in] ldA A的列数（行主序步长）
 * @param[in] srcRowA A的子块起始行索引
 * @param[in] srcColA A的子块起始列索引
 * @param[in] B 输入矩阵B
 * @param[in] ldB B的列数（行主序步长）
 * @param[in] srcRowB B的子块起始行索引
 * @param[in] srcColB B的子块起始列索引
 * @param[out] C 输出矩阵C
 * @param[in] ldC C的列数（行主序步长）
 * @param[in] destRow C的目标子块起始行索引
 * @param[in] destCol C的目标子块起始列索引
 */
void mat_subblock_sub_ptr(int m, int n, 
                          const float *A, int ldA, int srcRowA, int srcColA, 
                          const float *B, int ldB, int srcRowB, int srcColB, 
                          float *C, int ldC, int destRow, int destCol) {
    int i, j;
    float temp_A[MAX_MATRIX_DIM * MAX_MATRIX_DIM];
    float temp_B[MAX_MATRIX_DIM * MAX_MATRIX_DIM];
    float temp_C[MAX_MATRIX_DIM * MAX_MATRIX_DIM];
    
    // 提取子块
    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {
            temp_A[i*n + j] = A[(srcRowA+i)*ldA + (srcColA+j)];
            temp_B[i*n + j] = B[(srcRowB+i)*ldB + (srcColB+j)];
        }
    }
    
    // 使用HALDSP库的矩阵减法
    arm_matrix_instance_f32 mat_A, mat_B, mat_C;
    arm_mat_init_f32(&mat_A, m, n, temp_A);
    arm_mat_init_f32(&mat_B, m, n, temp_B);
    arm_mat_init_f32(&mat_C, m, n, temp_C);
    arm_mat_sub_f32(&mat_A, &mat_B, &mat_C);
    
    // 将结果复制回原矩阵
    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {
            C[(destRow+i)*ldC + (destCol+j)] = temp_C[i*n + j];
        }
    }
}

/**
 * 矩阵子块复制: C(destRow:destRow+m-1, destCol:destCol+n-1) = A(srcRow:srcRow+m-1, srcCol:srcCol+n-1)
 * @param[in] m 子块的行数
 * @param[in] n 子块的列数
 * @param[in] A 输入矩阵A
 * @param[in] ldA A的列数（行主序步长）
 * @param[in] srcRow A的子块起始行索引
 * @param[in] srcCol A的子块起始列索引
 * @param[out] C 输出矩阵C
 * @param[in] ldC C的列数（行主序步长）
 * @param[in] destRow C的目标子块起始行索引
 * @param[in] destCol C的目标子块起始列索引
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
 * 三维向量直接叉乘
 * @param[in] a 输入向量a，长度为3
 * @param[in] b 输入向量b，长度为3
 * @param[out] c 输出向量c = a × b，长度为3
 */
void vec3_cross(const float a[3], const float b[3], float c[3]) {
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}

/**
 * 三维向量叉乘 (指针版本) (使用HALDSP库)
 * @param[in] a 输入向量a，长度为3
 * @param[in] b 输入向量b，长度为3
 * @param[out] c 输出向量c = a × b，长度为3
 */
void vec3_cross_ptr(const float *a, const float *b, float *c) {
    vec3_cross((float *)a, (float *)b, c);
}

/**
 * 计算矩阵的LU分解 (使用HALDSP库)
 * A = P^(-1) * L * U，其中P是置换矩阵(通过P数组表示)
 * 
 * @param[in] n 矩阵A的维度(n×n)
 * @param[in] A 输入矩阵A，大小为n×n，按行存储
 * @param[out] L 输出下三角矩阵L，对角线元素为1，大小为n×n，按行存储
 * @param[out] U 输出上三角矩阵U，大小为n×n，按行存储
 * @param[out] P 输出行交换记录数组，大小为n
 * @return 成功返回MAT_SUCCESS，失败返回MAT_ERROR
 */
int mat_lu_decomp_ptr(int n, const float *A, float *L, float *U, int *P) {
    if (n > MAX_MATRIX_DIM) {
        return MAT_ERROR;
    }
    
    // HALDSP库不直接提供LU分解，仅提供矩阵求逆
    // 此处需要自定义LU分解，或使用Cholesky分解代替
    // 实际实现取决于HALDSP库版本和具体需求
    
    // 这里简单复用原有代码
    int i, j, k;
    float sum;
    int pivot_row;
    float pivot_val;
    int temp_idx;
    
    // 初始化L和U矩阵
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
    
    // LU分解带部分主元
    for (i = 0; i < n; i++) {
        // 查找主元
        pivot_row = i;
        pivot_val = 0.0f;
        for (j = i; j < n; j++) {
            if (fabs(A[P[j]*n + i]) > pivot_val) {
                pivot_val = fabs(A[P[j]*n + i]);
                pivot_row = j;
            }
        }
        
        // 检查矩阵是否可逆
        if (pivot_val < 1e-5f) {
            return MAT_ERROR;
        }
        
        // 交换行索引
        if (pivot_row != i) {
            temp_idx = P[i];
            P[i] = P[pivot_row];
            P[pivot_row] = temp_idx;
        }
        
        // 计算U的第i行
        for (j = i; j < n; j++) {
            sum = 0.0f;
            for (k = 0; k < i; k++) {
                sum += L[P[i]*n + k] * U[k*n + j];
            }
            U[i*n + j] = A[P[i]*n + j] - sum;
        }
        
        // 计算L的第i列
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
 * 使用已计算的LU分解求解线性方程组 A * X = B，得到 X = A^-1 * B
 * 适用于需要多次使用同一个A对不同B求解的情况，避免重复计算LU分解
 * 
 * @param[in] n A矩阵的维度（n×n）
 * @param[in] L 已计算的下三角矩阵，大小为n×n，按行存储
 * @param[in] U 已计算的上三角矩阵，大小为n×n，按行存储
 * @param[in] P 行交换记录数组，大小为n
 * @param[in] m B矩阵的列数
 * @param[in] B 输入矩阵B，大小为n×m，按行存储
 * @param[out] X 输出矩阵X = A^-1 * B，大小为n×m，按行存储
 * @return 成功返回MAT_SUCCESS，失败返回MAT_ERROR
 */
int mat_solve_with_lu_ptr(int n, const float *L, const float *U, const int *P, 
                          int m, const float *B, float *X) {
    // HALDSP库不直接支持此操作，需要自定义实现
    // 这里保持原有实现...（与原代码相同）
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
        
        // 前向替换
        for (i = 0; i < n; i++) {
            y[i] = b[P[i]];
            for (k = 0; k < i; k++) {
                y[i] -= L[P[i]*n + k] * y[k];
            }
        }
        
        // 后向替换
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

// 矩阵求逆 (使用HALDSP库)
int mat_inverse_ptr(int n, const float *A, float *Ainv) {
    arm_matrix_instance_f32 mat_A, mat_Ainv;
    arm_status status;
    
    // 初始化矩阵实例
    arm_mat_init_f32(&mat_A, n, n, (float *)A);
    arm_mat_init_f32(&mat_Ainv, n, n, Ainv);
    
    // 执行矩阵求逆
    status = arm_mat_inverse_f32(&mat_A, &mat_Ainv);
    
    if (status != ARM_MATH_SUCCESS) {
        return MAT_ERROR;
    }
    
    return MAT_SUCCESS;
}

// 专门处理3x3矩阵的求逆函数 (使用HALDSP库)
int mat_inverse_3x3_ptr(const float *A, float *Ainv) {
    return mat_inverse_ptr(3, A, Ainv);
}

/**
 * 使用LU分解求解矩阵方程 A * X = B，得到 X = A^-1 * B
 * 此方法比先求逆矩阵再乘更高效
 * 
 * @param[in] n A矩阵的维度（n×n）
 * @param[in] A 输入矩阵A，大小为n×n，按行存储
 * @param[in] m B矩阵的列数
 * @param[in] B 输入矩阵B，大小为n×m，按行存储
 * @param[out] X 输出矩阵X = A^-1 * B，大小为n×m，按行存储
 * @return 成功返回MAT_SUCCESS，失败返回MAT_ERROR
 */
int mat_solve_ptr(int n, const float *A, int m, const float *B, float *X) {
    // 对于单一右侧向量的情况，可以使用HALDSP的矩阵求逆然后相乘
    if (m == 1) {
        float Ainv[MAX_MATRIX_DIM * MAX_MATRIX_DIM];
        
        if (mat_inverse_ptr(n, A, Ainv) != MAT_SUCCESS) {
            return MAT_ERROR;
        }
        
        mat_mult_vec_ptr(n, n, Ainv, B, X);
        return MAT_SUCCESS;
    }
    
    // 对于多列右侧向量，使用LU分解方法
    float L[MAX_MATRIX_DIM * MAX_MATRIX_DIM] = {0};
    float U[MAX_MATRIX_DIM * MAX_MATRIX_DIM] = {0};
    int P[MAX_MATRIX_DIM];
    
    if (mat_lu_decomp_ptr(n, A, L, U, P) != MAT_SUCCESS) {
        return MAT_ERROR;
    }
    
    return mat_solve_with_lu_ptr(n, L, U, P, m, B, X);
}

/**
 * 专门处理3x3矩阵方程求解的快速函数 A * X = B，得到 X = A^-1 * B
 * 使用克拉默法则直接求解
 * 
 * @param[in] A 输入矩阵A，大小为3×3，按行存储
 * @param[in] m B矩阵的列数
 * @param[in] B 输入矩阵B，大小为3×m，按行存储
 * @param[out] X 输出矩阵X = A^-1 * B，大小为3×m，按行存储
 * @return 成功返回MAT_SUCCESS，失败返回MAT_ERROR
 */
int mat_solve_3x3_ptr(const float *A, int m, const float *B, float *X) {
    return mat_solve_ptr(3, A, m, B, X);
}

