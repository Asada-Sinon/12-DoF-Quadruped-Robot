#include "Matrix.h"
#include "math.h"
#include "stdio.h"
// 自定义矩阵计算函数
/**
 * 矩阵乘法: C = A * B (使用指针形式)
 * @param[in] m A矩阵的行数
 * @param[in] n A矩阵的列数和B矩阵的行数
 * @param[in] p B矩阵的列数
 * @param[in] A 输入矩阵A，大小为m×n，按行存储
 * @param[in] B 输入矩阵B，大小为n×p，按行存储
 * @param[out] C 输出矩阵C，大小为m×p，按行存储
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
 * 矩阵乘以向量: y = A * x (使用指针形式)
 * @param[in] m A矩阵的行数
 * @param[in] n A矩阵的列数和x向量的维度
 * @param[in] A 输入矩阵A，大小为m×n，按行存储
 * @param[in] x 输入向量x，大小为n
 * @param[out] y 输出向量y，大小为m
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
 * 矩阵加法: C = A + B (使用指针形式)
 * @param[in] m 矩阵的行数
 * @param[in] n 矩阵的列数
 * @param[in] A 输入矩阵A，大小为m×n，按行存储
 * @param[in] B 输入矩阵B，大小为m×n，按行存储
 * @param[out] C 输出矩阵C，大小为m×n，按行存储
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
 * 矩阵减法: C = A - B (使用指针形式)
 * @param[in] m 矩阵的行数
 * @param[in] n 矩阵的列数
 * @param[in] A 输入矩阵A，大小为m×n，按行存储
 * @param[in] B 输入矩阵B，大小为m×n，按行存储
 * @param[out] C 输出矩阵C，大小为m×n，按行存储
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
 * 矩阵转置: B = A^T (使用指针形式)
 * @param[in] m A矩阵的行数
 * @param[in] n A矩阵的列数
 * @param[in] A 输入矩阵，大小为m×n，按行存储
 * @param[out] B 输出转置矩阵，大小为n×m，按行存储
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
 * 向量减法: c = a - b (使用指针形式)
 * @param[in] n 向量的维度
 * @param[in] a 输入向量a，大小为n
 * @param[in] b 输入向量b，大小为n
 * @param[out] c 输出向量c，大小为n
 */
void vec_sub_ptr(int n, const float *a, const float *b, float *c) {
    int i;
    for (i = 0; i < n; i++) {
        c[i] = a[i] - b[i];
    }
}

/**
 * 向量加法: c = a + b (使用指针形式)
 * @param[in] n 向量的维度
 * @param[in] a 输入向量a，大小为n
 * @param[in] b 输入向量b，大小为n
 * @param[out] c 输出向量c，大小为n
 */
void vec_add_ptr(int n, const float *a, const float *b, float *c) {
    int i;
    for (i = 0; i < n; i++) {
        c[i] = a[i] + b[i];
    }
}

/**
 * 矩阵子块加法: C = A + B
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
    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {
            C[(row + i) * ld + (col + j)] = 
                A[(row +i) * ld + (col + j)] + 
                B[(i) * n + (j)];
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
    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {
            C[(destRow + i) * ldC + (destCol + j)] = 
                A[(srcRowA + i) * ldA + (srcColA + j)] - 
                B[(srcRowB + i) * ldB + (srcColB + j)];
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
            C[(destRow + i) * ldC + (destCol + j)] = A[(srcRow + i) * ldA + (srcCol + j)];
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
 * 三维向量叉乘 (指针版本)
 * @param[in] a 输入向量a，长度为3
 * @param[in] b 输入向量b，长度为3
 * @param[out] c 输出向量c = a × b，长度为3
 */
void vec3_cross_ptr(const float *a, const float *b, float *c) {
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}

/**
 * 计算矩阵的LU分解 (使用部分主元法)
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
    int i, j, k;
    float sum;
    int pivot_row;
    float pivot_val;
    int temp_idx;
    
    if (n > MAX_MATRIX_DIM) {
        return MAT_ERROR; // 矩阵维度超出限制
    }
    
    // 初始化L和U矩阵
    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            if (i == j) {
                L[i*n + j] = 1.0f; // L对角线初始化为1
            } else {
                L[i*n + j] = 0.0f;
            }
            U[i*n + j] = 0.0f;
        }
        P[i] = i; // 初始无行交换，使用整数索引
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
            return MAT_ERROR; // 矩阵接近奇异，不可逆
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
        for (j = i+1; j < n; j++) { // 从i+1开始，因为对角线元素已设为1
            sum = 0.0f;
            for (k = 0; k < i; k++) {
                sum += L[P[j]*n + k] * U[k*n + i];
            }
            // 避免除以非常小的数
            if (fabs(U[i*n + i]) < 1e-5f) {
                return MAT_ERROR; // 主元太小，不可逆
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
    int i, j, k;
    float y[MAX_MATRIX_DIM];    // 中间解向量
    float b[MAX_MATRIX_DIM];    // 右侧向量
    
    if (n > MAX_MATRIX_DIM) {
        return MAT_ERROR; // 矩阵维度超出限制
    }
    
    // 求解方程组A*X = B，对B的每一列分别求解
    for (j = 0; j < m; j++) {
        // 构造当前列的右侧向量
        for (i = 0; i < n; i++) {
            b[i] = B[i*m + j];
        }
        
        // 前向替换求解Ly = b (考虑行交换)
        for (i = 0; i < n; i++) {
            y[i] = b[P[i]]; // 应用行交换
            for (k = 0; k < i; k++) {
                y[i] -= L[P[i]*n + k] * y[k];
            }
        }
        
        // 后向替换求解Ux = y
        for (i = n-1; i >= 0; i--) {
            X[i*m + j] = y[i];
            for (k = i+1; k < n; k++) {
                X[i*m + j] -= U[i*n + k] * X[k*m + j];
            }
            // 检查数值稳定性
            if (fabs(U[i*n + i]) < 1e-5f) {
                return MAT_ERROR; // 主元太小，可能导致数值不稳定
            }
            X[i*m + j] /= U[i*n + i];
        }
    }
    
    return MAT_SUCCESS;
}

// 矩阵求逆的指针版本 (使用LU分解法)
// 返回值: 0成功, -1失败
// A和Ainv是按行存储的一维数组形式
int mat_inverse_ptr(int n, const float *A, float *Ainv) {
    int i, j, k;
    float L[MAX_MATRIX_DIM * MAX_MATRIX_DIM] = {0}; // 下三角矩阵
    float U[MAX_MATRIX_DIM * MAX_MATRIX_DIM] = {0}; // 上三角矩阵
    int P[MAX_MATRIX_DIM];    // 行交换记录(整数索引)
    float y[MAX_MATRIX_DIM];    // 中间解向量
    float b[MAX_MATRIX_DIM];    // 右侧向量
    
    printf("A(指针版本):\n");
    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            printf("%f ", A[i*n + j]);
        }
        printf("\n");
    }
    
    if (n > MAX_MATRIX_DIM) {
        return MAT_ERROR; // 矩阵维度超出限制
    }
    
    // 使用LU分解函数计算L、U和P
    if (mat_lu_decomp_ptr(n, A, L, U, P) != MAT_SUCCESS) {
        return MAT_ERROR; // LU分解失败，矩阵可能不可逆
    }
    
    // 通过求解方程AX=I来获取逆矩阵
    // 对每一列分别求解
    for (j = 0; j < n; j++) {
        // 构造单位向量
        for (i = 0; i < n; i++) {
            b[i] = (i == j) ? 1.0f : 0.0f;
        }
        
        // 前向替换求解Ly = b (考虑行交换)
        for (i = 0; i < n; i++) {
            y[i] = b[P[i]]; // 应用行交换
            for (k = 0; k < i; k++) {
                y[i] -= L[P[i]*n + k] * y[k];
            }
        }
        
        // 后向替换求解Ux = y
        for (i = n - 1; i >= 0; i--) {
            Ainv[i*n + j] = y[i];
            for (k = i + 1; k < n; k++) {
                Ainv[i*n + j] -= U[i*n + k] * Ainv[k*n + j];
            }
            // 再次检查数值稳定性
            if (fabs(U[i*n + i]) < 1e-5f) {
                return MAT_ERROR; // 主元太小，可能导致数值不稳定
            }
            Ainv[i*n + j] /= U[i*n + i];
        }
    }
    
    return MAT_SUCCESS;
}

// 专门处理3x3矩阵的求逆函数的指针版本
int mat_inverse_3x3_ptr(const float *A, float *Ainv) {
    // 计算行列式
    float det = A[0] * (A[4] * A[8] - A[5] * A[7])
              - A[1] * (A[3] * A[8] - A[5] * A[6])
              + A[2] * (A[3] * A[7] - A[4] * A[6]);
    
    // 判断是否可逆
    if (fabs(det) < 1e-5f) {
        printf("矩阵奇异，行列式接近于0: %f\n", det);
        return MAT_ERROR; // 矩阵不可逆
    }
    
    // 计算伴随矩阵的转置除以行列式
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
    int i, j;
    float L[MAX_MATRIX_DIM * MAX_MATRIX_DIM] = {0}; // 下三角矩阵
    float U[MAX_MATRIX_DIM * MAX_MATRIX_DIM] = {0}; // 上三角矩阵
    int P[MAX_MATRIX_DIM];    // 行交换记录(整数索引)
    
    if (n > MAX_MATRIX_DIM) {
        return MAT_ERROR; // 矩阵维度超出限制
    }
    
    // 使用LU分解函数计算L、U和P
    if (mat_lu_decomp_ptr(n, A, L, U, P) != MAT_SUCCESS) {
        return MAT_ERROR; // LU分解失败，矩阵可能不可逆
    }
    
    // 使用已计算的LU分解求解方程组
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
    // 计算矩阵A的行列式
    float det = A[0] * (A[4] * A[8] - A[5] * A[7])
              - A[1] * (A[3] * A[8] - A[5] * A[6])
              + A[2] * (A[3] * A[7] - A[4] * A[6]);
    
    // 判断是否可逆
    if (fabs(det) < 1e-5f) {
        return MAT_ERROR; // 矩阵不可逆
    }
    
    // 计算伴随矩阵的元素
    float adj[9];
    adj[0] = A[4] * A[8] - A[5] * A[7];  // 余子式M11
    adj[1] = A[2] * A[7] - A[1] * A[8];  // -M12
    adj[2] = A[1] * A[5] - A[2] * A[4];  // M13
    adj[3] = A[5] * A[6] - A[3] * A[8];  // -M21
    adj[4] = A[0] * A[8] - A[2] * A[6];  // M22
    adj[5] = A[2] * A[3] - A[0] * A[5];  // -M23
    adj[6] = A[3] * A[7] - A[4] * A[6];  // M31
    adj[7] = A[1] * A[6] - A[0] * A[7];  // -M32
    adj[8] = A[0] * A[4] - A[1] * A[3];  // M33
    
    // 计算伴随矩阵与B的乘积，再除以行列式
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

