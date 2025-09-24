#ifndef MATRIX_H
#define MATRIX_H
#include "stm32f767xx.h"
#include "arm_math.h"
#define MAX_MATRIX_DIM 28  // 用于矩阵操作的最大维度

#define MAT_SUCCESS 0     // 操作成功
#define MAT_ERROR -1      // 操作失败

// 矩阵乘法: C = A * B (使用指针形式)
void mat_mult_ptr(int m, int n, int p, const float *A, const float *B, float *C);

// 矩阵乘以向量: y = A * x (使用指针形式)
void mat_mult_vec_ptr(int m, int n, const float *A, const float *x, float *y);

// 矩阵加法: C = A + B (使用指针形式)
void mat_add_ptr(int m, int n, const float *A, const float *B, float *C);

// 矩阵减法: C = A - B (使用指针形式)
void mat_sub_ptr(int m, int n, const float *A, const float *B, float *C);

// 矩阵转置: B = A^T (使用指针形式)
void mat_transpose_ptr(int m, int n, const float *A, float *B);

// 向量减法: c = a - b (使用指针形式)
void vec_sub_ptr(int n, const float *a, const float *b, float *c);

// 向量加法: c = a + b (使用指针形式)
void vec_add_ptr(int n, const float *a, const float *b, float *c);

// 矩阵子块加法: C = A + B
void mat_add_block_ptr(int row, int col, int m, int n, int ld,
                      const float *A, const float *B, float *C);

// 矩阵子块减法: C(destRow:destRow+m-1, destCol:destCol+n-1) = A(srcRowA:srcRowA+m-1, srcColA:srcColA+n-1) - B(srcRowB:srcRowB+m-1, srcColB:srcColB+n-1)
void mat_subblock_sub_ptr(int m, int n, 
                          const float *A, int ldA, int srcRowA, int srcColA, 
                          const float *B, int ldB, int srcRowB, int srcColB, 
                          float *C, int ldC, int destRow, int destCol);

// 矩阵子块复制: C(destRow:destRow+m-1, destCol:destCol+n-1) = A(srcRow:srcRow+m-1, srcCol:srcCol+n-1)
void mat_subblock_copy_ptr(int m, int n, 
                           const float *A, int ldA, int srcRow, int srcCol, 
                           float *C, int ldC, int destRow, int destCol);

// 三维向量直接叉乘
void vec3_cross(const float a[3], const float b[3], float c[3]);

// 三维向量叉乘 (指针版本)
void vec3_cross_ptr(const float *a, const float *b, float *c);

// 矩阵求逆的指针版本 (使用LU分解法)
int mat_inverse_ptr(int n, const float *A, float *Ainv);

// 专门处理3x3矩阵的求逆函数的指针版本
int mat_inverse_3x3_ptr(const float *A, float *Ainv);

// 使用LU分解求解矩阵方程 A * X = B，得到 X = A^-1 * B
int mat_solve_ptr(int n, const float *A, int m, const float *B, float *X);

// 专门处理3x3矩阵方程求解的快速函数 A * X = B，得到 X = A^-1 * B
int mat_solve_3x3_ptr(const float *A, int m, const float *B, float *X);

// 使用已计算的LU分解求解线性方程组 A * X = B，得到 X = A^-1 * B
int mat_solve_with_lu_ptr(int n, const float *L, const float *U, const int *P, 
                          int m, const float *B, float *X);

// 计算矩阵的LU分解 (使用部分主元法)
int mat_lu_decomp_ptr(int n, const float *A, float *L, float *U, int *P);

void mat_mult_18x18_optimized(float32_t *A, float32_t *B, float32_t *C);
#endif
