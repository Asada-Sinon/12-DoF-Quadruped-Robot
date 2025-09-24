#ifndef MATRIX_H
#define MATRIX_H
#include "stm32f767xx.h"
#include "arm_math.h"
#define MAX_MATRIX_DIM 28  // ���ھ�����������ά��

#define MAT_SUCCESS 0     // �����ɹ�
#define MAT_ERROR -1      // ����ʧ��

// ����˷�: C = A * B (ʹ��ָ����ʽ)
void mat_mult_ptr(int m, int n, int p, const float *A, const float *B, float *C);

// �����������: y = A * x (ʹ��ָ����ʽ)
void mat_mult_vec_ptr(int m, int n, const float *A, const float *x, float *y);

// ����ӷ�: C = A + B (ʹ��ָ����ʽ)
void mat_add_ptr(int m, int n, const float *A, const float *B, float *C);

// �������: C = A - B (ʹ��ָ����ʽ)
void mat_sub_ptr(int m, int n, const float *A, const float *B, float *C);

// ����ת��: B = A^T (ʹ��ָ����ʽ)
void mat_transpose_ptr(int m, int n, const float *A, float *B);

// ��������: c = a - b (ʹ��ָ����ʽ)
void vec_sub_ptr(int n, const float *a, const float *b, float *c);

// �����ӷ�: c = a + b (ʹ��ָ����ʽ)
void vec_add_ptr(int n, const float *a, const float *b, float *c);

// �����ӿ�ӷ�: C = A + B
void mat_add_block_ptr(int row, int col, int m, int n, int ld,
                      const float *A, const float *B, float *C);

// �����ӿ����: C(destRow:destRow+m-1, destCol:destCol+n-1) = A(srcRowA:srcRowA+m-1, srcColA:srcColA+n-1) - B(srcRowB:srcRowB+m-1, srcColB:srcColB+n-1)
void mat_subblock_sub_ptr(int m, int n, 
                          const float *A, int ldA, int srcRowA, int srcColA, 
                          const float *B, int ldB, int srcRowB, int srcColB, 
                          float *C, int ldC, int destRow, int destCol);

// �����ӿ鸴��: C(destRow:destRow+m-1, destCol:destCol+n-1) = A(srcRow:srcRow+m-1, srcCol:srcCol+n-1)
void mat_subblock_copy_ptr(int m, int n, 
                           const float *A, int ldA, int srcRow, int srcCol, 
                           float *C, int ldC, int destRow, int destCol);

// ��ά����ֱ�Ӳ��
void vec3_cross(const float a[3], const float b[3], float c[3]);

// ��ά������� (ָ��汾)
void vec3_cross_ptr(const float *a, const float *b, float *c);

// ���������ָ��汾 (ʹ��LU�ֽⷨ)
int mat_inverse_ptr(int n, const float *A, float *Ainv);

// ר�Ŵ���3x3��������溯����ָ��汾
int mat_inverse_3x3_ptr(const float *A, float *Ainv);

// ʹ��LU�ֽ������󷽳� A * X = B���õ� X = A^-1 * B
int mat_solve_ptr(int n, const float *A, int m, const float *B, float *X);

// ר�Ŵ���3x3���󷽳����Ŀ��ٺ��� A * X = B���õ� X = A^-1 * B
int mat_solve_3x3_ptr(const float *A, int m, const float *B, float *X);

// ʹ���Ѽ����LU�ֽ�������Է����� A * X = B���õ� X = A^-1 * B
int mat_solve_with_lu_ptr(int n, const float *L, const float *U, const int *P, 
                          int m, const float *B, float *X);

// ��������LU�ֽ� (ʹ�ò�����Ԫ��)
int mat_lu_decomp_ptr(int n, const float *A, float *L, float *U, int *P);

void mat_mult_18x18_optimized(float32_t *A, float32_t *B, float32_t *C);
#endif
