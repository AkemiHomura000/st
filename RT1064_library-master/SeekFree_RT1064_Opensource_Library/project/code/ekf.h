/*
卡尔曼滤波库，结构体统一命名为EKF，更新和预测方程可选kf或ekf
*/
#ifndef KF_H
#define KF_H

#include <stdlib.h>
#include "matrix.h"

typedef struct
{
    Matrix *x;
    Matrix *P;
    Matrix *I;
    Matrix *K;      // x_size * z_sizeE
    Matrix *result; // 暂存ekf中h(x)的值
    int x_size;
    int z_size;
} EKF;

// 初始化ekf,需传入状态量个数，状态向量和误差协方差矩阵,x的初值不要用零！！!
EKF *create_ekf(int x_size, int z_size, Matrix *x, Matrix *P)
{
    EKF *ekf = (EKF *)malloc(sizeof(EKF));
    ekf->x = x;
    ekf->P = P;
    ekf->K = M_Zeros(x_size, z_size);
    ekf->I = M_I(x_size);
    ekf->result = M_Zeros(z_size, 1);
    return ekf;
}
// 释放内存
void destroy_ekf(EKF *ekf)
{
    if (ekf != NULL)
    {
        M_free(ekf->I);
        M_free(ekf->P);
        M_free(ekf->x);
        free(ekf);
    }
}
// kf用，不带控制量输入,中间变量全部free
void predict_kf(EKF *ekf, Matrix *F, Matrix *Q)
{
    Matrix *re1 = M_mul(F, ekf->x);
    memcpy(ekf->x->data, re1->data, (re1->row * re1->column) * sizeof(MATRIX_TYPE));
    M_free(re1);
    Matrix *re5 = M_mul(F, ekf->P);
    Matrix *re2 = M_T(F);
    Matrix *re3 = M_mul(re5, re2);
    Matrix *re4 = M_add_sub(1, re3, -1, Q);
    memcpy(ekf->P->data, re4->data, (re4->row * re4->column) * sizeof(MATRIX_TYPE));
    M_free(re4);
    M_free(re5);
    M_free(re2);
    M_free(re3);
    // ekf->x = M_mul(F, ekf->x);
    // ekf->P = M_add_sub(1, M_mul(M_mul(F, ekf->P), M_T(F)), -1, Q);
}
void update_kf(EKF *ekf, Matrix *z, Matrix *H, Matrix *R)
{
    Matrix *re1 = M_T(H);
     Matrix *re2 = M_mul(H, ekf->P);
    Matrix *re3 = M_mul(re2, re1);
    M_free(re2);
    Matrix *re4 = M_add_sub(1, re3, -1, R);
    M_free(re3);
    Matrix *re5 = M_Inverse(re4);
    M_free(re4);
    Matrix *re6 = M_mul(ekf->P, re1);
    M_free(re1);
    Matrix *re01 = M_mul(re6, re5);
    memcpy(ekf->K->data, re01->data, (re01->row * re01->column) * sizeof(MATRIX_TYPE));
    M_free(re01);
    M_free(re5);
    M_free(re6);
    Matrix *re7 = M_mul(H, ekf->x);
    Matrix *re8 = M_add_sub(1, z, 1, re7);
    M_free(re7);
    Matrix *re9 = M_mul(ekf->K, re8);
    Matrix *re02 = M_add_sub(1, ekf->x, -1, re9);
    memcpy(ekf->x->data, re02->data, (re02->row * re02->column) * sizeof(MATRIX_TYPE));
    M_free(re02);
    M_free(re8);
    M_free(re9);
    Matrix *re10 = M_mul(ekf->K, H);
    Matrix *re11 = M_add_sub(1, ekf->I, 1, re10);
    Matrix *re03 = M_mul(re11, ekf->P);
    memcpy(ekf->P->data, re03->data, (re03->row * re03->column) * sizeof(MATRIX_TYPE));
    M_free(re03);
    M_free(re10);
    M_free(re11);
    // ekf->K = M_mul(M_mul(ekf->P, M_T(H)), M_Inverse(M_add_sub(1, M_mul(M_mul(H, ekf->P), M_T(H)), -1, R)));
    // ekf->x = M_add_sub(1, ekf->x, -1, M_mul(ekf->K, M_add_sub(1, z, 1, M_mul(H, ekf->x))));
    // ekf->P = M_mul(M_add_sub(1, ekf->I, 1, M_mul(ekf->K, H)), ekf->P);
}

// ekf用，不带控制量输入，需传入状态转移函数(参数和返回值均为指针类型)
void predict_ekf(EKF *ekf, Matrix *F, Matrix *Q, Matrix *(*f)(Matrix *))
{
    ekf->x = f(ekf->x);
    ekf->P = M_add_sub(1, M_mul(M_mul(F, ekf->P), M_T(F)), -1, Q);
}
void update_ekf(EKF *ekf, Matrix *z, Matrix *H, Matrix *R, Matrix *(*h)(Matrix *, Matrix *))
{
    ekf->K = M_mul(M_mul(ekf->P, M_T(H)), M_Inverse(M_add_sub(1, M_mul(M_mul(H, ekf->P), M_T(H)), -1, R)));
    ekf->x = M_add_sub(1, ekf->x, -1, M_mul(ekf->K, M_add_sub(1, z, 1, h(ekf->x, ekf->result))));
    ekf->P = M_mul(M_add_sub(1, ekf->I, 1, M_mul(ekf->K, H)), ekf->P);
}
#endif