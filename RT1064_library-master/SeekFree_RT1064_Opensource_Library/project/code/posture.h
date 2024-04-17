#ifndef POSTURE_H
#define POSTURE_H
#include "zf_common_headfile.h"
#include "matrix.h"
#include "ekf.h"

#define ENCODER_1 (QTIMER1_ENCODER1)
#define ENCODER_1_A (QTIMER1_ENCODER1_CH1_C0)
#define ENCODER_1_B (QTIMER1_ENCODER1_CH2_C1)
#define ENCODER_2 (QTIMER1_ENCODER2)
#define ENCODER_2_A (QTIMER1_ENCODER2_CH1_C2)
#define ENCODER_2_B (QTIMER1_ENCODER2_CH2_C24)
#define ENCODER_3 (QTIMER2_ENCODER1)
#define ENCODER_3_A (QTIMER2_ENCODER1_CH1_C3)
#define ENCODER_3_B (QTIMER2_ENCODER1_CH2_C4)
#define ENCODER_4 (QTIMER2_ENCODER2)
#define ENCODER_4_A (QTIMER2_ENCODER2_CH1_C5)
#define ENCODER_4_B (QTIMER2_ENCODER2_CH2_C25)
/**
 * @brief 编码器使用中断PIT_CH_0，陀螺仪使用PIT_CH_1
 */
#define PIT_CH_0 (PIT_CH0)
#define PIT_CH_1 (PIT_CH1)
double encoder_data_1 = 0;
double encoder_data_2 = 0;
double encoder_data_3 = 0;
double encoder_data_4 = 0;
double w_encoder = 0;  // 质心角速度，逆时针为正
double vx_encoder = 0; // 向右为正
double vy_encoder = 0; // 向前为正
double yaw = 0;        // yaw为全局变量
double test_yaw = 0;
int imu_init_num = 0; // 零漂计数
double init_w = 0;    // bias
double w_gyro = 0;

Matrix *x;
Matrix *P;
EKF *ekf;
Matrix *F;
Matrix *Q;
Matrix *z;
Matrix *kf_H;
Matrix *R;

void st_encoder_init(void);   // 编码器初始化
void st_imu_init(void);       // imu初始化，会一直等待 TODO：初始化失败的提示
void st_w_kf_init(void);      // kf初始化，改变中断周期需调整F矩阵！！！
void st_w_kf_recount(void);   // kf重新开始计算yaw
void st_print_kf_result(void);
void pit_handler_0(void); // 编码器读值
void pit_handler_1(void); // imu读值，并完成卡尔曼滤波


#endif