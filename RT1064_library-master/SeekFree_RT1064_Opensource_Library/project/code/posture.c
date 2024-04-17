#include "posture.h"
void st_w_kf_init(void)
{
    MATRIX_TYPE x_[2][1] = {0.0001, 0.0001};
    int row = sizeof(x_) / sizeof(x_[0]);
    int column = sizeof(x_[0]) / sizeof(x_[0][0]);
    x = Matrix_gen(row, column, x_);
    MATRIX_TYPE P_[2][2] = {1, 0, 0, 1};
    row = sizeof(P_) / sizeof(P_[0]);
    column = sizeof(P_[0]) / sizeof(P_[0][0]);
    P = Matrix_gen(row, column, P_);
    ekf = create_ekf(2, 1, x, P);
    MATRIX_TYPE F_[2][2] = {1, 0.05, 0, 1};
    row = sizeof(F_) / sizeof(F_[0]);
    column = sizeof(F_[0]) / sizeof(F_[0][0]);
    F = Matrix_gen(row, column, F_);
    Q = M_I(2);
    Q->data[0] = 1;
    Q->data[3] = 3;
    z = M_I(1);
    MATRIX_TYPE H_[1][2] = {0, 1};
    row = sizeof(H_) / sizeof(H_[0]);
    column = sizeof(H_[0]) / sizeof(H_[0][0]);
    kf_H = Matrix_gen(row, column, H_);
    R = M_I(1);
    R->data[0] = 2;
}
void st_encoder_init(void) // 编码器初始化
{
    st_w_kf_init();
    encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B);
    encoder_quad_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B);
    encoder_quad_init(ENCODER_3, ENCODER_3_A, ENCODER_3_B);
    encoder_quad_init(ENCODER_4, ENCODER_4_A, ENCODER_4_B);
    pit_ms_init(PIT_CH_0, 100);
}
void st_imu_init(void) // imu初始化，会一直等待 TODO：初始化失败的提示
{
    while (1)
    {
        if (imu660ra_init())
        {
            printf("\r\nIMU660RA init error.");
        }
        else
        {
            break;
        }
    }
    pit_ms_init(PIT_CH_1, 5);
}
void st_w_kf_recount(void) // kf重新开始计算yaw
{
    ekf->x->data[0] = 0.0001;
    ekf->x->data[1] = 0.0001;
    ekf->P->data[0] = 1;
    ekf->P->data[1] = 0;
    ekf->P->data[2] = 0;
    ekf->P->data[3] = 1;
}
void st_print_kf_result(void)
{
    printf("\r\nyaw: %f\r\n", ekf->x->data[0]);
    printf("\r\nw: %f\r\n", z->data[0]);
}
void pit_handler_0(void)
{
    encoder_data_1 = encoder_get_count(ENCODER_1); // ????±à???÷????
    encoder_data_1 = 6.3 * 3 * 2 * 3.1415926 * encoder_data_1 / (7 * 1024 * 0.1);
    encoder_clear_count(ENCODER_1); // ????±à???÷????

    encoder_data_2 = encoder_get_count(ENCODER_2); // ????±à???÷????
    encoder_data_2 = -6.3 * 3 * 2 * 3.1415926 * encoder_data_2 / (7 * 1024 * 0.1);
    encoder_clear_count(ENCODER_2); // ????±à???÷????

    encoder_data_3 = encoder_get_count(ENCODER_3); // ????±à???÷????
    encoder_data_3 = -6.3 * 3 * 2 * 3.1415926 * encoder_data_3 / (7 * 1024 * 0.1);
    encoder_clear_count(ENCODER_3); // ????±à???÷????

    encoder_data_4 = encoder_get_count(ENCODER_4); // ????±à???÷????
    encoder_data_4 = 6.3 * 3 * 2 * 3.1415926 * encoder_data_4 / (7 * 1024 * 0.1);
    encoder_clear_count(ENCODER_4); // ????±à???÷????

    vy_encoder = (encoder_data_1 + encoder_data_2 + encoder_data_3 + encoder_data_4) / 4;
    vx_encoder = (-encoder_data_1 + encoder_data_2 - encoder_data_3 + encoder_data_4) / 4;
    w_encoder = (+encoder_data_1 - encoder_data_2 - encoder_data_3 + encoder_data_4) / (18 * 2 + 20 * 2) / 3.1415926 * 180 * 0.5625;
}
void pit_handler_1(void) // imu读值，并完成卡尔曼滤波
{
    imu660ra_get_acc();
    imu660ra_get_gyro();
    if (imu_init_num < 50)
    {
        init_w += imu660ra_gyro_y;
        imu_init_num++;
    }
    if (imu_init_num == 50)
    {
        init_w /= 50;
        imu_init_num++;
    }
    if (imu_init_num > 50)
    {
        imu660ra_gyro_y -= init_w;
        w_gyro = -imu660ra_gyro_y * 200 / 1515 * 0.464;
        predict_kf(ekf, F, Q);
        // printf("w:%f\n", z->data[0]);
        if (abs(w_gyro) < 40)
        {
            z->data[0] = w_encoder * 0.9 + w_gyro * 0.1;
        }
        else
        {
            z->data[0] = w_encoder * 0.3 + w_gyro * 0.7;
        }
        update_kf(ekf, z, kf_H, R);
        yaw = ekf->x->data[0];
    }
}
