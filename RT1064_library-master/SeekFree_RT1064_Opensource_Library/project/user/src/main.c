#include "main.h"
#include "posture.h"
#include "debugger.h"
#include "flash_param.h"
#include "motor.h"
#include "circle.h"
#include "cross.h"

#include "debugger.h"
#include "camera_param.h"
#include "headfile.h"

#include <stdio.h>

#define DEBUGGER_PIN C31

void process_image();

void find_corners();

void check_straight();

double get_angel();
// �������ƫ��
float angle001;

// rt_sem_t camera_sem;
debugger_image_t img0 = CREATE_DEBUGGER_IMAGE("raw", MT9V03X_W, MT9V03X_H, NULL);
image_t img_raw = DEF_IMAGE(NULL, MT9V03X_W, MT9V03X_H);

AT_DTCM_SECTION_ALIGN(uint8_t img_thres_data[MT9V03X_H][MT9V03X_W], 64);
debugger_image_t img1 = CREATE_DEBUGGER_IMAGE("thres", MT9V03X_W, MT9V03X_H, img_thres_data);
image_t img_thres = DEF_IMAGE((uint8_t *)img_thres_data, MT9V03X_W, MT9V03X_H);

AT_DTCM_SECTION_ALIGN(uint8_t img_line_data[MT9V03X_H][MT9V03X_W], 64);
debugger_image_t img2 = CREATE_DEBUGGER_IMAGE("line", MT9V03X_W, MT9V03X_H, img_line_data);
image_t img_line = DEF_IMAGE((uint8_t *)img_line_data, MT9V03X_W, MT9V03X_H);

debugger_param_t p0 = CREATE_DEBUGGER_PARAM("thres", 0, 255, 1, &thres);

debugger_param_t p1 = CREATE_DEBUGGER_PARAM("block_size", 1, 21, 2, &block_size);

debugger_param_t p2 = CREATE_DEBUGGER_PARAM("clip_value", -20, 20, 1, &clip_value);

// Ѳ����ʼ�������ƫ��
debugger_param_t p3 = CREATE_DEBUGGER_PARAM("begin_x", 0, MT9V03X_W / 2, 1, &begin_x);

// Ѳ����ʼ��������
debugger_param_t p4 = CREATE_DEBUGGER_PARAM("begin_y", 0, MT9V03X_H, 1, &begin_y);

debugger_param_t p5 = CREATE_DEBUGGER_PARAM("line_blur_kernel", 1, 49, 2, &line_blur_kernel);

debugger_param_t p6 = CREATE_DEBUGGER_PARAM("pixel_per_meter", 0, 200, 1, &pixel_per_meter);

debugger_param_t p7 = CREATE_DEBUGGER_PARAM("sample_dist", 1e-2, 0.4, 1e-2, &sample_dist);

debugger_param_t p8 = CREATE_DEBUGGER_PARAM("angle_dist", 0, 0.4, 1e-2, &angle_dist);

debugger_param_t p9 = CREATE_DEBUGGER_PARAM("servo_kp", -100, 100, 1e-2, &servo_pid.kp);

debugger_param_t p10 = CREATE_DEBUGGER_PARAM("aim_distance", 1e-2, 1, 1e-2, &aim_distance);

bool line_show_sample = true;
debugger_option_t opt0 = CREATE_DEBUGGER_OPTION("line_show_sample", &line_show_sample);

bool line_show_blur = false;
debugger_option_t opt1 = CREATE_DEBUGGER_OPTION("line_show_blur", &line_show_blur);

bool track_left = false;
debugger_option_t opt2 = CREATE_DEBUGGER_OPTION("track_left", &track_left);

debugger_option_t opt3 = CREATE_DEBUGGER_OPTION("adc_cross", &adc_cross);

// ԭͼ���ұ���
AT_DTCM_SECTION_ALIGN(int ipts0[POINTS_MAX_LEN][2], 8);
AT_DTCM_SECTION_ALIGN(int ipts1[POINTS_MAX_LEN][2], 8);
int ipts0_num, ipts1_num;
// �任�����ұ���
AT_DTCM_SECTION_ALIGN(float rpts0[POINTS_MAX_LEN][2], 8);
AT_DTCM_SECTION_ALIGN(float rpts1[POINTS_MAX_LEN][2], 8);
int rpts0_num, rpts1_num;
// �任�����ұ���+�˲�
AT_DTCM_SECTION_ALIGN(float rpts0b[POINTS_MAX_LEN][2], 8);
AT_DTCM_SECTION_ALIGN(float rpts1b[POINTS_MAX_LEN][2], 8);
int rpts0b_num, rpts1b_num;
// �任�����ұ���+�Ⱦ����
AT_DTCM_SECTION_ALIGN(float rpts0s[POINTS_MAX_LEN][2], 8);
AT_DTCM_SECTION_ALIGN(float rpts1s[POINTS_MAX_LEN][2], 8);
int rpts0s_num, rpts1s_num;
// ���ұ��߾ֲ��Ƕȱ仯��
AT_DTCM_SECTION_ALIGN(float rpts0a[POINTS_MAX_LEN], 8);
AT_DTCM_SECTION_ALIGN(float rpts1a[POINTS_MAX_LEN], 8);
int rpts0a_num, rpts1a_num;
// ���ұ��߾ֲ��Ƕȱ仯��+�Ǽ�������
AT_DTCM_SECTION_ALIGN(float rpts0an[POINTS_MAX_LEN], 8);
AT_DTCM_SECTION_ALIGN(float rpts1an[POINTS_MAX_LEN], 8);
int rpts0an_num, rpts1an_num;
// ��/������
AT_DTCM_SECTION_ALIGN(float rptsc0[POINTS_MAX_LEN][2], 8);
AT_DTCM_SECTION_ALIGN(float rptsc1[POINTS_MAX_LEN][2], 8);
int rptsc0_num, rptsc1_num;
// ����
float (*rpts)[2];
int rpts_num;
// ��һ������
AT_DTCM_SECTION_ALIGN(float rptsn[POINTS_MAX_LEN][2], 8);
int rptsn_num;

// Y�ǵ�
int Ypt0_rpts0s_id, Ypt1_rpts1s_id;
bool Ypt0_found, Ypt1_found;

// L�ǵ�
int Lpt0_rpts0s_id, Lpt1_rpts1s_id;
bool Lpt0_found, Lpt1_found;

// ��ֱ��
bool is_straight0, is_straight1;

// ���
bool is_turn0, is_turn1;

// ��ǰѲ��ģʽ
enum track_type_e track_type = TRACK_RIGHT;

int main(void)
{
    /*
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();
    system_delay_ms(300);
    st_pwm_init();
    st_encoder_init();
    st_imu_init();
    interrupt_global_enable(0);
    while (1)
    {
        double yaw = st_get_yaw();
        st_w_kf_recount();
        st_yaw_control(30, yaw);
        st_print_kf_result();
        system_delay_ms(50);
    }
        */
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();
    system_delay_ms(300);
    st_pwm_init();
    st_encoder_init();
    st_imu_init();
    pit_ms_init(PIT_CH2, 50); // ���̿���Ƶ��20hz
    interrupt_global_enable(0);

    EnableGlobalIRQ(0);

    while (1)
    {
        /**
         * @brief ��������صĽǶȵĶ��壬���ҵ���С��ǰ����������ߵļн�
         * @todo ���ݽǶȲ���нǶȻ��Ŀ��ƣ��ݲ������ѭ��ÿ�κ�ʱ���ݶ�ʹ���ж������Ƶ����˶���ʹ�ýǶ�����������Ч�����ۼ��������
         */
        get_angel();

        system_delay_ms(50);
    }
}
int angle_form_center_line_status;  // ���߽ǶȲ��Ƿ����
double last_yaw;                    // ������һ�θ��µ�yaw
double last_angle_form_center_line; // ������һ�θ��µĽǶȲ�
double angle_control;               // �Ƕȿ�����
/**
 * @brief ���Ƶ��̵��жϺ�����Ŀǰֻ���ƽǶȣ���֤��������ٶȿ��ơ�������ѭ���ٶȽ���������ѭ���нǶȲ����δ���£��жϺ������ý����yaw���п���
 *
 */
void pit_handler_2(void)
{
    double yaw = st_get_yaw();
    if (angle001 != last_angle_form_center_line) // �ǶȲ����
    {
        last_angle_form_center_line = angle001;
        last_yaw = yaw;
        angle_control = angle001;
    }
    else
    {
        angle_control -= (st_get_yaw() - last_yaw);
        last_yaw = yaw;
    }
    st_yaw_control(angle001, yaw);
}
double get_angel()
{
    // �ȴ�����ͷ�ɼ����
    // rt_sem_take(camera_sem, RT_WAITING_FOREVER);
    img_raw.data = mt9v03x_image[0];
    img0.buffer = mt9v03x_image[0];

    // ��ʼ��������ͷͼ��
    process_image(); // ������ȡ&����
    find_corners();  // �ǵ���ȡ&ɸѡ

    // Ԥ�����,��̬Ч������
    aim_distance = 0.58;

    // �������٣��л�Ѳ�߷���  ������Բ
    if (rpts0s_num < rpts1s_num / 2 && rpts0s_num < 60)
    {
        track_type = TRACK_RIGHT;
    }
    else if (rpts1s_num < rpts0s_num / 2 && rpts1s_num < 60)
    {
        track_type = TRACK_LEFT;
    }
    else if (rpts0s_num < 20 && rpts1s_num > rpts0s_num)
    {
        track_type = TRACK_RIGHT;
    }
    else if (rpts1s_num < 20 && rpts0s_num > rpts1s_num)
    {
        track_type = TRACK_LEFT;
    }

    /**
     * @brief ����Ƿ�ص����λ��
     * @todo ɾ������ⲿ��
     */
    // check_garage();

    // �ֱ���ʮ�� ���� ��Բ��, ʮ�����ȼ����
    if (garage_type == GARAGE_NONE)
    {
        check_cross();
    }
    if (garage_type == GARAGE_NONE && cross_type == CROSS_NONE)
    {
        check_circle();
    }
    if (cross_type != CROSS_NONE)
    {
        circle_type = CIRCLE_NONE;
    }

    // ���ݼ����ִ��ģʽ
    if (cross_type != CROSS_NONE)
        run_cross();
    if (circle_type != CIRCLE_NONE)
        run_circle();

    // ���Openart�շ�
    // check_openart();

    // ���߸���
    if (cross_type != CROSS_IN)
    {
        if (track_type == TRACK_LEFT)
        {
            rpts = rptsc0;
            rpts_num = rptsc0_num;
        }
        else
        {
            rpts = rptsc1;
            rpts_num = rptsc1_num;
        }
    }
    else
    {
        // ʮ�ָ���Զ�߿���
        if (track_type == TRACK_LEFT)
        {
            track_leftline(far_rpts0s + far_Lpt0_rpts0s_id, far_rpts0s_num - far_Lpt0_rpts0s_id, rpts,
                           (int)round(angle_dist / sample_dist), pixel_per_meter * ROAD_WIDTH / 2);
            rpts_num = far_rpts0s_num - far_Lpt0_rpts0s_id;
        }
        else
        {
            track_rightline(far_rpts1s + far_Lpt1_rpts1s_id, far_rpts1s_num - far_Lpt1_rpts1s_id, rpts,
                            (int)round(angle_dist / sample_dist), pixel_per_meter * ROAD_WIDTH / 2);
            rpts_num = far_rpts1s_num - far_Lpt1_rpts1s_id;
        }
    }

    // ���ֶ�Ӧ��(��������ʼ��)
    float cx = mapx[MT9V03X_W / 2][(int)(MT9V03X_H * 0.78f)];
    float cy = mapy[MT9V03X_W / 2][(int)(MT9V03X_H * 0.78f)];
    // �������(��ʼ�����߹�һ��)
    float min_dist = 1e10;
    int begin_id = -1;
    for (int i = 0; i < rpts_num; i++)
    {
        float dx = rpts[i][0] - cx;
        float dy = rpts[i][1] - cy;
        float dist = sqrt(dx * dx + dy * dy);
        if (dist < min_dist)
        {
            min_dist = dist;
            begin_id = i;
        }
    }
    // ����ģʽ�£����������(���ڱ��߻���һȦ���������������Ϊ�������һ���㣬�Ӷ������޷���������)
    if (garage_type == GARAGE_IN_LEFT || garage_type == GARAGE_IN_RIGHT || cross_type == CROSS_IN)
        begin_id = 0;
    // �����е㣬ͬʱ����㲻����󼸸���
    if (begin_id >= 0 && rpts_num - begin_id >= 3)
    {
        // ��һ������
        rpts[begin_id][0] = cx;
        rpts[begin_id][1] = cy;
        rptsn_num = sizeof(rptsn) / sizeof(rptsn[0]);
        resample_points(rpts + begin_id, rpts_num - begin_id, rptsn, &rptsn_num, sample_dist * pixel_per_meter);
        // ԶԤê��λ��
        int aim_idx = clip(round(aim_distance / sample_dist), 0, rptsn_num - 1);
        // ��Ԥê��λ��
        int aim_idx_near = clip(round(0.25 / sample_dist), 0, rptsn_num - 1);
        // ����Զê��ƫ��ֵ
        float dx = rptsn[aim_idx][0] - cx;
        float dy = cy - rptsn[aim_idx][1] + 0.2 * pixel_per_meter;
        float dn = sqrt(dx * dx + dy * dy);
        float error = -atan2f(dx, dy) * 180 / PI;
        assert(!isnan(error));
        // �����ǽ���Զ��,�ɽ��ƹ���Stanley�㷨,����ײ·��
        // �����ê��ƫ��ֵ
        float dx_near = rptsn[aim_idx_near][0] - cx;
        float dy_near = cy - rptsn[aim_idx_near][1] + 0.2 * pixel_per_meter;
        float dn_near = sqrt(dx_near * dx_near + dy_near * dy_near);
        float error_near = -atan2f(dx_near, dy_near) * 180 / PI;
        assert(!isnan(error_near));
        // Զ��ê���ۺϿ���
        // angle = pid_solve(&servo_pid, error * far_rate + error_near * (1 - far_rate));
        // ����ƫ�����PD����
        // float angle = pid_solve(&servo_pid, error);
        // �������㷨(ֻ����Զ��)
        float pure_angle = -atanf(pixel_per_meter * 2 * 0.2 * dx / dn / dn) / PI * 180 / 2.4; // 2.4��SMOTOR_RATE;
        angle001 = pid_solve(&servo_pid, pure_angle);
        angle001 = MINMAX(angle001, -14.5, 14.5);
        return angle001;
    }
    else
    {
        // ���ߵ����(��������)���򲻿��ƶ��
        rptsn_num = 0;
    }
}
void process_image()
{
    // ԭͼ�����ұ���
    int x1 = img_raw.width / 2 - begin_x, y1 = begin_y;
    ipts0_num = sizeof(ipts0) / sizeof(ipts0[0]);
    for (; x1 > 0; x1--)
        if (AT_IMAGE(&img_raw, x1 - 1, y1) < thres)
            break;
    if (AT_IMAGE(&img_raw, x1, y1) >= thres)
        findline_lefthand_adaptive(&img_raw, block_size, clip_value, x1, y1, ipts0, &ipts0_num);
    else
        ipts0_num = 0;
    int x2 = img_raw.width / 2 + begin_x, y2 = begin_y;
    ipts1_num = sizeof(ipts1) / sizeof(ipts1[0]);
    for (; x2 < img_raw.width - 1; x2++)
        if (AT_IMAGE(&img_raw, x2 + 1, y2) < thres)
            break;
    if (AT_IMAGE(&img_raw, x2, y2) >= thres)
        findline_righthand_adaptive(&img_raw, block_size, clip_value, x2, y2, ipts1, &ipts1_num);
    else
        ipts1_num = 0;

    // ȥ����+͸�ӱ任
    for (int i = 0; i < ipts0_num; i++)
    {
        rpts0[i][0] = mapx[ipts0[i][1]][ipts0[i][0]];
        rpts0[i][1] = mapy[ipts0[i][1]][ipts0[i][0]];
    }
    rpts0_num = ipts0_num;
    for (int i = 0; i < ipts1_num; i++)
    {
        rpts1[i][0] = mapx[ipts1[i][1]][ipts1[i][0]];
        rpts1[i][1] = mapy[ipts1[i][1]][ipts1[i][0]];
    }
    rpts1_num = ipts1_num;

    // �����˲�
    blur_points(rpts0, rpts0_num, rpts0b, (int)round(line_blur_kernel));
    rpts0b_num = rpts0_num;
    blur_points(rpts1, rpts1_num, rpts1b, (int)round(line_blur_kernel));
    rpts1b_num = rpts1_num;

    // ���ߵȾ����
    rpts0s_num = sizeof(rpts0s) / sizeof(rpts0s[0]);
    resample_points(rpts0b, rpts0b_num, rpts0s, &rpts0s_num, sample_dist * pixel_per_meter);
    rpts1s_num = sizeof(rpts1s) / sizeof(rpts1s[0]);
    resample_points(rpts1b, rpts1b_num, rpts1s, &rpts1s_num, sample_dist * pixel_per_meter);

    // ���߾ֲ��Ƕȱ仯��
    local_angle_points(rpts0s, rpts0s_num, rpts0a, (int)round(angle_dist / sample_dist));
    rpts0a_num = rpts0s_num;
    local_angle_points(rpts1s, rpts1s_num, rpts1a, (int)round(angle_dist / sample_dist));
    rpts1a_num = rpts1s_num;

    // �Ƕȱ仯�ʷǼ�������
    nms_angle(rpts0a, rpts0a_num, rpts0an, (int)round(angle_dist / sample_dist) * 2 + 1);
    rpts0an_num = rpts0a_num;
    nms_angle(rpts1a, rpts1a_num, rpts1an, (int)round(angle_dist / sample_dist) * 2 + 1);
    rpts1an_num = rpts1a_num;

    // �������߸���
    track_leftline(rpts0s, rpts0s_num, rptsc0, (int)round(angle_dist / sample_dist), pixel_per_meter * ROAD_WIDTH / 2);
    rptsc0_num = rpts0s_num;
    track_rightline(rpts1s, rpts1s_num, rptsc1, (int)round(angle_dist / sample_dist), pixel_per_meter * ROAD_WIDTH / 2);
    rptsc1_num = rpts1s_num;
}
void find_corners()
{
    // ʶ��Y,L�յ�
    Ypt0_found = Ypt1_found = Lpt0_found = Lpt1_found = false;
    is_straight0 = rpts0s_num > 1. / sample_dist;
    is_straight1 = rpts1s_num > 1. / sample_dist;
    for (int i = 0; i < rpts0s_num; i++)
    {
        if (rpts0an[i] == 0)
            continue;
        int im1 = clip(i - (int)round(angle_dist / sample_dist), 0, rpts0s_num - 1);
        int ip1 = clip(i + (int)round(angle_dist / sample_dist), 0, rpts0s_num - 1);
        float conf = fabs(rpts0a[i]) - (fabs(rpts0a[im1]) + fabs(rpts0a[ip1])) / 2;

        // Y�ǵ���ֵ
        if (Ypt0_found == false && 30. / 180. * PI < conf && conf < 65. / 180. * PI && i < 0.8 / sample_dist)
        {
            Ypt0_rpts0s_id = i;
            Ypt0_found = true;
        }
        // L�ǵ���ֵ
        if (Lpt0_found == false && 70. / 180. * PI < conf && conf < 140. / 180. * PI && i < 0.8 / sample_dist)
        {
            Lpt0_rpts0s_id = i;
            Lpt0_found = true;
        }
        // ��ֱ����ֵ
        if (conf > 5. / 180. * PI && i < 1.0 / sample_dist)
            is_straight0 = false;
        if (Ypt0_found == true && Lpt0_found == true && is_straight0 == false)
            break;
    }
    for (int i = 0; i < rpts1s_num; i++)
    {
        if (rpts1an[i] == 0)
            continue;
        int im1 = clip(i - (int)round(angle_dist / sample_dist), 0, rpts1s_num - 1);
        int ip1 = clip(i + (int)round(angle_dist / sample_dist), 0, rpts1s_num - 1);
        float conf = fabs(rpts1a[i]) - (fabs(rpts1a[im1]) + fabs(rpts1a[ip1])) / 2;
        if (Ypt1_found == false && 30. / 180. * PI < conf && conf < 65. / 180. * PI && i < 0.8 / sample_dist)
        {
            Ypt1_rpts1s_id = i;
            Ypt1_found = true;
        }
        if (Lpt1_found == false && 70. / 180. * PI < conf && conf < 140. / 180. * PI && i < 0.8 / sample_dist)
        {
            Lpt1_rpts1s_id = i;
            Lpt1_found = true;
        }

        if (conf > 5. / 180. * PI && i < 1.0 / sample_dist)
            is_straight1 = false;

        if (Ypt1_found == true && Lpt1_found == true && is_straight1 == false)
            break;
    }
    // Y����μ��,�������ǵ���뼰�ǵ���ſ�����
    if (Ypt0_found && Ypt1_found)
    {
        float dx = rpts0s[Ypt0_rpts0s_id][0] - rpts1s[Ypt1_rpts1s_id][0];
        float dy = rpts0s[Ypt0_rpts0s_id][1] - rpts1s[Ypt1_rpts1s_id][1];
        float dn = sqrtf(dx * dx + dy * dy);
        if (fabs(dn - 0.45 * pixel_per_meter) < 0.15 * pixel_per_meter)
        {
            float dwx = rpts0s[clip(Ypt0_rpts0s_id + 50, 0, rpts0s_num - 1)][0] -
                        rpts1s[clip(Ypt1_rpts1s_id + 50, 0, rpts1s_num - 1)][0];
            float dwy = rpts0s[clip(Ypt0_rpts0s_id + 50, 0, rpts0s_num - 1)][1] -
                        rpts1s[clip(Ypt1_rpts1s_id + 50, 0, rpts1s_num - 1)][1];
            float dwn = sqrtf(dwx * dwx + dwy * dwy);
            if (!(dwn > 0.7 * pixel_per_meter &&
                  rpts0s[clip(Ypt0_rpts0s_id + 50, 0, rpts0s_num - 1)][0] < rpts0s[Ypt0_rpts0s_id][0] &&
                  rpts1s[clip(Ypt1_rpts1s_id + 50, 0, rpts1s_num - 1)][0] > rpts1s[Ypt1_rpts1s_id][0]))
            {
                Ypt0_found = Ypt1_found = false;
            }
        }
        else
        {
            Ypt0_found = Ypt1_found = false;
        }
    }
    // L����μ�飬����ģʽ�����, ����L�ǵ���뼰�ǵ���ſ�����
    if (garage_type == GARAGE_NONE)
    {
        if (Lpt0_found && Lpt1_found)
        {
            float dx = rpts0s[Lpt0_rpts0s_id][0] - rpts1s[Lpt1_rpts1s_id][0];
            float dy = rpts0s[Lpt0_rpts0s_id][1] - rpts1s[Lpt1_rpts1s_id][1];
            float dn = sqrtf(dx * dx + dy * dy);
            if (fabs(dn - 0.45 * pixel_per_meter) < 0.15 * pixel_per_meter)
            {
                float dwx = rpts0s[clip(Lpt0_rpts0s_id + 50, 0, rpts0s_num - 1)][0] -
                            rpts1s[clip(Lpt1_rpts1s_id + 50, 0, rpts1s_num - 1)][0];
                float dwy = rpts0s[clip(Lpt0_rpts0s_id + 50, 0, rpts0s_num - 1)][1] -
                            rpts1s[clip(Lpt1_rpts1s_id + 50, 0, rpts1s_num - 1)][1];
                float dwn = sqrtf(dwx * dwx + dwy * dwy);
                if (!(dwn > 0.7 * pixel_per_meter &&
                      rpts0s[clip(Lpt0_rpts0s_id + 50, 0, rpts0s_num - 1)][0] < rpts0s[Lpt0_rpts0s_id][0] &&
                      rpts1s[clip(Lpt1_rpts1s_id + 50, 0, rpts1s_num - 1)][0] > rpts1s[Lpt1_rpts1s_id][0]))
                {
                    Lpt0_found = Lpt1_found = false;
                }
            }
            else
            {
                Lpt0_found = Lpt1_found = false;
            }
        }
    }
}
