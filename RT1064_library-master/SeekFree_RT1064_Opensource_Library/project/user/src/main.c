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
// 舵机控制偏差
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

// 巡线起始点距中线偏差
debugger_param_t p3 = CREATE_DEBUGGER_PARAM("begin_x", 0, MT9V03X_W / 2, 1, &begin_x);

// 巡线起始点纵坐标
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

// 原图左右边线
AT_DTCM_SECTION_ALIGN(int ipts0[POINTS_MAX_LEN][2], 8);
AT_DTCM_SECTION_ALIGN(int ipts1[POINTS_MAX_LEN][2], 8);
int ipts0_num, ipts1_num;
// 变换后左右边线
AT_DTCM_SECTION_ALIGN(float rpts0[POINTS_MAX_LEN][2], 8);
AT_DTCM_SECTION_ALIGN(float rpts1[POINTS_MAX_LEN][2], 8);
int rpts0_num, rpts1_num;
// 变换后左右边线+滤波
AT_DTCM_SECTION_ALIGN(float rpts0b[POINTS_MAX_LEN][2], 8);
AT_DTCM_SECTION_ALIGN(float rpts1b[POINTS_MAX_LEN][2], 8);
int rpts0b_num, rpts1b_num;
// 变换后左右边线+等距采样
AT_DTCM_SECTION_ALIGN(float rpts0s[POINTS_MAX_LEN][2], 8);
AT_DTCM_SECTION_ALIGN(float rpts1s[POINTS_MAX_LEN][2], 8);
int rpts0s_num, rpts1s_num;
// 左右边线局部角度变化率
AT_DTCM_SECTION_ALIGN(float rpts0a[POINTS_MAX_LEN], 8);
AT_DTCM_SECTION_ALIGN(float rpts1a[POINTS_MAX_LEN], 8);
int rpts0a_num, rpts1a_num;
// 左右边线局部角度变化率+非极大抑制
AT_DTCM_SECTION_ALIGN(float rpts0an[POINTS_MAX_LEN], 8);
AT_DTCM_SECTION_ALIGN(float rpts1an[POINTS_MAX_LEN], 8);
int rpts0an_num, rpts1an_num;
// 左/右中线
AT_DTCM_SECTION_ALIGN(float rptsc0[POINTS_MAX_LEN][2], 8);
AT_DTCM_SECTION_ALIGN(float rptsc1[POINTS_MAX_LEN][2], 8);
int rptsc0_num, rptsc1_num;
// 中线
float (*rpts)[2];
int rpts_num;
// 归一化中线
AT_DTCM_SECTION_ALIGN(float rptsn[POINTS_MAX_LEN][2], 8);
int rptsn_num;

// Y角点
int Ypt0_rpts0s_id, Ypt1_rpts1s_id;
bool Ypt0_found, Ypt1_found;

// L角点
int Lpt0_rpts0s_id, Lpt1_rpts1s_id;
bool Lpt0_found, Lpt1_found;

// 长直道
bool is_straight0, is_straight1;

// 弯道
bool is_turn0, is_turn1;

// 当前巡线模式
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
    pit_ms_init(PIT_CH2, 50); // 底盘控制频率20hz
    interrupt_global_enable(0);

    EnableGlobalIRQ(0);

    while (1)
    {
        /**
         * @brief 不清楚返回的角度的定义，姑且当作小车前进方向和中线的夹角
         * @todo 根据角度差进行角度环的控制，暂不清楚主循环每次耗时，暂定使用中断来控制地盘运动：使用角度增量控制有效避免累计误差问题
         */
        get_angel();

        system_delay_ms(50);
    }
}
int angle_form_center_line_status;  // 中线角度差是否更新
double last_yaw;                    // 储存上一次更新的yaw
double last_angle_form_center_line; // 储存上一次更新的角度差
double angle_control;               // 角度控制量
/**
 * @brief 控制底盘的中断函数，目前只控制角度，验证后加入线速度控制。假设主循环速度较慢，在主循环中角度差可能未更新，中断函数利用解算的yaw进行控制
 *
 */
void pit_handler_2(void)
{
    double yaw = st_get_yaw();
    if (angle001 != last_angle_form_center_line) // 角度差更新
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
    // 等待摄像头采集完毕
    // rt_sem_take(camera_sem, RT_WAITING_FOREVER);
    img_raw.data = mt9v03x_image[0];
    img0.buffer = mt9v03x_image[0];

    // 开始处理摄像头图像
    process_image(); // 边线提取&处理
    find_corners();  // 角点提取&筛选

    // 预瞄距离,动态效果更佳
    aim_distance = 0.58;

    // 单侧线少，切换巡线方向  切外向圆
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
     * @brief 检查是否回到起点位置
     * @todo 删除出入库部分
     */
    // check_garage();

    // 分别检查十字 三叉 和圆环, 十字优先级最高
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

    // 根据检查结果执行模式
    if (cross_type != CROSS_NONE)
        run_cross();
    if (circle_type != CIRCLE_NONE)
        run_circle();

    // 检查Openart收发
    // check_openart();

    // 中线跟踪
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
        // 十字根据远线控制
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

    // 车轮对应点(纯跟踪起始点)
    float cx = mapx[MT9V03X_W / 2][(int)(MT9V03X_H * 0.78f)];
    float cy = mapy[MT9V03X_W / 2][(int)(MT9V03X_H * 0.78f)];
    // 找最近点(起始点中线归一化)
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
    // 特殊模式下，不找最近点(由于边线会绕一圈回来，导致最近点为边线最后一个点，从而中线无法正常生成)
    if (garage_type == GARAGE_IN_LEFT || garage_type == GARAGE_IN_RIGHT || cross_type == CROSS_IN)
        begin_id = 0;
    // 中线有点，同时最近点不是最后几个点
    if (begin_id >= 0 && rpts_num - begin_id >= 3)
    {
        // 归一化中线
        rpts[begin_id][0] = cx;
        rpts[begin_id][1] = cy;
        rptsn_num = sizeof(rptsn) / sizeof(rptsn[0]);
        resample_points(rpts + begin_id, rpts_num - begin_id, rptsn, &rptsn_num, sample_dist * pixel_per_meter);
        // 远预锚点位置
        int aim_idx = clip(round(aim_distance / sample_dist), 0, rptsn_num - 1);
        // 近预锚点位置
        int aim_idx_near = clip(round(0.25 / sample_dist), 0, rptsn_num - 1);
        // 计算远锚点偏差值
        float dx = rptsn[aim_idx][0] - cx;
        float dy = cy - rptsn[aim_idx][1] + 0.2 * pixel_per_meter;
        float dn = sqrt(dx * dx + dy * dy);
        float error = -atan2f(dx, dy) * 180 / PI;
        assert(!isnan(error));
        // 若考虑近点远点,可近似构造Stanley算法,避免撞路肩
        // 计算近锚点偏差值
        float dx_near = rptsn[aim_idx_near][0] - cx;
        float dy_near = cy - rptsn[aim_idx_near][1] + 0.2 * pixel_per_meter;
        float dn_near = sqrt(dx_near * dx_near + dy_near * dy_near);
        float error_near = -atan2f(dx_near, dy_near) * 180 / PI;
        assert(!isnan(error_near));
        // 远近锚点综合考虑
        // angle = pid_solve(&servo_pid, error * far_rate + error_near * (1 - far_rate));
        // 根据偏差进行PD计算
        // float angle = pid_solve(&servo_pid, error);
        // 纯跟踪算法(只考虑远点)
        float pure_angle = -atanf(pixel_per_meter * 2 * 0.2 * dx / dn / dn) / PI * 180 / 2.4; // 2.4是SMOTOR_RATE;
        angle001 = pid_solve(&servo_pid, pure_angle);
        angle001 = MINMAX(angle001, -14.5, 14.5);
        return angle001;
    }
    else
    {
        // 中线点过少(出现问题)，则不控制舵机
        rptsn_num = 0;
    }
}
void process_image()
{
    // 原图找左右边线
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

    // 去畸变+透视变换
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

    // 边线滤波
    blur_points(rpts0, rpts0_num, rpts0b, (int)round(line_blur_kernel));
    rpts0b_num = rpts0_num;
    blur_points(rpts1, rpts1_num, rpts1b, (int)round(line_blur_kernel));
    rpts1b_num = rpts1_num;

    // 边线等距采样
    rpts0s_num = sizeof(rpts0s) / sizeof(rpts0s[0]);
    resample_points(rpts0b, rpts0b_num, rpts0s, &rpts0s_num, sample_dist * pixel_per_meter);
    rpts1s_num = sizeof(rpts1s) / sizeof(rpts1s[0]);
    resample_points(rpts1b, rpts1b_num, rpts1s, &rpts1s_num, sample_dist * pixel_per_meter);

    // 边线局部角度变化率
    local_angle_points(rpts0s, rpts0s_num, rpts0a, (int)round(angle_dist / sample_dist));
    rpts0a_num = rpts0s_num;
    local_angle_points(rpts1s, rpts1s_num, rpts1a, (int)round(angle_dist / sample_dist));
    rpts1a_num = rpts1s_num;

    // 角度变化率非极大抑制
    nms_angle(rpts0a, rpts0a_num, rpts0an, (int)round(angle_dist / sample_dist) * 2 + 1);
    rpts0an_num = rpts0a_num;
    nms_angle(rpts1a, rpts1a_num, rpts1an, (int)round(angle_dist / sample_dist) * 2 + 1);
    rpts1an_num = rpts1a_num;

    // 左右中线跟踪
    track_leftline(rpts0s, rpts0s_num, rptsc0, (int)round(angle_dist / sample_dist), pixel_per_meter * ROAD_WIDTH / 2);
    rptsc0_num = rpts0s_num;
    track_rightline(rpts1s, rpts1s_num, rptsc1, (int)round(angle_dist / sample_dist), pixel_per_meter * ROAD_WIDTH / 2);
    rptsc1_num = rpts1s_num;
}
void find_corners()
{
    // 识别Y,L拐点
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

        // Y角点阈值
        if (Ypt0_found == false && 30. / 180. * PI < conf && conf < 65. / 180. * PI && i < 0.8 / sample_dist)
        {
            Ypt0_rpts0s_id = i;
            Ypt0_found = true;
        }
        // L角点阈值
        if (Lpt0_found == false && 70. / 180. * PI < conf && conf < 140. / 180. * PI && i < 0.8 / sample_dist)
        {
            Lpt0_rpts0s_id = i;
            Lpt0_found = true;
        }
        // 长直道阈值
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
    // Y点二次检查,依据两角点距离及角点后张开特性
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
    // L点二次检查，车库模式不检查, 依据L角点距离及角点后张开特性
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
