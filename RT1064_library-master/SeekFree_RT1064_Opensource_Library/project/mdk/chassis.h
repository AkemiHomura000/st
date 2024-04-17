#ifndef CHASSIS_H
#define CHASSIS_H
#include "zf_common_headfile.h"
#include "pid.h"
#include "posture.h"
#define MOTOR1_DIR (C9)
#define MOTOR1_PWM (PWM2_MODULE1_CHA_C8)
#define MOTOR2_DIR (C7)
#define MOTOR2_PWM (PWM2_MODULE0_CHA_C6)
#define MOTOR4_DIR (D2)
#define MOTOR4_PWM (PWM2_MODULE3_CHB_D3)
#define MOTOR3_DIR (C10)
#define MOTOR3_PWM (PWM2_MODULE2_CHB_C11)
/**
 * @brief 使车辆前进为正方向，从右前方开始逆时针编号，分别为1、2、3、4
 */
static double speed_tar_1 = 0;
static double speed_tar_2 = 0;
static double speed_tar_3 = 0;
static double speed_tar_4 = 0;
static double speed_tar = 35;
static pid_param_t angle_pid = PID_CREATE(1.0, 1.0, 0.2, 0.1, 4.0, 3, 9.0);     // 角度环
static pid_param_t liner_vel_pid = PID_CREATE(1.0, 1.0, 0.2, 0.1, 4.0, 3, 9.0); // 速度环
static void st_pwm_init()                                                       // pwm初始化
{
    gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR1_PWM, 17000, 0);

    gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR2_PWM, 17000, 0);

    gpio_init(MOTOR3_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR3_PWM, 17000, 0);

    gpio_init(MOTOR4_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR4_PWM, 17000, 0);
}
static void st_pwm_control() // 输出pwm控制量
{
    if (speed_tar_1 > 0)
    {
        gpio_set_level(MOTOR1_DIR, GPIO_HIGH);
        pwm_set_duty(MOTOR1_PWM, speed_tar_1 * (PWM_DUTY_MAX / 100));
    }
    else
    {
        gpio_set_level(MOTOR1_DIR, GPIO_LOW);
        pwm_set_duty(MOTOR1_PWM, -speed_tar_1 * (PWM_DUTY_MAX / 100));
    }
    if (speed_tar_2 > 0)
    {
        gpio_set_level(MOTOR2_DIR, GPIO_HIGH);
        pwm_set_duty(MOTOR2_PWM, speed_tar_2 * (PWM_DUTY_MAX / 100));
    }
    else
    {
        gpio_set_level(MOTOR2_DIR, GPIO_LOW);
        pwm_set_duty(MOTOR2_PWM, -speed_tar_2 * (PWM_DUTY_MAX / 100));
    }
    if (speed_tar_3 > 0)
    {
        gpio_set_level(MOTOR3_DIR, GPIO_HIGH);
        pwm_set_duty(MOTOR3_PWM, speed_tar_3 * (PWM_DUTY_MAX / 100));
    }
    else
    {
        gpio_set_level(MOTOR3_DIR, GPIO_LOW);
        pwm_set_duty(MOTOR3_PWM, -speed_tar_3 * (PWM_DUTY_MAX / 100));
    }
    if (speed_tar_4 > 0)
    {
        gpio_set_level(MOTOR4_DIR, GPIO_HIGH);
        pwm_set_duty(MOTOR4_PWM, speed_tar_4 * (PWM_DUTY_MAX / 100));
    }
    else
    {
        gpio_set_level(MOTOR4_DIR, GPIO_LOW);
        pwm_set_duty(MOTOR4_PWM, -speed_tar_4 * (PWM_DUTY_MAX / 100));
    }
}
/**
 * @brief 测试电机编号是否正确，测试电机驱动板正负线是否接反，从一号开始依次正转3s，再从一号开始依次反转3s,不需要在main中延时!!!
 * num为计数器，50ms的延时情况下，计数到60进行状态切换；stage表示电机编号及正反转
 */
static int num = 0;
static int stage = 1;
static void number_test()
{
    if (num < 60)
    {
        speed_tar_1 = 0;
        speed_tar_2 = 0;
        speed_tar_3 = 0;
        speed_tar_4 = 0;
        switch (stage)
        {
        case 1:
            speed_tar_1 = 35;
            break;
        case 2:
            speed_tar_2 = 35;
            break;
        case 3:
            speed_tar_3 = 35;
            break;
        case 4:
            speed_tar_4 = 35;
            break;
        case 5:
            speed_tar_1 = -35;
            break;
        case 6:
            speed_tar_2 = -35;
            break;
        case 7:
            speed_tar_3 = -35;
            break;
        case 8:
            speed_tar_4 = -35;
            break;
        default:
            break;
        }
        st_pwm_control();
        num++;
        printf("\r\nstage:%d", stage);
        printf("\r\n1:%f", speed_tar_1);
        printf("\r\n2:%f", speed_tar_2);
        printf("\r\n3:%f", speed_tar_3);
        printf("\r\n4:%f", speed_tar_4);
    }
    else
    {
        num = 0;
        stage++;
    }
    if (stage > 8)
        stage = 1;
    system_delay_ms(50);
}
/**
 * @brief 基础运动功能，均为匀速
 */
static void st_car_ahead() // 前进
{
    speed_tar_1 = speed_tar;
    speed_tar_2 = speed_tar;
    speed_tar_3 = speed_tar;
    speed_tar_4 = speed_tar;
    st_pwm_control();
}
static void st_car_back()
{
    speed_tar_1 = -speed_tar;
    speed_tar_2 = -speed_tar;
    speed_tar_3 = -speed_tar;
    speed_tar_4 = -speed_tar;
    st_pwm_control();
}
static void st_car_lsideWay()
{
    speed_tar_1 = speed_tar;
    speed_tar_2 = -speed_tar;
    speed_tar_3 = speed_tar;
    speed_tar_4 = -speed_tar;
    st_pwm_control();
}
static void st_car_rsideWay()
{
    speed_tar_1 = -speed_tar;
    speed_tar_2 = speed_tar;
    speed_tar_3 = -speed_tar;
    speed_tar_4 = speed_tar;
    st_pwm_control();
}
// 对角线移动
static void st_car_diagonal()
{
    speed_tar_1 = speed_tar;
    speed_tar_2 = 0;
    speed_tar_3 = 0;
    speed_tar_4 = speed_tar;
    st_pwm_control();
}
static void st_car_turnround()
{
    speed_tar_1 = speed_tar;
    speed_tar_2 = speed_tar;
    speed_tar_3 = -speed_tar;
    speed_tar_4 = -speed_tar;
    st_pwm_control();
}
static void st_car_anticlockwise()
{
    speed_tar_1 = -speed_tar;
    speed_tar_2 = -speed_tar;
    speed_tar_3 = speed_tar;
    speed_tar_4 = speed_tar;
    st_pwm_control();
}
static void st_car_concerning()
{
    speed_tar_1 = speed_tar;
    speed_tar_2 = 0;
    speed_tar_3 = speed_tar;
    speed_tar_4 = 0;
    st_pwm_control();
}
static void st_car_stop()
{
    speed_tar_1 = 0;
    speed_tar_2 = 0;
    speed_tar_3 = 0;
    speed_tar_4 = 0;
    st_pwm_control();
}
/**
 * @brief 加入pid控制
 */
static void st_yaw_control(float yaw_target, double yaw_now)
{
    float angle_control = pid_solve(&angle_pid, yaw_now - yaw_target);
    speed_tar_1 = -angle_control;
    speed_tar_2 = angle_control;
    speed_tar_3 = angle_control;
    speed_tar_4 = -angle_control;
    st_pwm_control();
}
/**
 * @brief 线速度控制
 *
 * @param v_x 前进方向
 * @param v_y 右手坐标系确定
 * @todo 增加pid控制,确定是否使用位置控制
 */
static void st_liner_vel_control(float v_x, float v_y)
{
    speed_tar_1 = v_x + v_y;
    speed_tar_2 = v_x - v_y;
    speed_tar_3 = v_x + v_y;
    speed_tar_4 = v_x - v_y;
    st_pwm_control();
}
/**
 * @brief 底盘控制，角度+线速度
 *
 * @param delta_yaw 角度差
 * @param v_x
 * @param v_y
 * @todo 暂未确定角度的定义，正负号还需要确定，小心实验！！！
 */
static void st_chassis_control(double delta_yaw, float v_x, float v_y)
{
    float angle_control = pid_solve(&angle_pid, delta_yaw);
    speed_tar_1 = -angle_control + v_x + v_y;
    speed_tar_2 = angle_control + v_x - v_y;
    speed_tar_3 = angle_control + v_x + v_y;
    speed_tar_4 = -angle_control + v_x - v_y;
    st_pwm_control();
}
#endif