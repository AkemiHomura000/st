#include "motor.h"
#include "pid.h"

#define MOTOR1_DIR (C9)
#define MOTOR1_PWM (PWM2_MODULE1_CHA_C8)
#define MOTOR2_DIR (C7)
#define MOTOR2_PWM (PWM2_MODULE0_CHA_C6)
#define MOTOR4_DIR (D2)
#define MOTOR4_PWM (PWM2_MODULE3_CHB_D3)
#define MOTOR3_DIR (C10)
#define MOTOR3_PWM (PWM2_MODULE2_CHB_C11)


//#define MINMAX(x, l, u) MIN(MAX(x, l), u)
#define MOTOR_PWM_DUTY_MAX    50000

//�����PID
//motor_param_t motor_r = MOTOR_CREATE(12, 18, 1, 15, 2500, 250, 10,MOTOR_PWM_DUTY_MAX/3 ,MOTOR_PWM_DUTY_MAX/3 ,MOTOR_PWM_DUTY_MAX/3);

//��������PID
motor_param_t motor_l = MOTOR_CREATE(12, 1000, 25, 100, 5000, 500, 600, 1, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
motor_param_t motor_r = MOTOR_CREATE(12, 1000, 25, 100, 5000, 500, 600, 1, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);

// Matlabλ��PID
// 0.5s
pid_param_t motor_pid_l = PID_CREATE(7021, 10000 / 1e3, 0, 1, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
pid_param_t motor_pid_r = PID_CREATE(7021, 10000 / 1e3, 0, 1, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
// 0.4s
//pid_param_t motor_pid_l = PID_CREATE(8830, 14117/1e3, 0, 1, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
//pid_param_t motor_pid_r = PID_CREATE(8830, 14117/1e3, 0, 1, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
// 0.3s
//pid_param_t motor_pid_l = PID_CREATE(11864, 22367/1e3, 0, 1, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
//pid_param_t motor_pid_r = PID_CREATE(11864, 22367/1e3, 0, 1, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);

// �������������������Ŀ���ٶ�
pid_param_t target_speed_pid = PID_CREATE(5., 0, 30., 0.6, 5, 5, 5);

// λ�û�PID
pid_param_t posloop_pid = PID_CREATE(200., 0, 0., 0.7, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);

// ��׼�ٶ�
float NORMAL_SPEED = 17.;  //16.4
// ��ǰĿ���ٶ�
float target_speed;

//Բ���ٶ� + NORMAL_SPEED
float CIRCLE_MAX_SPEED = 0, CIRCLE_MIN_SPEED = -6;
//�ٶ���+  NORMAL_SPEED
float NORMAL_MAX_SPEED = 0, NORMAL_MIN_SPEED = -8;



// ����Բ���뾶
float radius_3pts(float pt0[2], float pt1[2], float pt2[2]) {
    float a, b, c, d, e, f, r, x, y;
    a = 2 * (pt1[0] - pt0[0]);
    b = 2 * (pt1[1] - pt0[1]);
    c = pt1[0] * pt1[0] + pt1[1] * pt1[1] - pt0[0] * pt0[0] - pt0[1] * pt0[1];
    d = 2 * (pt2[0] - pt1[0]);
    e = 2 * (pt2[1] - pt1[1]);
    f = pt2[0] * pt2[0] + pt2[1] * pt2[1] - pt1[0] * pt1[0] - pt1[1] * pt1[1];
    x = (b * f - e * c) / (b * d - e * a);
    y = (d * c - a * f) / (b * d - e * a);
    r = sqrt((x - pt0[0]) * (x - pt0[0]) + (y - pt0[1]) * (y - pt0[1]));
    return r;
}

// �������ɣ����ڵ���
uint32_t clk;
void square_signal(void) {
    clk += 1;
    if (clk > 10000) { clk = 0; }
    if (clk < 2000) {
        motor_l.target_speed = 20;
        motor_r.target_speed = 20;
    } else if (clk < 4000) {
        motor_l.target_speed = 0;
        motor_r.target_speed = 0;
    } else if (clk < 6000) {
        motor_l.target_speed = 15;
        motor_r.target_speed = 15;
    } else if (clk < 8000) {
        motor_l.target_speed = 28;
        motor_r.target_speed = 28;
    } else if (clk < 10000) {
        motor_l.target_speed = 0;
        motor_r.target_speed = 0;
    }
}

void speed_control(void) {
    //����
    float diff = 15.8 * tan(angle001 / 180 * PI * 2.4) / 40 / 2;
    //diff = MINMAX(diff, -NORMAL_SPEED / 4, NORMAL_SPEED / 4);

    // Ĭ�ϳ������ģʽ
    motor_l.motor_mode = MODE_NORMAL;
    motor_r.motor_mode = MODE_NORMAL;
/**
    extern float laser_angle;
    //����ͣ�� ����
    if (rt_tick_get_millisecond() - openart.animaltime < 2500 && openart.fa_type == ANIMAL) {
        target_speed = 0;
        diff = 0;
    } else if (openart.fruit_delta < 0 && (laser_angle < 5 || laser_angle > 175)) { // ����ת�����ˣ�����
        target_speed = -1;
    } else if (openart.tag_type == TAG_SEARCH) {                                    // Ѱ�Ұ��ƣ�����ǰ��
        target_speed = 1;
    } else if (openart.tag_type == TAG_STOP || openart.tag_type == TAG_SHOOTING) {  // ��ʼ��У�ͣ��
        target_speed = 0;
    }
       // �ڰ�apriltimeͣ��
    else if (apriltag_type == APRILTAG_FOUND) {
        target_speed = 0.5; // ����ǰ��������ȫͣ����OpenArtȴûʶ��apriltag�����
        diff = 0;

    } else if (apriltag_type == APRILTAG_MAYBE) {
        // Զ���ڰߣ�����
        target_speed = 1;
        motor_l.motor_mode = MODE_NORMAL;
        motor_r.motor_mode = MODE_NORMAL;
    } else if (garage_type == GARAGE_OUT_LEFT || garage_type == GARAGE_OUT_RIGHT) {
        // ���⻺����ģʽ������һ�³��̫�ͣ��������
        motor_l.motor_mode = MODE_SOFT;
        motor_r.motor_mode = MODE_SOFT;
        target_speed = 14;
    } else if (garage_type == GARAGE_IN_RIGHT || garage_type == GARAGE_IN_LEFT) {
        // ����м���
        target_speed = 10;
    } else if (enable_adc) {
        // �µ����٣�������·���
        target_speed = 9;
        motor_l.motor_mode = MODE_BANGBANG;
        motor_r.motor_mode = MODE_BANGBANG;
    } else if (yroad_type == YROAD_NEAR) {
        //����near, ����ͣ��
        target_speed = YROAD_NEAR_SPEED;
    } else if (yroad_type == YROAD_FOUND) {
        //����found, ����
        target_speed = YROAD_FOUND_SPEED;
    } else */
		if (circle_type == CIRCLE_LEFT_BEGIN || circle_type == CIRCLE_RIGHT_BEGIN) {
        // Բ���ٶ�  ��Բ��max 16.2 -1.5
        // Բ����ʼ��б�º�������
        target_speed = MINMAX(target_speed - 0.02, NORMAL_SPEED + CIRCLE_MIN_SPEED, NORMAL_SPEED + CIRCLE_MAX_SPEED);
    }
//    else if (circle_type == CIRCLE_LEFT_END || circle_type == CIRCLE_RIGHT_END) {
//        // ��������
//        target_speed = MINMAX(target_speed + 0.01, NORMAL_SPEED + CIRCLE_MIN_SPEED, NORMAL_SPEED);
//    }
    else if (rptsn_num > 20) {
        // ֱ��/����ٶ�
        int id = MIN(70, rptsn_num - 1);
        float error = fabs((rptsn[id][0] - rptsn[0][0]) / (rptsn[id][1] - rptsn[0][1]));

        // ���ټ���kd, ͻ�����
//        if(error >= 0.5) target_speed_pid.kd = 20;
//        else target_speed_pid.kd = 0;

        float speed = -pid_solve(&target_speed_pid, error);
        target_speed = MINMAX(NORMAL_SPEED + speed, NORMAL_SPEED + NORMAL_MIN_SPEED, NORMAL_SPEED + NORMAL_MAX_SPEED);
    } else if (rptsn_num > 5) {
        //��̫��,���Ծ�ֱ������
        target_speed = NORMAL_SPEED + NORMAL_MIN_SPEED;
    }
    // ��ͣ(����ֵ̫С���������)
 /*   if (  elec_data[0] + elec_data[1] < 60)) {
        motor_l.motor_mode = MODE_NORMAL;
        motor_r.motor_mode = MODE_NORMAL;
        target_speed = 0;
        diff = 0;
    }
*/
    // ��̬���Kp
    //servo_pid.kp = 1. + (motor_l.encoder_speed + motor_r.encoder_speed) / 40;

    // ���Ӳ���
    motor_l.target_speed = target_speed - diff * target_speed;
    motor_r.target_speed = target_speed + diff * target_speed;
}

void motor_control_001() {
    //square_signal();

    //wireless_show();

    speed_control();


    if (motor_l.motor_mode == MODE_NORMAL) {
        // ����ģʽ
        float error = (motor_l.target_speed - motor_l.encoder_speed);
        motor_l.duty = pid_solve(&motor_pid_l, error);
        motor_l.duty = MINMAX(motor_l.duty, -MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
    } else if (motor_l.motor_mode == MODE_BANGBANG) {
        // ���ģʽ�����ͼ��ͼ�
        motor_pid_l.out_i = 0;  // ���Ĭ��ģʽ�Ļ�����

        motor_l.duty += bangbang_pid_solve(&motor_l.brake_pid, (float) (motor_l.target_speed - motor_l.encoder_speed));
        motor_l.duty = MINMAX(motor_l.duty, -MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
    } else if (motor_l.motor_mode == MODE_SOFT) {
        // ������ģʽ
        motor_pid_l.out_i = 0;  // ���Ĭ��ģʽ�Ļ�����

        motor_l.duty += changable_pid_solve(&motor_l.pid, (float) (motor_l.target_speed - motor_l.encoder_speed));
        motor_l.duty = MINMAX(motor_l.duty, -MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
    } else if (motor_l.motor_mode == MODE_POSLOOP) {
        //Apriltagͣ��λ�û�
        motor_pid_l.out_i = 0; // ���Ĭ��ģʽ�Ļ�����

        motor_l.duty = pid_solve(&posloop_pid, (float) (motor_l.target_encoder - motor_l.total_encoder));
        motor_l.duty = MINMAX(motor_l.duty, -MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
    }

    // �ҵ����ͬ����
    if (motor_r.motor_mode == MODE_NORMAL) {
        float error = (motor_r.target_speed - motor_r.encoder_speed);
        motor_r.duty = pid_solve(&motor_pid_r, error);
        motor_r.duty = MINMAX(motor_r.duty, -MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
    } else if (motor_r.motor_mode == MODE_BANGBANG) {
        motor_pid_r.out_i = 0;

        motor_r.duty += bangbang_pid_solve(&motor_r.brake_pid, (float) (motor_r.target_speed - motor_r.encoder_speed));
        motor_r.duty = MINMAX(motor_r.duty, -MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
    } else if (motor_r.motor_mode == MODE_SOFT) {
        motor_pid_r.out_i = 0;

        motor_r.duty += changable_pid_solve(&motor_r.pid, (float) (motor_r.target_speed - motor_r.encoder_speed));
        motor_r.duty = MINMAX(motor_r.duty, -MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
    } else if (motor_r.motor_mode == MODE_POSLOOP) {
        motor_pid_r.out_i = 0;

        motor_r.duty = pid_solve(&posloop_pid, (float) (motor_r.target_encoder - motor_r.total_encoder));
        motor_r.duty = MINMAX(motor_r.duty, -MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX);
    }

    //��Ƕ��޷�
    if (fabs(angle001) > 10) {
        //����
        if (target_speed - (motor_r.encoder_speed + motor_l.encoder_speed) / 2 < 0) {
            motor_l.duty = MINMAX(motor_l.duty, -MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX * 8 / 10);
            motor_r.duty = MINMAX(motor_r.duty, -MOTOR_PWM_DUTY_MAX, MOTOR_PWM_DUTY_MAX * 8 / 10);
        } else {
            motor_l.duty = MINMAX(motor_l.duty, -MOTOR_PWM_DUTY_MAX * 8 / 10, MOTOR_PWM_DUTY_MAX * 8 / 10);
            motor_r.duty = MINMAX(motor_r.duty, -MOTOR_PWM_DUTY_MAX * 8 / 10, MOTOR_PWM_DUTY_MAX * 8 / 10);
        }
    }


    /* //PWM����
    pwm_duty(MOTOR1_PWM1, (motor_l.duty >= 0) ? motor_l.duty : 0);
    pwm_duty(MOTOR1_PWM2, (motor_l.duty >= 0) ? 0 : (-motor_l.duty));

    pwm_duty(MOTOR2_PWM1, (motor_r.duty >= 0) ? motor_r.duty : 0);
    pwm_duty(MOTOR2_PWM2, (motor_r.duty >= 0) ? 0 : (-motor_r.duty));
		*/
		//PWM����

    if (motor_l.duty > 0)
    {
        gpio_set_level(MOTOR1_DIR, GPIO_HIGH);
        pwm_set_duty(MOTOR1_PWM, motor_l.duty * (PWM_DUTY_MAX / 100));
    }
    else
    {
        gpio_set_level(MOTOR1_DIR, GPIO_LOW);
        pwm_set_duty(MOTOR1_PWM, -motor_l.duty * (PWM_DUTY_MAX / 100));
    }
    if (motor_r.duty > 0)
    {
        gpio_set_level(MOTOR2_DIR, GPIO_HIGH);
        pwm_set_duty(MOTOR2_PWM, motor_r.duty * (PWM_DUTY_MAX / 100));
    }
    else
    {
        gpio_set_level(MOTOR2_DIR, GPIO_LOW);
        pwm_set_duty(MOTOR2_PWM, -motor_r.duty * (PWM_DUTY_MAX / 100));
    }
   
}


int64_t get_total_encoder() {
    return (int64_t) ((motor_l.total_encoder + motor_r.total_encoder) / 2);
}