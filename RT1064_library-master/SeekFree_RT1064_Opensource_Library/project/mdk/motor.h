#ifndef _motor_h
#define _motor_h

#include "headfile.h" 


extern int32 duty1,duty2,duty3,duty4;//电机PWM值

void motor_init(void);


void car_ahead();
void car_back();
void car_lsideWay();
void car_rsideWay();
void car_diagonal();
void car_turnround();
void car_anticlockwise();
void car_concerning();
void car_stop();
void motor_control(bool run);

#include "headfile.h"
#include "pid.h"


#define BYTE0(dwTemp) (*(char *)(&dwTemp))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))

typedef struct motor_param_t {
    float total_encoder;
    float target_encoder;
    float encoder_raw;
    float encoder_speed; //Measured speed
    float target_speed;
    int32_t duty;         //Motor PWM duty

    enum {
        MODE_NORMAL, MODE_BANGBANG, MODE_SOFT, MODE_POSLOOP,
    } motor_mode;

    pid_param_t pid;      //Motor PID param
    pid_param_t brake_pid;      //Motor PID param
} motor_param_t;


#define MOTOR_CREATE(ts, kp, ki, kd, brake_kp, brake_ki, brake_kd, low_pass, p_max, i_max, d_max)       \
    {                                           \
        .total_encoder = 0,                     \
        .encoder_speed = 0,                     \
        .target_speed = ts,                     \
        .motor_mode = MODE_NORMAL,              \
        .pid = PID_CREATE(kp, ki, kd, low_pass, p_max ,i_max ,d_max), \
        .brake_pid = PID_CREATE(brake_kp, brake_ki, brake_kd, low_pass, p_max ,i_max ,d_max), \
    }

extern motor_param_t motor_l, motor_r;


#define ENCODER_PER_METER   (5800)

void wireless_show(void);

void motor_control001(void);



int64_t get_total_encoder();


//舵机控制PID
static pid_param_t servo_pid = PID_CREATE(1.5, 0, 1.0, 0.8, 15, 5, 15);		
		
#endif


#define ENCODER_PER_METER   (5800)

