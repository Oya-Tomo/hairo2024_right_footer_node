#include <cmath>

#include "features/flipper/flipper.h"
#include "features/connection/connection.h"
#include "features/ticker/ticker.h"
#include "utils/utils.h"

#include "sclp/sclp.h"

MotorDriver3Pins front_flipper_motor(6, 7);
MotorDriver3Pins::config_t front_flipper_motor_config = {
    .pwm_clkdiv = pwm_clkdiv_calc(10 * 1000, 1000),
    .pwm_wrap = 1000,
    .reverse = false,
};

MotorDriver3Pins back_flipper_motor(8, 9);
MotorDriver3Pins::config_t back_flipper_motor_config = {
    .pwm_clkdiv = pwm_clkdiv_calc(10 * 1000, 1000),
    .pwm_wrap = 1000,
    .reverse = false,
};

QEI front_flipper_qei(18, 19);
QEI::config_t front_flipper_qei_config = {
    .ppr = 48,
    .reverse = false,
};

QEI back_flipper_qei(20, 21);
QEI::config_t back_flipper_qei_config = {
    .ppr = 48,
    .reverse = false,
};

PID front_flipper_pid(PID::sPID);
PID::config_t front_flipper_pid_config = {
    .Kp = 0.9,
    .Ki = 0.000,
    .Kd = 0.01,
    .Kf = 0,
    .guard = true,
    .min = -0.99,
    .max = 0.99,
};

PID back_flipper_pid(PID::sPID);
PID::config_t back_flipper_pid_config = {
    .Kp = 0.9,
    .Ki = 0.000,
    .Kd = 0.01,
    .Kf = 0,
    .guard = true,
    .min = -0.99,
    .max = 0.99,
};

void flipper_callback_register(uint gpio, uint32_t events)
{
    front_flipper_qei.configure(front_flipper_qei_config);
    back_flipper_qei.configure(back_flipper_qei_config);
}

void flipper_setup()
{
    front_flipper_motor.configure(front_flipper_motor_config);
    back_flipper_motor.configure(back_flipper_motor_config);

    front_flipper_qei.configure(front_flipper_qei_config);
    back_flipper_qei.configure(back_flipper_qei_config);

    front_flipper_pid.configure(front_flipper_pid_config);
    back_flipper_pid.configure(back_flipper_pid_config);
}

void flipper_task()
{
    flipper_state_t flipper_state = get_flipper_state();

    set_front_flipper_angle(flipper_state.front_angle);
    set_back_flipper_angle(flipper_state.back_angle);
}

void set_front_flipper_angle(double angle)
{
    static float front_flipper_duty_ratio = 0.0;
    // motor duty state

    double target_angle = angle;
    double current_angle = front_flipper_qei.get_radians();

    front_flipper_duty_ratio = front_flipper_pid.calculate(target_angle, current_angle, get_dt());
    front_flipper_motor.set_duty_ratio(front_flipper_duty_ratio);
}

void set_back_flipper_angle(double angle)
{
    static float back_flipper_duty_ratio = 0.0;
    // motor duty state

    double target_angle = angle;
    double current_angle = back_flipper_qei.get_radians();

    back_flipper_duty_ratio = back_flipper_pid.calculate(target_angle, current_angle, get_dt());
    back_flipper_motor.set_duty_ratio(back_flipper_duty_ratio);
}
