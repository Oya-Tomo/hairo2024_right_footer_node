#include <cmath>

#include "features/drive/drive.h"
#include "features/connection/connection.h"
#include "features/ticker/ticker.h"
#include "utils/utils.h"

#include "sclp/sclp.h"

constexpr double MAX_RPS = 1 * 71 * (2 * M_PI); // tire * gear * 2PI

MotorDriver3Pins center_belt_motor(0, 1);
MotorDriver3Pins::config_t center_belt_motor_config = {
    .pwm_clkdiv = pwm_clkdiv_calc(10 * 1000, 1000),
    .pwm_wrap = 1000,
    .reverse = false,
};

MotorDriver3Pins front_belt_motor(2, 3);
MotorDriver3Pins::config_t front_belt_motor_config = {
    .pwm_clkdiv = pwm_clkdiv_calc(10 * 1000, 1000),
    .pwm_wrap = 1000,
    .reverse = false,
};

MotorDriver3Pins back_belt_motor(4, 5);
MotorDriver3Pins::config_t back_belt_motor_config = {
    .pwm_clkdiv = pwm_clkdiv_calc(10 * 1000, 1000),
    .pwm_wrap = 1000,
    .reverse = false,
};

QEI center_belt_qei(10, 11);
QEI::config_t center_belt_qei_config = {
    .ppr = 48,
    .reverse = false,
};

QEI front_belt_qei(12, 13);
QEI::config_t front_belt_qei_config = {
    .ppr = 48,
    .reverse = false,
};

QEI back_belt_qei(14, 15);
QEI::config_t back_belt_qei_config = {
    .ppr = 48,
    .reverse = false,
};

PID center_belt_pid(PID::sPID);
PID::config_t center_belt_pid_config = {
    .Kp = 0.001,
    .Ki = 0.000,
    .Kd = 0.001,
    .Kf = 0,
    .guard = true,
    .min = -0.1,
    .max = 0.1,
};

PID front_belt_pid(PID::sPID);
PID::config_t front_belt_pid_config = {
    .Kp = 0.001,
    .Ki = 0.000,
    .Kd = 0.001,
    .Kf = 0,
    .guard = true,
    .min = -0.1,
    .max = 0.1,
};

PID back_belt_pid(PID::sPID);
PID::config_t back_belt_pid_config = {
    .Kp = 0.001,
    .Ki = 0.000,
    .Kd = 0.001,
    .Kf = 0,
    .guard = true,
    .min = -0.1,
    .max = 0.1,
};

void drive_callback_register(uint gpio, uint32_t events)
{
    center_belt_qei.callback_register(gpio, events);
    front_belt_qei.callback_register(gpio, events);
    back_belt_qei.callback_register(gpio, events);
}

void drive_setup()
{
    center_belt_motor.configure(center_belt_motor_config);
    front_belt_motor.configure(front_belt_motor_config);
    back_belt_motor.configure(back_belt_motor_config);

    center_belt_qei.configure(center_belt_qei_config);
    front_belt_qei.configure(front_belt_qei_config);
    back_belt_qei.configure(back_belt_qei_config);

    center_belt_pid.configure(center_belt_pid_config);
    front_belt_pid.configure(front_belt_pid_config);
    back_belt_pid.configure(back_belt_pid_config);
}

void drive_stop()
{
    set_center_belt_speed(0.0);
    set_front_belt_speed(0.0);
    set_back_belt_speed(0.0);
}

void drive_task()
{
    drive_state_t drive_state = get_drive_state();

    set_center_belt_speed(drive_state.speed);
    set_front_belt_speed(drive_state.speed);
    set_back_belt_speed(drive_state.speed);
}

void set_center_belt_speed(double speed)
{
    static float center_belt_duty_ratio = 0.0;
    /// motor duty state

    double target_rps = MAX_RPS * speed;
    double current_rps = center_belt_qei.get_radians() / get_dt();

    center_belt_duty_ratio = guard(
        center_belt_duty_ratio +
            center_belt_pid.calculate(target_rps, current_rps, get_dt()),
        1.0, -1.0);
    center_belt_motor.set_duty_ratio(center_belt_duty_ratio);
}

void set_front_belt_speed(double speed)
{
    static float front_belt_duty_ratio = 0.0;
    /// motor duty state

    double target_rps = MAX_RPS * speed;
    double current_rps = front_belt_qei.get_radians() / get_dt();

    front_belt_duty_ratio = guard(
        front_belt_duty_ratio +
            front_belt_pid.calculate(target_rps, current_rps, get_dt()),
        1.0, -1.0);
    front_belt_motor.set_duty_ratio(front_belt_duty_ratio);
}

void set_back_belt_speed(double speed)
{
    static float back_belt_duty_ratio = 0.0;
    /// motor duty state

    double target_rps = MAX_RPS * speed;
    double current_rps = back_belt_qei.get_radians() / get_dt();

    back_belt_duty_ratio = guard(
        back_belt_duty_ratio +
            back_belt_pid.calculate(target_rps, current_rps, get_dt()),
        1.0, -1.0);
    back_belt_motor.set_duty_ratio(back_belt_duty_ratio);
}
