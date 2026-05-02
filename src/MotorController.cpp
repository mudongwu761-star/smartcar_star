/*
 * @Author: ilikara 3435193369@qq.com
 * @Date: 2024-10-10 14:36:42
 * @LastEditors: ilikara 3435193369@qq.com
 * @LastEditTime: 2025-03-21 10:41:17
 * @FilePath: /smartcar/src/MotorController.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "MotorController.h"
#include <global.h>

MotorController::MotorController(int pwmchip, int pwmnum, int gpioNum, unsigned int period_ns,unsigned int duty_cycle_ns,
                                 double kp, double ki, double kd, double targetSpeed,
                                 int encoder_pwmchip, int encoder_gpioNum, int encoder_dir_)
    :  directionGPIO(gpioNum),
    pwmController(pwmchip, pwmnum, period_ns,duty_cycle_ns),
    pidController(kp, ki, kd, targetSpeed),
    encoder(std::make_unique<ENCODER>(encoder_pwmchip, encoder_gpioNum)),
    encoder_dir(encoder_dir_),
    last_speed(0.0),
    last_update(std::chrono::steady_clock::now())
{
    pwmController.setPeriod(period_ns); // 设置 PWM 周期
    directionGPIO.setDirection("out");
    pwmController.enable(); // 启用 PWM
    // encoder = new ENCODER(encoder_pwmchip, encoder_gpioNum);
    last_update = std::chrono::steady_clock::now();
}

MotorController::~MotorController(void)
{
    pwmController.disable();
}

double MotorController::getCurrentSpeed() 
{
    auto now = std::chrono::steady_clock::now();
    last_update = now;
    
    // 从编码器获取脉冲计数并计算速度
    double pulse_count = encoder->pulse_counter_update() * encoder_dir;
    
    // 转换为实际速度（示例公式，需根据实际参数调整）
    // 假设: 每转脉冲数=1024，轮周长=0.2m
    const double pulse_per_rev = 1024.0;
    const double wheel_circumference = 0.2; // 米
    
    double speed_rps = pulse_count / pulse_per_rev;
    double speed_mps = speed_rps * wheel_circumference;
    
    // 低通滤波平滑速度值
    last_speed = 0.8 * last_speed + 0.2 * speed_mps;
    
    return last_speed;
}

double MotorController::encoderSpeed() 
{
    // 从编码器获取脉冲计数并计算速度
    double pulse_count = encoder->pulse_counter_update() * encoder_dir;
    return pulse_count;
}

void MotorController::updateduty(double dutyCycle)
{
    int newduty = pwmController.readPeriod() * abs(dutyCycle) / 100.0;
    if (newduty != pwmController.readDutyCycle())
    {
        pwmController.setDutyCycle(newduty);
    }

    // 根据 PID 输出设置 GPIO 的方向
    if (dutyCycle > 0)
    {
        directionGPIO.setValue(1); // 正向
    }
    else
    {
        directionGPIO.setValue(0); // 反向
    }
    //std::cout << encoder.pulse_counter_update() << std::endl;
}

void MotorController::updateSpeed(void)
{
     
    double encoderReading = encoder->pulse_counter_update() * encoder_dir;
    // std::cout << encoderReading << std::endl;
    double output = pidController.updateservo(encoderReading);
    // int dutyCycle = static_cast<int>(output);
    std::cout << "  " << output << std::endl;

    if(output >= 10)
    {
        output = 10;
    }

    // 设置 PWM 占空比
    // updateduty(target_speed);
    std::cout << encoderReading << "  " << output << std::endl;
}

void MotorController::updateTarget(int speed)
{
    pidController.setTarget(speed);
}
