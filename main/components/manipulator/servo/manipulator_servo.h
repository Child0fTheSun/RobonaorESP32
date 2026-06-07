#pragma once

#include "driver/gpio.h"
#include "iot_servo.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MANIPULATOR_SERVO_COUNT 6

struct ServoConfig {
    gpio_num_t pin;
    float min_angle; // Software limit for minimum angle
    float max_angle; // Software limit for maximum angle
};

class ManipulatorServos {
public:
    ManipulatorServos();
    ~ManipulatorServos();

    // Initialize all servos and start the control task
    void init();

    // Run the physical initialization sequence (min -> max -> min)
    void runInitSequence();
    
    // Set target angle for a specific servo with software limit check
    // Returns true if angle was within limits, false if it was clamped
    bool setTargetAngle(uint8_t index, float angle);
    
    // Set speed in degrees per second for a specific servo
    void setSpeed(uint8_t index, float deg_per_sec);

    // Get current actual angle
    float getCurrentAngle(uint8_t index) const;

    // Get the configured software limits
    void getLimits(uint8_t index, float& min_angle, float& max_angle) const;

    // Dynamically adjust software limits
    void setLimits(uint8_t index, float min_angle, float max_angle);

private:
    ServoConfig m_configs[MANIPULATOR_SERVO_COUNT];
    
    float m_current_angles[MANIPULATOR_SERVO_COUNT];
    float m_target_angles[MANIPULATOR_SERVO_COUNT];
    float m_speeds[MANIPULATOR_SERVO_COUNT]; // degrees per second
    
    TaskHandle_t m_control_task_handle;

    static void controlTask(void* arg);
    void updateServos(float dt);
};
