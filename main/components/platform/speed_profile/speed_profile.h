#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "mecanum.h"

enum class MotionDirection {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
};

struct ProfileCommand {
    MotionDirection dir;
    float distance;
    float max_pwm;
    float accel_dist;
    float decel_dist;
};

class SpeedProfileGenerator {
public:
    SpeedProfileGenerator(MecanumPlatform* platform);
    
    // Инициализация (создание очереди и задачи)
    void init();
    
    // Неблокирующий вызов. Добавляет команду в очередь.
    bool executeLinearMotion(MotionDirection dir, float distance_m, float max_pwm, float accel_dist_m, float decel_dist_m);
    
private:
    MecanumPlatform* platform;
    QueueHandle_t cmd_queue;
    TaskHandle_t task_handle;
    
    static void taskWrapper(void* arg);
    void taskLoop();
};
