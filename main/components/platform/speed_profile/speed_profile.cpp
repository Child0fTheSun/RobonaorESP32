#include "speed_profile.h"
#include "esp_log.h"
#include <cmath>
#include <cstdio>

static const char *TAG = "SpeedProfile";

SpeedProfileGenerator::SpeedProfileGenerator(MecanumPlatform *plat)
    : platform(plat), cmd_queue(nullptr), task_handle(nullptr) {}

void SpeedProfileGenerator::init() {
  // Очередь на 5 команд
  cmd_queue = xQueueCreate(5, sizeof(ProfileCommand));
  if (cmd_queue == nullptr) {
    ESP_LOGE(TAG, "Failed to create command queue");
    return;
  }

  // Создаем задачу с приоритетом 5 (можно поменять в зависимости от системы)
  xTaskCreate(taskWrapper, "SpeedProfileTask", 4096, this, 5, &task_handle);
}

bool SpeedProfileGenerator::executeLinearMotion(MotionDirection dir,
                                                float distance_m, float max_pwm,
                                                float accel_dist_m,
                                                float decel_dist_m) {
  if (cmd_queue == nullptr)
    return false;

  ProfileCommand cmd = {dir, distance_m, max_pwm, accel_dist_m, decel_dist_m};
  return xQueueSend(cmd_queue, &cmd, 0) == pdTRUE;
}

void SpeedProfileGenerator::taskWrapper(void *arg) {
  SpeedProfileGenerator *generator = static_cast<SpeedProfileGenerator *>(arg);
  generator->taskLoop();
}

void SpeedProfileGenerator::taskLoop() {
  ProfileCommand cmd;

  while (true) {
    // Ожидаем команду (блокируем задачу до получения)
    if (xQueueReceive(cmd_queue, &cmd, portMAX_DELAY) == pdTRUE) {
      ESP_LOGI(TAG, "Starting profile: dist=%.2f, max_pwm=%.2f", cmd.distance,
               cmd.max_pwm);

      // Сбрасываем счетчики одометрии
      platform->resetDistance();

      // Небольшая задержка, чтобы моторы успели сбросить счетчики
      vTaskDelay(pdMS_TO_TICKS(10));

      // Фактические расстояния разгона/торможения (для поддержки треугольного профиля)
      float actual_accel_dist = cmd.accel_dist;
      float actual_decel_dist = cmd.decel_dist;

      if (actual_accel_dist + actual_decel_dist > cmd.distance) {
        // Дистанция слишком мала для трапеции, делаем треугольный профиль
        float ratio = cmd.distance / (actual_accel_dist + actual_decel_dist);
        actual_accel_dist *= ratio;
        actual_decel_dist *= ratio;
      }

      while (true) {
        // Калибровочный коэффициент: платформа проезжала в 3 раза больше
        float current_dist = platform->getAverageDistance() * 3.0f;

        // Проверка достижения цели
        if (current_dist >= cmd.distance) {
          platform->stop();
          ESP_LOGI(TAG, "Target reached (%.2f / %.2f)", current_dist,
                   cmd.distance);
          break;
        }

        float remaining_dist = cmd.distance - current_dist;
        float current_pwm = 0.0f;

        // Трапецеидальный профиль
        float min_pwm =
            5.0f; // Минимальный ШИМ, при котором платформа трогается

        if (current_dist < actual_accel_dist) {
          // Разгон (сохраняем оригинальный наклон / cmd.accel_dist для правильного ШИМа в пике треугольника)
          current_pwm = min_pwm + (cmd.max_pwm - min_pwm) *
                                      (current_dist / cmd.accel_dist);
        } else if (remaining_dist < actual_decel_dist) {
          // Торможение
          current_pwm = min_pwm + (cmd.max_pwm - min_pwm) *
                                      (remaining_dist / cmd.decel_dist);
        } else {
          // Равномерное движение
          current_pwm = cmd.max_pwm;
        }

        // Ограничение ШИМ
        if (current_pwm > cmd.max_pwm)
          current_pwm = cmd.max_pwm;
        if (current_pwm < 0.0f)
          current_pwm = 0.0f;

        // Отправка команды движения на платформу
        switch (cmd.dir) {
        case MotionDirection::FORWARD:
          platform->moveForward(
              current_pwm,
              0); // distance = 0, так как мы сами контролируем остановку
          break;
        case MotionDirection::BACKWARD:
          platform->moveBackward(current_pwm, 0);
          break;
        case MotionDirection::LEFT:
          platform->moveLeft(current_pwm, 0);
          break;
        case MotionDirection::RIGHT:
          platform->moveRight(current_pwm, 0);
          break;
        }

        // Частота обновления ~50 Гц (каждые 20 мс)
        vTaskDelay(pdMS_TO_TICKS(20));
      }
    }
  }
}
