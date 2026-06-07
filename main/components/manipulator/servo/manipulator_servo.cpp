#include "manipulator_servo.h"
#include "esp_log.h"
#include <algorithm>
#include <cmath>

static const char *TAG = "ManipulatorServos";

#define CONTROL_TASK_INTERVAL_MS 20

ManipulatorServos::ManipulatorServos() : m_control_task_handle(nullptr) {
  // GPIOs bottom to top: 19, 18, 5, 17, 16, 4
  const gpio_num_t pins[MANIPULATOR_SERVO_COUNT] = {GPIO_NUM_19, GPIO_NUM_18,
                                                    GPIO_NUM_5,  GPIO_NUM_17,
                                                    GPIO_NUM_16, GPIO_NUM_4};

  const float default_limits[MANIPULATOR_SERVO_COUNT][2] = {
      {0.0f, 180.0f},  // Servo 0 (GPIO 19)
      {0.0f, 180.0f},  // Servo 1 (GPIO 18)
      {80.0f, 140.0f}, // Servo 2 (GPIO 5)
      {5.0f, 60.0f},   // Servo 3 (GPIO 17)
      {30.0f, 121.0f}, // Servo 4 (GPIO 16)
      {10.0f, 70.0f}   // Servo 5 (GPIO 4)
  };

  // Калибровочные углы: именно в эти позиции сервы перейдут при первом
  // включении (резкий рывок только 1 раз, зато сразу в нужную точку)
  const float init_angles[MANIPULATOR_SERVO_COUNT] = {80.0f, 0.0f,   90.0f,
                                                      5.0f,  121.0f, 10.0f};

  for (int i = 0; i < MANIPULATOR_SERVO_COUNT; ++i) {
    m_configs[i].pin = pins[i];
    m_configs[i].min_angle = default_limits[i][0];
    m_configs[i].max_angle = default_limits[i][1];

    // Говорим системе, что мы уже находимся в калибровочном положении.
    // Первый ШИМ-сигнал прыгнет сюда мгновенно — дальше только плавное
    // движение.
    m_current_angles[i] = init_angles[i];
    m_target_angles[i] = init_angles[i];
    m_speeds[i] = 22.5f; // Скорость уменьшена вдвое (22.5 град/сек)
  }
}

ManipulatorServos::~ManipulatorServos() {
  if (m_control_task_handle != nullptr) {
    vTaskDelete(m_control_task_handle);
  }
}

void ManipulatorServos::init() {
  ESP_LOGI(TAG, "Initializing Manipulator Servos...");

  servo_config_t servo_cfg = {
      .max_angle = 180,
      .min_width_us = 500,
      .max_width_us = 2500,
      .freq = 50,
      .timer_number = LEDC_TIMER_0,
      .channels = {.servo_pin = {m_configs[0].pin, m_configs[1].pin,
                                 m_configs[2].pin, m_configs[3].pin,
                                 m_configs[4].pin, m_configs[5].pin},
                   .ch = {LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2,
                          LEDC_CHANNEL_3, LEDC_CHANNEL_4, LEDC_CHANNEL_5}},
      .channel_number = MANIPULATOR_SERVO_COUNT};

  iot_servo_init(LEDC_LOW_SPEED_MODE, &servo_cfg);

  // Initial positioning
  for (int i = 0; i < MANIPULATOR_SERVO_COUNT; ++i) {
    iot_servo_write_angle(LEDC_LOW_SPEED_MODE, i, m_current_angles[i]);
  }

  // Start control task
  xTaskCreate(controlTask, "ServoControlTask", 4096, this, 5,
              &m_control_task_handle);
  ESP_LOGI(TAG, "Manipulator Servos Initialized");
}

void ManipulatorServos::runInitSequence() {
  ESP_LOGI(TAG,
           "Starting physical initialization sequence to calibration angles");

  const float init_angles[MANIPULATOR_SERVO_COUNT] = {100.0f, 180.0f, 90.0f,
                                                      5.0f,   121.0f, 70.0f};

  for (int i = 0; i < MANIPULATOR_SERVO_COUNT; ++i) {
    setTargetAngle(i, init_angles[i]);
  }

  vTaskDelay(pdMS_TO_TICKS(10000));

  ESP_LOGI(TAG, "Initialization sequence completed");
}

bool ManipulatorServos::setTargetAngle(uint8_t index, float angle) {
  if (index >= MANIPULATOR_SERVO_COUNT)
    return false;

  bool clamped = false;
  if (angle < m_configs[index].min_angle) {
    angle = m_configs[index].min_angle;
    clamped = true;
  } else if (angle > m_configs[index].max_angle) {
    angle = m_configs[index].max_angle;
    clamped = true;
  }

  m_target_angles[index] = angle;
  return !clamped;
}

void ManipulatorServos::setSpeed(uint8_t index, float deg_per_sec) {
  if (index >= MANIPULATOR_SERVO_COUNT)
    return;
  if (deg_per_sec < 0.1f)
    deg_per_sec = 0.1f; // minimum speed
  m_speeds[index] = deg_per_sec;
}

float ManipulatorServos::getCurrentAngle(uint8_t index) const {
  if (index >= MANIPULATOR_SERVO_COUNT)
    return 0.0f;
  return m_current_angles[index];
}

void ManipulatorServos::getLimits(uint8_t index, float &min_angle,
                                  float &max_angle) const {
  if (index >= MANIPULATOR_SERVO_COUNT)
    return;
  min_angle = m_configs[index].min_angle;
  max_angle = m_configs[index].max_angle;
}

void ManipulatorServos::setLimits(uint8_t index, float min_angle,
                                  float max_angle) {
  if (index >= MANIPULATOR_SERVO_COUNT)
    return;
  if (min_angle < 0.0f)
    min_angle = 0.0f;
  if (max_angle > 180.0f)
    max_angle = 180.0f;
  if (min_angle > max_angle)
    std::swap(min_angle, max_angle);

  m_configs[index].min_angle = min_angle;
  m_configs[index].max_angle = max_angle;

  // Apply limits immediately to target angle
  setTargetAngle(index, m_target_angles[index]);
}

void ManipulatorServos::controlTask(void *arg) {
  ManipulatorServos *self = static_cast<ManipulatorServos *>(arg);
  const float dt = CONTROL_TASK_INTERVAL_MS / 1000.0f;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (true) {
    self->updateServos(dt);
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CONTROL_TASK_INTERVAL_MS));
  }
}

void ManipulatorServos::updateServos(float dt) {
  for (int i = 0; i < MANIPULATOR_SERVO_COUNT; ++i) {
    if (m_current_angles[i] != m_target_angles[i]) {
      float diff = m_target_angles[i] - m_current_angles[i];
      float step = m_speeds[i] * dt;

      if (std::abs(diff) <= step) {
        m_current_angles[i] = m_target_angles[i];
      } else {
        if (diff > 0) {
          m_current_angles[i] += step;
        } else {
          m_current_angles[i] -= step;
        }
      }

      iot_servo_write_angle(LEDC_LOW_SPEED_MODE, i, m_current_angles[i]);
    }
  }
}
