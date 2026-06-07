#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "iarduino_I2C_Motor.h"
#include "manipulator_servo.h"
#include "mecanum.h"
#include "soc/gpio_num.h"
#include "speed_profile.h"
#include "stdio.h"

i2c_master_bus_handle_t bus_handle;
MecanumPlatform *platform;
SpeedProfileGenerator *profileGenerator;

extern "C" void app_main(void) {
  printf("Init I2C...\n");
  i2c_bus_init(&bus_handle);

  printf("Init Mecanum Platform...\n");
  platform = new MecanumPlatform(bus_handle);
  platform->init();

  printf("Init Speed Profile Generator...\n");
  profileGenerator = new SpeedProfileGenerator(platform);
  profileGenerator->init();

  printf("Platform ready\n");

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(2000));
    printf("Move forward 50cm with trapezoidal profile\n");
    profileGenerator->executeLinearMotion(MotionDirection::FORWARD, 0.5f, 30.0f,
                                          0.075f, 0.075f);

    vTaskDelay(pdMS_TO_TICKS(6000)); // Ждем завершения (зависит от скорости)

    printf("Move backward 50cm with trapezoidal profile\n");
    profileGenerator->executeLinearMotion(MotionDirection::BACKWARD, 0.5f,
                                          30.0f, 0.075f, 0.075f);

    vTaskDelay(pdMS_TO_TICKS(6000));
  }

  /* ManipulatorServos manipulator;
  manipulator.init(); */
}
