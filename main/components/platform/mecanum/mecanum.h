#pragma once
#include "iarduino_I2C_Motor.h"

#define MOT1_ADDR 0x09 // Правый верхний
#define MOT2_ADDR 0x0A // Правый нижний
#define MOT3_ADDR 0x0B // Левый нижний
#define MOT4_ADDR 0x0C // Левый верхний

class MecanumPlatform {
public:
  MecanumPlatform(i2c_master_bus_handle_t bus_handle, uint8_t addr1 = MOT1_ADDR,
                  uint8_t addr2 = MOT2_ADDR, uint8_t addr3 = MOT3_ADDR,
                  uint8_t addr4 = MOT4_ADDR);

  void init();

  // Простые команды движения
  void moveForward(float speed = 50, float distance = 0);
  void moveBackward(float speed = 50, float distance = 0);
  void moveRight(float speed = 50, float distance = 0);
  void moveLeft(float speed = 50, float distance = 0);
  void rotateClockwise(float speed = 50);
  void rotateCounterClockwise(float speed = 50);
  void stop();

  // Работа с одометрией
  float getAverageDistance();
  void resetDistance();

  // Расширенное управление: задание скоростей платформы
  // Vx – продольная, Vy – поперечная, W – угловая (рад/с)
  // Скорости колёс вычисляются по кинематике mecanum
  void setVelocity(float Vx, float Vy, float W);

private:
  iarduino_I2C_Motor mot1, mot2, mot3, mot4;
  i2c_master_bus_handle_t bus;

  // Вспомогательная настройка одного мотора
  void motor_setup(iarduino_I2C_Motor &mot, bool isRightWheel);
  // Установка скоростей четырёх колёс (об/мин или PWM) напрямую
  void setWheelSpeeds(float s1, float s2, float s3, float s4,
                      float distance = 0);
};