#include "mecanum.h"
#include "iarduino_I2C_Motor.h"
#include <cmath>
#include <cstdio>

MecanumPlatform::MecanumPlatform(i2c_master_bus_handle_t bus_handle,
                                 uint8_t addr1, uint8_t addr2, uint8_t addr3,
                                 uint8_t addr4)
    : mot1(addr1), mot2(addr2), mot3(addr3), mot4(addr4), bus(bus_handle) {}

void MecanumPlatform::init() {
  motor_setup(mot1, true);  // Правый верхний
  motor_setup(mot2, true);  // Правый нижний
  motor_setup(mot3, false); // Левый нижний
  motor_setup(mot4, false); // Левый верхний
  printf("Mecanum motors initialized\n");
}

void MecanumPlatform::motor_setup(iarduino_I2C_Motor &mot, bool isRightWheel) {
  mot.begin(bus);
  mot.setMagnet(22);
  mot.setVoltage(12);
  mot.setReducer(27.0);
  mot.setInvGear(false, false);
  mot.setStopNeutral(true);
  mot.setDirection(isRightWheel);
  mot.setError(10);

  mot.radius = 6;
}

// Вперёд
void MecanumPlatform::moveForward(float speed, float distance) {
  setWheelSpeeds(speed, speed, speed, speed, distance);
}

// Назад
void MecanumPlatform::moveBackward(float speed, float distance) {
  setWheelSpeeds(-speed, -speed, -speed, -speed, distance);
}

// Вправо
void MecanumPlatform::moveRight(float speed, float distance) {
  setWheelSpeeds(-speed, speed, -speed, speed, distance);
}

// Влево
void MecanumPlatform::moveLeft(float speed, float distance) {
  setWheelSpeeds(speed, -speed, speed, -speed, distance);
}

// Вращение по часовой
void MecanumPlatform::rotateClockwise(float speed) {
  setWheelSpeeds(-speed, -speed, speed, speed);
}

// Вращение против часовой
void MecanumPlatform::rotateCounterClockwise(float speed) {
  setWheelSpeeds(speed, speed, -speed, -speed);
}

// Стоп
void MecanumPlatform::stop() { setWheelSpeeds(0, 0, 0, 0); }

// Установка скоростей колёс
void MecanumPlatform::setWheelSpeeds(float s1, float s2, float s3, float s4,
                                     float distance) {
  if (distance > 0) {
    mot1.setSpeed(s1, MOT_PWM, distance, MOT_MET);
    mot2.setSpeed(s2, MOT_PWM, distance, MOT_MET);
    mot3.setSpeed(s3, MOT_PWM, distance, MOT_MET);
    mot4.setSpeed(s4, MOT_PWM, distance, MOT_MET);
  } else {
    mot1.setSpeed(s1, MOT_PWM);
    mot2.setSpeed(s2, MOT_PWM);
    mot3.setSpeed(s3, MOT_PWM);
    mot4.setSpeed(s4, MOT_PWM);
  }
}

// ----- Кинематика mecanum (заглушка) -----
void MecanumPlatform::setVelocity(float Vx, float Vy, float W) {
  // Пока используем простые коэффициенты, в будущем подставите
  // реальные формулы из кинематики вашей платформы
  int s1 = Vx + Vy + W;
  int s2 = Vx - Vy + W;
  int s3 = -(Vx - Vy - W);
  int s4 = -(Vx + Vy - W);
  setWheelSpeeds(s1, s2, s3, s4);
}

void MecanumPlatform::resetDistance() {
  mot1.delSum();
  mot2.delSum();
  mot3.delSum();
  mot4.delSum();
}
float MecanumPlatform::getAverageDistance() {
  float d1 = std::fabs(mot1.getSum(MOT_MET));
  float d2 = std::fabs(mot2.getSum(MOT_MET));
  float d3 = std::fabs(mot3.getSum(MOT_MET));
  float d4 = std::fabs(mot4.getSum(MOT_MET));
  return (d1 + d2 + d3 + d4) / 4.0f;
}