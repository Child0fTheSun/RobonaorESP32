// Реализация I2C-транспорта для библиотеки iarduino_I2C_Motor на ESP-IDF v5+
// Этот файл заменяет оригинальный абстрактный слой поверх Arduino Wire.h

#pragma once

#include "driver/i2c_master.h"
#include <cstdint>
#include <cstdlib>
#include <cstring>

// Класс-обёртка для работы с шиной I2C в ESP-IDF
class iarduino_I2C_Select {
public:
  iarduino_I2C_Select() : dev_handle_(nullptr) {}

  ~iarduino_I2C_Select() {
    if (dev_handle_) {
      i2c_master_bus_rm_device(dev_handle_);
    }
  }

  // Инициализация устройства на шине
  bool init(i2c_master_bus_handle_t bus, uint8_t adr) {
    if (dev_handle_) {
      i2c_master_bus_rm_device(dev_handle_);
      dev_handle_ = nullptr;
    }

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = adr;
    dev_cfg.scl_speed_hz = 100000;

    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &dev_handle_);

    return err == ESP_OK;
  }

  void begin() {
    // Ничего не делаем, инициализация происходит через init(bus, adr)
  }

  // Статическая проверка наличия устройства на шине по адресу
  static bool checkAddress(i2c_master_bus_handle_t bus, uint8_t adr) {
    if (!bus)
      return false;

    esp_err_t ret = i2c_master_probe(bus, adr, 100);

    return (ret == ESP_OK);
  }

  // Пакетное чтение данных из регистров модуля (запись адреса регистра, потом
  // чтение)
  bool readBytes(uint8_t reg, uint8_t *data, uint8_t sum) {
    if (!dev_handle_)
      return false;

    esp_err_t ret =
        i2c_master_transmit_receive(dev_handle_, &reg, 1, data, sum, 100);

    return (ret == ESP_OK);
  }

  // Пакетное чтение данных напрямую
  bool readBytes(uint8_t *data, uint8_t sum) {
    if (!dev_handle_)
      return false;

    esp_err_t ret = i2c_master_receive(dev_handle_, data, sum, 100);

    return (ret == ESP_OK);
  }

  // Чтение одного байта данных из регистра
  uint8_t readByte(uint8_t reg) {
    uint8_t data = 0;
    readBytes(reg, &data, 1);

    return data;
  }

  // Пакетная запись нескольких байт данных в регистр
  bool writeBytes(uint8_t reg, uint8_t *data, uint8_t sum) {
    if (!dev_handle_)
      return false;

    uint8_t buffer[16]; // Максимальная длина передачи в библиотеке <= 6 байт,
                        // стек безопасен

    if (sum + 1 > sizeof(buffer))
      return false;

    buffer[0] = reg;
    memcpy(&buffer[1], data, sum);
    esp_err_t ret = i2c_master_transmit(dev_handle_, buffer, sum + 1, 100);

    return (ret == ESP_OK);
  }

  // Пакетная запись данных напрямую
  bool writeBytes(uint8_t *data, uint8_t sum) {
    if (!dev_handle_)
      return false;

    esp_err_t ret = i2c_master_transmit(dev_handle_, data, sum, 100);

    return (ret == ESP_OK);
  }

  // Запись одного байта в регистр
  bool writeByte(uint8_t reg, uint8_t data) {
    return writeBytes(reg, &data, 1);
  }

private:
  i2c_master_dev_handle_t dev_handle_;
};
