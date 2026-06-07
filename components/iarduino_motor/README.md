# iarduino_I2C_Motor (ESP-IDF Port)

Это портированная для ESP-IDF версия библиотеки [iarduino_I2C_Motor](https://github.com/tremaru/iarduino_I2C_Motor), предназначенной для работы с I2C драйверами моторов от iarduino.ru.

Оригинальная документация и подробное описание функций доступно по ссылкам:
- [Описание функций библиотеки](https://wiki.iarduino.ru/page/motor-driver-i2c/)
- [Настройка драйвера для работы с мотором](https://wiki.iarduino.ru/page/motor-driver-controller-i2c/)

## Отличия от оригинальной (Arduino) версии

Данный порт был адаптирован для использования в среде **ESP-IDF (v5.x / v6.x)** со строгими правилами компилятора C++ (`-Werror`).

1. **Отказ от `Wire.h` в пользу нативного I2C драйвера ESP-IDF:**
   Вместо Arduino-класса `TwoWire`, библиотека теперь использует нативный `driver/i2c.h`. Вы должны самостоятельно инициализировать I2C шину перед тем, как вызывать метод `begin()` у мотора.

2. **Изменённая сигнатура `begin()`:**
   Теперь метод `begin()` принимает номер порта I2C (тип `i2c_port_t`), а не указатель на `TwoWire`. По умолчанию используется `I2C_NUM_0`.
   ```cpp
   // Было (Arduino):
   motor.begin(&Wire);
   
   // Стало (ESP-IDF):
   motor.begin(I2C_NUM_0);
   ```

3. **Слой совместимости:**
   Внутри библиотеки используется заголовочный файл `compat_arduino.h`, который прозрачно переводит Arduino-вызовы (такие как `delay()`, `millis()`, `delayMicroseconds()`) в соответствующие вызовы FreeRTOS (`vTaskDelay`) и ESP-IDF (`esp_timer_get_time()`, `ets_delay_us()`). 

4. **Зависимости CMake:**
   Компонент автоматически требует подключения стандартных ESP-IDF библиотек: `driver`, `esp_timer`, `esp_rom`. Никаких дополнительных настроек с вашей стороны не требуется, просто укажите `iarduino_motor` в `REQUIRES` вашего `main/CMakeLists.txt`.

## Пример использования

Ниже представлен базовый пример инициализации I2C шины и запуска мотора в ESP-IDF:

```cpp
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "iarduino_I2C_Motor.h"

#define I2C_MASTER_SCL_IO           GPIO_NUM_22
#define I2C_MASTER_SDA_IO           GPIO_NUM_21
#define I2C_MASTER_FREQ_HZ          100000
#define MOTOR_I2C_ADDRESS           0x0A // Замените на адрес вашего модуля

extern "C" void app_main(void) {
    // 1. Инициализация шины I2C
    i2c_port_t i2c_master_port = I2C_NUM_0;
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = 0;

    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);

    // 2. Инициализация библиотеки мотора
    iarduino_I2C_Motor motor(MOTOR_I2C_ADDRESS);

    if (!motor.begin(i2c_master_port)) {
        printf("Ошибка: Мотор не найден на шине I2C!\n");
    } else {
        printf("Мотор успешно инициализирован.\n");
    }

    // 3. Управление мотором (ШИМ 50%)
    motor.setSpeed(50, MOT_PWM);

    while (1) {
        printf("Текущая скорость (ШИМ): %.2f\n", motor.getSpeed(MOT_PWM));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

## Доступные методы
Большинство методов аналогичны Arduino версии (например, `setSpeed`, `setStop`, `getSpeed` и т.д.). Для полного списка загляните в `iarduino_I2C_Motor.h` или на [Wiki iarduino](https://wiki.iarduino.ru/page/motor-driver-i2c/).
