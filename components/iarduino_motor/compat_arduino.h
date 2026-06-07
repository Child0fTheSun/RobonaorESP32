//  Слой совместимости с Arduino API для ESP-IDF.
//  Предоставляет функции delay(), delayMicroseconds(), millis() и макросы PI,
//  bit(). Используется при портировании библиотек iarduino на ESP-IDF.

#pragma once

#include <cmath> // M_PI
#include <cstdint>
#include <cstdlib> // abs()

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h" // ets_delay_us()

// --- Математические константы ---
#ifndef PI
#define PI M_PI
#endif

// --- Битовые макросы ---
#ifndef bit
#define bit(b) (1UL << (b))
#endif

// --- Функции задержки ---

/// @brief Блокирующая задержка в миллисекундах (через FreeRTOS vTaskDelay).
inline void delay(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

/// @brief Блокирующая задержка в микросекундах (busy-wait).
inline void delayMicroseconds(uint32_t us) { ets_delay_us(us); }

// --- Функции времени ---

/// @brief Возвращает количество миллисекунд с момента запуска.
inline uint32_t millis() { return (uint32_t)(esp_timer_get_time() / 1000ULL); }
