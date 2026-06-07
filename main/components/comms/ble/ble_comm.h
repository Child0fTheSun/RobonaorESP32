#pragma once

#include <cstdint>
#include <cstddef>

// =============================================================================
// BLEComm — BLE UART-like communication component (ESP-NimBLE)
//
// Схема работы:
//   1. ПК подключается к роботу по BLE.
//   2. ESP32 отправляет приветственное сообщение "CN".
//   3. ПК отправляет команды в RX-характеристику (Write).
//      Приём команды вызывает зарегистрированный обратный вызов onCommand.
//   4. ESP32 может отправлять данные ПК через TX-характеристику (Notify).
//
// Используется протокол Nordic UART Service (NUS):
//   Service UUID: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
//   RX char UUID: 6E400002-...  (Write: ПК → ESP32)
//   TX char UUID: 6E400003-...  (Notify: ESP32 → ПК)
// =============================================================================

// Максимальный размер одного BLE-пакета (байт)
#define BLE_COMM_MTU 240

// Тип обратного вызова для приёма команды от ПК (заглушка)
// data — указатель на принятые данные, len — длина в байтах
typedef void (*ble_on_command_cb_t)(const uint8_t* data, size_t len, void* arg);

class BLEComm {
public:
    // -------------------------------------------------------------------------
    // Singleton: единственный экземпляр на всё приложение
    // -------------------------------------------------------------------------
    static BLEComm& instance();

    // -------------------------------------------------------------------------
    // Инициализация: запускает NimBLE-стек и начинает рекламу.
    // Вызывать из app_main ПОСЛЕ nvs_flash_init().
    //   device_name — имя устройства в BLE-рекламе (≤ 20 символов)
    // -------------------------------------------------------------------------
    void init(const char* device_name = "Robonaor");

    // Устройство подключено?
    bool isConnected() const { return m_connected; }

    // -------------------------------------------------------------------------
    // Отправить строку / буфер клиенту (BLE Notify).
    // Возвращает false, если клиент не подключён или MTU превышен.
    // -------------------------------------------------------------------------
    bool send(const char* str);
    bool send(const uint8_t* data, size_t len);

    // -------------------------------------------------------------------------
    // Хелперы для команд обратной связи
    // -------------------------------------------------------------------------
    void sendCompletedCommand(int n);
    void sendScenarioComplete();

    // -------------------------------------------------------------------------
    // Зарегистрировать обратный вызов для приёма команд (заглушка).
    //   cb  — функция, вызываемая при каждой записи в RX-характеристику
    //   arg — пользовательский указатель, передаётся в cb
    // -------------------------------------------------------------------------
    void setOnCommand(ble_on_command_cb_t cb, void* arg = nullptr);

    // -------------------------------------------------------------------------
    // NimBLE callback — вызывается из C-кода BLE-стека (не для прямого вызова)
    // -------------------------------------------------------------------------
    static int  gapEventHandler(struct ble_gap_event* event, void* arg);
    static int  gattAccessHandler(uint16_t conn_handle, uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt* ctxt, void* arg);
    static void nimbleHostTask(void* arg);
    static void onNimbleSync();
    static void onNimbleReset(int reason);

private:
    BLEComm() = default;
    BLEComm(const BLEComm&)            = delete;
    BLEComm& operator=(const BLEComm&) = delete;

    void startAdvertising();
    void onConnect(uint16_t conn_handle);
    void onDisconnect();

    bool           m_initialized    = false;
    volatile bool  m_connected      = false;
    uint16_t       m_conn_handle    = 0;
    const char*    m_device_name    = "Robonaor";

    ble_on_command_cb_t m_on_command_cb  = nullptr;
    void*               m_on_command_arg = nullptr;
};
