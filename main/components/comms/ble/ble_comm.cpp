#include "ble_comm.h"

#include "esp_log.h"
#include "nvs_flash.h"
#include <cstring>

#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "BLEComm";

// =============================================================================
// NUS (Nordic UART Service) UUIDs — little-endian byte order
//   Service:  6E400001-B5A3-F393-E0A9-E50E24DCCA9E
//   RX char:  6E400002-...  (Write, ПК → ESP32)
//   TX char:  6E400003-...  (Notify, ESP32 → ПК)
// =============================================================================
static const ble_uuid128_t NUS_SVC_UUID =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3,
                     0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E);
static const ble_uuid128_t NUS_RX_UUID =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3,
                     0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E);
static const ble_uuid128_t NUS_TX_UUID =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3,
                     0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E);

// Хэндл TX-характеристики (нужен для ble_gatts_notify_custom)
// Устанавливается NimBLE при регистрации GATT-сервиса
static uint16_t s_tx_val_handle = 0;

// =============================================================================
// =============================================================================
// GATT-сервис (статическая таблица, требуемая NimBLE C-API)
// =============================================================================
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"

static const struct ble_gatt_chr_def s_nus_chars[] = {
    {
        .uuid = &NUS_RX_UUID.u,
        .access_cb = BLEComm::gattAccessHandler,
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
    },
    {
        .uuid = &NUS_TX_UUID.u,
        .access_cb = BLEComm::gattAccessHandler,
        .flags = BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &s_tx_val_handle,
    },
    {0} // конец характеристик
};

static const struct ble_gatt_svc_def s_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &NUS_SVC_UUID.u,
        .characteristics = s_nus_chars,
    },
    {0} // конец сервисов
};

#pragma GCC diagnostic pop

// =============================================================================
// Singleton
// =============================================================================
BLEComm &BLEComm::instance() {
  static BLEComm inst;
  return inst;
}

// =============================================================================
// init
// =============================================================================
void BLEComm::init(const char *device_name) {
  if (m_initialized)
    return;
  m_device_name = device_name;

  ESP_LOGI(TAG, "Initializing NimBLE stack...");

  // NimBLE требует инициализированного NVS
  esp_err_t nvs_err = nvs_flash_init();
  if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES ||
      nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
  }

  // Инициализация NimBLE-порта
  nimble_port_init();

  // Регистрируем обратные вызовы хоста
  ble_hs_cfg.sync_cb = BLEComm::onNimbleSync;
  ble_hs_cfg.reset_cb = BLEComm::onNimbleReset;

  // Регистрируем GATT-сервис
  int rc = ble_gatts_count_cfg(s_gatt_svcs);
  assert(rc == 0);
  rc = ble_gatts_add_svcs(s_gatt_svcs);
  assert(rc == 0);

  // Устанавливаем имя GAP-устройства
  rc = ble_svc_gap_device_name_set(m_device_name);
  assert(rc == 0);

  ble_svc_gap_init();
  ble_svc_gatt_init();

  // Запускаем задачу NimBLE-хоста
  nimble_port_freertos_init(BLEComm::nimbleHostTask);

  m_initialized = true;
  ESP_LOGI(TAG, "NimBLE initialized. Device name: \"%s\"", m_device_name);
}

// =============================================================================
// Реклама (Advertising)
// =============================================================================
void BLEComm::startAdvertising() {
  struct ble_gap_adv_params adv_params = {};
  struct ble_hs_adv_fields fields = {};

  // Флаги: только BLE, обнаруживаем всеми
  fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
  fields.tx_pwr_lvl_is_present = 1;
  fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

  // Имя устройства
  fields.name = (const uint8_t *)m_device_name;
  fields.name_len = (uint8_t)strlen(m_device_name);
  fields.name_is_complete = 1;

  int rc = ble_gap_adv_set_fields(&fields);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gap_adv_set_fields error: %d", rc);
    return;
  }

  // Неограниченная реклама (connectable, undirected)
  adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

  rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params,
                         BLEComm::gapEventHandler, nullptr);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gap_adv_start error: %d", rc);
    return;
  }

  ESP_LOGI(TAG, "Advertising started");
}

// =============================================================================
// send
// =============================================================================
bool BLEComm::send(const char *str) {
  return str ? send(reinterpret_cast<const uint8_t *>(str), strlen(str))
             : false;
}

bool BLEComm::send(const uint8_t *data, size_t len) {
  if (!m_connected || s_tx_val_handle == 0 || data == nullptr || len == 0) {
    return false;
  }

  struct os_mbuf *om = ble_hs_mbuf_from_flat(data, (uint16_t)len);
  if (!om) {
    ESP_LOGE(TAG, "Failed to allocate mbuf");
    return false;
  }

  int rc = ble_gatts_notify_custom(m_conn_handle, s_tx_val_handle, om);
  if (rc != 0) {
    ESP_LOGE(TAG, "Notify failed: %d", rc);
    return false;
  }
  return true;
}

void BLEComm::sendCompletedCommand(int n) {
  char buf[32];
  snprintf(buf, sizeof(buf), "CP %d", n);
  send(buf);
}

void BLEComm::sendScenarioComplete() {
  send("CPA");
}

// =============================================================================
// setOnCommand
// =============================================================================
void BLEComm::setOnCommand(ble_on_command_cb_t cb, void *arg) {
  m_on_command_cb = cb;
  m_on_command_arg = arg;
}

// =============================================================================
// Внутренние обработчики событий подключения
// =============================================================================
void BLEComm::onConnect(uint16_t conn_handle) {
  m_conn_handle = conn_handle;
  m_connected = true;
  ESP_LOGI(TAG, "Client connected (conn_handle=%d)", conn_handle);
}

void BLEComm::onDisconnect() {
  m_connected = false;
  m_conn_handle = 0;
  ESP_LOGI(TAG, "Client disconnected. Restarting advertising...");
  startAdvertising();
}

// =============================================================================
// NimBLE sync callback: вызывается когда стек готов к работе
// =============================================================================
void BLEComm::onNimbleSync() {
  ESP_LOGI(TAG, "NimBLE synced — starting advertising");
  BLEComm::instance().startAdvertising();
}

// =============================================================================
// NimBLE reset callback: вызывается при аварийном сбросе стека
// =============================================================================
void BLEComm::onNimbleReset(int reason) {
  ESP_LOGW(TAG, "NimBLE host reset (reason=%d)", reason);
}

// =============================================================================
// GAP event handler — обрабатывает подключение/отключение/обновление MTU
// =============================================================================
int BLEComm::gapEventHandler(struct ble_gap_event *event, void * /*arg*/) {
  BLEComm &self = BLEComm::instance();

  switch (event->type) {
  case BLE_GAP_EVENT_CONNECT:
    if (event->connect.status == 0) {
      self.onConnect(event->connect.conn_handle);
    } else {
      ESP_LOGW(TAG, "Connection failed (status=%d)", event->connect.status);
      self.startAdvertising();
    }
    return 0;

  case BLE_GAP_EVENT_DISCONNECT:
    self.onDisconnect();
    return 0;

  case BLE_GAP_EVENT_MTU:
    ESP_LOGI(TAG, "MTU updated: conn=%d mtu=%d", event->mtu.conn_handle,
             event->mtu.value);
    return 0;

  default:
    return 0;
  }
}

// =============================================================================
// GATT access handler — вызывается при чтении/записи характеристик
// =============================================================================
int BLEComm::gattAccessHandler(uint16_t /*conn_handle*/, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt,
                               void * /*arg*/) {
  BLEComm &self = BLEComm::instance();

  // Нас интересует только WRITE (RX характеристика, ПК → ESP32)
  if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
    uint16_t data_len = OS_MBUF_PKTLEN(ctxt->om);
    uint8_t buf[BLE_COMM_MTU];

    if (data_len > BLE_COMM_MTU) {
      ESP_LOGW(TAG, "RX: packet too large (%d bytes), truncating", data_len);
      data_len = BLE_COMM_MTU;
    }

    int rc = ble_hs_mbuf_to_flat(ctxt->om, buf, data_len, &data_len);
    if (rc != 0) {
      ESP_LOGE(TAG, "ble_hs_mbuf_to_flat failed: %d", rc);
      return BLE_ATT_ERR_UNLIKELY;
    }

    ESP_LOGI(TAG, "RX: received %d bytes (attr_handle=%d)", data_len,
             attr_handle);

    // TODO: здесь будет разбор команд. Пока вызываем заглушку.
    if (self.m_on_command_cb) {
      self.m_on_command_cb(buf, data_len, self.m_on_command_arg);
    }
  }
  return 0;
}

// =============================================================================
// Задача NimBLE-хоста (запускается nimble_port_freertos_init)
// =============================================================================
void BLEComm::nimbleHostTask(void * /*arg*/) {
  ESP_LOGI(TAG, "NimBLE host task started");
  nimble_port_run(); // блокирует до shutdown
  nimble_port_freertos_deinit();
}
