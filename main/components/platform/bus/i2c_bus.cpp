#include "i2c_bus.h"
#include "driver/i2c_master.h"

#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_MASTER_SCL_IO GPIO_NUM_22 // GPIO для I2C SCL
#define I2C_MASTER_SDA_IO GPIO_NUM_21 // GPIO для I2C SDA
#define I2C_MASTER_FREQ_HZ 100000     // Частота I2C шины

void i2c_bus_init(i2c_master_bus_handle_t *bus_handle) {
  i2c_master_bus_config_t i2c_bus_config = {};
  i2c_bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
  i2c_bus_config.i2c_port = I2C_MASTER_PORT;
  i2c_bus_config.scl_io_num = I2C_MASTER_SCL_IO;
  i2c_bus_config.sda_io_num = I2C_MASTER_SDA_IO;
  i2c_bus_config.glitch_ignore_cnt = 7;
  i2c_bus_config.flags.enable_internal_pullup = true;

  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, bus_handle));
}