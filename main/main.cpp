#include "ble_comm.h"
#include "command_parser.h"
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
CommandParser parser;

extern "C" void app_main(void) {
  /* printf("Init I2C...\n");
  i2c_bus_init(&bus_handle);

  printf("Init Mecanum Platform...\n");
  platform = new MecanumPlatform(bus_handle);
  platform->init();

  printf("Init Speed Profile Generator...\n");
  profileGenerator = new SpeedProfileGenerator(platform);
  profileGenerator->init(); */

  printf("Platform ready\n");

  printf("Init BLE...\n");
  BLEComm::instance().init("RobonatorESP32");
  
  // =========================================================================
  // Регистрация команд парсера
  // =========================================================================
  
  // --- Команды управления системой ---
  parser.registerHandler("ST", [](const std::vector<std::string>& args) {
      // ST - старт выполнения
      printf("CMD: Start execution\n");
  });
  
  parser.registerHandler("SC", [](const std::vector<std::string>& args) {
      if (args.size() == 5) {
          // SC x1 y1 x2 y2 - координаты склада
          printf("CMD: Set Warehouse coords (%s, %s) to (%s, %s)\n", 
                 args[1].c_str(), args[2].c_str(), args[3].c_str(), args[4].c_str());
      }
  });
  parser.registerHandler("SP", [](const auto& args) { printf("CMD: Stop\n"); });
  parser.registerHandler("HM", [](const auto& args) { printf("CMD: Home position\n"); });

  // --- Команды ручного управления манипулятором ---
  parser.registerHandler("GR", [](const auto& args) { 
      if(args.size() > 1) printf("CMD: Gripper %s\n", args[1].c_str()); 
  });
  parser.registerHandler("MU", [](const auto& args) { printf("CMD: Manipulator UP\n"); });
  parser.registerHandler("MD", [](const auto& args) { printf("CMD: Manipulator DOWN\n"); });
  parser.registerHandler("MR", [](const auto& args) { printf("CMD: Manipulator RIGHT\n"); });
  parser.registerHandler("ML", [](const auto& args) { printf("CMD: Manipulator LEFT\n"); });
  parser.registerHandler("GRR", [](const auto& args) { 
      if(args.size() > 1) printf("CMD: Gripper Rotate %s\n", args[1].c_str()); 
  });

  // --- Команды ручного управления платформой ---
  parser.registerHandler("PF", [](const auto& args) { printf("CMD: Platform FORWARD\n"); });
  parser.registerHandler("PD", [](const auto& args) { printf("CMD: Platform BACKWARD\n"); });
  parser.registerHandler("PR", [](const auto& args) { printf("CMD: Platform RIGHT\n"); });
  parser.registerHandler("PL", [](const auto& args) { printf("CMD: Platform LEFT\n"); });
  parser.registerHandler("PRR", [](const auto& args) { printf("CMD: Platform ROTATE CW\n"); });
  parser.registerHandler("PRL", [](const auto& args) { printf("CMD: Platform ROTATE CCW\n"); });
  parser.registerHandler("PFR", [](const auto& args) { printf("CMD: Platform FWD-RIGHT 45\n"); });
  parser.registerHandler("PFL", [](const auto& args) { printf("CMD: Platform FWD-LEFT 45\n"); });
  parser.registerHandler("PDR", [](const auto& args) { printf("CMD: Platform BWD-RIGHT 45\n"); });
  parser.registerHandler("PDL", [](const auto& args) { printf("CMD: Platform BWD-LEFT 45\n"); });

  // --- Команды автоматического режима ---
  parser.registerHandler("W", [](const auto& args) {
      if(args.size() == 3) printf("CMD: Work area %s x %s\n", args[1].c_str(), args[2].c_str());
  });
  parser.registerHandler("OBJ", [](const auto& args) {
      if(args.size() == 3) printf("CMD: Object size %s x %s\n", args[1].c_str(), args[2].c_str());
  });
  parser.registerHandler("J", [](const auto& args) {
      if(args.size() == 5) printf("CMD: Job at (%s, %s, %s) action: %s\n", 
                                  args[1].c_str(), args[2].c_str(), args[3].c_str(), args[4].c_str());
  });

  // Передаем парсер как контекст (arg) в callback
  BLEComm::instance().setOnCommand(
      [](const uint8_t *data, size_t len, void *arg) {
        CommandParser* p = static_cast<CommandParser*>(arg);
        p->pushBytes(data, len);
      }, &parser);

  /* while (true) {
    vTaskDelay(pdMS_TO_TICKS(2000));
    printf("Move forward 50cm with trapezoidal profile\n");
    profileGenerator->executeLinearMotion(MotionDirection::FORWARD, 0.5f, 30.0f,
                                          0.075f, 0.075f);

    vTaskDelay(pdMS_TO_TICKS(6000)); // Ждем завершения (зависит от скорости)

    printf("Move backward 50cm with trapezoidal profile\n");
    profileGenerator->executeLinearMotion(MotionDirection::BACKWARD, 0.5f,
                                          30.0f, 0.075f, 0.075f);

    vTaskDelay(pdMS_TO_TICKS(6000));
  } */

  /* ManipulatorServos manipulator;
  manipulator.init(); */
}
