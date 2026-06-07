#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <functional>
#include <cstdint>
#include <cstddef>

class CommandParser {
public:
    // Тип обработчика команды. 
    // На вход подаются аргументы (args[0] - это сама команда (OPCODE), 
    // args[1] и далее - параметры).
    using CommandHandler = std::function<void(const std::vector<std::string>& args)>;

    // -------------------------------------------------------------------------
    // Добавление сырых байт от BLE в буфер.
    // Если в буфере появляется '\n', строка извлекается и парсится.
    // -------------------------------------------------------------------------
    void pushBytes(const uint8_t* data, size_t len);

    // -------------------------------------------------------------------------
    // Регистрация обработчика для конкретного OPCODE (например, "ST" или "MU").
    // -------------------------------------------------------------------------
    void registerHandler(const std::string& opcode, CommandHandler handler);

private:
    void processLine(const std::string& line);

    std::string m_buffer;
    std::unordered_map<std::string, CommandHandler> m_handlers;
};
