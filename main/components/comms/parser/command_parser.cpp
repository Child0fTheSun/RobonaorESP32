#include "command_parser.h"
#include <sstream>
#include <esp_log.h>

static const char* TAG = "CommandParser";

void CommandParser::pushBytes(const uint8_t* data, size_t len) {
    if (data == nullptr || len == 0) return;

    // Добавляем пришедшие байты в буфер
    m_buffer.append(reinterpret_cast<const char*>(data), len);

    // Ищем символ переноса строки
    size_t pos;
    while ((pos = m_buffer.find('\n')) != std::string::npos) {
        // Извлекаем строку
        std::string line = m_buffer.substr(0, pos);
        
        // Удаляем извлеченную часть (вместе с \n) из буфера
        m_buffer.erase(0, pos + 1);

        // Обрабатываем извлеченную команду (игнорируя возможный \r)
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }

        if (!line.empty()) {
            processLine(line);
        }
    }
}

void CommandParser::registerHandler(const std::string& opcode, CommandHandler handler) {
    m_handlers[opcode] = handler;
}

void CommandParser::processLine(const std::string& line) {
    std::vector<std::string> args;
    std::istringstream iss(line);
    std::string token;

    // Разбиваем строку по пробелам
    while (iss >> token) {
        args.push_back(token);
    }

    if (args.empty()) {
        return; // Пустая строка
    }

    const std::string& opcode = args[0];

    auto it = m_handlers.find(opcode);
    if (it != m_handlers.end()) {
        // Вызываем зарегистрированный обработчик
        it->second(args);
    } else {
        ESP_LOGW(TAG, "Unknown command opcode: %s", opcode.c_str());
    }
}
