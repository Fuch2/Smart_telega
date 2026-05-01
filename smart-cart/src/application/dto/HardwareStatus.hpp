#pragma once

#include <chrono>
#include <string>

namespace smartcart::application::dto {

struct HardwareStatus {
    // STM32 status
    bool stm32Online {false};
    std::chrono::system_clock::time_point stm32LastSeen {};
    std::string stm32LastError {};

    // RFID status
    bool rfidOnline {false};
    std::chrono::system_clock::time_point rfidLastSeen {};
    std::string rfidLastError {};

    // Helper methods
    bool isHealthy() const {
        return stm32Online && rfidOnline;
    }

    std::string summary() const {
        std::string result;
        result += "STM32: " + std::string(stm32Online ? "ONLINE" : "OFFLINE");
        if (!stm32LastError.empty()) {
            result += " (error: " + stm32LastError + ")";
        }
        result += "\nRFID: " + std::string(rfidOnline ? "ONLINE" : "OFFLINE");
        if (!rfidLastError.empty()) {
            result += " (error: " + rfidLastError + ")";
        }
        return result;
    }
};

} // namespace smartcart::application::dto
