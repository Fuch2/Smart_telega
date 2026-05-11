#pragma once

#include "application/ports/IOrderRepository.hpp"
#include "application/ports/IStm32Link.hpp"
#include "application/ports/IWorkflowRepository.hpp"

#include <cstdint>
#include <vector>

namespace smartcart::application::services {

struct CartLightingConfig {
    int moduleId{1};
    // LED-индексы "подсветки тележки" (статусные полосы).
    // Если пустой — подсветка отключена.
    std::vector<int> statusLedIndices;
};

/// Вычисляет и отправляет цвет статусной подсветки тележки на STM32.
///
/// Цвет определяется по правилам ТЗ п.5.2:
///   urgency 0–60%  → зелёный
///   urgency 61–90% → жёлтый (с плавным переходом за 10% до порога)
///   urgency 91%+   → красный + мигание
///
/// Urgency = max(приоритет заказа, дедлайн в %)
/// Вызывать tick() с периодом ≤1 с (например из потока Stm32PollingService).
class CartLightingService {
public:
    struct RgbColor { uint8_t r, g, b; };

    CartLightingService(ports::IStm32Link& stm32Link,
                        ports::IWorkflowRepository& workflowRepo,
                        ports::IOrderRepository& orderRepo,
                        CartLightingConfig config);

    /// Обновить подсветку: пересчитать цвет → отправить на STM32.
    /// Безопасен для вызова из фонового потока.
    void tick();

private:
    void sendColor(RgbColor color);
    void sendBlack();
    static double priorityToUrgency(const std::string& priority);
    static double deadlineUrgency(const std::string& deadline,
                                  const std::string& createdAt);
    static RgbColor urgencyToColor(double urgency);
    static RgbColor blendColors(RgbColor a, RgbColor b, double t);

    ports::IStm32Link& stm32_;
    ports::IWorkflowRepository& workflowRepo_;
    ports::IOrderRepository& orderRepo_;
    CartLightingConfig config_;

    bool blinkOn_{true};   // состояние мигания (переключается каждый tick)
    int  tickCount_{0};    // счётчик вызовов для управления частотой мигания
};

} // namespace smartcart::application::services
