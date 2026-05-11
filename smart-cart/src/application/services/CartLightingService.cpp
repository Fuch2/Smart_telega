#include "application/services/CartLightingService.hpp"

#include "infrastructure/hw/stm32/Protocol.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace smartcart::application::services {

namespace domain = smartcart::domain;
namespace stm32  = smartcart::infrastructure::hw::stm32;

// ── Цвета (RGB) ──────────────────────────────────────────────────────────────
static constexpr CartLightingService::RgbColor kGreen  {  0, 200,   0};
static constexpr CartLightingService::RgbColor kYellow {255, 200,   0};
static constexpr CartLightingService::RgbColor kRed    {220,   0,   0};
static constexpr CartLightingService::RgbColor kBlack  {  0,   0,   0};

// ── Пороги urgency (%) ────────────────────────────────────────────────────────
static constexpr double kGreenMax     = 60.0;  // < 60% — зелёный
static constexpr double kYellowMax    = 90.0;  // < 90% — жёлтый
static constexpr double kTransitionW  = 10.0;  // ширина плавного перехода
static constexpr int    kBlinkPeriod  = 2;     // мигание: 1 из 2 тиков ON

// ── Helpers ───────────────────────────────────────────────────────────────────

CartLightingService::RgbColor CartLightingService::blendColors(
    RgbColor a, RgbColor b, double t)
{
    t = std::clamp(t, 0.0, 1.0);
    return {
        static_cast<uint8_t>(std::round(a.r + (b.r - a.r) * t)),
        static_cast<uint8_t>(std::round(a.g + (b.g - a.g) * t)),
        static_cast<uint8_t>(std::round(a.b + (b.b - a.b) * t))
    };
}

CartLightingService::RgbColor CartLightingService::urgencyToColor(double urgency)
{
    urgency = std::clamp(urgency, 0.0, 100.0);

    if (urgency <= kGreenMax - kTransitionW) {
        // Чисто зелёный
        return kGreen;
    }
    if (urgency <= kGreenMax) {
        // Плавный переход green → yellow
        const double t = (urgency - (kGreenMax - kTransitionW)) / kTransitionW;
        return blendColors(kGreen, kYellow, t);
    }
    if (urgency <= kYellowMax - kTransitionW) {
        // Чисто жёлтый
        return kYellow;
    }
    if (urgency <= kYellowMax) {
        // Плавный переход yellow → red
        const double t = (urgency - (kYellowMax - kTransitionW)) / kTransitionW;
        return blendColors(kYellow, kRed, t);
    }
    // Красный — вызывающий код управляет миганием
    return kRed;
}

// Маппинг строки приоритета → urgency (%)
double CartLightingService::priorityToUrgency(const std::string& priority)
{
    if (priority == "low")      return  0.0;
    if (priority == "normal")   return 40.0;
    if (priority == "medium")   return 65.0;
    if (priority == "high")     return 85.0;
    if (priority == "critical") return 96.0;
    return 40.0; // по умолчанию — normal
}

// Вычисляет urgency (%) по дедлайну: прогресс от createdAt до deadline
double CartLightingService::deadlineUrgency(const std::string& deadline,
                                            const std::string& createdAt)
{
    if (deadline.empty() || createdAt.empty()) {
        return 0.0;
    }

    // Парсим ISO-8601: "YYYY-MM-DDTHH:MM:SS" или "YYYY-MM-DD HH:MM:SS"
    auto parseIso = [](const std::string& s) -> std::time_t {
        std::tm tm{};
        std::istringstream ss(s);
        // Пробуем оба формата
        ss >> std::get_time(&tm, "%Y-%m-%dT%H:%M:%S");
        if (ss.fail()) {
            ss.clear();
            ss.str(s);
            ss >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S");
        }
        if (ss.fail()) return -1;
        tm.tm_isdst = -1;
        return std::mktime(&tm);
    };

    const std::time_t tDeadline  = parseIso(deadline);
    const std::time_t tCreated   = parseIso(createdAt);
    if (tDeadline == -1 || tCreated == -1 || tDeadline <= tCreated) {
        return 0.0;
    }

    const std::time_t tNow = std::time(nullptr);
    const double total   = static_cast<double>(tDeadline - tCreated);
    const double elapsed = static_cast<double>(tNow - tCreated);
    return std::clamp(elapsed / total * 100.0, 0.0, 100.0);
}

// ── STM32 commands ────────────────────────────────────────────────────────────

void CartLightingService::sendColor(RgbColor color)
{
    if (config_.statusLedIndices.empty()) return;

    for (const int ledIdx : config_.statusLedIndices) {
        stm32::Frame cmd;
        cmd.type  = stm32::FrameType::Cmd;
        cmd.cmdId = stm32::CommandId::LedSetSlot;
        cmd.payload = {
            static_cast<uint8_t>(ledIdx),
            color.r, color.g, color.b
        };
        stm32_.sendCommand(cmd);
    }

    stm32::Frame apply;
    apply.type  = stm32::FrameType::Cmd;
    apply.cmdId = stm32::CommandId::LedApply;
    stm32_.sendCommand(apply);
}

void CartLightingService::sendBlack()
{
    sendColor(kBlack);
}

// ── Constructor ───────────────────────────────────────────────────────────────

CartLightingService::CartLightingService(
    ports::IStm32Link& stm32Link,
    ports::IWorkflowRepository& workflowRepo,
    ports::IOrderRepository& orderRepo,
    CartLightingConfig config)
    : stm32_(stm32Link)
    , workflowRepo_(workflowRepo)
    , orderRepo_(orderRepo)
    , config_(std::move(config))
{}

// ── tick() ────────────────────────────────────────────────────────────────────

void CartLightingService::tick()
{
    if (config_.statusLedIndices.empty()) return;

    try {
        const auto workflow = workflowRepo_.get();

        // Нет активного заказа → гасим
        if (!workflow.currentOrderId.has_value() ||
            workflow.state == domain::CartWorkflowState::Free)
        {
            sendBlack();
            blinkOn_ = true;
            tickCount_ = 0;
            return;
        }

        const auto orderOpt = orderRepo_.getOrderById(*workflow.currentOrderId);
        if (!orderOpt.has_value()) {
            sendBlack();
            return;
        }
        const auto& order = *orderOpt;

        // Вычисляем urgency
        const double priUrgency  = priorityToUrgency(order.priority);
        const double deadUrgency = deadlineUrgency(order.deadline, order.createdAt);
        const double urgency     = std::max(priUrgency, deadUrgency);

        const bool blinking = (urgency > kYellowMax); // > 90% → мигание

        if (blinking) {
            // Мигание: чередуем red и off каждый тик
            ++tickCount_;
            blinkOn_ = (tickCount_ % kBlinkPeriod == 0);
            if (blinkOn_) {
                sendColor(kRed);
            } else {
                sendBlack();
            }
        } else {
            blinkOn_   = true;
            tickCount_ = 0;
            sendColor(urgencyToColor(urgency));
        }
    } catch (...) {
        // Не должны крашить polling-поток — проглатываем
    }
}

} // namespace smartcart::application::services
