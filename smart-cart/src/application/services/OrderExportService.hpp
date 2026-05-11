#pragma once

#include "application/ports/IOrderExporter.hpp"
#include "application/ports/IOrderRepository.hpp"
#include "application/ports/IEventLogger.hpp"

#include <string>

namespace smartcart::application::services {

/// Записывает JSON с результатом выполненного заказа в директорию completedOrdersDir.
/// Имя файла: <external_order_id>_result.json
/// Структура JSON совместима со входным форматом плюс поле "completion_result".
class OrderExportService : public ports::IOrderExporter {
public:
    OrderExportService(ports::IOrderRepository& orderRepo,
                       ports::IEventLogger& eventLogger,
                       std::string completedOrdersDir);

    void exportOrder(int orderId) override;

private:
    ports::IOrderRepository& orderRepo_;
    ports::IEventLogger& eventLogger_;
    std::string completedOrdersDir_;
};

} // namespace smartcart::application::services
